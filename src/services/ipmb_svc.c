/*
 * ipmb_svc.c
 *
 *  Created on: Oct 12, 2010
 *      Author: tgorski
 */

#include <stdio.h>
#include <string.h>
#include "swevent.h"
#include "utils.h"
#include "timer_callback.h"
#include "sio_usart.h"
#include "twidriver.h"
#include "rtc.h"
#include "ipmb_svc.h"
#include "ipmi_cmd_parser.h"
#include "sensor_svc.h"
#include "LEDdrivers.h"

#include "pins.h"
#include "gpio.h"

// request message table
#define REQ_MSG_TBL_SIZE			(16)			// size of request message table -- make bigger in case more sensors crap out?
#define REQ_MSG_TIMEOUT_LIMIT		(3)		// 100 ms ticks until message times out
#define REQ_MSG_XMT_TRY_LIMIT		(5)			// max number of times message transmitted until we give up

typedef struct {
  unsigned char ocflag_rqSeq;				// entry occupied flag (bit 7) plus msg rqSeq in bit 5-0
  ipmb_msg_desc_t msg;						// message buffer and length field
  unsigned char xmtcnt;						// times transmitted
  unsigned char timer;						// 100ms ticks since last transmit
  ptrMsgCallback prspcallback;				// pointer to callback function
} req_msg_tbl_entry_t;

void raw_byte_char_dump(const unsigned char* puc, const unsigned int len);
void process_rsp_msg(ipmb_msg_desc_t* pmsg);
int reserve_req_msg_table_entry(unsigned char msg_rqSEQ);
int xmt_retry_timer_callback(event evID, void* arg);
void check_req_msg_table(void);
void check_incoming_msgs(void);
int match_rsp_to_req(unsigned char rsp_rqSeq, unsigned char rsp_rsSA);
void free_req_msg_table_entry(int entry_index);
void request_msg_dump(ipmb_msg_desc_t* pmsg);

unsigned char event_rcvr_ipmbl_addr;
unsigned char event_rcvr_lun;

unsigned char rqSEQ;
req_msg_tbl_entry_t ReqMsgTbl[REQ_MSG_TBL_SIZE];

void ipmb_init(void) {
  rqSEQ = 1;
  //memset((void*) &ReqMsgTbl, 0, sizeof(req_msg_tbl_entry_t));       // only zeroes out first entry!!
  memset((void*) &ReqMsgTbl, 0, sizeof(ReqMsgTbl));                   // fixed at 2.2a, zeroes entire table
  event_rcvr_ipmbl_addr = DEFAULT_EVT_RCVR_IPMBL_ADDR;
  event_rcvr_lun = 0;
  register_timer_callback(xmt_retry_timer_callback, TEVENT_100MSEC_2);
}


void ipmb_init_req_hdr(unsigned char* pmsg, unsigned char rsSA, unsigned char netFn, unsigned char rsLUN,
  unsigned char rqLUN) {
  IPMB_RQ_rsSA(pmsg) = rsSA;
  IPMB_RQ_netFN_SET(pmsg, netFn);
  IPMB_RQ_rsLUN_SET(pmsg, rsLUN);
  IPMB_RQ_HXSUM(pmsg) = calc_ipmi_xsum(pmsg, 2);
  IPMB_RQ_rqSA(pmsg) = twi_state.ipmbl_addr;
  IPMB_RQ_rqSEQ_SET(pmsg, rqSEQ++);
  rqSEQ = rqSEQ & 0x3f;
  IPMB_RQ_rqLUN_SET(pmsg, rqLUN);
}


void ipmb_init_rsp_hdr(unsigned char* pmsg, const unsigned char *preq) {
  // build response header from request header
  IPMB_RS_rqSA(pmsg) = IPMB_RQ_rqSA(preq);
  IPMB_RS_netFN_SET(pmsg, (IPMB_RQ_netFN_GET(preq) | 1));
  IPMB_RS_rqLUN_SET(pmsg, IPMB_RQ_rqLUN_GET(preq));
  IPMB_RS_HXSUM(pmsg) = calc_ipmi_xsum(pmsg,2);
  IPMB_RS_rsSA(pmsg) = IPMB_RQ_rsSA(preq);
  IPMB_RS_rqSEQ_SET(pmsg, IPMB_RQ_rqSEQ_GET(preq));
  IPMB_RS_rsLUN_SET(pmsg, IPMB_RQ_rsLUN_GET(preq));
  IPMB_RS_CMD(pmsg)= IPMB_RQ_CMD(preq);
}


unsigned char calc_ipmi_xsum(const unsigned char* pbuf, unsigned short len) {
  unsigned char sum = 0;
  const unsigned char *pcur = pbuf;
  unsigned short remlen = len;

  while (remlen) {
    sum += *pcur++;
    remlen--;
  }
  return (~sum) + 1;
}


void ipmb_service(void) {
  CAPBUF1_SAVEPC;
  // function checks for any incoming messages and also if any retries need to be sent on
  // outgoing requests
  check_incoming_msgs();
  check_req_msg_table();
}


void check_incoming_msgs(void) {
  // this function checks the twi queue for incoming IPMI requests, and posts
  // events to allow the assigned callbacks to run
  ipmb_msg_desc_t curmsg;
  unsigned char xsumval;
  int i1;

  // zero out message buffer before get
  memset((void*) curmsg.buf, 0, sizeof(curmsg.buf));
  if (!get_twi_msg(curmsg.buf, &curmsg.len, IPMIMAXMSGLEN))
    // queue empty
    return;

#ifdef TRACE
#endif

  // check if first byte is zero.  If it is, then this is a broadcast command.  Discard
  // the zero by shifting all of the higher bytes down and decrementing the count
  if (curmsg.buf[0] == 0x00) {
    curmsg.len--;
    for (i1=0; i1<curmsg.len; i1++)
      curmsg.buf[i1] = curmsg.buf[i1+1];
  }

  // check minimum/maximum message length
  if (curmsg.len < IPMIMINMSGLEN) {
	  sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "?Error, incoming IPMI message too short\n");
	  sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, "  Raw message:  ");
	  raw_byte_char_dump(curmsg.buf, curmsg.len);
	  return;
  }

  // perform checksums on header and body
  xsumval = calc_ipmi_xsum(curmsg.buf, 2);
  if (xsumval != curmsg.buf[2]) {
	  // header xsum mismatch
	  sprintf(spbuf, "?Error, IPMI message header xsum error\n  Calculated value=%02X, Data value = %02X\n", xsumval, curmsg.buf[2]);
	  sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, spbuf);
	  sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, "  Raw Message:  ");
	  raw_byte_char_dump(curmsg.buf, curmsg.len);
	  return;
  }
  xsumval = calc_ipmi_xsum(curmsg.buf+3, curmsg.len-4);
  if (xsumval != curmsg.buf[curmsg.len-1]) {
	  // body xsum mismatch
	  sprintf(spbuf, "?Error, IPMI message body xsum error\n  Calculated value=%02X, Data value = %02X\n", xsumval, curmsg.buf[curmsg.len-1]);
	  sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, spbuf);
	  sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, " Raw Message:  ");
	  raw_byte_char_dump(curmsg.buf, curmsg.len);
    return;
  }

  // check to see if this is request or response
  if (IPMB_RQ_netFN_GET(curmsg.buf) & 1)
    process_rsp_msg(&curmsg);
  else
    // even numbered function, this is a request
    post_swevent(IPMBEV_REQ_RCVD, (void*) &curmsg);
}


void raw_byte_char_dump(const unsigned char* puc, const unsigned int len) {
  const unsigned char* pcur = puc;
  unsigned int curlen = len;

  while (curlen) {
	  sprintf(spbuf, "%02X ", *pcur++);
	  sio_filt_putstr(TXTFILT_DBG_DETAIL, 0, spbuf);
	  curlen--;
  }
  sio_filt_putstr(TXTFILT_DBG_DETAIL, 0, "\n");
}


int ipmb_send_request(ipmb_msg_desc_t* pmsg, ptrMsgCallback rspCallback) {
  // attempts to load a message into the request table and send it via the I2C interface
  // returns 1 if request loaded into table, 0 otherwise
  req_msg_tbl_entry_t* preq;
  int i1;
  int tblidx = reserve_req_msg_table_entry(IPMB_RQ_rqSEQ_GET(pmsg->buf));
  if (tblidx == -1) {
    sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "?Error, request message table full\n");
    return 0;				// no entries available
  }

  // copy message into table
  preq = &ReqMsgTbl[tblidx];
//  for (i1=0; i1<pmsg->len; i1++)
  for (i1=0; i1<sizeof(pmsg->buf); i1++)       // New at 2.2a--copy entire buffer, not just active length
    preq->msg.buf[i1] = pmsg->buf[i1];
  preq->msg.len = pmsg->len;
  preq->prspcallback = rspCallback;				// copy callback pointer
  request_msg_dump(pmsg);
#ifdef TRACE
  if (get_cap_buf_ena())
    cap_buf_capture_TXreq(pmsg);
#endif
  // try to transmit message
  if (!put_twi_msg(preq->msg.buf, preq->msg.len)) {
    sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? Queue full, request message NOT enqueued\n  ");
  }
  else {
	  preq->timer = 0;				// zero out timer
	  preq->xmtcnt++;				// bump transmit count
  }
	  
  return 1;
}


int ipmb_send_response(ipmb_msg_desc_t* pmsg) {
  // sends a response to the I2C driver.  Returns 1 if driver accepts message, 0 otherwise
  if (pmsg->len) {
    // add data checksum
    pmsg->buf[pmsg->len-1] = calc_ipmi_xsum(pmsg->buf+3, pmsg->len-4);
    if (!put_twi_msg(pmsg->buf, pmsg->len)) {
      sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? Queue full, response message NOT enqueued\n  ");
      raw_byte_char_dump(pmsg->buf, pmsg->len);
      return 0;
    }
   return 1;
  }
  return 0;
}


void process_rsp_msg(ipmb_msg_desc_t* pmsg) {
  // first look for a match in the request msg table
  req_msg_tbl_entry_t* preq;
  int tblidx = match_rsp_to_req(IPMB_RS_rqSEQ_GET(pmsg->buf), IPMB_RS_rsSA(pmsg->buf));
  if (tblidx == -1) {
    sprintf(spbuf, "(%s)  Unmatched received response:\n", systimestr());
    sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, spbuf);
    ipmb_msg_dump(pmsg, TXTFILT_DBG_DETAIL);
    post_swevent(IPMBEV_UNMATCHED_RSP_MSG, (void*) pmsg);
    return;
  }
  preq = &ReqMsgTbl[tblidx];
  // check netFN and command code to determine if this is a event message
  if ((IPMB_RQ_netFN_GET(preq->msg.buf) == NETFN_SE) && (IPMB_RQ_CMD(preq->msg.buf) == IPMICMD_SE_PLATFORM_EVENT)) {
    sprintf(spbuf, "(%s) Received Response, Completion Code=0x%02x", systimestr(), IPMB_RS_CCODE(pmsg->buf));
    sio_filt_putstr(TXTFILT_EVENTS, 1, spbuf);
    if (IPMB_RS_CCODE(pmsg->buf) == IPMI_RS_NORMAL_COMP_CODE)
      sio_filt_putstr(TXTFILT_EVENTS, 0, " (normal)\n");
    else
      sio_filt_putstr(TXTFILT_EVENTS, 0, "\n");
  }	
  // if a callback is defined, make the call for further processing of the response message
  if (preq->prspcallback != NULL)
	  (*preq->prspcallback)((void*) &(preq->msg), (void*) pmsg);			// request is arg1, response is arg2

  free_req_msg_table_entry(tblidx);										// delete request from table
}


int reserve_req_msg_table_entry(unsigned char msg_rqSEQ) {
  // finds an unused location in the request message table and reserves it
  // returns the table index if successful, otherwise -1
  int i1;
  req_msg_tbl_entry_t* preq = &ReqMsgTbl[0];
  for (i1=0; i1<REQ_MSG_TBL_SIZE; i1++) {
	if (!(preq->ocflag_rqSeq & 0x80)) {
	  // this one is free
	  preq->msg.len = 0;				// no message in buffer yet
	  preq->xmtcnt = 0;
	  preq->ocflag_rqSeq = 0x80 | msg_rqSEQ;		// set reservation key based on rqSEQ value
      return i1;
    }
    preq++;
  }
  return -1;			// no slots open
}


int xmt_retry_timer_callback(event evID, void* arg) {
  unsigned char i1;
  req_msg_tbl_entry_t* preq = &ReqMsgTbl[0];

  for (i1=0; i1<REQ_MSG_TBL_SIZE; i1++) {
	// bump the timer for each occupied entry in the request table
	if ((preq->ocflag_rqSeq & 0x80) && (preq->xmtcnt>0)) {
	  // this table entry is occupied and has been transmitted--bump the response timer count
      preq->timer++;
	    preq++;					// to next entry
	  }
  }
  return 1;
}

void check_req_msg_table(void) {
  // check the request message table for messages that need to be transmitted--or retransmitted
  unsigned char i1;
  unsigned char xmtcnt, timerval;
  req_msg_tbl_entry_t* preq;
  for (i1=0; i1<REQ_MSG_TBL_SIZE; i1++) {
	  preq = &ReqMsgTbl[i1];
	  if ((!(preq->ocflag_rqSeq & 0x80)) || (preq->msg.len == 0))
	    continue;					// this table position is currently empty or not ready to be transmitted
    xmtcnt = preq->xmtcnt;
    timerval = preq->timer;
    if (!xmtcnt) {
      // this request hasn't been transmitted yet, so send it
      if (!put_twi_msg(preq->msg.buf, preq->msg.len)) {
        sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? Queue full, request message NOT enqueued\n  ");
      }
      else {
      	preq->timer = 0;			// zero out timer
      	preq->xmtcnt = 1;			// initialize transmit count
      }
      return;
    }
    if (timerval < REQ_MSG_TIMEOUT_LIMIT)
      // message still waiting for response to last xmt, skip it
      continue;
    if (xmtcnt >= REQ_MSG_XMT_TRY_LIMIT) {
      // this message has been around too long without getting a response--delete it
      sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? No ack received for request message, giving up on retries\n");
      post_swevent(IPMBEV_UNACK_REQ_MSG, (void*) &preq->msg);
	    // if this request registered a callback function, make the callback, with a NULL value for the response message
      if (preq->prspcallback != NULL)
	      (*preq->prspcallback)((void*) &(preq->msg), NULL);			// request is arg1, NULL for arg2
      free_req_msg_table_entry(i1);
      continue;
    }
    // message needs retransmit
    if (!put_twi_msg(preq->msg.buf, preq->msg.len)) {
      sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "? Queue full, request message NOT enqueued\n  ");
	    // A full message queue indicates that the I2C bus is crashed, and I2C messages are getting NAK'd.  In this case it is
	    // fair to say that something major has gone awry in the crate, and a reset of some sort will be forthcoming.  In the mean time
		  // the transmit count will be bumped and the timer zeroed out, in order to limit the number of retries into a stuck queue.
		  // This is no fix for the underlying problem of a hosed I2C bus, but it will stop the retry engine from calling 'put_twi_msg' an
		  // unlimited number of times for this message
		  preq->timer = 0;
		  preq->xmtcnt++;
    }
    else {
  	  preq->timer = 0;				// zero out timer
  	  preq->xmtcnt++;				// bump transmit count
    }
    return;
  }
}


int match_rsp_to_req(unsigned char rsp_rqSeq, unsigned char rsp_rsSA) {
  // scans request message table, looking for a request with the matching rqSeq and rsSA
  // returns index of entry if it finds one, otherwise -1 if it doesn't
  unsigned char key = 0x80 | (rsp_rqSeq & 0x3f);			// matching key
  int i1;
  req_msg_tbl_entry_t* preq = &ReqMsgTbl[0];
  for (i1=0; i1<REQ_MSG_TBL_SIZE; i1++) {
	  if (preq->ocflag_rqSeq == key)
	    if (IPMB_RQ_rsSA(preq->msg.buf) == rsp_rsSA)
		  // have match also on rsSA
	      return i1;
    preq++;
  }
  return -1;
}


void free_req_msg_table_entry(int entry_index) {
  // function to free up an entry in the request message table
  if ((entry_index <0) || (entry_index >= REQ_MSG_TBL_SIZE))
	  return;			// out of range
  ReqMsgTbl[entry_index].ocflag_rqSeq = 0x00;				// clear occupied flag

}

void ipmb_msg_dump(const ipmb_msg_desc_t* pmsg, int filtcode) {
  // formatted dump of message to console
  int remlen;
  if (!(IPMB_RQ_netFN_GET(pmsg->buf) & 1)) {
    sio_filt_putstr(filtcode, 1, "rsSA  netFn  rsLUN  Hxsum  rqSA  rqSeq  rqLUN  cmd  Mxsum\n");
    sprintf(spbuf, " %02x    %02x      %01x     %02x     %02x    %02x      %01x    %02x    %02x\n",
      IPMB_RQ_rsSA(pmsg->buf), IPMB_RQ_netFN_GET(pmsg->buf), IPMB_RQ_rsLUN_GET(pmsg->buf),
      IPMB_RQ_HXSUM(pmsg->buf), IPMB_RQ_rqSA(pmsg->buf), IPMB_RQ_rqSEQ_GET(pmsg->buf),
      IPMB_RQ_rqLUN_GET(pmsg->buf), IPMB_RQ_CMD(pmsg->buf), pmsg->buf[pmsg->len-1]);
    sio_filt_putstr(filtcode, 1, spbuf);
    remlen = pmsg->len-IPMB_RQ_DATA_OFFSET-1;
    sio_filt_putstr(filtcode, 1, "Data Dump:\n");
    if (remlen) {
      raw_byte_char_dump(&pmsg->buf[IPMB_RQ_DATA_OFFSET], remlen);
      sio_filt_putstr(filtcode, 0, "\n");
    }
    else
      sio_filt_putstr(filtcode, 0, "(no data)\n\n");
  }
  else {
    sio_filt_putstr(filtcode, 1, "rqSA  netFn  rqLUN  Hxsum  rsSA  rqSeq  rsLUN  cmd  ccode  Mxsum\n");
    sprintf(spbuf, " %02x    %02x      %01x     %02x     %02x    %02x      %01x    %02x    %02x    %02x\n",
      IPMB_RS_rqSA(pmsg->buf), IPMB_RS_netFN_GET(pmsg->buf), IPMB_RS_rqLUN_GET(pmsg->buf),
      IPMB_RS_HXSUM(pmsg->buf), IPMB_RS_rsSA(pmsg->buf), IPMB_RS_rqSEQ_GET(pmsg->buf),
      IPMB_RS_rsLUN_GET(pmsg->buf), IPMB_RS_CMD(pmsg->buf), IPMB_RS_CCODE(pmsg->buf), pmsg->buf[pmsg->len-1]);
    sio_filt_putstr(filtcode, 1, spbuf);
    sio_filt_putstr(filtcode, 1, "Data Dump:\n");
    remlen = pmsg->len-IPMB_RS_DATA_OFFSET-1;
    if (remlen) {
      raw_byte_char_dump(&pmsg->buf[IPMB_RS_DATA_OFFSET], remlen);
      sio_filt_putstr(filtcode, 0, "\n");
    }
    else
      sio_filt_putstr(filtcode, 0, "(no data)\n\n");
  }
}


void ipmb_set_event_rcvr_ipmb_addr(unsigned char ipmbl_addr) {
  event_rcvr_ipmbl_addr = ipmbl_addr;
}


unsigned char ipmb_get_event_rcvr_ipmb_addr(void) {
  return event_rcvr_ipmbl_addr;
}


void ipmb_set_event_rcvr_lun(unsigned char lun) {
  event_rcvr_lun = lun;
}


unsigned char ipmb_get_event_rcvr_lun(void) {
  return event_rcvr_lun;
}


void request_msg_dump(ipmb_msg_desc_t* pmsg) {

  const char* Threshold_Event_str[] = {"Lower Non-Critical going low",
    "Lower Non-Critical going high",
    "Lower Critical going low",
    "Lower Critical going high",
    "Lower Non-Recoverable going low",
    "Lower Non-Recoverable going high",
    "Upper Non-Critical going low",
    "Upper Non-Critical going high",
    "Upper Critical going low",
    "Upper Critical going high",
    "Upper Non-Recoverable going low",
    "Upper Non-Recoverable going high" };
	const char* Digital_Event_str[] = {"Inactive", "Active"};
	const char* Severity_Event_str[] = {"OK", "OK to Noncrit", "Crit from Lower", "Nonrec from Lower",
	  "Noncrit from Higher", "Crit from Nonrec", "Nonrecoverable", "Monitor", "Informational"};
  const char* Hotswap_Event_str[] = {"Handle closed",
    "Handle_opened",
    "Quiesced",
    "Backend_power failure",
    "Backend power shutdown"};
	const char* Config_Event_str[] = {"firmware load done (0x0001)", "FPGA firmware req (0x0002)", "FPGA 0 SPI detect (0x0004)",
	  "FPGA 0 req cfg (0x0008)", "FPGA 0 cfg ready (0x0010)",
	  "FPGA 1 SPI detect (0x0020)", "FPGA 1 req cfg (0x0040)", "FPGA 1 cfg rdy (0x0080)",
	  "FPGA 2 SPI detect (0x0100)", "FPGA 2 req cfg (0x0200)", "FPGA 2 cfg rdy (0x0400)" };
	const char* Event_Direction_str[] = {"assert", "deassert" };

  // dump of request message
  int code_offset, sensor_type, sensor_number, event_type, assert_flag, messageID, sensoreventID;
  char sensor_namestr[20];
  const unsigned char* prqdata = &pmsg->buf[IPMB_RQ_DATA_OFFSET-1];   // offset pointer for 1-base refs to req data
  // decode the message to get key parameters
  sensor_number = prqdata[3];
  sensor_type = prqdata[2];
  event_type = prqdata[4] & 0x7f;
  assert_flag = prqdata[4] & 0x80 ? 1 : 0;
  get_sensor_name_str(sensor_number, &sensor_namestr[0]);         // get null-terminated string containing sensor name from SDR
  code_offset = prqdata[5] & 0xf;  
  
  // define the message ID as a 2-byte code, consisting of the netFN in the top byte, and the command ID in the bottom byte
  messageID = (IPMB_RQ_netFN_GET(pmsg->buf) << 8) | IPMB_RQ_CMD(pmsg->buf);
  // define the sensor/event ID as a 2-byte code, consisting of the sensorID type in the top byte, and the event ID in the bottom byte
  sensoreventID = (sensor_type << 8) | event_type;
  
  switch (messageID) {
	  case ((NETFN_SE << 8) | IPMICMD_SE_PLATFORM_EVENT) :
	    // decode platform event message
		  // display general header
		  sprintf(spbuf, "(%s  rqSEQ=0x%02x) Sensor %i (%s) Event: ", systimestr(), IPMB_RQ_rqSEQ_GET(pmsg->buf), sensor_number, sensor_namestr);
      sio_filt_putstr(TXTFILT_EVENTS, 1, spbuf);
		  
		  switch (sensoreventID) {
			  case ((TEMPERATURE_SENSOR_TYPE << 8) | THRESHOLD_EVENT_READING_TYPE):
			  case ((VOLTAGE_SENSOR_TYPE << 8) | THRESHOLD_EVENT_READING_TYPE):
			    if (code_offset <= SENSOREV_UPPER_NONRECOVER_HIGH_OFFSET)
			      sprintf(spbuf, "%s %s state, Thr=0x%02x, Val=0x%02x\n", Event_Direction_str[assert_flag], Threshold_Event_str[code_offset], prqdata[7], prqdata[6]);
					else
			      sprintf(spbuf, "%s Offset=%i, Thr=0x%02x, Val=0x%02x\n", Event_Direction_str[assert_flag], code_offset, prqdata[7], prqdata[6]);
				  sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);
				  break;
		
			  case ((POWER_SUPPLY_SENSOR_TYPE << 8) | DIGITAL_EVENT_READING_TYPE):
			    if (code_offset <= DIGITALEV_STATE_ASSERTED)
				    sprintf(spbuf, "%s %s state\n", Event_Direction_str[assert_flag], Digital_Event_str[code_offset]);
					else
					  sprintf(spbuf, "%s Offset=%i\n", Event_Direction_str[assert_flag], code_offset);
					sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);
					break;
			  
			  case ((MODULE_BOARD_SENSOR_TYPE << 8) | SEVERITY_EVENT_READING_TYPE):
			    if (code_offset <= ALARMLVLEV_INFORMATIONAL)
				    sprintf(spbuf, "%s %s state\n", Event_Direction_str[assert_flag], Severity_Event_str[code_offset]);
					else
					  sprintf(spbuf, "%s Offset=%i\n", Event_Direction_str[assert_flag], code_offset);
					sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);
					break;
					
			  
			  case ((HOTSWAP_SENSOR_TYPE << 8) | SENSOR_SPECIFIC_READING_TYPE):
			    if (code_offset <= HOTSWAP_EVENT_BACKEND_SHUTDOWN)
				    sprintf(spbuf, "%s %s state\n", Event_Direction_str[assert_flag], Hotswap_Event_str[code_offset]);
					else
					  sprintf(spbuf, "%s Offset=%i\n", Event_Direction_str[assert_flag], code_offset);
					sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);
					break;
			  
			  case ((FPGA_CONFIG_SENSOR_TYPE << 8) | SENSOR_SPECIFIC_READING_TYPE):
			    if (code_offset <= FPGACFGEV_CFGRDY2)
				    sprintf(spbuf, "%s %s state\n", Event_Direction_str[assert_flag], Config_Event_str[code_offset]);
					else
					  sprintf(spbuf, "%s Offset=%i\n", Event_Direction_str[assert_flag], code_offset);
					sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);
					break;
			  
			  default:
			    // unknown sensor type/event type combination
				  sprintf(spbuf, "Event type=0x%02x  Sensor type=0x%02x Offset=%i Direction=%s\n", event_type, sensor_type, code_offset, Event_Direction_str[assert_flag]);
          sio_filt_putstr(TXTFILT_EVENTS, 0, spbuf);
		  }
		  break;
	  
	  default:
	    // undecoded default dump for event messages
	    //sprintf(spbuf, "(systime 0x%08lx) Sending Request:\n", systime);
	    sprintf(spbuf, "(%s) Sending Request of length %i bytes:\n", systimestr(), pmsg->len);
      sio_filt_putstr(TXTFILT_EVENTS, 1, spbuf);
      ipmb_msg_dump(pmsg, TXTFILT_EVENTS);
	  
  }
}

