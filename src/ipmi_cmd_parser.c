/*
 * ipmi_cmd_parser.c
 *
 *  Created on: Oct 21, 2010
 *      Author: tgorski
 */


#include <avr32/io.h>
#include <stdio.h>
#include <string.h>
#include "mmc_version.h"
#include "utils.h"
#include "swevent.h"
#include "timer_callback.h"
#include "ejecthandle.h"
#include "adc.h"
#include "rtc.h"
#include "sio_usart.h"
#include "twidriver.h"
#include "LEDdrivers.h"
#include "nonvolatile.h"
#include "ipmb_svc.h"
#include "ipmi_cmd_parser.h"
#include "sensor_svc.h"
#include "payload_mgr.h"
#include "spi1.h"
#include "fpgaspi.h"
#include "fpga_autoconfig.h"

typedef struct {
  unsigned char eep_erase_key[4];
  unsigned long issue_time;
} EEP_erase_key_rec_t;

#define EEP_KEY_VALID_DURATION            (30)          // length of time in seconds that an issued erase key is valid
#define EEP_ERASE_KEY_SUBFUNC             (0xaa)        // subfunction code for obtaining erase key
#define EEP_ERASE_EXECUTE_SUBFUNC         (0x55)        // subfunction code for performing actual EEPROM erase

// storage for IPMI command override LED activitiy descriptors
LED_activity_desc_t IPMI_LED_overrides[IPMI_LED_CNT];

// storage for EEP erase command key
EEP_erase_key_rec_t EEP_erase_key = {{0, 0, 0, 0}, 0xffff8000};

int ipmi_req_parser(event eventID, void* parg);

void parse_App_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

void parse_SensEvt_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

void parse_Storage_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

void parse_PICMG_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

void parse_UWMMC_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

void parse_CMS_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp);

void ipmi_cmd_parser_init(void) {
  register_swevent_callback(ipmi_req_parser, IPMBEV_REQ_RCVD);		// register for received requests
}


int ipmi_req_parser(event eventID, void* parg) {
  // this routine is called through the swevent handler in response to the posting of a
  // IPMBEV_REQ_RCVD event by the IPMB service
  const ipmb_msg_desc_t* preq = (const ipmb_msg_desc_t*) parg;
  ipmb_msg_desc_t rspmsg;
  unsigned char netFN;

  // initialize response message header
  memset((void *) &rspmsg.buf, 0, sizeof(rspmsg.buf));    // zero out entire response message first
  ipmb_init_rsp_hdr(rspmsg.buf, preq->buf);			// response header generated from request header
  rspmsg.len = IPMBRSPOVHEAD; 				// length of response including hdr, comp code and xsums, but no data
  // get netFN
  netFN = IPMB_RQ_netFN_GET(preq->buf);

  switch(netFN) {
    case NETFN_SE:
      parse_SensEvt_cmds(preq, &rspmsg);
      break;

    case NETFN_STORAGE:
      parse_Storage_cmds(preq, &rspmsg);
      break;

    case NETFN_APP:
      parse_App_cmds(preq, &rspmsg);
      break;

    case NETFN_UWMMC:
      parse_UWMMC_cmds(preq, &rspmsg);
      break;

    case NETFN_CMS:
      parse_CMS_cmds(preq, &rspmsg);
      break;

    case NETFN_GRPEXT:
  	  if (preq->len < NETFN_PICMG_MIN_RQ_LENGTH) {
  		  // msg too short
  	  	rspmsg.len = 0;
    		break;
  	  }
      if (preq->buf[IPMB_RQ_DATA_OFFSET] == NETFN_PICMG_IDENTIFIER)
      	parse_PICMG_cmds(preq, &rspmsg);
      else
       	rspmsg.len = 0;
      break;

    // these are unsupported
    case NETFN_FIRMWARE:
    case NETFN_TRANSPORT:
    case NETFN_CHASSIS:
    case NETFN_BRIDGE:
    // completely unknown netFN
    default:
      rspmsg.len = 0;
  }

  // check response message length--if zero, it has been trapped as a bad or as-yet-unsupported
  // message
  if (rspmsg.len)
  	ipmb_send_response(&rspmsg);
  else {
  	sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "?Invalid or Unsupported Request Message:\n");
    ipmb_msg_dump(preq, TXTFILT_DBG_DETAIL);
    rspmsg.len = IPMBRSPOVHEAD;			// put length back
    IPMB_RS_CCODE(rspmsg.buf) = IPMI_RS_INVALID_CMD;
	  ipmb_send_response(&rspmsg);
  }

  return 1;					// callback stays resident
}


void parse_App_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp) {
//  const unsigned char* prqdata = &preq->buf[IPMB_RQ_DATA_OFFSET-1];		// offset pointer for 1-base refs to req data
  unsigned char* prsdata = &prsp->buf[IPMB_RS_DATA_OFFSET-1];			// offset pointer for 1-base refs to rsp data
  App_Device_ID_record_t devIDrec;

  switch (IPMB_RQ_CMD(preq->buf)) {
    case IPMICMD_APP_GET_DEVICE_ID:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_APP_GET_DEVICE_ID\n");
      // read device ID record from EEPROM
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;				// node is busy, can't get EEPROM access
        break;
      }
      eepspi_read((unsigned char*) &devIDrec, APP_DEV_ID_BYTE_OFFSET, sizeof(App_Device_ID_record_t));
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = devIDrec.deviceID;
      prsdata[3] = devIDrec.devicerev;
      prsdata[4] = devIDrec.fwrev1;
      prsdata[5] = devIDrec.fwrev2;
      prsdata[6] = devIDrec.ipmivers;
      prsdata[7] = devIDrec.adddevsuppt;
      prsdata[8] = devIDrec.manfIDlbyte;
      prsdata[9] = devIDrec.manfIDmidbyte;
      prsdata[10] = devIDrec.manfIDhbyte;
      prsdata[11] = devIDrec.prodIDlbyte;
      prsdata[12] = devIDrec.prodIDhbyte;
      prsp->len += 11;
      break;

    case IPMICMD_APP_COLD_RESET:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_APP_COLD_RESET\n");
      // use FRU control command to generate cold reset
      pyldmgr_ipmicmd_fru_ctrl(FRU_CTLCODE_COLD_RST);
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      break;

    default:
      prsp->len = 0;			// nothing supported here yet
  }
}


void parse_SensEvt_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp) {
  const unsigned char* prqdata = &preq->buf[IPMB_RQ_DATA_OFFSET-1];		// offset pointer for 1-base refs to req data
  unsigned char* prsdata = &prsp->buf[IPMB_RS_DATA_OFFSET-1];			// offset pointer for 1-base refs to rsp data
  unsigned short rsvID, recordID, nextrecID;
  SDR_entry_hdr_t* pSDRhdr;
  SDR_type_01h_t* pSensorSDR;
  unsigned char* pSDRdata;
  unsigned char* prsbyte;
  unsigned char rdlength;
  int i1;

  switch (IPMB_RQ_CMD(preq->buf)) {
    case IPMICMD_SE_SET_EVENT_RECEIVER:
      sio_filt_putstr(TXTFILT_EVENTS, 1, "IPMICMD_SE_SET_EVENT_RECEIVER\n");
      ipmb_set_event_rcvr_ipmb_addr(prqdata[1]);
      ipmb_set_event_rcvr_lun(prqdata[2] & 0x3);
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
	    update_sensor_events();                // signals that sensor events should be reissued
      break;

    case IPMICMD_SE_GET_EVENT_RECEIVER:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_EVENT_RECEIVER\n");
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = ipmb_get_event_rcvr_ipmb_addr();
      prsdata[3] = ipmb_get_event_rcvr_lun() & 0x3;
      prsp->len += 2;
      break;

    case IPMICMD_SE_GET_DEVICE_SDR_INFO:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_DEVICE_SDR_INFO\n");
     IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      if (IPMB_RQ_rsLUN_GET(preq->buf) != 0)
        prsdata[2] = 0;								// no sensors or devices for LUN != 0
      else {
        if (preq->len <= IPMBREQOVHEAD) {
      	  // operation data byte missing, so by definition return the number of sensors
          prsdata[2] = SDRstate.sensor_cnt;
			    sprintf(spbuf, "%i Sensors Reported to Mgr\n", SDRstate.sensor_cnt);
          sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, spbuf);
        }
        else {
      	  if (prqdata[1] & 0x1) {
      		  prsdata[2] = SDRstate.SDR_cnt;
			      sprintf(spbuf, "%i SDRs Reported to Mgr\n", SDRstate.SDR_cnt);
            sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, spbuf);
			    }			    
          else {
            prsdata[2] = SDRstate.sensor_cnt;
			      sprintf(spbuf, "%i Sensors Reported to Mgr\n", SDRstate.sensor_cnt);
            sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, spbuf);
		      }			    
        }
      }
      prsdata[3] = 0x01;				// per AMC.0 3.11.1, static sensors, LUN 0 only
      prsp->len += 2;
      break;

    case IPMICMD_SE_GET_DEVICE_SDR:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_DEVICE_SDR\n");
      rsvID = prqdata[2] << 8 | prqdata[1];			// reservation ID for this command
      if ((prqdata[5] != 0) && (rsvID != SDRstate.SDRreservationID)) {
        // reservation ID doesn't match currently active one, so send an error completion code
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RSV_ID;
        break;
      }
      recordID = prqdata[4] << 8 | prqdata[3];
      pSDRdata = get_SDR_entry_addr(recordID);
      if (!pSDRdata) {
        // record ID out of range
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
      pSDRhdr = (SDR_entry_hdr_t*) pSDRdata;
      if (prqdata[6] == 0xff)
	      // return rest of record (from offset)
    	  rdlength = pSDRhdr->reclength - prqdata[5];
      else
        rdlength = prqdata[6];

      if (rdlength > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-3)) {
    	  IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
    	  break;
      }
  	  else {
  	    // return read data
  	    IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
  	    nextrecID = get_SDR_next_recID(pSDRhdr);
  	    prsdata[2] = nextrecID & 0xff;
  	    prsdata[3] = nextrecID >> 8;
        prsp->len += 2+rdlength;
 	      prsbyte = &prsdata[4];
 	      pSDRdata += prqdata[5];         // advance data pointer to starting offset
 	      while (rdlength) {
 	        *prsbyte++ = *pSDRdata++;
 	        rdlength--;
 	      }
 		    if (!prqdata[5]) {
				  // null offset, return info about read & next record
				  sprintf(spbuf, "Read SDR Rec ID = 0x%04x, next SDR Rec ID = 0x%04x\n", recordID, nextrecID);
	        sio_filt_putstr(TXTFILT_DBG_DETAIL, 1, spbuf);
			  }			  
      }
      break;

    case IPMICMD_SE_RES_DEVICE_SDR_REPOSITORY:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_RES_DEVICE_SDR_REPOSITORY\n");
      // bump SDR repository reservation counter and return the value
      SDRstate.SDRreservationID++;
	    IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = SDRstate.SDRreservationID & 0xff;
      prsdata[3] = SDRstate.SDRreservationID >> 8;
      prsp->len += 2;
      break;

    case IPMICMD_SE_GET_SENSOR_HYSTERESIS:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_HYSTERESIS\n");
      // find SDR number from sensor number
      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
      if (pSensorSDR == NULL) {
        // bad sensor ID
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      if (((pSensorSDR->sensorcap & 0x30) < 0x10) || ((pSensorSDR->sensorcap & 0x30) > 0x20) || (pSensorSDR->event_reading_type != THRESHOLD_EVENT_READING_TYPE)) {
        // hysteresis not readable on this sensor
		    IPMB_RS_CCODE(prsp->buf) = IPMI_RS_COMMAND_ILLEGAL_FOR_TYPE;
      }
      else {
        prsdata[2] = pSensorSDR->pos_thr_hysteresis;
        prsdata[3] = pSensorSDR->neg_thr_hysteresis;
        prsp->len += 2;
      }
      break;

    case IPMICMD_SE_SET_SENSOR_THRESHOLD:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_SET_SENSOR_THRESHOLD\n");
      // first check to see if EEPROM is available
      if (eepspi_chk_write_in_progress()) {
    	  IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
	  
      // This command will update the sensor thresholds in both the RAM and EEPROM-based SDR records for
      // the specified sensor.
      // Here's the trick:  we're taking advantage of the fact that the thresholds all fall within the
      // same 32-byte block in the EEPROM, so we can build a buffer image of the current thresholds,
      // update those that are changed by the command, and write the settings back to EEPROM with a single
      // write operation, such that the EEPROM updates itself while the response is returned.  We avoid the
      // situation where we have to wait for one or more block writes to complete before finishing the
      // others, which would put the EEPROM internal update time for the earlier writes in the command
      // processing path, which we want to avoid--lest the controller decide to reissue the commands

      // find SDR number from sensor number
      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
      if (pSensorSDR == NULL) {
        // bad sensor ID
      	IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
	    // check to see if sensor has settable thresholds
      if (((pSensorSDR->sensorcap & 0x0c) != 0x08) || (pSensorSDR->event_reading_type != THRESHOLD_EVENT_READING_TYPE)) {
		    // no thresholds or not settable
			  IPMB_RS_CCODE(prsp->buf) = IPMI_RS_COMMAND_ILLEGAL_FOR_TYPE;
				break;
		  }
	  
      // initialize buffer with current or new threshold values
      pSDRdata = (unsigned char*) &pSensorSDR->upper_nonrecover_thr;
      for (i1=0; i1<6; i1++)
        if ((prqdata[2] & (1 << i1)) & pSensorSDR->settable_threshold_mask)
          // new value for threshold
          pSDRdata[5-i1] = prqdata[3+i1];
 	    IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      eepspi_write(pSDRdata, SDR_AREA_BYTE_OFFSET+(unsigned short) (pSDRdata - (unsigned char*) &SDRtbl), 6);
      break;

    case IPMICMD_SE_GET_SENSOR_THRESHOLD:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_THRESHOLD\n");
      // find SDR number from sensor number
      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
      if (pSensorSDR == NULL) {
        // bad sensor ID
      	IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
	  
	    if (((pSensorSDR->sensorcap & 0x0c) < 0x04) || ((pSensorSDR->sensorcap & 0x0c) > 0x08) || (pSensorSDR->event_reading_type != THRESHOLD_EVENT_READING_TYPE)) {
        // hysteresis not readable on this sensor
		    IPMB_RS_CCODE(prsp->buf) = IPMI_RS_COMMAND_ILLEGAL_FOR_TYPE;
      }

	    IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = pSensorSDR->readable_threshold_mask;
      prsdata[8] = pSensorSDR->upper_nonrecover_thr;
      prsdata[7] = pSensorSDR->upper_critical_thr;
      prsdata[6] = pSensorSDR->upper_noncritical_thr;
      prsdata[5] = pSensorSDR->lower_nonrecover_thr;
      prsdata[4] = pSensorSDR->lower_critical_thr;
      prsdata[3] = pSensorSDR->lower_noncritical_thr;
      prsp->len += 7;
      break;

    case IPMICMD_SE_GET_SENSOR_READING:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_READING\n");
      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
      if (pSensorSDR == NULL) {
        // bad sensor ID
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
      // access sensor data table to get sensor value
	    IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      if (prqdata[1] == HOTSWAP_SENSOR) {
	      // return AMC-specified response for hotswap sensor
	      prsdata[2] = 0x00;
        prsdata[3] = SensorData[prqdata[1]].event_msg_ctl | 0x40;   // scanning always enabled, plus event status
	      prsdata[4] = SensorData[HOTSWAP_SENSOR].cur_masked_comp;
		    prsp->len += 3;
	    }
	    else {
			  switch (pSensorSDR->event_reading_type) {
				  case THRESHOLD_EVENT_READING_TYPE:
	          prsdata[2] = SensorData[prqdata[1]].readout_value;
	          prsdata[3] = SensorData[prqdata[1]].event_msg_ctl | 0x40;		// scanning always enabled, plus event status
            prsdata[4] = SensorData[prqdata[1]].comparator_status;
				    prsp->len += 3;
					  break;
					  
				  case DIGITAL_EVENT_READING_TYPE:
				  case SEVERITY_EVENT_READING_TYPE:
          case SENSOR_SPECIFIC_READING_TYPE:
				    prsdata[2] = 0x00;        // ignore on discrete sensors
	          prsdata[3] = SensorData[prqdata[1]].event_msg_ctl | 0x40;		// scanning always enabled, plus event status
				    prsdata[4] = SensorData[prqdata[1]].cur_masked_comp & 0xff;     // event state assertion lines 0-7
				    prsdata[5] = ((SensorData[prqdata[1]].cur_masked_comp >> 8) & 0x7f) | 0x80;       // states 8-14, plus msb set, per IPMI spec
				    prsp->len += 4;     // extra byte for this response
				    break;
					
				  default:
            IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
            break;
			  }
      }
      break;

    case IPMICMD_SE_GET_SENSOR_EVENT_ENABLE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_GET_SENSOR_EVENT_ENABLE\n");
      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
      if (pSensorSDR == NULL) {
        // bad sensor ID
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
      if ((pSensorSDR->sensorcap & 0x3) == 0x3) {
        // sensor event/reading type not supported or per event enable/disable not allowed for this sensor
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_COMMAND_ILLEGAL_FOR_TYPE;
        break;
      }
      prsdata[2] = 0x40 | (SensorData[prqdata[1]].event_msg_ctl & SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK);
      prsdata[3] = pSensorSDR->assertion_event_mask[LOWBYTE];
      prsdata[5] = pSensorSDR->deassertion_event_mask[LOWBYTE];
      if (pSensorSDR->event_reading_type == THRESHOLD_EVENT_READING_TYPE) {
        // threshold type
        prsdata[4] = pSensorSDR->assertion_event_mask[HIGHBYTE] & 0xf;
        prsdata[6] = pSensorSDR->deassertion_event_mask[HIGHBYTE] & 0xf;
      }
      else {
        // discrete type
        prsdata[4] = pSensorSDR->assertion_event_mask[HIGHBYTE];
        prsdata[6] = pSensorSDR->deassertion_event_mask[HIGHBYTE];
      }
      prsp->len += 5;
      break;

    case IPMICMD_SE_SET_SENSOR_EVENT_ENABLE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_SE_SET_SENSOR_EVENT_ENABLE\n");
      // first check to see if EEPROM is available
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      pSensorSDR = get_sensor_SDR_addr(prqdata[1]);
      if (pSensorSDR == NULL) {
        // bad sensor ID
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
      if ((pSensorSDR->sensorcap & 0x3) == 0x3) {
        // no sensor events
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_COMMAND_ILLEGAL_FOR_TYPE;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      if (!(prqdata[2] & SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK))
        // disable event messages from this sensor
        SensorData[prqdata[1]].event_msg_ctl &= ~SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
      else
        // enable event messages from this sensor
        SensorData[prqdata[1]].event_msg_ctl |= SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK;
      if (preq->len <= 9)
        break;      // no extra bytes
      switch (prqdata[2] & SENSOREV_MSG_CTL_SELECTED_EVENT_ACTION_MASK) {
        case 0x10:
          // enable selected events
          pSensorSDR->assertion_event_mask[LOWBYTE] |= prqdata[3];
          if (preq->len >= 11)
            pSensorSDR->assertion_event_mask[HIGHBYTE] |= prqdata[4];
          if (preq->len >= 12)
            pSensorSDR->deassertion_event_mask[LOWBYTE] |= prqdata[5];
            if (preq->len >= 13)
          pSensorSDR->deassertion_event_mask[HIGHBYTE] |= prqdata[6];
          pSDRdata = (unsigned char *) &(pSensorSDR->assertion_event_mask[LOWBYTE]);
          eepspi_write(pSDRdata, SDR_AREA_BYTE_OFFSET+(unsigned short) (pSDRdata - (unsigned char*) &SDRtbl), 4);
          break;

        case 0x20:
          // disable selected events
          pSensorSDR->assertion_event_mask[LOWBYTE] &= ~prqdata[3];
          if (preq->len >= 11)
            pSensorSDR->assertion_event_mask[HIGHBYTE] &= ~prqdata[4];
          if (preq->len >= 12)
            pSensorSDR->deassertion_event_mask[LOWBYTE] &= ~prqdata[5];
          if (preq->len >= 13)
            pSensorSDR->deassertion_event_mask[HIGHBYTE] &= ~prqdata[6];
          pSDRdata = (unsigned char *) &(pSensorSDR->assertion_event_mask[LOWBYTE]);
          eepspi_write(pSDRdata, SDR_AREA_BYTE_OFFSET+(unsigned short) (pSDRdata - (unsigned char*) &SDRtbl), 4);
          break;

        case 0x00:
        case 0x30:
        default:
          // nothing to do
          break;
      }
      break;

    case IPMICMD_SE_PLATFORM_EVENT:
      // should never get one of these incoming!!
    default:
      prsp->len = 0;			// nothing supported here yet
  }
}


void parse_Storage_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp) {
  const unsigned char* prqdata = &preq->buf[IPMB_RQ_DATA_OFFSET-1];		// offset pointer for 1-base refs to req data
  unsigned char* prsdata = &prsp->buf[IPMB_RS_DATA_OFFSET-1];			// offset pointer for 1-base refs to rsp data
  unsigned short fruoffset;
  unsigned char datalen;
//  int i1;

  switch (IPMB_RQ_CMD(preq->buf)) {
    case IPMICMD_STOR_GET_FRU_INVEN_AREA_INFO:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_STOR_GET_FRU_INVEN_AREA_INFO\n");
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = FRU_AREA_SIZE_L;
      prsdata[3] = FRU_AREA_SIZE_H;
      prsdata[4] = 0;               // byte addressing
      prsp->len += 3;
      break;

    case IPMICMD_STOR_READ_FRU_DATA:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_STOR_READ_FRU_DATA\n");
      if (eepspi_chk_write_in_progress()) {
      IPMB_RS_CCODE(prsp->buf) = 0x81;                // command-specified code when resource is busy
        break;
      }
      fruoffset = prqdata[2] + (prqdata[3] << 8);
      datalen = prqdata[4];
      if (datalen > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-1)) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = datalen;
      eepspi_read(&prsdata[3], COMMON_HEADER_BYTE_OFFSET+fruoffset, datalen);
      prsp->len += datalen+1;
      break;

    case IPMICMD_STOR_WRITE_FRU_DATA:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_STOR_WRITE_FRU_DATA\n");
      if (eepspi_chk_write_in_progress()) {
      IPMB_RS_CCODE(prsp->buf) = 0x81;                // command-specified code when resource is busy
        break;
      }
      fruoffset = prqdata[2] + (prqdata[3] << 8);
      datalen = prqdata[4];
      if (datalen > (IPMIMAXMSGLEN-IPMBRSPOVHEAD-1)) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = datalen;
      eepspi_write(&prqdata[4], COMMON_HEADER_BYTE_OFFSET+fruoffset, datalen);
      prsp->len += 1;
      break;

    default:
      prsp->len = 0;			// nothing supported here yet
  }
}


void parse_PICMG_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp) {
  const unsigned char* prqdata = &preq->buf[IPMB_RQ_DATA_OFFSET-1];		// offset pointer for 1-base refs to req data
  unsigned char* prsdata = &prsp->buf[IPMB_RS_DATA_OFFSET-1];			// offset pointer for 1-base refs to rsp data
  int i1;
  LED_activity_desc_t *pLEDact;
  const LED_state_rec_t* pLEDstate;
  switch (IPMB_RQ_CMD(preq->buf)) {
    case IPMICMD_PICMG_GET_PICMG_PROP:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_PICMG_PROP\n");
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsdata[3] = 0x14;                      // per AMC Spec 3.1.5
      prsdata[4] = 0;                         // max FRU device ID
      prsdata[5] = 0;                         // FRU device ID for device containing this MMC
      prsp->len += 4;
      break;

    case IPMICMD_PICMG_FRU_CONTROL_CAPABILITIES:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_FRU_CONTROL_CAPABILITIES\n");
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      // capable of cold reset and graceful reboot
      prsdata[3] = 0x04;
      prsp->len += 2;
      break;

    case IPMICMD_PICMG_FRU_CONTROL:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_FRU_CONTROL\n");
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsp->len += 1;
      if ((prqdata[3] == 3) || (prqdata[3] > 4))
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
      else {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
        pyldmgr_ipmicmd_fru_ctrl(prqdata[3]);
      }
      break;

    case IPMICMD_PICMG_GET_FRU_LED_PROP:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_FRU_LED_PROP\n");
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsdata[3] = 0x07;        // blue LED, LED1, LED2
      prsdata[4] = 0;           // don't report any application specific LEDs
      prsp->len += 3;
      break;

    case IPMICMD_PICMG_GET_LED_COLOR_CAP:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_LED_COLOR_CAP\n");
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      switch (prqdata[3]) {
        case IPMI_BLUELED_TBL_IDX:
          prsdata[3] = 1 << LEDCOLOR_BLUE;      // blue only capability
          prsdata[4] = LEDCOLOR_BLUE;           // blue in local ctl state
          prsdata[5] = LEDCOLOR_BLUE;           // blue in override state
          prsdata[6] = 0;                       // no flags
          prsp->len += 5;
          break;

        case IPMI_LED1_TBL_IDX:
          prsdata[3] = 1 << LEDCOLOR_GREEN;      // red only capability     -- note:  AMX13XG boards built with Green LEDs, not red ones, at IPMI LED1
          prsdata[4] = LEDCOLOR_GREEN;           // red in local ctl state
          prsdata[5] = LEDCOLOR_GREEN;           // red in override state
          prsdata[6] = 0;                       // no flags
          prsp->len += 5;
          break;

        case IPMI_LED2_TBL_IDX:
          prsdata[3] = 1 << LEDCOLOR_GREEN;      // green only capability
          prsdata[4] = LEDCOLOR_GREEN;           // green in local ctl state
          prsdata[5] = LEDCOLOR_GREEN;           // green in override state
          prsdata[6] = 0;                       // no flags
          prsp->len += 5;
          break;

        default:
          // unknown LED
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
          prsp->len += 1;
          break;
      }
      break;

    case IPMICMD_PICMG_SET_FRU_LED_STATE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_SET_FRU_LED_STATE\n");
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsp->len += 2;
      for (i1=0; i1<IPMI_LED_CNT; i1++)
        if ((prqdata[3] == i1) || (prqdata[3] == 0xff)) {
          // program this LED
          pLEDact = &IPMI_LED_overrides[i1];
          switch (prqdata[4]) {
            case 0x00:
              // off override
              program_LED(i1, Override, &LED_Off_Activity);
              break;
            case 0xfb:
              // lamp test
              pLEDact->action = Blink;
              pLEDact->initstate = LEDON;
              pLEDact->delay1 = prqdata[5]*10;            // duration given in 100ms units
              pLEDact->delay2 = 0;
              program_LED(i1, Override, pLEDact);
              break;

            case 0xfc:
              // restore to local control
              program_LED(i1, Local_Control, NULL);
              break;
            case 0xfd:
            case 0xfe:
              // don't do anything
              break;
            case 0xff:
              // on override
              program_LED(i1, Override, &LED_On_Activity);
              break;
            default:
              // blink override
              pLEDact->action = Blink;
              pLEDact->initstate = LEDOFF;
              pLEDact->delay1 = prqdata[4];            // off time
              pLEDact->delay2 = prqdata[5];            // on time
              program_LED(i1, Override, pLEDact);
          }
        }
      break;

    case IPMICMD_PICMG_GET_FRU_LED_STATE:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_FRU_LED_STATE\n");
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      if (prqdata[3] >= IPMI_LED_CNT) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
        prsp->len += 1;
        break;
      }
      // need to poke around in the LED state, so turn off interrupts for a little
      // bit while we see what's going on with the LED
      Disable_global_interrupt();
      pLEDstate = &LEDstate[prqdata[3]];      // pointer to LED state
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      // response byte 3, LED states
      prsdata[3] = 0x00;
      if (pLEDstate->LEDstate == Local_Control)
        prsdata[3] |= 0x01;
      else {
        if (pLEDstate->pOvrideDesc->delay2 == 0)
          prsdata[3] |= 0x04;         // lamp test
        else
          prsdata[3] |= 0x02;         // override
      }
      // response byte 4/5/6, local control
      switch (pLEDstate->pLocalDesc->action) {
        case On:
          prsdata[4] = 0xff;
          prsdata[5] = 0;
          break;
        case Blink:
          if (pLEDstate->pLocalDesc->initstate == LEDON) {
            prsdata[4] = (unsigned char) (pLEDstate->pLocalDesc->delay2 & 0xff);
            prsdata[5] = (unsigned char) (pLEDstate->pLocalDesc->delay1 & 0xff);
          }
          else {
            prsdata[4] = (unsigned char) (pLEDstate->pLocalDesc->delay1 & 0xff);
            prsdata[5] = (unsigned char) (pLEDstate->pLocalDesc->delay2 & 0xff);
          }
          break;
        case Off:
        case Bypass:
        default:
          prsdata[4] = 0;
          prsdata[5] = 0;
      }
      prsdata[6] = pLEDstate->Color;
      prsp->len += 5;             // update length for bytes encoded so far
      // response bytes 7-9, override state
      if (prsdata[3] > 1) {
        // have override bytes to report
        prsdata[9] = pLEDstate->Color;
        switch (pLEDstate->pOvrideDesc->action) {
          case On:
            prsdata[7] = 0xff;
            prsdata[8] = 0;
            prsp->len += 3;
            break;
          case Blink:
            if (pLEDstate->pOvrideDesc->delay2 == 0) {
              // lamp test
              prsdata[7] = 0xfb;
              prsdata[8] = 0;
              prsdata[10] = (unsigned char) ((pLEDstate->pOvrideDesc->delay1/10) & 0xff);
              prsp->len += 4;
            }
            else {
              if (pLEDstate->pOvrideDesc->initstate == LEDON) {
                prsdata[7] = (unsigned char) (pLEDstate->pOvrideDesc->delay2 & 0xff);
                prsdata[8] = (unsigned char) (pLEDstate->pOvrideDesc->delay1 & 0xff);
              }
              else {
                prsdata[7] = (unsigned char) (pLEDstate->pOvrideDesc->delay1 & 0xff);
                prsdata[8] = (unsigned char) (pLEDstate->pOvrideDesc->delay2 & 0xff);
              }
              prsp->len += 3;
            }
          break;
          case Off:
          case Bypass:
          default:
            prsdata[7] = 0;
            prsdata[8] = 0;
            prsp->len += 3;
        }
      }
      Enable_global_interrupt();
      break;

    case IPMICMD_PICMG_GET_DEVICE_LOCATOR_REC_ID:
      sio_filt_putstr(TXTFILT_IPMI_STD_REQ, 1, "IPMICMD_PICMG_GET_DEVICE_LOCATOR_REC_ID\n");
      // since the AMC specificaiton calls for a max device ID of zero, we expect this
      // command to give a device ID of zero as byte 2 of the command data, which is interpreted
      // as asking for the MMC device record index in the SDR
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = NETFN_PICMG_IDENTIFIER;
      prsdata[3] = (unsigned char) (SDR_MMC & 0xff);
      prsdata[4] = (unsigned char) (SDR_MMC >> 8);                // upper byte of MMC SDR record
      prsp->len += 3;
      break;

    default:
      prsp->len = 0;      // nothing supported here yet
  }
}


void parse_UWMMC_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp) {
  const unsigned char* prqdata = &preq->buf[IPMB_RQ_DATA_OFFSET-1];   // offset pointer for 1-base refs to req data
  unsigned char* prsdata = &prsp->buf[IPMB_RS_DATA_OFFSET-1];     // offset pointer for 1-base refs to rsp data
  unsigned char xbuf[NONVOLATILE_MAX_XFER_LEN];
  unsigned char* xbptr;
  unsigned char* pmsgdata;
  unsigned char xlength, curxlen;
  unsigned short eepsaddr;
  unsigned long systime;
  int i1;
  module_current_req_record_t currentrecbuf;
  nonvolatile_settings_t PMsettingsbuf;

  switch (IPMB_RQ_CMD(preq->buf)) {
    case IPMICMD_UWMMC_SET_BACKEND_PWR:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_SET_BACKEND_PWR\n");
      // BYTE         DATA FIELD
      // Command Format:
      //   1          [7] - 1b=update setting in nonvolatile storage
      //              [6] - 1b=update current operational setting
	    //              [5:1] - unused, reserved
		  //              [0] - 1b=backend power enabled,
		  //                    0b=backend power disabled
      // Response Format:
      //   1          Completion Code.
	    if (prqdata[1] & 0x80) {
			  // need EEPROM
        if (eepspi_chk_write_in_progress()) {
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
          break;
        }
		    eepspi_read((unsigned char*) &PMsettingsbuf, PAYLDMGR_AREA_BYTE_OFFSET, 4);           // read first few bytes, enough to load settings
			  PMsettingsbuf.bkend_pwr_ena = prqdata[1] & 0x01 ? 1 : 0;                              // set non volatile power bit
			  eepspi_write((unsigned char*) &PMsettingsbuf, PAYLDMGR_AREA_BYTE_OFFSET, 4);          // rewrite bytes with modified field
		  }		
		  if (prqdata[1] & 0x40)
			  // update current working setting
        pyldmgr_ipmicmd_backend_pwr(preq);
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      break;

    case IPMICMD_UWMMC_GET_BACKEND_PWR:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_BACKEND_PWR\n");
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.
      //   2          Current operational setting for backend power (1=enabled, 0=disabled)
      //   3          Backend Power Pin State (1=enabled, 0=disabled)
	    //   4          Nonvolatile memory power enable setting (1=enabled, 0=disabled)
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
		  eepspi_read((unsigned char*) &PMsettingsbuf, PAYLDMGR_AREA_BYTE_OFFSET, 4);           // read first few bytes, enough to load settings
			prsdata[2] = pyldmgr_state.settings.bkend_pwr_ena;                                    // return backend power enable on/off setting
			prsdata[3] = (pyldmgr_get_backend_power_pin_status() == power_on) ? 1 : 0;            // return backend power current pin state
			prsdata[4] = PMsettingsbuf.bkend_pwr_ena;                                             // return nonvolatile setting       
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsp->len += 3;
      break;
	  
  
    case IPMICMD_UWMMC_SET_PYLD_MGR_SETTING_REC:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_SET_PYLD_MGR_SETTING_REC\n");
      // Sets control field values for Payload Manager.  Settings are non-volatile.  Fields
      // not being updated are ignored but should have data bytes present in the command.  At startup
	    // the settings are loaded from nonvolatile (EEPROM) storage.  Setting changes are made to the
		  // current operational and/or nonvolatile values per command parameters.  The exception is the
		  // auxiliary 12V select, for which only the nonvolatile setting can be changed, taking effect at the
		  // next initialization of the MMC.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          [7] - 1b=update setting in nonvolatile storage
      //              [6] - 1b=update current operational setting
      //              Field select mask.  Bit=1 selects associated field for update, bit=0
      //              leaves field unchanged.  Bit definitions as follows:
      //              [5] - FPGA auto-configuration enable
      //              [4] - Backend power shutdown event-enable
      //              [3] - Backend Monitor global alarm level
	    //              [2] - Auxillary 12V power select
      //              [1:0] - Unused, reserved
      //   2          FPGA auto-configuration enable setting (1=enable FPGA-auto-configuration
      //              function, 0=disable FPGA-auto-configuration)
      //   3          Backend power shutdown event-enable setting (1=send hotswap event if
      //              backend power is commanded to shut down, 0=do not send event)
      //   4          Backend Monitor global alarm mask level (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
	    //   5          Auxiliary 12V supply enable (1=enable auxiliary 12V input, 0=ignore auxiliary 12V input).  Change is made
		  //              to nonvolatile storage only
      // Response Format:
      //   1          Completion Code.
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      if (!(prqdata[1] & 0xc0))
        break;      // nothing to write
      if (preq->len < IPMBREQOVHEAD+5) {
        // not enough params
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
        break;
      }
		  if (prqdata[1] & 0x80) {
			  // apply updates to nonvolatile storage
        if (eepspi_chk_write_in_progress()) {
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
          break;
        }
		    eepspi_read((unsigned char*) &PMsettingsbuf, PAYLDMGR_AREA_BYTE_OFFSET, 4);           // read first few bytes, enough to load settings
			  if (prqdata[1] & 0x20)
			    PMsettingsbuf.autocfg_ena = prqdata[2] ? 1 : 0;
				if (prqdata[1] & 0x10)
				  PMsettingsbuf.bkend_shtdn_evt_ena = prqdata[3] ? 1 : 0;
				if (prqdata[1] & 0x08)
				  PMsettingsbuf.global_mask_level = prqdata[4] & 0x3;
				if (prqdata[1] & 0x04)
				  PMsettingsbuf.aux_12V_pwr_select = prqdata[5] ? 1 : 0;
			  eepspi_write((unsigned char*) &PMsettingsbuf, PAYLDMGR_AREA_BYTE_OFFSET, 4);          // rewrite bytes with modified field
		  }
		  if (prqdata[1] & 0x40) {
			  // apply updates to current settings
			  if (prqdata[1] & 0x20)
			    pyldmgr_state.settings.autocfg_ena = prqdata[2] ? 1 : 0;
				if (prqdata[1] & 0x10)
				  pyldmgr_state.settings.bkend_shtdn_evt_ena = prqdata[3] ? 1 : 0;
				if (prqdata[1] & 0x08)
				  pyldmgr_state.settings.global_mask_level = prqdata[4] & 0x3;
		  }
		  break;
		  
    case IPMICMD_UWMMC_GET_PYLD_MGR_SETTING_REC:

      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_PYLD_MGR_SETTING_REC\n");
      // BYTE         DATA FIELD
      // Command Format:
      //   1          [7] - 1b=return settings from nonvolatile storage, 0b=return current settings
      //              [6:0] - Unused, reserved
      // Response Format:
      //   1          Completion Code.
      //   2          FPGA auto-configuration enable setting (1=enable FPGA-auto-configuration
      //              function, 0=disable FPGA-auto-configuration)
      //   3          Backend power shutdown event-enable setting (1=send hotswap event if
      //              backend power is commanded to shut down, 0=do not send event)
      //   4          Backend Monitor global alarm mask level (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
	    //   5          Auxiliary 12V supply enable (1=enable auxiliary 12V input, 0=ignore auxiliary 12V input).  Change is made
		  //              to nonvolatile storage only
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
	    if (prqdata[1] & 0x80) {
			  // return settings from non-volatile memory
        if (eepspi_chk_write_in_progress()) {
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
          break;
        }
		    eepspi_read((unsigned char*) &PMsettingsbuf, PAYLDMGR_AREA_BYTE_OFFSET, 4);           // read first few bytes, enough to load settings
        prsdata[2] = PMsettingsbuf.autocfg_ena;
		    prsdata[3] = PMsettingsbuf.bkend_shtdn_evt_ena;
			  prsdata[4] = PMsettingsbuf.global_mask_level;
        prsdata[5] = PMsettingsbuf.aux_12V_pwr_select;
		  }
		  else {
			  // return current settings
        prsdata[2] = pyldmgr_state.settings.autocfg_ena;
		    prsdata[3] = pyldmgr_state.settings.bkend_shtdn_evt_ena;
			  prsdata[4] = pyldmgr_state.settings.global_mask_level;
        prsdata[5] = pyldmgr_state.settings.aux_12V_pwr_select;
		  }			
      prsp->len += 4;
      break;
		
    case IPMICMD_UWMMC_GET_FAULT_STATUS:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_FAULT_STATUS\n");
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.
      //   2          alarm severity level (0=normal, 1=noncritical, 2=critical, 3=nonrecoverable)
      //   3          alarm source (0=none, 1=temperature sensor, 2=voltage sensor, 3=backend device)
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = (unsigned char) pyldmgr_state.alarm.cur_alarm_level;
      prsdata[3] = (unsigned char) pyldmgr_state.alarm.cur_alarm_source;
      prsp->len += 2;
      break;

    case IPMICMD_UWMMC_SET_BACKEND_PWR_ENA_MASK:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_SET_BACKEND_PWR_ENA_MASK\n");
      // BYTE         DATA FIELD
      // Command Format:
	    //   1          Number of stages in power up sequence (N=1 to 4)
		  //   2 to N+1   Power on mask for each stage, where N is the number of stages defined in
		  //              byte 1 of the command.  Each stage mask has the following bit field
		  //              defnintion:
      //              [7:6] - Unused, reserved
	    //              [5] - mask bit for power enable 6
	    //              [4] - mask bit for power enable 5
	    //              [3] - mask bit for power enable 4
	    //              [2] - mask bit for power enable 3
	    //              [1] - mask bit for power enable 2
	    //              [0] - mask bit for power enable 1
		  //              Power enable mask bits should be set to 1b in the stage in which they first
		  //              become active.  They may be set to 0b or 1b in subsequent stages, as once
		  //              a power enable bit is set, it remains active in all remaining stages whether
		  //              its mask bit in those stages is set or not.  Mask bits for unused (unconnected)
		  //              power enables may be set to either 0b or 1b.
		  //   N+2 to 2N  Intra-stage delay in ms.  Not used for single stage sequences (N=1).  The first
		  //              value gives the delay in ms between the first and second stages, etc.
		  // Response Format:
      //   1          Completion Code.
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
	    // check stage count
		  if ((prqdata[1] < 1) || (prqdata[1] > MAX_PWR_ENA_STAGE_CNT)) {
			  IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
			  break;
		  }
		  // make sure there are enough bytes in command to define the power sequence
		  if (preq->len < (IPMB_RQ_DATA_OFFSET+2*prqdata[1]+1)) {
			  IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
			  break;
		  }			
		  memset((void *) &PMsettingsbuf.bkend_pwr_cfg, 0, sizeof(PMsettingsbuf.bkend_pwr_cfg));          // zero out mask record  
	    PMsettingsbuf.bkend_pwr_cfg.defined_stage_cnt = prqdata[1];
		  for (i1=0; i1<prqdata[1]; i1++)
			  PMsettingsbuf.bkend_pwr_cfg.enamask[i1] = prqdata[2+i1];
			for (i1=0; i1<prqdata[1]-1; i1++)
			  PMsettingsbuf.bkend_pwr_cfg.stagedelay_ms[i1] = prqdata[prqdata[1]+2+i1];
		  // write to EEPROM
		  eepspi_write((unsigned char*) &(PMsettingsbuf.bkend_pwr_cfg), PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void *) (&PMsettingsbuf.bkend_pwr_cfg) - (void *) &PMsettingsbuf)), sizeof(PMsettingsbuf.bkend_pwr_cfg));  
	    break;
	
    case IPMICMD_UWMMC_GET_BACKEND_PWR_ENA_MASK:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_BACKEND_PWR_ENA_MASK\n");
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
		  // Response Format:
      //   1          Completion Code.
	    //   2          Number of stages in power up sequence (N=1 to 4)
		  //   3 to N+2   Power on mask for each stage.  Each stage mask has the following bit field
		  //              defnintion:
      //              [7:6] - Unused, returned as 00b
	    //              [5] - mask bit for power enable 6
	    //              [4] - mask bit for power enable 5
	    //              [3] - mask bit for power enable 4
	    //              [2] - mask bit for power enable 3
	    //              [1] - mask bit for power enable 2
	    //              [0] - mask bit for power enable 1
		  //              Mask bits wil be set to 1b in those stages where they are active and 0b
		  //              in those stages where they are inactive.
		  //              power enables may be set to either 0b or 1b.
		  //   N+3 to     Intra-stage delay in ms.  Not used for single stage sequences (N=1).  The first
		  //     2N+1     value gives the delay in ms between the first and second stages, etc.
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
		  prsdata[2] = pyldmgr_state.settings.bkend_pwr_cfg.defined_stage_cnt;
		  prsdata[3] = 0x3f & pyldmgr_state.settings.bkend_pwr_cfg.enamask[0];
		  for (i1=1; i1<prsdata[2]; i1++) {
			  prsdata[3+i1] = prsdata[2+i1] | (0x3f & pyldmgr_state.settings.bkend_pwr_cfg.enamask[i1]);         // logical ORing of bits of successive stages for returned mask values
		  }
		  for (i1=0; i1<prsdata[2]-1; i1++)
		    prsdata[prsdata[2]+3+i1] = pyldmgr_state.settings.bkend_pwr_cfg.stagedelay_ms[i1];
      prsp->len += 2*prsdata[2];
	    break;

    case IPMICMD_UWMMC_SET_CURRENT_REQUIREMENT:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_SET_CURRENT_REQUIREMENT\n");
      // Command updates the current requirement record in the FRU multirecord area in
      // nonvolatile storage.  The change will take effect the next time the FRU data area
      // is read by the Carrier Manager.  Current is given in 100 mA units, to a maximum of
      // 7.0 amps, with a minimum of 0.5A
      // BYTE         DATA FIELD
      // Command Format:
      //   1          12V Current Requirement in 100 mA units
      // Response Format:
      //   1          Completion Code.
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      if ((prqdata[1] < 5) || (prqdata[1] > 70)) {
        // bad current specification
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
        break;
      }
      // load current record into memory and inspect it
      eepspi_read((unsigned char*) &currentrecbuf, MULTIRECORD_AREA_BYTE_OFFSET, sizeof(currentrecbuf));
      if ((currentrecbuf.PICMG_recID != 0x16) || (currentrecbuf.mfgID_LSB != 0x5a) || (currentrecbuf.mfgID_MidB != 0x31) ||
        (currentrecbuf.mfgID_MSB != 0x00)) {
        // something wrong, but this isn't the current requirement record--so abort
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_RECORD_CODE;
        break;
      }
      currentrecbuf.current_draw_100mA = prqdata[1];    // set new current requirement
      currentrecbuf.hdr.recxsum = calc_ipmi_xsum((unsigned char*) &(currentrecbuf.mfgID_LSB), sizeof(module_current_req_record_t)-
        sizeof(multirecord_header_t));                  // update checksum
      currentrecbuf.hdr.hdrxsum = calc_ipmi_xsum((unsigned char*) &(currentrecbuf.hdr.rectypeID), sizeof(multirecord_header_t)-1);
      eepspi_write((unsigned char*) &currentrecbuf, MULTIRECORD_AREA_BYTE_OFFSET, sizeof(currentrecbuf));
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      break;

    case IPMICMD_UWMMC_SET_ANALOG_SCALE_FACTOR:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_SET_ANALOG_SCALE_FACTOR\n");
      // Command sets the scaling factors for an ADC input channel in nonvolatile storage.
      // For changes to be observed in sensor operation, the MMC must be restarted by a power
      // cycle or IPMI Application Cold Reset command (netFN=06h, Cmd=02h).
      // BYTE         DATA FIELD
      // Command Format:
      //   1          ADC channel number.  If omitted, command returns the number of ADC channels.
      //              Channels are numbered consecutively, beginning at 0.
      //   2          Field index.  Fields are indexed as follows:
      //                0 - Scaling factor M numerator
      //                1 - Scaling factor M denominator
      //                2 - Offset B
      //   3-6        Field value, as 32-bit signed integer, LS byte first
      // Response Format:
      //   1          Completion Code.
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      if ((prqdata[1] > MAX_ADC_CH_NUM) || (prqdata[2] > 2)) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
        break;
      }
      // compute EEPROM address
      eepsaddr = ADC_SCALING_AREA_BYTE_OFFSET + prqdata[1]*sizeof(front_end_scaling_factors_t) + prqdata[2]*sizeof(long);
      // use 'systime' as a scratch pad variable to build a 32-bit version of the new field value
      systime = prqdata[6];
      systime = (systime << 8) + prqdata[5];
      systime = (systime << 8) + prqdata[4];
      systime = (systime << 8) + prqdata[3];
      eepspi_write((unsigned char*) &systime, eepsaddr, 4);
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      break;

    case IPMICMD_UWMMC_GET_ANALOG_SCALE_FACTOR:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_ANALOG_SCALE_FACTOR\n");
      // Command returns the scaling factors for an ADC input channel.  If no arguments are
      // specified, it returns the number of ADC channels.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          ADC channel number.  If omitted, command returns the number of ADC channels.
      //              Channels are numbered consecutively, beginning at 0.
      // Response Format:
      //   1          Completion Code.
      //   2          Returns the ADC channel number specified in the command.  If omitted from
      //   3-6        Scaling factor (M) numerator, as 32-bit signed integer, LS byte first.
      //   7-10       Scaling factor (M) denominator, as 32-bit signed integer, LS byte first
      //   11-14      Offset (B), as 32-bit signed integer, LS byte first.
      if (preq->len > (IPMBREQOVHEAD)) {
        // channel number specified
        if (prqdata[1] > MAX_ADC_CH_NUM) {
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
          break;
        }
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
        prsdata[2] = prqdata[1];
        prsdata[3] = ADCscaletbl[prqdata[1]].Mnum & 0xff;
        prsdata[4] = (ADCscaletbl[prqdata[1]].Mnum >> 8) & 0xff;
        prsdata[5] = (ADCscaletbl[prqdata[1]].Mnum >> 16) & 0xff;
        prsdata[6] = (ADCscaletbl[prqdata[1]].Mnum >> 24) & 0xff;
        prsdata[7] = ADCscaletbl[prqdata[1]].Mdenom & 0xff;
        prsdata[8] = (ADCscaletbl[prqdata[1]].Mdenom >> 8) & 0xff;
        prsdata[9] = (ADCscaletbl[prqdata[1]].Mdenom >> 16) & 0xff;
        prsdata[10] = (ADCscaletbl[prqdata[1]].Mdenom >> 24) & 0xff;
        prsdata[11] = ADCscaletbl[prqdata[1]].B & 0xff;
        prsdata[12] = (ADCscaletbl[prqdata[1]].B >> 8) & 0xff;
        prsdata[13] = (ADCscaletbl[prqdata[1]].B >> 16) & 0xff;
        prsdata[14] = (ADCscaletbl[prqdata[1]].B >> 24) & 0xff;
        prsp->len += 13;
        break;
      }
      // channel number not specified, return channel count
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = ADC_CHANNEL_CNT;
      prsp->len += 1;
      break;

    case IPMICMD_UWMMC_SET_SENSOR_ALARM_MASK:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_SET_SENSOR_ALARM_MASK\n");
      // Command updates the alarm mask for the specified sensor.  Alarms at or below
      // the specified threshold are masked off such that the power monitor does not
      // see them.  IPMI sensor events for enabled thresholds are still generated.  Settings are
	    // initialized at startup from nonvolatile memory.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          [7] - 1b=update setting in nonvolatile storage
      //              [6] - 1b=update current operational setting
      //              [5:0] - IPMI Sensor Number
      //   2          Alarm Mask Value (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
      // Response Format:
      //   1          Completion Code.
	    i1 = prqdata[1] & 0x3f;           // sensor number
      if ((i1 >= SDRstate.sensor_cnt) || (prqdata[2] > (unsigned char) fault)) {
        // bad sensor number or alarm mask value
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
        break;
      }
	    if (prqdata[1] & 0x80) {
			  // set in EEPROM
        if (eepspi_chk_write_in_progress()) {
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
          break;
		    }		    
		    eepspi_write(&prqdata[2], PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void *) &(pyldmgr_state.settings.sensor_mask_level[i1]) - (void *) &pyldmgr_state.settings)), 1);  
      }
			if (prqdata[1] & 0x40)
			  pyldmgr_state.settings.sensor_mask_level[i1] = prqdata[2];  
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      break;

    case IPMICMD_UWMMC_GET_SENSOR_ALARM_MASK:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_SENSOR_ALARM_MASK\n");
      // Command returns the alarm mask for the specified sensor.  Alarms at or below
      // the specified threshold are masked off such that the power monitor does not
      // see them.  IPMI sensor events for enabled thresholds are still generated.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          IPMI Sensor number
      // Response Format:
      //   1          Completion Code.
      //   2          Current Alarm Mask Value (0=no mask, 1=noncritical, 2=critical, 3=nonrecoverable)
	    //   3          Alarm mask value in nonvolatile memory (MMC initialization value)
      if (prqdata[1] >= SDRstate.sensor_cnt) {
        // bad sensor number
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
        break;
      }
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
		  }		    
      prsdata[2] = (unsigned char) pyldmgr_state.settings.sensor_mask_level[prqdata[1]];
		  eepspi_read(&prsdata[3], PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void *) &(pyldmgr_state.settings.sensor_mask_level[prqdata[1]]) - (void *) &pyldmgr_state.settings)), 1);  
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsp->len += 2;
      break;

    case IPMICMD_UWMMC_SET_HANDLE_OVERRIDE:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_SET_HANDLE_OVERRIDE\n");
      // BYTE         DATA FIELD
      // Command Format:
      //   1          [7] - 0b = normal mode, 1b = override mode
      //              [6:1] - reserved
      //              [0] - 0b = force handle to report closed (in) state, 1b = force
      //              handle to report an open (out) state.  This bit is ignored
      //              if bit 7 is set to 0.
      //   2          Override duration (optional).  If this byte is present, it specifies
      //              the duration of the override, in 100ms units, with zero indicating
      //              that the override is indefinite.  Ignored if normal mode is specified.
      // Response Format:
      //   1          Completion Code.
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      if (!(prqdata[1] & 0x80)) {
        // normal mode
        clear_handle_position_override();
        break;
      }
      if (preq->len > (IPMBREQOVHEAD+1))
        xbuf[0] = prqdata[2];     // duration specified
      else
        xbuf[0] = 0;        // no duration specified
      if (prqdata[1] & 0x01)
        set_handle_position_override(out_open, xbuf[0]);
      else
        set_handle_position_override(in_closed, xbuf[0]);
      break;

    case IPMICMD_UWMMC_GET_NONVOLATILE_AREA_INFO:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_NONVOLATILE_AREA_INFO\n");
      // This returns area information for EEPROM-based nonvolatile storage
      // BYTE         DATA FIELD
      // Command Format:
      //   1          Requested page index.  Nonvolatile area information returned
      //              depends on which page index is specified.
      // Response Format:
      //   1          Completion Code.
      // ----Response bytes for page index 0----------------------------------------
      //   2          Format code for nonvolatile storage.  Returns 0x01 for this version
      //   3          Requested page index supplied in command (equals 0 for this section)
      //   4          EEPROM size in bytes, LS byte
      //   5          EEPROM size in bytes, MS byte
      //   6          Hardware Header Area byte offset, LS byte
      //   7          Hardware Header Area byte offset, MS byte
      //   8          Hardware Header Area size, 8-byte units
      //   9          Application Device Info Area byte offset, LS byte
      //   10         Application Device Info Area byte offset, MS byte
      //   11         Application Device Info Area size, 8-byte units
      //   12         FRU Data Area byte offset, LS byte
      //   13         FRU Data Area byte offset, MS byte
      //   14         FRU Data Area size, 8-byte units
      //   15         FPGA Configuration Area byte offset, LS byte
      //   16         FPGA Configuration Area byte offset, MS byte
      //   17         FPGA Configuration Area size, 32-byte units
      // ----Response bytes for page index 1----------------------------------------
      //   2          Format code for nonvolatile storage.  Returns 0x01 for this version
      //   3          Requested page index supplied in command (equals 1 for this section)
      //   4          SDR Area byte offset, LS byte
      //   5          SDR Area byte offset, MS byte
      //   6          SDR Area size, 32-byte units
      //   7          Payload Manager Area byte offset, LS byte
      //   8          Payload Manager Area byte offset, MS byte
      //   9          Payload Manager Area size, 8-byte units
      //   10         ADC Scaling Factor Area byte offset, LS byte
      //   11         ADC Scaling Factor Area byte offset, MS byte
      //   12         ADC Scaling Factor Area size, 8-byte units
	    //   13         Fault Log Area byte offset, LS byte
		  //   14         Fault Log Area byte offset, MS byte
		  //   15         Fault Log Area size, 8-byte units
      switch (prqdata[1]) {
        case 0:
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
          prsdata[2] = NONVOLATILE_FORMAT_VERSION;
          prsdata[3] = 0;
          prsdata[4] = EEPSIZE & 0xff;
          prsdata[5] = EEPSIZE >> 8;
          prsdata[6] = HW_HEADER_BYTE_OFFSET & 0xff;
          prsdata[7] = HW_HEADER_BYTE_OFFSET >> 8;
          prsdata[8] = HW_HEADER_SIZE >> 3;
          prsdata[9] = APP_DEV_ID_BYTE_OFFSET & 0xff;
          prsdata[10] = APP_DEV_ID_BYTE_OFFSET >> 8;
          prsdata[11] = APP_DEV_ID_SIZE >> 3;
          prsdata[12] = COMMON_HEADER_BYTE_OFFSET & 0xff;
          prsdata[13] = COMMON_HEADER_BYTE_OFFSET >> 8;
          prsdata[14] = FRU_AREA_SIZE >> 3;
          prsdata[15] = FPGA_CONFIG_AREA_BYTE_OFFSET & 0xff;
          prsdata[16] = FPGA_CONFIG_AREA_BYTE_OFFSET >> 8;
          prsdata[17] = FPGA_CONFIG_AREA_SIZE >> 5;
          prsp->len += 16;
          break;

        case 1:
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
          prsdata[2] = NONVOLATILE_FORMAT_VERSION;
          prsdata[3] = 1;
          prsdata[4] = SDR_AREA_BYTE_OFFSET & 0xff;
          prsdata[5] = SDR_AREA_BYTE_OFFSET >> 8;
          prsdata[6] = SDR_AREA_SIZE >> 5;
          prsdata[7] = PAYLDMGR_AREA_BYTE_OFFSET & 0xff;
          prsdata[8] = PAYLDMGR_AREA_BYTE_OFFSET >> 8;
          prsdata[9] = PAYLDMGR_AREA_SIZE >> 3;
          prsdata[10] = ADC_SCALING_AREA_BYTE_OFFSET & 0xff;
          prsdata[11] = ADC_SCALING_AREA_BYTE_OFFSET >> 8;
          prsdata[12] = ADC_SCALING_AREA_SIZE >> 3;
		      prsdata[13] = FAULT_LOG_HDR_BYTE_OFFSET & 0xff;
		      prsdata[14] = FAULT_LOG_HDR_BYTE_OFFSET >> 8;
		      prsdata[15] = FAULT_LOG_SIZE >> 3;
          prsp->len += 14;
          break;

        default:
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_ILLEGAL_PARAMETER;
        break;
      }
      break;

    case IPMICMD_UWMMC_RAW_NONVOLATILE_WRITE:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_RAW_NONVOLATILE_WRITE\n");
      // This function performs a raw write of the nonvolatile storage EEPROM.  It should
      // be used with great care, as it cause corruption of one or more EEPROM storage areas
      // and affect the function of the module.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          Starting EEPROM byte offset, LS byte
      //   2          Starting EEPROm byte offset, MS byte
      //   3          Count of Bytes to write (n)
      //   4..4+(n-1) Write Data
      // Response Format:
      //   1          Completion Code.
      // first check to see if EEPROM is available
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      eepsaddr = prqdata[1] | (prqdata[2] << 8);
      xlength = prqdata[3];
      if (((eepsaddr+xlength-1) > EEPSIZE) || (xlength > NONVOLATILE_MAX_XFER_LEN)) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
      // transfer data from command to buffer
      curxlen = xlength;
      xbptr = &xbuf[0];
      pmsgdata = (unsigned char*) &prqdata[4];
      while (curxlen) {
        *xbptr++ = *pmsgdata++;
        curxlen--;
      }
      eepspi_write(xbuf, eepsaddr, xlength);
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      break;

    case IPMICMD_UWMMC_RAW_NONVOLATILE_READ:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_RAW_NONVOLATILE_READ\n");
      // This function performs a raw read of the nonvolatile storage EEPROM.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          Starting EEPROM byte offset, LS byte
      //   2          Starting EEPROM byte offset, MS byte
      //   3          Count of Bytes to read (n)
      // Response Format:
      //   1          Completion Code.
      //   2..2+(n-1) Read Data
      // first check to see if EEPROM is available
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      eepsaddr = prqdata[1] | (prqdata[2] << 8);
      xlength = prqdata[3];
      if (((eepsaddr+xlength-1) >= EEPSIZE) || (xlength > NONVOLATILE_MAX_XFER_LEN)) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
      eepspi_read(xbuf, eepsaddr, xlength);         // read from EEPROM
      // transfer data from buffer to response msg
      curxlen = xlength;
      xbptr = &xbuf[0];
      pmsgdata = (unsigned char*) &prsdata[2];
      while (curxlen) {
        *pmsgdata++ = *xbptr++;
        curxlen--;
      }
      eepspi_write(xbuf, eepsaddr, xlength);
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsp->len += xlength;
     break;

    case IPMICMD_UWMMC_CHK_EEPROM_BUSY:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_CHK_EEPROM_BUSY\n");
      // Function checks the EEPROM status to determine if it is busy with a write access or not.
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.
      //   2          Busy status--returns 0x01 if EEPROM busy, 0x00 if it is not
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = eepspi_chk_write_in_progress();
      prsp->len += 1;
      break;

    case IPMICMD_UWMMC_EEPROM_ERASE:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_EEPROM_ERASE\n");
      // Command to erase EEPROM.  In order to avoid accidental erasures of the EEPROM,
      // the command must be executed twice, each time with a different subfunction code.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          Subfunction code.  Use value 0xaa on the first call to have the function
      //              return a 4-byte erase key.  Use value 0x55 on the second call, along with
      //              the erase key in bytes 2-5 of the request, to complete the erase operation.
      //              Note that the erase key is valid for approximately 30 seconds after it is
      //              issued.
      //   2          Erase key byte 0.  Ignored on first (0xaa) call, required on second (0x55) call
      //   3          Erase key byte 1.  Ignored on first (0xaa) call, required on second (0x55) call
      //   4          Erase key byte 2.  Ignored on first (0xaa) call, required on second (0x55) call
      //   5          Erase key byte 3.  Ignored on first (0xaa) call, required on second (0x55) call
      // Response Format:
      //   1          Completion Code.
      //   2          Erase key byte 0.  Returned on first (0xaa) call only.
      //   3          Erase key byte 1.  Returned on first (0xaa) call only.
      //   4          Erase key byte 2.  Returned on first (0xaa) call only.
      //   5          Erase key byte 3.  Returned on first (0xaa) call only.
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      switch (prqdata[1]) {
        case EEP_ERASE_KEY_SUBFUNC:
          // get new key
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
          EEP_erase_key.issue_time = TIMESTATREC.uptime;      // take time snapshot
          // choose some dynamic data values to make the key
          EEP_erase_key.eep_erase_key[0] = AVR32_TC.channel[1].cv & 0xff;      // request msg checksum
          EEP_erase_key.eep_erase_key[1] = TIMESTATREC.uptime & 0xff;
          EEP_erase_key.eep_erase_key[2] = AVR32_TC.channel[0].cv & 0xff;
          EEP_erase_key.eep_erase_key[3] = (get_rtc_value() >> 8) & 0xff;
          prsdata[2] = EEP_erase_key.eep_erase_key[0];
          prsdata[3] = EEP_erase_key.eep_erase_key[1];
          prsdata[4] = EEP_erase_key.eep_erase_key[2];
          prsdata[5] = EEP_erase_key.eep_erase_key[3];
          prsp->len += 4;
          break;

        case EEP_ERASE_EXECUTE_SUBFUNC:
          // check key to make sure it is accurate and unexpired
          if (((TIMESTATREC.uptime - EEP_erase_key.issue_time) > EEP_KEY_VALID_DURATION) ||
            (prqdata[2] != EEP_erase_key.eep_erase_key[0]) || (prqdata[3] != EEP_erase_key.eep_erase_key[1]) ||
            (prqdata[4] != EEP_erase_key.eep_erase_key[2]) || (prqdata[5] != EEP_erase_key.eep_erase_key[3])) {
            // time expired or bad key
            IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INSUFFICIENT_PRIVILEGE;
            break;
          }
          // perform erase
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
          eepspi_chip_erase();
          break;

        default:
          IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
          break;
      }
      break;


    case IPMICMD_UWMMC_GET_TIMESTATS:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_TIMESTATS\n");
      // Function returns the timer values for MMC up time and Back end hot time, in seconds
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.
      //   2          32-bit mmc up time, LS byte
      //   3          32-bit mmc up time, bits [15:8]
      //   4          32-bit mmc up time, bits [23:16]
      //   5          32-bit mmc up time, MS byte
      //   6          32-bit backend hot time, LS byte
      //   7          32-bit backend hot time, bits [15:8]
      //   8          32-bit backend hot time, bits [23:16]
      //   9          32-bit backend hot time, MS byte
      // ADDITIONAL BITS AT AMC13 MMC VERSION 3.0 BELOW
      //   10         Reset Counter, LS byte
      //   11         Reset Counter, bits [15:8]
      //   12         Reset Counter, bits [23:16]
      //   13         Reset Counter, MS byte
      //   14         32-bit time since last reset, LS byte
      //   15         32-bit time since last reset, bits [15:8]
      //   16         32-bit time since last reset, bits [23:16]
      //   17         32-bit time since last reset, MS byte

      systime = get_rtc_value();
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = TIMESTATREC.uptime & 0xff;
      prsdata[3] = (TIMESTATREC.uptime >> 8) & 0xff;
      prsdata[4] = (TIMESTATREC.uptime >> 16) & 0xff;
      prsdata[5] = (TIMESTATREC.uptime >> 24) & 0xff;
      prsdata[6] = TIMESTATREC.hottime & 0xff;
      prsdata[7] = (TIMESTATREC.hottime >> 8) & 0xff;
      prsdata[8] = (TIMESTATREC.hottime >> 16) & 0xff;
      prsdata[9] = (TIMESTATREC.hottime >> 24) & 0xff;
      prsdata[10] = TIMESTATREC.reset_cnt & 0xff;
      prsdata[11] = (TIMESTATREC.reset_cnt >> 8) & 0xff;
      prsdata[12] = (TIMESTATREC.reset_cnt >> 16) & 0xff;
      prsdata[13] = (TIMESTATREC.reset_cnt >> 24) & 0xff;
      prsdata[14] = TIMESTATREC.lastrsttime & 0xff;
      prsdata[15] = (TIMESTATREC.lastrsttime >> 8) & 0xff;
      prsdata[16] = (TIMESTATREC.lastrsttime >> 16) & 0xff;
      prsdata[17] = (TIMESTATREC.lastrsttime >> 24) & 0xff;
      prsp->len += 16;
      break;

    case IPMICMD_UWMMC_MMC_RESET:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_MMC_RESET\n");
      // Function resets the MMC by disabling service of the watchdog timer.
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      Service_Watchdog();     // service it now to reset the counters (to increase chances for response code to be sent before reset kicks in)
      Force_Watchdog_Reset();   // disable further service, causing eventual reset
      break;

    case IPMICMD_UWMMC_GET_MMC_VERSION:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_MMC_VERSION\n");
      // Function resets the MMC by disabling service of the watchdog timer.
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.
      //   2          Major Firmware Revision Byte
      //   3          Minor Firmware Revision Byte
      //   4          Date String Length (N)
      //   5..(5+N-1) Date String
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = MAJOR_MMC_VERSION_NUMBER;
      prsdata[3] = MINOR_MMC_VERSION_NUMBER;
      prsdata[4] = MMC_VERSION_STRLEN;
      strncpy(spbuf, MMC_VERSION_STRING, MMC_VERSION_STRLEN);
        for (i1=0; i1<MMC_VERSION_STRLEN; i1++)
      prsdata[5+i1] = spbuf[i1];
      prsp->len += MMC_VERSION_STRLEN+3;
      break;

    case IPMICMD_UWMMC_CLR_MMC_RESET_CTR:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_CLR_MMC_RESET_CTR\n");
      // Function clears the MMC reset counter, setting it back to zero.
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      TIMESTATREC.reset_cnt = 0;
      break;

    case IPMICMD_UWMMC_SET_SYSTIME:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_SET_SYSTIME\n");
      // This function sets the 32-bit system time kept by the RTC clock and the 32KHz oscillator
      // BYTE         DATA FIELD
      // Command Format:
      //   1          32-bit system time, LS byte
      //   2          32-bit system time, bits [15:8
      //   3          32-bit system time, bits [23:16]
      //   4          32-bit system time, MS byte
      // Response Format:
      //   1          Completion Code.
      if (preq->len < IPMBREQOVHEAD+4) {
        // not enough params
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_DATA_LENGTH;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      systime = prqdata[4];
      systime = (systime << 8) + prqdata[3];
      systime = (systime << 8) + prqdata[2];
      systime = (systime << 8) + prqdata[1];
      set_rtc_value(systime);
      break;

    case IPMICMD_UWMMC_GET_SYSTIME:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_GET_SYSTIME\n");
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.
      //   2          32-bit system time, LS byte
      //   3          32-bit system time, bits [15:8
      //   4          32-bit system time, bits [23:16]
      //   5          32-bit system time, MS byte
      systime = get_rtc_value();
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = systime & 0xff;
      prsdata[3] = (systime >> 8) & 0xff;
      prsdata[4] = (systime >> 16) & 0xff;
      prsdata[5] = (systime >> 24) & 0xff;
      prsp->len += 4;
      break;

    case IPMICMD_UWMMC_POLL_FPGA_CFG_PORT:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_POLL_FPGA_CFG_PORT\n");
      // This command polls the SPI1 port for connected FPGA Config Port SPI slaves.  It
      // asserts the ~SCANSLV signal from the Microcontroller and samples the ~CSx pins
      // as logic inputs.  Any ~CSx lines observed to go to a logic 0 are considered to
      // have FPGA Config Ports attached to them.
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
      //   2          [7:4] - reserved, returns 1111b
      //              [3] - SPI1 CS3 -- 1b = FPGA port detected, 0b = no FPGA detected
      //              [2] - SPI1 CS2 -- 1b = FPGA port detected, 0b = no FPGA detected
      //              [1] - SPI1 CS1 -- 1b = FPGA port detected, 0b = no FPGA detected
      //              [0] - SPI1 CS0 -- 1b = FPGA port detected, 0b = no FPGA detected
      if (!spi1_lock()) {
        // SPI1 unavailable
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = 0xf0 | fpgaspi_slave_detect();
      spi1_unlock();
      prsp->len += 1;
      break;

    case IPMICMD_UWMMC_FPGA_CFG_RDSR:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_FPGA_CFG_RDSR\n");
      // BYTE         DATA FIELD
      // This command reads the FPGA Config Port Status register using the RDSR SPI command
      // Command Format:
      //   1          SPI1 Chip Select ID (0-3)
      // Response Format:
      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
      //   2          FPGA SPI Configuration Port Status Register
      //              [7] - Upper Handshake Flag (UHF)
      //              [6] - Lower Handshake Flag (LHF)
      //              [5] - Config Ready Flag (CFGRDY)-- reset to 0b by FPGA firmware @ startup,
      //                    set to 1b after config image written to port
      //              [4] - Request Config Flag (REQCFG) -- set to 1b by FPGA at init to request
      //                    config data, cleared by FPGA after detecting CFGRDY set to 1b by SPI write
      //              [3:0] - reserved
      if (prqdata[1] > MAX_FPGA_ID) {
        // bad param
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
      if (!spi1_lock()) {
        // SPI1 unavailable
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = fpgaspi_status_read(prqdata[1]);
      spi1_unlock();
      prsp->len += 1;
      break;

    case IPMICMD_UWMMC_FPGA_CFG_WRCTL:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_FPGA_CFG_WRCTL\n");
      // This function writes a byte to the FPGA SPI Config Port Control register
      // using the WRCTL SPI command.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          SPI1 Chip Select ID (0-3)
      //   2          Data Byte Written to Control Register
      //              [7] - 1b = select UHF, 0b = no select
      //              [6] - 1b = select LHF, 0b = no select
      //              [5] - 1b = select CFGRDY, 0b = no select
      //              [4:2] - reserved
      //              [1] - 1b = selected bits 7:5 should be set, 0b = no change
      //              [0] - 1b = selected bits 7:5 should be cleared, 0b = no change
      // Response Format:
      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
      if (prqdata[1] > MAX_FPGA_ID) {
        // bad param
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
      if (!spi1_lock()) {
        // SPI1 unavailable
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      fpgaspi_ctl_write(prqdata[1], prqdata[2]);
      spi1_unlock();
      break;

    case IPMICMD_UWMMC_FPGA_CFG_WRITE:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_FPGA_CFG_WRITE\n");
      // This function performs a raw write to the FPGA SPI Config Port.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          SPI1 Chip Select ID (0-3)
      //   2          Config Port Destination byte address, LS byte
      //   3          Config Port Destination byte address, MS byte
      //   4          Count of Bytes to write (n)
      //   5..5+(n-1) Write Data
      // Response Format:
      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
      if ((prqdata[1] > MAX_FPGA_ID) || (prqdata[4] > NONVOLATILE_MAX_XFER_LEN)) {
        // bad param
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
      if (!spi1_lock()) {
        // SPI1 unavailable
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      eepsaddr = prqdata[2] | (prqdata[3] << 8);
      // transfer bytes to SPI port
      fpgaspi_data_write(prqdata[1], &prqdata[5], eepsaddr, prqdata[4]);
      spi1_unlock();
      break;

    case IPMICMD_UWMMC_FPGA_CFG_READ:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_FPGA_CFG_READ\n");
      // This function performs a raw read from the FPGA SPI Config Port.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          SPI1 Chip Select ID (0-3)
      //   2          Config Port Source byte address, LS byte
      //   3          Config Port Source byte address, MS byte
      //   4          Count of Bytes to read (n)
       // Response Format:
      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
      //   2..2+(n-1) Read Data
      if ((prqdata[1] > MAX_FPGA_ID) || (prqdata[4] > NONVOLATILE_MAX_XFER_LEN)) {
        // bad param
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
      if (!spi1_lock()) {
        // SPI1 unavailable
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      eepsaddr = prqdata[2] | (prqdata[3] << 8);
      fpgaspi_data_read(prqdata[1], &prsdata[2], eepsaddr, prqdata[4]);
      prsp->len += prqdata[4];
      spi1_unlock();
      break;

    case IPMICMD_UWMMC_FPGA_CFG_NONVOL_HDR_WRITE:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_FPGA_CFG_NONVOL_HDR_WRITE\n");
      // This function writes the header for the 256 byte Autoconfig section
      // in the nonvolatile storage (EEPROM).  NOTE:  Any changes to the header
      // or Autoconfig section of nonvolatile storage will not take effect until
      // the MMC is reset.
      // BYTE         DATA FIELD
      // Command Format:
      //   1          Config Flags, 2 bits per chip select.  (x0b = config data undefined,
      //              01b = defined, but autoconfig disabled, 11b = autoconfig enabled)
      //              [7:6] CS3 autoconfig setting
      //              [5:4] CS2 autoconfig setting
      //              [3:2] CS1 autoconfig setting
      //              [1:0] CS0 autoconfig setting
      //   2          Byte offset to CS0 config data
      //   3          Byte offset to CS1 config data
      //   4          Byte offset to CS2 config data
      //   5          Byte offset to CS3 config data
      //   6          Header Checksum
      // Response Format:
      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      eepsaddr = FPGA_CONFIG_AREA_BYTE_OFFSET+1;      // skip past format flag
      eepspi_write(&prqdata[1], eepsaddr, sizeof(FPGA_config_area_header_t)-1);
      break;

    case IPMICMD_UWMMC_FPGA_CFG_NONVOL_HDR_READ:
      sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_UWMMC_FPGA_CFG_NONVOL_HDR_READ\n");
      // This function reads and returns the header for the 256 byte Autoconfig section
      // in the nonvolatile storage (EEPROM)
      // BYTE         DATA FIELD
      // Command Format:
      //   (no bytes)
      // Response Format:
      //   1          Completion Code.  (Returns C0h if SPI1 is unavailable)
      //   2          Config Flags, 2 bits per chip select.  (x0b = config data undefined,
      //              01b = defined, but autoconfig disabled, 11b = autoconfig enabled)
      //              [7:6] CS3 autoconfig setting
      //              [5:4] CS2 autoconfig setting
      //              [3:2] CS1 autoconfig setting
      //              [1:0] CS0 autoconfig setting
      //   3          Byte offset to CS0 config data
      //   4          Byte offset to CS1 config data
      //   5          Byte offset to CS2 config data
      //   6          Byte offset to CS3 config data
      //   7          Header Checksum
      if (eepspi_chk_write_in_progress()) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
        break;
      }
      eepsaddr = FPGA_CONFIG_AREA_BYTE_OFFSET+1;      // skip past format flag
      eepspi_read(&prsdata[2], eepsaddr, sizeof(FPGA_config_area_header_t)-1);
      prsp->len += sizeof(FPGA_config_area_header_t)-1;
      break;

    default:
      prsp->len = 0;      // nothing supported here yet
  }
}

void parse_CMS_cmds(const ipmb_msg_desc_t*preq, ipmb_msg_desc_t* prsp) {
  const unsigned char* prqdata = &preq->buf[IPMB_RQ_DATA_OFFSET-1];   // offset pointer for 1-base refs to req data
  unsigned char* prsdata = &prsp->buf[IPMB_RS_DATA_OFFSET-1];			// offset pointer for 1-base refs to rsp data
  unsigned char deviceID;
  unsigned char MACbuf[16];
  unsigned short saddr;
  int i1;

  switch (IPMB_RQ_CMD(preq->buf)) {
    case IPMICMD_CMS_GET_MAC_ADDRESS:
    sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_CMS_GET_MAC_ADDRESS\n");
    // This returns the MAC address as reported by the payload via the SPI interface
    // BYTE         DATA FIELD
    // Command Format:
    //   1          Optional DeviceID (use 0 if only single MAC supported).  If omitted,
    //              a DeviceID of 0 is implied
    // Response Format:
    //   1          Completion Code
    //                00h - normal completion
    //                C0h - MAC address supported by FRU/DeviceID, but unavailable for reporting at this time
    //                      (e.g., FPGA not booted)
    //                C1h - FRU/DeviceID does not support MAC addresses
    //                CCh - if DeviceID is out of range supported by FRU
    //   2          Address_Type code -- type of address being reported
    //                00h - reserved
    //                01h - MAC-48 (6 bytes)
    //                02h - EUI-64 (8 bytes)
    //                03h-FFh - reserved
    //   3            Checksum on MAC address - standard IPMI checksum
    //   4 to 4+N-1   MAC address, where N=# of bytes, per the address type

    // check device ID
    if (preq->len == IPMIMINMSGLEN)
    // implied DeviceID = 0
    deviceID = 0;
    else {
      deviceID = prqdata[1];
      if (deviceID > 1) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
    }
    // check if backend power up
    if (pyldmgr_get_backend_power_status() == power_off) {
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
      break;
    }
    // get SPI port
    if (!spi1_lock()) {
      // SPI1 unavailable
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
      break;
    }
    // check to see if an active MMI listener is detected
    if (!(fpgaspi_slave_detect() & (1 << deviceID))) {
      // FPGA not booted or does not contain interface core
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
      spi1_unlock();
      break;
    }
    // read MAC address from pre-defined address
    saddr = (MAC_ADDRESS_UPPER_SPI_ADDR << 8) | MAC_ADDRESS_LOWER_SPI_ADDR;
    if (!fpgaspi_data_read(deviceID, &MACbuf[0], saddr, 9)) {
      // problem with read
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
      spi1_unlock();
      break;
    }
    // parse MAC address
    switch (MACbuf[0]) {
      case 0x01:    // MAC-48
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = 0x01;                  // set type
      prsdata[3] = calc_ipmi_xsum(&MACbuf[1], 6);
      for (i1=0; i1<6; i1++)
      prsdata[4+i1] = MACbuf[i1+1];     // transfer 6-byte MAC address
      prsp->len += 8;
      break;

      case 0x02:    // EUI-64
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = 0x02;                  // set type
      prsdata[3] = calc_ipmi_xsum(&MACbuf[1], 8);
      for (i1=0; i1<8; i1++)
      prsdata[4+i1] = MACbuf[i1+1];     // transfer 6-byte MAC address
      prsp->len += 8;
      break;

      default:
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
    }
    spi1_unlock();
    break;

    case IPMICMD_CMS_GET_IP_ADDRESS:
    sio_filt_putstr(TXTFILT_IPMI_CUSTOM_REQ, 1, "IPMICMD_CMS_GET_IP_ADDRESS\n");
    // This command returns the IP address as reported by the payload via the SPI interface
    // BYTE         DATA FIELD
    // Command Format:
    //   1          Optional DeviceID (use 0 if only single IP endpoint supported).  If omitted,
    //              a DeviceID of 0 is implied
    // Response Format:
    //   1          Completion Code
    //                00h - normal completion
    //                C0h - IP address supported by FRU/DeviceID, but unavailable for reporting at this time
    //                      (e.g., FPGA not booted or address unknown/unitialized)
    //                C1h - FRU/DeviceID does not support MAC addresses
    //                C2h - FRU/DeviceID is in RARP mode and IP address is unavailable
    //                CCh - if DeviceID is out of range supported by FRU
    //   2          Address_Type code -- type of address being reported
    //                00h - reserved
    //                01h - IPv4 (4 bytes)
    //                02h-FFh - reserved
    //   3            Checksum on returned address/subnet field
    //   4 to 4+N-1   IP address, where N=# of bytes, per the address type
    //   4+N to     Optional subnet mask.  Where omitted, the subnet is assumed to
    //     4+2*N-1    be fixed and specific to the FRU type.
    //   4+2*N to   Optional default gateway.
    //     4+3*N-1
    // check device ID
    if (preq->len == IPMIMINMSGLEN)
      // implied DeviceID = 0
      deviceID = 0;
    else {
      deviceID = prqdata[1];
      if (deviceID > 1) {
        IPMB_RS_CCODE(prsp->buf) = IPMI_RS_INVALID_FIELD_IN_REQ;
        break;
      }
    }
    // check if backend power up
    if (pyldmgr_get_backend_power_status() == power_off) {
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
      break;
    }
    // get SPI port
    if (!spi1_lock()) {
      // SPI1 unavailable
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
      break;
    }
    // check to see if an active MMI listener is detected
    if (!(fpgaspi_slave_detect() & (1 << deviceID))) {
      // FPGA not booted or does not contain interface core
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
      spi1_unlock();
      break;
    }

    // read IP address and subnet from pre-defined location in boot vector.  By definition AMC13 uses IPv4 addressing,
    // with netmask in bytes 1-4, and IP address in bytes 5-8 of config vector.  But the SPI read skips the slot ID byte,
    // so the netmask is in bytes 0-3 of the buffer, and the IP address is in bytes 4-7.
    // use the MACbuf local variable as a handy stack buffer for the 8-byte read
    saddr = (IP_ADDRESS_UPPER_SPI_ADDR << 8) | IP_ADDRESS_LOWER_SPI_ADDR;
    if (!fpgaspi_data_read(deviceID, &MACbuf[0], saddr, 12)) {
      // problem with read
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;
      spi1_unlock();
      break;
    }
    // If address in buffer is all zeros, then the vector is still uninitialized
    if (MACbuf[0] | MACbuf[1] | MACbuf[2] | MACbuf[3]) {
      // non-zero address.  Encode in the response and add a checksum
      IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NORMAL_COMP_CODE;
      prsdata[2] = 0x01;
      for (i1=0; i1<4; i1++) {
        prsdata[4+i1] = MACbuf[4+i1];         // copy IP address bytes
        prsdata[8+i1] = MACbuf[i1];           // coppy netmask
      }
      prsdata[3] = calc_ipmi_xsum(&prsdata[4], 8);
      prsp->len += 10;
    }
    else
    IPMB_RS_CCODE(prsp->buf) = IPMI_RS_NODE_BUSY;

    spi1_unlock();
    break;

    default:
    prsp->len = 0;			// nothing supported here yet
  }
}


void encapsulate_message(ipmb_msg_desc_t* pwrapmsg, const ipmb_msg_desc_t* pencapmsg) {
	int encaplen, i1;
	// build a send message request that encapsulates another message 
  if ((pwrapmsg == NULL) || (pencapmsg == NULL))
    return;
	encaplen = pencapmsg->len;
  ipmb_init_req_hdr(pwrapmsg->buf, ipmb_get_event_rcvr_ipmb_addr(), NETFN_APP, 0, 0);
  pwrapmsg->buf[5] = IPMICMD_SEND_MESSAGE;
  pwrapmsg->buf[6] = 0x47;          // tracking = 01h, channel number = 7 (IPMB-L)
  for (i1=0; i1<encaplen; i1++)
    pwrapmsg->buf[7+i1] = pencapmsg->buf[i1];
	pwrapmsg->buf[7+encaplen] = calc_ipmi_xsum(&pwrapmsg->buf[3], 7+encaplen);
	pwrapmsg->len = 8+encaplen;
}


int deencapsulate_message(const ipmb_msg_desc_t* pwrapmsg, ipmb_msg_desc_t* pencapmsg) {
	// extract encapsulated message.  Check to make sure that the command code is for the send message command
	// returns 1 if a deencapsulation occurs, otherwise 0
	int encaplen, i1;
  if ((pwrapmsg == NULL) || (pencapmsg == NULL))
    return 0;
	if ((pwrapmsg->buf[5] != IPMICMD_SEND_MESSAGE) || (pwrapmsg->len < 15))
	  // not a send message command or length too short to be viable
	  return 0;
	encaplen = pwrapmsg->len-8;
	for (i1=0; i1<encaplen; i1++)
	  pencapmsg->buf[i1] = pwrapmsg->buf[7+i1];
	pencapmsg->len = encaplen;
	return 1;  
}


