/*
 * utils.c
 *
 *  Created on: Oct 7, 2010
 *      Author: tgorski
 */

#include <avr32/io.h>
#include <stdio.h>
#include <time.h>
#include "utils.h"
#include "sio_usart.h"
#include "ipmb_svc.h"
#include "nonvolatile.h"
#include "rtc.h"

static int wddebug;
int cap_buf_ena = 0;
int watchdog_service_enable = 1;
char timebuf[32];            // buffer for the time string

void Enable_Watchdog(void) {
  watchdog_service_enable = 1;
  AVR32_WDT.ctrl = 0x55000f01;        // first write to enable watchdog
  AVR32_WDT.ctrl = 0xaa000f01;        // second write, watchdog now armed!
}


void Disable_Watchdog(void) {
  AVR32_WDT.ctrl = 0x55000f00;        // first write to disable watchdog
  AVR32_WDT.ctrl = 0xaa000f00;        // second write, watchdog now disarmed
}

void Force_Watchdog_Reset(void) {
  watchdog_service_enable = 0;        // clear the enable variable, which disables futher service of the register
}

void Service_Watchdog(void) {
  if (watchdog_service_enable) {
    AVR32_WDT.clr = 0xffff;                // service watchdog
    wddebug = 0;
  }
  else {
    wddebug += 1;
  }
  if (wddebug == 20)
    sio_putstr("WDT Service Disabled for 20 cycles, reset likely pending\n");
}


#ifdef TRACE
void set_cap_buf_ena(void) {
  cap_buf_ena = 1;
  CAPBUF.pccapidx1 = 0;                // reset index pointer
  CAPBUF.pccapidx2 = 0;                // reset the other index pointer
  CAPBUF.codebittags = 0;
}


inline int get_cap_buf_ena(void) {
  return cap_buf_ena;
}

void reset_cap_buf_ena(void) {
  cap_buf_ena = 0;
}


void print_cap_buf(void) {
  int i1;
  unsigned long curidx;
  unsigned char tempfiltmask = GPparambuf.sio_filtermask;
  GPparambuf.sio_filtermask |= TXTFILT_DBG_SUMMARY | TXTFILT_DBG_DETAIL;

  sprintf(spbuf, "Total size of buffer = %li bytes (decimal)\n", sizeof(capture_buf_rec_t));
  sio_putstr(spbuf);


  sio_putstr("Capture Buffer 1 Program Counter Dump (most recent first):\n");
  curidx = (RDOUTBUF.pccapidx1-1) >> 2;
  if (curidx > 14) {
    curidx=0;
    sio_putstr("PC buffer index out of range, resetting to zero\n");
  }
  for (i1=0; i1<15; i1++) {
    sprintf(spbuf, "0x%08lx\n", RDOUTBUF.pccapbuf1[curidx]-0x16);         //offset to start of pc capture macro
    sio_putstr(spbuf);
    if (!curidx)
    curidx = 14;
    else
    curidx--;
  }
  
  sio_putstr("\nCapture Buffer 2 Program Counter Dump (most recent first):\n");
  curidx = (RDOUTBUF.pccapidx2-1) >> 2;
  if (curidx > 14) {
    curidx=0;
    sio_putstr("PC buffer index out of range, resetting to zero\n");
  }
  for (i1=0; i1<15; i1++) {
    sprintf(spbuf, "0x%08lx\n", RDOUTBUF.pccapbuf2[curidx]-0x16);          //offset to start of pc capture macro
    sio_putstr(spbuf);
    if (!curidx)
    curidx = 14;
    else
    curidx--;
  }
  
  sio_putstr("\nFlag Register binary dump:  ");
  for (i1=0; i1<32; i1++) {
    if (RDOUTBUF.codebittags & (1<<i1))
    sio_putstr("1");
    else
    sio_putstr("0");
  }
  sio_putstr("\n\n");
  
  if ((RDOUTBUF.lastRXreq.len < IPMIMINMSGLEN) || (RDOUTBUF.lastRXreq.len > IPMIMAXMSGLEN))
    sio_putstr("Capture Buffer Last RX Request Corrupt or Uninitialized\n");
  else {
    sprintf(spbuf, "0x%08lx  Capture Buffer Last RX Request, length = %i:\n", RDOUTBUF.lastRXreqTstamp, RDOUTBUF.lastRXreq.len);
    sio_putstr(spbuf);
    ipmb_msg_dump(&(RDOUTBUF.lastRXreq), TXTFILT_DBG_SUMMARY);
  }

  if ((RDOUTBUF.lastTXrsp.len < IPMIMINMSGLEN) || (RDOUTBUF.lastTXrsp.len > IPMIMAXMSGLEN))
    sio_putstr("Capture Buffer Last TX Response Corrupt or Uninitialized\n");
  else {
    sprintf(spbuf, "0x%08lx  Capture Buffer Last TX Response, length = %i:\n", RDOUTBUF.lastTXrspTstamp, RDOUTBUF.lastTXrsp.len);
    sio_putstr(spbuf);
    ipmb_msg_dump(&(RDOUTBUF.lastTXrsp), TXTFILT_DBG_SUMMARY);
  }

  if ((RDOUTBUF.lastTXreq.len < IPMIMINMSGLEN) || (RDOUTBUF.lastTXreq.len > IPMIMAXMSGLEN))
    sio_putstr("Capture Buffer Last TX Request Corrupt or Uninitialized\n");
  else {
    sprintf(spbuf, "\n0x%08lx  Capture Buffer Last TX Request, length = %i:\n", RDOUTBUF.lastTXreqTstamp, RDOUTBUF.lastTXreq.len);
    sio_putstr(spbuf);
    ipmb_msg_dump(&(RDOUTBUF.lastTXreq), TXTFILT_DBG_SUMMARY);
  }

  if ((RDOUTBUF.lastRXrsp.len < IPMIMINMSGLEN) || (RDOUTBUF.lastRXrsp.len > IPMIMAXMSGLEN))
    sio_putstr("Capture Buffer Last RX Response Corrupt or Uninitialized\n");
  else {
    sprintf(spbuf, "0x%08lx  Capture Buffer Last RX Response, length = %i:\n", RDOUTBUF.lastRXrspTstamp, RDOUTBUF.lastRXrsp.len);
    sio_putstr(spbuf);
    ipmb_msg_dump(&(RDOUTBUF.lastRXrsp), TXTFILT_DBG_SUMMARY);
  }
 
  sio_putstr("Capture Buffer IPMB-L Driver Stats:\n");
  sprintf(spbuf, "  Rx Msg Cnt:  %lu\n", RDOUTBUF.ipmbstats.rxmsg_cnt);
  sio_putstr(spbuf);
  sprintf(spbuf, "  Tx Msg Cnt:  %lu\n", RDOUTBUF.ipmbstats.txmsg_cnt);
  sio_putstr(spbuf);
  sprintf(spbuf, "  Tx Msg Retried Cnt:  %lu\n", RDOUTBUF.ipmbstats.txmsgretried_cnt);
  sio_putstr(spbuf);
  sprintf(spbuf, "  Tx Msg Failed Cnt:  %lu\n", RDOUTBUF.ipmbstats.txmsgfailed_cnt);
  sio_putstr(spbuf);
  sprintf(spbuf, "  Tx Arb Lost Cnt:  %lu\n", RDOUTBUF.ipmbstats.txarblost_cnt);
  sio_putstr(spbuf);
  sio_putstr("  Capture Buffer Driver State:  ");
  switch (RDOUTBUF.twistate) {
    case disabled:
    sio_putstr("disabled\n");
    break;
    case slave_listen:
    sio_putstr("listening (idle)\n");
    break;
    case slave_write:
    sio_putstr("slave (write op)\n");
    break;
    case slave_read:
    sio_putstr("slave (read op)\n");
    break;
    case master_write:
    sio_putstr("master write\n");
    break;
    case failed:
    sio_putstr("FAILED\n");
    break;
    default:
    sio_putstr("UNKNOWN\n");
    break;
  }
  sio_putstr("  Capture Buffer Driver Regs:\n");
  sprintf(spbuf, "    SR:  0x%08lx\n", RDOUTBUF.twiSR);
  sio_putstr(spbuf);
  sprintf(spbuf, "    IMR:  0x%08lx\n", RDOUTBUF.twiMR);
  sio_putstr(spbuf);
  
  GPparambuf.sio_filtermask = tempfiltmask;
  
}


inline void cap_buf_capture_RXreq(const ipmb_msg_desc_t* pmsg) {
  const unsigned char* psrc = (const unsigned char*) pmsg;
  unsigned char* pdst = (unsigned char*) &(CAPBUF.lastRXreq);
  int i;
  for (i=0; i<sizeof(ipmb_msg_desc_t); i++)
  *pdst++ = *psrc++;
  CAPBUF.lastRXreqTstamp = (AVR32_RTC).val;
}


inline void cap_buf_capture_TXreq(const ipmb_msg_desc_t* pmsg) {
  const unsigned char* psrc = (const unsigned char*) pmsg;
  unsigned char* pdst = (unsigned char*) &(CAPBUF.lastTXreq);
  int i;
  for (i=0; i<sizeof(ipmb_msg_desc_t); i++)
  *pdst++ = *psrc++;
  CAPBUF.lastTXreqTstamp = (AVR32_RTC).val;
}


inline void cap_buf_capture_RXrsp(const ipmb_msg_desc_t* pmsg) {
  const unsigned char* psrc = (const unsigned char*) pmsg;
  unsigned char* pdst = (unsigned char*) &(CAPBUF.lastRXrsp);
  int i;
  for (i=0; i<sizeof(ipmb_msg_desc_t); i++)
  *pdst++ = *psrc++;
  CAPBUF.lastRXrspTstamp = (AVR32_RTC).val;
}


inline void cap_buf_capture_TXrsp(const ipmb_msg_desc_t* pmsg) {
  const unsigned char* psrc = (const unsigned char*) pmsg;
  unsigned char* pdst = (unsigned char*) &(CAPBUF.lastTXrsp);
  int i;
  for (i=0; i<sizeof(ipmb_msg_desc_t); i++)
  *pdst++ = *psrc++;
  CAPBUF.lastTXrspTstamp = (AVR32_RTC).val;
}


void cap_buf_capture_twi_state(void) {
  const unsigned char* psrc = (const unsigned char*) &IPMB_stats;
  unsigned char* pdst = (unsigned char*) &(CAPBUF.ipmbstats);
  int i;
  CAPBUF.twiMR = (AVR32_TWI).imr;
  CAPBUF.twiSR = (AVR32_TWI).sr;
  CAPBUF.twistate = twi_state.state;
  
  for (i=0; i<sizeof(IPMB_stat_rec_t); i++)
  *pdst++ = *psrc++;
}

#endif


const char* systimestr(void) {
  long systime;
  int i1;
  // returns a ctime formatted string without newline character termination from the current system time
  systime = get_rtc_value();
  ctime_r(&systime, timebuf);
  timebuf[sizeof(timebuf)-1] = 0;     // null terminate last byte
  for (i1 = 0; i1<sizeof(timebuf)-1; i1++) 
    if ((timebuf[i1] == '\r') || (timebuf[i1] == '\n')) {
      timebuf[i1] = 0;   // null terminate
      break;
    }
  return &timebuf[0];
}
