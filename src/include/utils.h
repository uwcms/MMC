/*
 * utils.h
 *
 *  Created on: Oct 6, 2010
 *      Author: tgorski
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "pins.h"
#include "gpio.h"
#include "twidriver.h"
#include "ipmb_svc.h"

#ifdef TRACE
//************ stuff for tracing analysis
typedef struct {
  unsigned long pccapidx1;
  unsigned long pccapbuf1[15];
  unsigned long pccapidx2;
  unsigned long pccapbuf2[15];
  unsigned long codebittags;
  ipmb_msg_desc_t lastRXreq;
  ipmb_msg_desc_t lastTXreq;
  ipmb_msg_desc_t lastRXrsp;
  ipmb_msg_desc_t lastTXrsp;
  unsigned long lastRXreqTstamp;
  unsigned long lastTXreqTstamp;
  unsigned long lastRXrspTstamp;
  unsigned long lastTXrspTstamp;
  IPMB_stat_rec_t ipmbstats;
  enum TWISTATE twistate;
  unsigned long twiSR;
  unsigned long twiMR;
} capture_buf_rec_t;

typedef struct {
  unsigned long hottime;
  unsigned long uptime;
  unsigned long reset_cnt;
  unsigned long lastrsttime;
} timestats_rec_t;

#define CAPBUFBASEADDR            (0x0000fe00)
#define CAPBUF                    (*((capture_buf_rec_t*) CAPBUFBASEADDR))
#define RDOUTBUFBASEADDR          (0x0000fc00)
#define RDOUTBUF                  (*((capture_buf_rec_t*) RDOUTBUFBASEADDR))
#define TIMESTATSADDR             (0x0000ffe8)
#define TIMESTATREC               (*((timestats_rec_t*) TIMESTATSADDR))


#define SETCBBITFIELD(b)            (CAPBUF.codebittags |= (1<<b))
#define CLRCBBITFIELD(b)            (CAPBUF.codebittags &= ~(1<<b))

#define CAPBUF1_SAVEPC             {  asm volatile("\n" \
  "pushm r8-r9\n" \
  "mov r8,0xfe00\n" \
  "ld.w r9,r8\n" \
  "cp.w r9,60\n" \
  "movge r9,0\n" \
  "sub r9,-4\n" \
  "st.w r8,r9\n" \
  "add r8,r9\n" \
  "st.w r8,pc\n" \
  "popm r8-r9\n" \
); }

#define CAPBUF2_SAVEPC             {  asm volatile("\n" \
  "pushm r8-r9\n" \
  "mov r8,0xfe40\n" \
  "ld.w r9,r8\n" \
  "cp.w r9,60\n" \
  "movge r9,0\n" \
  "sub r9,-4\n" \
  "st.w r8,r9\n" \
  "add r8,r9\n" \
  "st.w r8,pc\n" \
  "popm r8-r9\n" \
); }



#define BITFIELDMASK(w, p)      ((0xffffffff >> (31-(w))) << (p))
#define SETTRACEBITFIELD(v, w, p)   ((CAPBUF.codebittags & ~BITFIELDMASK(w,p)) | ((v) & BITFIELDMASK(w,0) << (p)))

void set_cap_buf_ena(void);
void reset_cap_buf_ena(void);
inline int get_cap_buf_ena(void);
void print_cap_buf(void);
inline void cap_buf_capture_RXreq(const ipmb_msg_desc_t* pmsg);
inline void cap_buf_capture_TXreq(const ipmb_msg_desc_t* pmsg);
inline void cap_buf_capture_RXrsp(const ipmb_msg_desc_t* pmsg);
inline void cap_buf_capture_TXrsp(const ipmb_msg_desc_t* pmsg);
void cap_buf_capture_twi_state(void);
#else
#define CAPBUF1_SAVEPC            // alternate null definition
#define CAPBUF2_SAVEPC            // alternate null definition
#define SETTRACEBITFIELD(v, w, p)
#endif


#define Is_global_interrupt_enabled()         (!Tst_bits(Get_system_register(AVR32_SR), AVR32_SR_GM_MASK))

/*! \brief Disables interrupts globally.
 */
#if (defined __GNUC__)
  #define Disable_global_interrupt()          ({__asm__ __volatile__ ("ssrf\t%0" :  : "i" (AVR32_SR_GM_OFFSET));})
#elif (defined __ICCAVR32__)
  #define Disable_global_interrupt()          (__disable_interrupt())
#endif

/*! \brief Enables interrupts globally.
 */
#if (defined __GNUC__)
  #define Enable_global_interrupt()           ({__asm__ __volatile__ ("csrf\t%0" :  : "i" (AVR32_SR_GM_OFFSET));})
#elif (defined __ICCAVR32__)
  #define Enable_global_interrupt()           (__enable_interrupt())
#endif

#define Int_Disable(b)					{ if (b) Disable_global_interrupt(); }
#define Int_Restore(b)					{ if (b) Enable_global_interrupt();}

void Enable_Watchdog(void);
void Disable_Watchdog(void);
void Service_Watchdog(void);
void Force_Watchdog_Reset(void);

const char* systimestr(void);


#endif /* UTILS_H_ */
