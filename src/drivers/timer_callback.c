/*
 * timer_callback.c
 *
 *  Created on: Jun 23, 2010
 *      Author: tgorski
 */

#include <avr32/io.h>
#include <stdio.h>
#include <string.h>
#include "intc.h"
#include "utils.h"
#include "timer_callback.h"
#include "pwrmgr.h"

#define TIMERCALLBACKTBLSIZE    (24)

#define FAST_IRQ_PRIORITY	AVR32_INTC_INT1
#define SLOW_IRQ_PRIORITY	AVR32_INTC_INT0
#define FASTTC				(0)
#define SLOWTC				(1)
#define FASTTC_IRQ			AVR32_TC_IRQ0
#define SLOWTC_IRQ			AVR32_TC_IRQ1

#define TC0_CKDIV			(2)						// use timer_clock2 input source
#define TC0_CKFREQ			(PBAFREQ/TC0_CKDIV)
#define TC0_IRQFREQ			(5000)					// 200us interrupt timer
#define TC0_RCVAL			(TC0_CKFREQ/TC0_IRQFREQ-1)		// value for compare register C
#define TC0_PSMAX			(10)					// divider to get 2ms events from 200us events

#define TC1_CKDIV			(32)					// use timer_clock4 input source
#define TC1_CKFREQ			(PBAFREQ/TC1_CKDIV)
#define TC1_IRQFREQ			(100)					// 10ms interrupt timer
#define TC1_RCVAL			(TC1_CKFREQ/TC1_IRQFREQ-1)		// value for compare register C
#define TC1_PS1MAX			(10)					// prescaler to get 100ms events from 10ms events
#define TC1_PS2MAX			(10)					// divider to get 1sec events from 100ms events


CallbackTblEntry TimerCallbackTbl[TIMERCALLBACKTBLSIZE];
int cbi;
event cbe;
volatile avr32_tc_t *tc = (&AVR32_TC);
unsigned short prescale_TC0, prescale1_TC1, prescale2_TC1;

void make_callbacks(event eventID);


#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void fasttc_irq_hdlr(void) {
  unsigned long junk;
  junk = tc->channel[FASTTC].sr;
  make_callbacks(TEVENT_200USEC);
  prescale_TC0++;
  if (prescale_TC0 >= TC0_PSMAX) {
	prescale_TC0 = 0;
	make_callbacks(TEVENT_2MSEC);
  }
}


#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void slowtc_irq_hdlr(void) {
  unsigned long junk;
  junk = tc->channel[SLOWTC].sr;
  make_callbacks(TEVENT_10MSEC);
  prescale1_TC1++;
  if (prescale1_TC1 >= TC1_PS1MAX)
  	prescale1_TC1 = 0;
  switch (prescale1_TC1) {
    case 0:
      make_callbacks(TEVENT_100MSEC_0);
      break;
    case 2:
      make_callbacks(TEVENT_100MSEC_1);
      break;
    case 4:
      make_callbacks(TEVENT_100MSEC_2);
      break;
    case 6:
      make_callbacks(TEVENT_100MSEC_3);
      break;
    case 8:
      make_callbacks(TEVENT_100MSEC_4);
      break;
    case 9:
      // bump prescaler for 1 second callbacks
    	prescale2_TC1++;
    	if (prescale2_TC1 >= TC1_PS2MAX) {
      	prescale2_TC1 = 0;
      	make_callbacks(TEVENT_1SEC);
	    }
      break;
    default:
      break;
  }
}


void timer_callback_init(void) {
  volatile avr32_tc_channel_t  *chptr;					// channel pointer
  // initialize tables to empty
  memset((void*) TimerCallbackTbl, 0, sizeof(TimerCallbackTbl));

  chptr = &(tc->channel[FASTTC]);
  chptr->cmr = 0x0000c001;			// wave mode, wavesel=2, tcclk=clk2
  chptr->rc = TC0_RCVAL;			// value for compare register C
  chptr->ier = 0x00000010;			// enable RC compare interrupt

  chptr = &(tc->channel[SLOWTC]);
  chptr->cmr = 0x0000c003;			// wave mode, wavesel=2, tcclk=clk4
  chptr->rc = TC1_RCVAL;			// value for compare register C
  chptr->ier = 0x00000010;			// enable RC compare interrupt

  tc->bmr = 0;
  prescale_TC0 = 0;
  prescale1_TC1 = 0;
  prescale2_TC1 = 0;

  INTC_register_interrupt(&fasttc_irq_hdlr, FASTTC_IRQ, FAST_IRQ_PRIORITY);
  INTC_register_interrupt(&slowtc_irq_hdlr, SLOWTC_IRQ, SLOW_IRQ_PRIORITY);
}


int register_timer_callback(ptrCallback funcaddr, event eventID) {
  // installs callback in standard table.  Returns 0 if successful, 1 otherwise
  int i1;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  Disable_global_interrupt();
  for (i1=0; i1<TIMERCALLBACKTBLSIZE; i1++) {
    if (!TimerCallbackTbl[i1].active) {
      TimerCallbackTbl[i1].active = 1;
      TimerCallbackTbl[i1].eventID = eventID;
      TimerCallbackTbl[i1].ptr = funcaddr;
      Int_Restore(giflag);
      return 0;
    }
  }
  Int_Restore(giflag);
  return 1;   // table full
}


void make_callbacks(event eventID) {
  cbe = eventID;

  for (cbi=0; cbi<TIMERCALLBACKTBLSIZE; cbi++)
    if (TimerCallbackTbl[cbi].active)
      if (TimerCallbackTbl[cbi].eventID == cbe) {
        TimerCallbackTbl[cbi].active = (*TimerCallbackTbl[cbi].ptr)(cbe, NULL);
      }
}


void start_timers(void) {
  tc->channel[FASTTC].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
  tc->channel[SLOWTC].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}


