/*
 * rtc.c
 *
 *  Created on: Oct 7, 2010
 *      Author: tgorski
 */

#include <avr32/io.h>
#include "rtc.h"

#define RTC			(AVR32_RTC)

void rtc_init(void) {
  unsigned long mask;
  // initialize the RTC to count at 1 Hz from the 32.768 kHz crystal
  while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_OSC32RDY_MASK));			// wait for 32kHz to be stable and ready
  while (RTC.CTRL.busy);      // wait for busy to clear
  RTC.top = 0xffffffff;           // set top value register
  while (RTC.CTRL.busy);      // wait for busy to clear
  RTC.val = 0x0;           // set top value register
  mask = (14 << AVR32_RTC_CTRL_PSEL_OFFSET) | (1 << AVR32_RTC_CTRL_CLK32_OFFSET) |
    (1 << AVR32_RTC_CTRL_CLKEN_OFFSET) | (1 << AVR32_RTC_CTRL_EN_OFFSET);
  while (RTC.CTRL.busy);      // wait for busy to clear
  RTC.ctrl = mask;
}


unsigned long get_rtc_value(void) {
  return RTC.val;
}


void set_rtc_value(unsigned long systime) {
  while (RTC.CTRL.busy);      // wait for busy to clear
  RTC.val = systime;
}

