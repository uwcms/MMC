/*
 * rtc.h
 *
 *  Created on: Oct 7, 2010
 *      Author: tgorski
 */

#ifndef RTC_H_
#define RTC_H_

void rtc_init(void);

unsigned long get_rtc_value(void);

void set_rtc_value(unsigned long systime);

#endif /* RTC_H_ */
