/*
 * timer_callback.h
 *
 *  Created on: Jun 23, 2010
 *      Author: tgorski
 */

#ifndef TIMER_CALLBACK_H_
#define TIMER_CALLBACK_H_

#include "events.h"


void timer_callback_init(void);

int register_timer_callback(ptrCallback funcaddr, event eventID);

void start_timers(void);

#endif /* TIMER_CALLBACK_H_ */
