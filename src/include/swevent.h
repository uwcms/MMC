/*
 * swevent.h
 *
 *  Created on: Oct 11, 2010
 *      Author: tgorski
 */

#ifndef SWEVENT_H_
#define SWEVENT_H_

#include "events.h"

void swevent_init(void);

int register_swevent_callback(ptrCallback funcaddr, event eventID);

void post_swevent(event eventID, void* arg);

#endif /* SWEVENT_H_ */
