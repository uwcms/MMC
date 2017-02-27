/*
 * ejecthandle.h
 *
 *  Created on: Aug 4, 2010
 *      Author: tgorski
 */

#ifndef EJECTHANDLE_H_
#define EJECTHANDLE_H_

#include "events.h"

typedef enum {in_closed=0, out_open, indeterminate} Eject_Hdl_Pos;

void eject_handle_init(void);

void register_eject_handle_change_callback(ptrDriverISRCallback funcaddr);

Eject_Hdl_Pos get_eject_handle_stable_state(void);

void eject_handle_event_service(void);

void set_handle_position_override(Eject_Hdl_Pos pos, unsigned char duration);

void clear_handle_position_override(void);

#endif /* EJECTHANDLE_H_ */
