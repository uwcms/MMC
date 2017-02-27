/*
 * ejecthandle.c
 *
 *  Created on: Aug 4, 2010
 *      Author: tgorski
 */

#include <stdio.h>
#include "pins.h"
#include "timer_callback.h"
#include "utils.h"
#include "gpio.h"
#include "ejecthandle.h"
#include "swevent.h"


#define SWITCHPOLLLIMIT		(5)					// number of consecutive polls to accept new state


int pinchangeflag;
unsigned char handle_state_change_flag;
unsigned char handle_pos_override_flag;
short override_duration;
Eject_Hdl_Pos OverridePosition;
Eject_Hdl_Pos HandleStableState;
ptrDriverISRCallback hdlchgcallbackptr;

#define gethandlepinstate()					(gpio_get_pin_value(IPMI_EJTHDL)==0 ? in_closed : out_open)

int eject_poll_callback(event evID, void* arg);

int override_timer_callback(event evID, void* arg);

void eject_handle_init(void) {
  hdlchgcallbackptr = 0;
  HandleStableState = gethandlepinstate();
  pinchangeflag = 0;
  handle_state_change_flag = 0;
  handle_pos_override_flag = 0;
  override_duration = 0;
  register_timer_callback(eject_poll_callback, TEVENT_2MSEC);
  register_timer_callback(override_timer_callback, TEVENT_100MSEC_1);
}


void register_eject_handle_change_callback(ptrDriverISRCallback funcaddr) {
  // this function is called when the eject handle changes state
  hdlchgcallbackptr = funcaddr;
}



Eject_Hdl_Pos get_eject_handle_stable_state(void) {
  return HandleStableState;
}

void set_handle_position_override(Eject_Hdl_Pos pos, unsigned char duration) {
  if (pos == indeterminate)
    return;
  Disable_global_interrupt();
  if (!duration)
    override_duration = -1;             // set to indefinite
  else
    override_duration = duration;
  OverridePosition = pos;
  handle_pos_override_flag = 1;
  Enable_global_interrupt();
}

void clear_handle_position_override(void) {
  handle_pos_override_flag = 0;
}


int eject_poll_callback(event evID, void* arg) {
  // callback for polling the switch.  Callback runs on 2ms events
  static int switchpollcnt;
  static Eject_Hdl_Pos pollinitstate;
  Eject_Hdl_Pos curpos = handle_pos_override_flag ? OverridePosition : gethandlepinstate();
  if (!pinchangeflag) {
	  // as far as we know, things are stable with the eject handle position
	  // compare the current position with the stable state
	  if (curpos != HandleStableState) {
	    // detected initial change
	    pollinitstate = curpos;				// mark new position
	    pinchangeflag = 1;					// set flag to indicate now searching for new stable switch state
	    switchpollcnt = 0;					// reset poll count
	  }
  }
  else {
	  // working on finding new stable state
    if (curpos == pollinitstate) {
      // current position same as start
      switchpollcnt++;
      if (switchpollcnt >= SWITCHPOLLLIMIT) {
    	  // enough ticks in a row--assign new state as stable
    	  if (HandleStableState != curpos) {
    	    // new stable state, so set switch event flag
    	    HandleStableState = curpos;
   	      if (hdlchgcallbackptr != 0)
            (*hdlchgcallbackptr)();            // call handling function
    	    //handle_state_change_flag = 1;
    	  }
    	  pinchangeflag = 0;					// clear the change flag
      }
    }
    else {
      // value changed from initial state (switch bounce), so restart polling
      pollinitstate = curpos;
      switchpollcnt = 0;
    }
  }
  return 1;
}


int override_timer_callback(event evID, void* arg) {
  if (override_duration > 0) {
    override_duration--;
    if (!override_duration)
      handle_pos_override_flag = 0;
  }
  return 1;
}



