/*
 * swevent.c
 *
 *  Created on: Oct 11, 2010
 *      Author: tgorski
 */

#include <string.h>
#include "swevent.h"
#include "sio_usart.h"

#define SWEVENTTBLSIZE				(24)

CallbackTblEntry SwEventCallbackTbl[SWEVENTTBLSIZE];

void swevent_init(void) {
  memset((void*) SwEventCallbackTbl, 0, sizeof(SwEventCallbackTbl));
}


int register_swevent_callback(ptrCallback funcaddr, event eventID) {
  // installs callback in standard table.  Returns 1 if successful, 0 otherwise
  int i1;
  for (i1=0; i1<SWEVENTTBLSIZE; i1++) {
	if (!SwEventCallbackTbl[i1].active) {
	  SwEventCallbackTbl[i1].active = 1;
	  SwEventCallbackTbl[i1].eventID = eventID;
	  SwEventCallbackTbl[i1].ptr = funcaddr;
      return 1;
    }
  }
  sio_putstr("?Error, software callback table full!!\n");
  return 0;			// table full
}


void post_swevent(event eventID, void* arg) {
  event cbe = eventID;
  unsigned short cbi;

  for (cbi=0; cbi<SWEVENTTBLSIZE; cbi++)
    if (SwEventCallbackTbl[cbi].active)
      if (SwEventCallbackTbl[cbi].eventID == cbe) {
    	SwEventCallbackTbl[cbi].active = (*SwEventCallbackTbl[cbi].ptr)(cbe, arg);
      }
}




