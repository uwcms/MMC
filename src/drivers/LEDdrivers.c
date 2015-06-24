/*
 * LEDdrivers.c
 *
 *  Created on: Sep 3, 2010
 *      Author: tgorski
 */

#include <stdio.h>
#include "utils.h"
#include "pins.h"
#include "timer_callback.h"
#include "LEDdrivers.h"

const LED_activity_desc_t LED_Off_Activity = {Off, LEDOFF, 0, 0};
const LED_activity_desc_t LED_On_Activity = {On, LEDON, 0, 0};
const LED_activity_desc_t LED_50ms_Flash_Activity = {Blink, LEDON, 5, 0};
const LED_activity_desc_t LED_1Hz_Blink_Activity = {Blink, LEDON, 50, 50};
const LED_activity_desc_t LED_driver_bypass = {Bypass, LEDOFF, 0, 0};
const LED_activity_desc_t LED_2Hz_Blink_Activity = {Blink, LEDON, 25, 25};
const LED_activity_desc_t LED_3sec_Lamp_Test_Activity = {Blink, LEDON, 300, 0};
const LED_activity_desc_t LED_1Hz_Blink_Activity_7525 = {Blink, LEDON, 75, 25};

LED_state_rec_t LEDstate[LED_CNT] = {
  {Local_Control, &LED_Off_Activity, NULL, 0, 0, LEDCOLOR_BLUE, _IPMI_BLLED},
  {Local_Control, &LED_Off_Activity, NULL, 0, 0, LEDCOLOR_RED, _IPMI_LED1},
  {Local_Control, &LED_Off_Activity, NULL, 0, 0, LEDCOLOR_GREEN, _IPMI_LED2},
  {Local_Control, &LED_Off_Activity, NULL, 0, 0, LEDCOLOR_GREEN, _GP_LED0},
  {Local_Control, &LED_Off_Activity, NULL, 0, 0, LEDCOLOR_GREEN, _GP_LED1}};

int IPMI_LED_callback(event evID, void* arg);


void program_LED(unsigned short LEDID, LEDstate_t newLEDstate, const LED_activity_desc_t* pLEDact) {
  LED_state_rec_t* pLED;
  if ((LEDID >= LED_CNT) || ((newLEDstate == Override) && (pLEDact == NULL)))
    return;						// bad argument

  pLED = &LEDstate[LEDID];
  Disable_global_interrupt();
  if (newLEDstate == Local_Control) {
    if (pLED->pLocalDesc != pLEDact) {
      // change in LED descriptor, update the local control pointer/prescaler
      if (pLEDact)
      pLED->pLocalDesc = pLEDact;                     // update local activity pointer
      pLED->LocalPscale = pLED->pLocalDesc->delay1;
    }
  }
  else {
    // update the override pointer/prescaler
	  pLED->pOvrideDesc = pLEDact;
	  pLED->OvridePscale = pLEDact->delay1;
  }
  pLED->LEDstate = newLEDstate;
  LED_set_state(pLED->Pin, pLEDact->initstate);
  Enable_global_interrupt();
}


void LED_init(void) {
  register_timer_callback(IPMI_LED_callback, TEVENT_10MSEC);
}



int IPMI_LED_callback(event evID, void* arg) {
  // service LEDs
  unsigned char i1;
  unsigned short* pCurPscale;
  const LED_activity_desc_t* pCurDesc;

  LED_state_rec_t* pLED;
  pLED = &LEDstate[0];
  for (i1=0; i1<LED_CNT; i1++) {
    if (pLED->LEDstate == Local_Control) {
      // point to local activity params
      pCurDesc = pLED->pLocalDesc;
      pCurPscale = &(pLED->LocalPscale);
    }
    else {
      // point to override activity params
      pCurDesc = pLED->pOvrideDesc;
      pCurPscale = &(pLED->OvridePscale);
    }

    // process activity
    switch (pCurDesc->action) {
      case On:
    	LED_on(pLED->Pin);
    	break;
      case Off:
    	LED_off(pLED->Pin);
    	break;
      case Blink:
    	(*pCurPscale)--;
    	if (!*pCurPscale) {
    	  // prescaler reached bottom--have to do some work now to figure out what to do next
          if (pCurDesc->initstate == LED_get_state(pLED->Pin))
        	  // LED in initial state
        	  *pCurPscale = pCurDesc->delay2;
          else
        	  // LED in toggled state
          	*pCurPscale = pCurDesc->delay1;
          LED_tog(pLED->Pin);
          if (!*pCurPscale) {
          	// loaded a zero prescale value--means the activity descriptor is a single-shot descriptor
          	// that has expired--like a lamp test or a pulse.
        	  // if LED is in an override state, revert to the local control state.
        	  // if LED is already in local control, revert to a static state which is opposite of the
        	  // single-shot state
        	  if (pLED->LEDstate == Local_Control) {
        	    if (pLED->pLocalDesc->initstate == LEDON) {
        		    pLED->pOvrideDesc = &LED_Off_Activity;
            	  LED_off(pLED->Pin);
              }
              else {
          		  pLED->pOvrideDesc = &LED_On_Activity;
              	LED_on(pLED->Pin);
       		    }
       	    }
        	  else {
    		      pLED->LEDstate = Local_Control;
    		      pCurDesc = pLED->pLocalDesc;
    		      pLED->LocalPscale = pCurDesc->delay1;
    		      LED_set_state(pLED->Pin, pCurDesc->initstate);
    		    }
          }
    	}
    	break;
      case Bypass:
      default:
    	// don't do anything
    	break;
    }
    pLED++;
  }
  return 1;
}

