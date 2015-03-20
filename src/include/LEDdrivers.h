/*
 * LEDdrivers.h
 *
 *  Created on: Sep 3, 2010
 *      Author: tgorski
 */

#ifndef LEDDRIVERS_H_
#define LEDDRIVERS_H_

#include "gpio.h"

// macros for driving LEDs directly from code.  Useful for debugging, but recommended method
// for production code is to use activity descriptors & the timer-based driver
#define LEDOFF				(1)
#define LEDON				(0)

#define LED_on(pin)			(gpio_clr_gpio_pin(pin))
#define LED_off(pin)		(gpio_set_gpio_pin(pin))
#define LED_tog(pin)		(gpio_tgl_gpio_pin(pin))
#define LED_set_state(pin, val)		{if (val==LEDOFF) LED_off(pin); else LED_on(pin); }

#define LED_is_off(pin)     (gpio_get_pin_value(pin))
#define LED_is_on(pin)      (!gpio_get_pin_value(pin))
#define LED_get_state(pin)	((unsigned char) gpio_get_pin_value(pin))


// PICMG-defined color codes for set/get LED state commands
#define LEDCOLOR_BLUE					(1)
#define LEDCOLOR_RED					(2)
#define LEDCOLOR_GREEN					(3)
#define LEDCOLOR_AMBER					(4)
#define LEDCOLOR_ORANGE					(5)
#define LEDCOLOR_WHITE					(6)
#define LEDCOLOR_NOCHANGE				(0xe)
#define LEDCOLOR_DEFAULT				(0xf)

#define LED_CNT							(6)
#define IPMI_LED_CNT					(3)				// count of IPMI LEDs in table
#define IPMI_BLUELED_TBL_IDX	(0)
#define IPMI_LED1R_TBL_IDX		(1)
#define IPMI_LED2_TBL_IDX			(2)
#define LED0_TBL_IDX					(3)
#define IPMI_LED1Y_TBL_IDX		(4)       // do not report this LED to the carrier as an IPMI LED (used for local control but in the IPMI LED1 position)
#define LED2_TBL_IDX					(5)

typedef enum {Local_Control=0, Override} LEDstate_t;

typedef enum {On=0, Off, Blink, Bypass} LEDactivity_t;

typedef struct {
  LEDactivity_t action;
  unsigned char initstate;						// initial state, either LEDOFF or LEDON
  unsigned short delay1;							// period in initial state in 10ms units
  unsigned short delay2;							// period in opposite state in 10ms units, 0=skip
} LED_activity_desc_t;


typedef struct {
  LEDstate_t LEDstate;
  const LED_activity_desc_t* pLocalDesc;
  const LED_activity_desc_t* pOvrideDesc;
  unsigned short LocalPscale;
  unsigned short OvridePscale;
  unsigned char Color;
  unsigned short Pin;							// GPIO pin identifier
} LED_state_rec_t;


extern const LED_activity_desc_t LED_Off_Activity;
extern const LED_activity_desc_t LED_On_Activity;
extern const LED_activity_desc_t LED_50ms_Flash_Activity;
extern const LED_activity_desc_t LED_1Hz_Blink_Activity;
extern const LED_activity_desc_t LED_1Hz_Blink_Activity_8020;
extern const LED_activity_desc_t LED_1Hz_Blink_Activity_7525;
extern const LED_activity_desc_t LED_1p5Hz_Blink_Activity;
extern const LED_activity_desc_t LED_0p5Hz_Blink_Activity;
extern const LED_activity_desc_t LED_driver_bypass;
extern const LED_activity_desc_t LED_2Hz_Blink_Activity;
extern const LED_activity_desc_t LED_3Hz_Blink_Activity;
extern const LED_activity_desc_t LED_3sec_Lamp_Test_Activity;

extern LED_state_rec_t LEDstate[LED_CNT];

void program_LED(unsigned short LEDID, LEDstate_t newLEDstate, const LED_activity_desc_t* pLEDact);

void LED_init(void);


#endif /* LEDDRIVERS_H_ */
