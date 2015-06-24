/*
 * adcreadout.c
 *
 *  Created on: Aug 5, 2010
 *      Author: tgorski
 */

#include <avr32/io.h>
#include <stdio.h>
#include <string.h>
#include "pins.h"
#include "gpio.h"
#include "intc.h"
#include "utils.h"
#include "timer_callback.h"
#include "adc.h"
#include "eepspi.h"
#include "spi1.h"
#include "sio_usart.h"
#include "nonvolatile.h"
#include "payload_mgr.h"


#define ADC				(&AVR32_ADC)

unsigned long raw_adc_vals[ADC_CHANNEL_CNT];
front_end_scaling_factors_t ADCscaletbl[ADC_CHANNEL_CNT];
IPMI_readout_scaling_factors_t ADCreadouttbl[ADC_CHANNEL_CNT];

ptrDriverISRCallback adccallbackptr;

int adc_timer_callback(event evID, void* arg);


#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void adc_int_handler(void) {
  // handler for convertion complete interrupt
  CAPBUF2_SAVEPC;
  Disable_global_interrupt();
  ADC->sr;
  raw_adc_vals[IADC_CH0] = ADC->cdr0;
  raw_adc_vals[IADC_CH1] = ADC->cdr1;
  raw_adc_vals[IADC_CH2] = ADC->cdr2;
  raw_adc_vals[IADC_CH3] = ADC->cdr3;
  raw_adc_vals[IADC_CH4] = ADC->cdr4;
  raw_adc_vals[IADC_CH5] = ADC->cdr5;
  raw_adc_vals[IADC_CH6] = ADC->cdr6;
  raw_adc_vals[IADC_CH7] = ADC->cdr7;
  if (adccallbackptr != 0)
	(*adccallbackptr)();						// call handling function
  Enable_global_interrupt();
}


void adc_init(void) {
  adccallbackptr = 0;						// initialize callback pointer
  ADC->mr = 0x00013320;						// SHTIM=0, STARTUP=0x1, PRESCAL = 0x33 (99d), SLEEP=1, LOWRES=0, TRIGEN=0
  ADC->cher = 0xff;							// enable all 8 channels
  ADC->idr = 0xffff;						// disable all interrupts
  ADC->ier = 0x80;							// enable EOC7 interrupt
  memset((void*) &raw_adc_vals, 0, sizeof(raw_adc_vals));
  memset((void*) &ADCreadouttbl, 0, sizeof(ADCreadouttbl));

  // read the ADC front end scaling factors from EEPROM.  If they are blank, then initialize them
  // with default valules
  while (eepspi_chk_write_in_progress());     // wait for EEPROM to be available
  eepspi_read((unsigned char*) &ADCscaletbl, ADC_SCALING_AREA_BYTE_OFFSET, sizeof(ADCscaletbl));
  if (ADCscaletbl[IADC_CH0].Mnum == -1) {
    // scaling factor section is blank.  Initialize
    sio_putstr("Uninitialized ADC Front End Scaling Factor area detected.  Writing default image....\n");

    // channel 0 - 5.65X attenuator with V-to-mV conversion
    ADCscaletbl[IADC_CH0].Mnum = mVperV*IADC0_RLOW;
    ADCscaletbl[IADC_CH0].Mdenom = IADC0_RLOW+IADC0_RHIGH;
    ADCscaletbl[IADC_CH0].B = 0;

    // channel 1 - 5.65X attenuator with V-to-mV conversion
    ADCscaletbl[IADC_CH1].Mnum = mVperV*IADC1_RLOW;
    ADCscaletbl[IADC_CH1].Mdenom = IADC1_RLOW+IADC1_RHIGH;
    ADCscaletbl[IADC_CH1].B = 0;

    // channel 2 - 1.33X attenuator with V-to-mV conversion
    ADCscaletbl[IADC_CH2].Mnum = mVperV*IADC2_RLOW;
    ADCscaletbl[IADC_CH2].Mdenom = IADC2_RLOW+IADC2_RHIGH;
    ADCscaletbl[IADC_CH2].B = 0;

    // channel 3 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[IADC_CH3].Mnum = mVperV*IADC3_RLOW;
    ADCscaletbl[IADC_CH3].Mdenom = IADC3_RLOW+IADC3_RHIGH;
    ADCscaletbl[IADC_CH3].B = 0;

    // channel 4 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[IADC_CH4].Mnum = mVperV*IADC4_RLOW;
    ADCscaletbl[IADC_CH4].Mdenom = IADC4_RLOW+IADC4_RHIGH;
    ADCscaletbl[IADC_CH4].B = 0;

    // channel 5 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[IADC_CH5].Mnum = mVperV*IADC5_RLOW;
    ADCscaletbl[IADC_CH5].Mdenom = IADC5_RLOW+IADC5_RHIGH;
    ADCscaletbl[IADC_CH5].B = 0;

    // channel 6 - TMP36 sensor+amplifier, M=14.64 mV/degC, B=732mV
    ADCscaletbl[IADC_CH6].Mnum = mVperV*IADC6_RLOW;
    ADCscaletbl[IADC_CH6].Mdenom = IADC6_RLOW+IADC6_RHIGH;
    ADCscaletbl[IADC_CH6].B = 0;

    // channel 7 - TMP36 sensor+amplifier, M=10.00 mV/degC, B=500mV
    ADCscaletbl[IADC_CH7].Mnum = 10;
    ADCscaletbl[IADC_CH7].Mdenom = 1;
    ADCscaletbl[IADC_CH7].B = 500; 

    eepspi_write((unsigned char*) &ADCscaletbl, ADC_SCALING_AREA_BYTE_OFFSET, sizeof(ADCscaletbl));
  }
  INTC_register_interrupt(&adc_int_handler, AVR32_ADC_IRQ, AVR32_INTC_INT0);
  register_timer_callback(adc_timer_callback, TEVENT_100MSEC_0);
}



void adc_register_convert_complete_callback(ptrDriverISRCallback funcaddr) {
  // this function stores the specified address as the routine to be called by the
  // adc conversion complete IRQ
  adccallbackptr = funcaddr;
}


unsigned char get_adc_channel_sensor_readout_value(long chnum) {
  long workingval;
  
  if ((chnum < MIN_ADC_CH_NUM) || (chnum > MAX_ADC_CH_NUM))
    return 0;			// out of range
  workingval = ADC_MIN_READOUT_VAL + (ADC_MAX_READOUT_VAL-ADC_MIN_READOUT_VAL)*((long) raw_adc_vals[chnum]-ADCreadouttbl[chnum].ReadoutScalingMinVal);
  workingval = workingval / (ADCreadouttbl[chnum].ReadoutScalingMaxVal-ADCreadouttbl[chnum].ReadoutScalingMinVal);
  if (workingval < 0)
    return 0;					// underflow
  else
    if (workingval > 0xff)
      return 0xff;				// overflow
  return (unsigned char) workingval;
}


int adc_timer_callback(event evID, void* arg) {
  // this function is called by the timer callback facility
  // it starts an internal adc conversion cycle using the software trigger
  ADC->cr = 0x2;				// start conversion
  return 1;
}



long get_full_scale_lsb(int chnum) {
	// returns the full scale range in lsb's for a given ADC channel
  return INTADCFULLSCALElsb;
}


long get_full_scale_mV(int chnum) {
	// returns the full scale range in mV for a given ADC channel
  return INTADCFULLSCALEmV;
}


int get_analog_voltage_string(int chnum, char* pValuestr) {
	// uses the full resolution of the ADC channel to return an ascii string corresponding to the
	// raw signal value at the front end of the specified channel, as back-calculated using the
	// given attenuator values.  String value is given in volts with three digits to right of decimal point.
	// returns a value of 1 if a string is calculated, and 0 if the channel number is out of range or if
	// there is some other problem.
	float ADCvalV, Bf, Mnf, Mdf, sig;
	if ((chnum < 0) || (chnum > MAX_ADC_CH_NUM))	
	  return 0;
	
	// convert all variables to floating point first
	ADCvalV = get_full_scale_mV(chnum) * raw_adc_vals[chnum];
	ADCvalV = ADCvalV / ((float) get_full_scale_lsb(chnum) * 1000); 
	Bf = (float) ADCscaletbl[chnum].B / 1000;               // convert offset to volts
  Mnf = (float) ADCscaletbl[chnum].Mnum / 1000;                   // convert gain to volts
  Mdf = ADCscaletbl[chnum].Mdenom;
  sig = (ADCvalV - Bf) * Mdf / Mnf;                           // front end signal value
  sprintf(pValuestr, "%6.3f", sig);
  return 1;	
}


