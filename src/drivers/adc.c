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

// external ADC SPI command constants (AD7298)
#define EXT_ADC_CMD_MASK          (0x8004)
#define EXT_ADC_CH0_FLAG          (0x2000)
#define EXT_ADC_CH1_FLAG          (0x1000)
#define EXT_ADC_CH2_FLAG          (0x0800)
#define EXT_ADC_CH3_FLAG          (0x0400)
#define EXT_ADC_CH4_FLAG          (0x0200)
#define EXT_ADC_CH5_FLAG          (0x0100)
#define EXT_ADC_CH6_FLAG          (0x0080)
#define EXT_ADC_CH7_FLAG          (0x0040)


unsigned long raw_adc_vals[ADC_CHANNEL_CNT];
front_end_scaling_factors_t ADCscaletbl[ADC_CHANNEL_CNT];
IPMI_readout_scaling_factors_t ADCreadouttbl[ADC_CHANNEL_CNT];
void AD7298_Bitbang_Cycle(const unsigned short wdata, unsigned short* rdatabuf);

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

    // channel 1 - 1X attenuator with V-to-mV conversion
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

    // channel 5 - 5.65X attenuator with V-to-mV conversion
    ADCscaletbl[IADC_CH5].Mnum = mVperV*IADC5_RLOW;
    ADCscaletbl[IADC_CH5].Mdenom = IADC5_RLOW+IADC5_RHIGH;
    ADCscaletbl[IADC_CH5].B = 0;

    // channel 6 - TMP36 sensor+amplifier, M=14.64 mV/degC, B=732mV
    ADCscaletbl[IADC_CH6].Mnum = 1464;
    ADCscaletbl[IADC_CH6].Mdenom = 100;
    ADCscaletbl[IADC_CH6].B = 732;

    // channel 7 - TMP36 sensor+amplifier, M=14.64 mV/degC, B=732mV
    ADCscaletbl[IADC_CH7].Mnum = 1464;
    ADCscaletbl[IADC_CH7].Mdenom = 100;
    ADCscaletbl[IADC_CH7].B = 732; 

    // channel 8 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[XADC_CH0].Mnum = mVperV*XADC0_RLOW;
    ADCscaletbl[XADC_CH0].Mdenom = XADC0_RLOW+XADC0_RHIGH;
    ADCscaletbl[XADC_CH0].B = 0;
	
    // channel 9 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[XADC_CH1].Mnum = mVperV*XADC1_RLOW;
    ADCscaletbl[XADC_CH1].Mdenom = XADC1_RLOW+XADC1_RHIGH;
    ADCscaletbl[XADC_CH1].B = 0;
	
    // channel 10 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[XADC_CH2].Mnum = mVperV*XADC2_RLOW;
    ADCscaletbl[XADC_CH2].Mdenom = XADC2_RLOW+XADC2_RHIGH;
    ADCscaletbl[XADC_CH2].B = 0;
	
    // channel 11 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[XADC_CH3].Mnum = mVperV*XADC3_RLOW;
    ADCscaletbl[XADC_CH3].Mdenom = XADC3_RLOW+XADC3_RHIGH;
    ADCscaletbl[XADC_CH3].B = 0;
	
    // channel 12 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[XADC_CH4].Mnum = mVperV*XADC4_RLOW;
    ADCscaletbl[XADC_CH4].Mdenom = XADC4_RLOW+XADC4_RHIGH;
    ADCscaletbl[XADC_CH4].B = 0;
	
    // channel 13 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[XADC_CH5].Mnum = mVperV*XADC5_RLOW;
    ADCscaletbl[XADC_CH5].Mdenom = XADC5_RLOW+XADC5_RHIGH;
    ADCscaletbl[XADC_CH5].B = 0;
	
    // channel 14 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[XADC_CH6].Mnum = mVperV*XADC6_RLOW;
    ADCscaletbl[XADC_CH6].Mdenom = XADC6_RLOW+XADC6_RHIGH;
    ADCscaletbl[XADC_CH6].B = 0;
	
    // channel 15 - 1X attenuator with V-to-mV conversion
    ADCscaletbl[XADC_CH7].Mnum = mVperV*XADC7_RLOW;
    ADCscaletbl[XADC_CH7].Mdenom = XADC7_RLOW+XADC7_RHIGH;
    ADCscaletbl[XADC_CH7].B = 0;

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


void readout_external_ADC(void) {
	unsigned short rdbuffer;
	
	if (pyldmgr_get_backend_power_status() == power_off) {
		// zero out converter values and return
    raw_adc_vals[XADC_CH0] = 0;
    raw_adc_vals[XADC_CH1] = 0;
    raw_adc_vals[XADC_CH2] = 0;
    raw_adc_vals[XADC_CH3] = 0;
    raw_adc_vals[XADC_CH4] = 0;
    raw_adc_vals[XADC_CH5] = 0;
    raw_adc_vals[XADC_CH6] = 0;
    raw_adc_vals[XADC_CH7] = 0;
		return;
	}
	
	// this function reads out the exteral ADC.  It is called from sensor service during execution of the main loop
  
	// readout of AD7298 consists of 10 (8+2) 16-bit SPI port transfers, executed in sequence.  Interrupts are disabled while SPI transfers are in process,
	// but are reenabled in between the transfers.
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH0_FLAG), &rdbuffer);
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH1_FLAG), &rdbuffer);
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH2_FLAG), &rdbuffer);
  raw_adc_vals[XADC_CH0] = 0xfff & rdbuffer;
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH3_FLAG), &rdbuffer);
  raw_adc_vals[XADC_CH1] = 0xfff & rdbuffer;
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH4_FLAG), &rdbuffer);
  raw_adc_vals[XADC_CH2] = 0xfff & rdbuffer;
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH5_FLAG), &rdbuffer);
  raw_adc_vals[XADC_CH3] = 0xfff & rdbuffer;
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH6_FLAG), &rdbuffer);
  raw_adc_vals[XADC_CH4] = 0xfff & rdbuffer;
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH7_FLAG), &rdbuffer);
  raw_adc_vals[XADC_CH5] = 0xfff & rdbuffer;
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH0_FLAG), &rdbuffer);
  raw_adc_vals[XADC_CH6] = 0xfff & rdbuffer;
	AD7298_Bitbang_Cycle((EXT_ADC_CMD_MASK | EXT_ADC_CH1_FLAG), &rdbuffer);
  raw_adc_vals[XADC_CH7] = 0xfff & rdbuffer;
}


long get_full_scale_lsb(int chnum) {
	// returns the full scale range in lsb's for a given ADC channel
	if (chnum <= IADC_CH7)
	  return INTADCFULLSCALElsb;
	else
	  return EXTADCFULLSCALElsb;
}


long get_full_scale_mV(int chnum) {
	// returns the full scale range in mV for a given ADC channel
	if (chnum <= IADC_CH7)
	  return INTADCFULLSCALEmV;
	else
	  return EXTADCFULLSCALEmv;
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


void AD7298_Bitbang_Cycle(const unsigned short wdata, unsigned short* rdatabuf) {
	// bit-banged SPI-like cycle for AD7298 ADC, which is regrettibly not compatible with the AVR SPI port,
	// owing to the fact that the AD7298 uses the falling edge of the clock for both reading and writing.
	#define BITBANGDELAYVAL             (2)
	#define BitBangDelay                {int i1; for (i1=0; i1<BITBANGDELAYVAL; i1++);}
	unsigned short wrbuffer = wdata;
	unsigned short rdbuffer = 0;
	int remclk = 16;
  
	Disable_global_interrupt();
	GPIO.port[1].ovrs = (1 << (ADC_SCK-32)) | (1 << (_ADC_SS0-32));
  BitBangDelay;
  GPIO.port[1].ovrc = (1 << (_ADC_SS0-32));   // drop slave select
	while (remclk) {
	  // load next data bit from device
	  rdbuffer = (rdbuffer << 1) & 0xfffe;          // right shift data, assume 0 at lsb
	  if (GPIO.port[1].pvr & (1 << (ADC_MISO-32)))
	    rdbuffer |= 0x0001;                         // lsb is a 1
		// shift out next MOSI bit
		if (wrbuffer & 0x8000)
		  GPIO.port[1].ovrs = (1 << (ADC_MOSI-32));
		else
		  GPIO.port[1].ovrc = (1 << (ADC_MOSI-32));
		wrbuffer = wrbuffer << 1;
		// BitBangDelay;  -- not needed due to inherent delay of above lines of code
		// falling edge of clock -- loads MSB of write buffer at AD7298, 
		GPIO.port[1].ovrc = (1 << (ADC_SCK-32));
		BitBangDelay;
    // rising edge of clock
		GPIO.port[1].ovrs = (1 << (ADC_SCK-32));
		remclk--;
	}
  // raise chip select
  BitBangDelay;
	GPIO.port[1].ovrs = (1 << (_ADC_SS0-32));
  *rdatabuf = rdbuffer;
  	
	Enable_global_interrupt();
}
