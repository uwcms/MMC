/*
 * adcreadout.h
 *
 *  Created on: Aug 5, 2010
 *      Author: tgorski
 */

#ifndef ADCREADOUT_H_
#define ADCREADOUT_H_

#include "events.h"

#define INTERNAL_ADC_CHANNEL_CNT					(8)
#define EXTERNAL_ADC_CHANNEL_CNT          (8)

#define UNDEFINED_ADC_CH_NUM			        (-1)

#define ADC_CHANNEL_CNT                   (INTERNAL_ADC_CHANNEL_CNT + EXTERNAL_ADC_CHANNEL_CNT)

#define MIN_ADC_CH_NUM					          (0)
#define MAX_ADC_CH_NUM                    (ADC_CHANNEL_CNT-1)

#define INTADCFULLSCALEmV						      (3000)				// internal ADC full scale range in mV
#define INTADCFULLSCALElsb						    (1024)				// internal ADC full scale range in lsb's
#define EXTADCFULLSCALEmv                 (2048)        // external ADC full scale range in mV
#define EXTADCFULLSCALElsb                (4096)        // external ADC full scale range in lsb's
#define mVperV                            (1000)        // mV per volt

#define IADC_CH0							(0)
#define IADC_CH1							(1)
#define IADC_CH2							(2)
#define IADC_CH3							(3)
#define IADC_CH4							(4)
#define IADC_CH5							(5)
#define IADC_CH6							(6)
#define IADC_CH7							(7)
#define XADC_CH0							(8)
#define XADC_CH1							(9)
#define XADC_CH2							(10)
#define XADC_CH3							(11)
#define XADC_CH4							(12)
#define XADC_CH5							(13)
#define XADC_CH6							(14)
#define XADC_CH7							(15)

// resistor divider values for ADC inputs (in ohms)
#define IADC0_RHIGH						(4640)
#define IADC0_RLOW						(1000)
#define IADC1_RHIGH           (0)
#define IADC1_RLOW            (1)
#define IADC2_RHIGH						(1540)
#define IADC2_RLOW						(4640)
#define IADC3_RHIGH						(0)
#define IADC3_RLOW						(1)
#define IADC4_RHIGH						(0)
#define IADC4_RLOW						(1)
#define IADC5_RHIGH           (4640)
#define IADC5_RLOW            (1000)

#define XADC0_RHIGH           (0)
#define XADC0_RLOW            (1)
#define XADC1_RHIGH           (0)
#define XADC1_RLOW            (1)
#define XADC2_RHIGH           (0)
#define XADC2_RLOW            (1)
#define XADC3_RHIGH           (0)
#define XADC3_RLOW            (1)
#define XADC4_RHIGH           (0)
#define XADC4_RLOW            (1)
#define XADC5_RHIGH           (0)
#define XADC5_RLOW            (1)
#define XADC6_RHIGH           (0)
#define XADC6_RLOW            (1)
#define XADC7_RHIGH           (0)
#define XADC7_RLOW            (1)


#define ADC_MIN_READOUT_VAL				(0)						// min value for readout scaling (IPMI uses 8-bit raw values)
#define ADC_MAX_READOUT_VAL				(255)					// max value for readout scaling (IPMI uses 8-bit raw values)

typedef struct {
  long Mnum;			  // multiplier scaling factor numerator
  long Mdenom;			// multiplier scaling factor denominator
  long B;			      // offset scaling factor
} front_end_scaling_factors_t;

typedef struct {
  long ReadoutScalingMinVal;				// minimum ADC value for 8-bit readout range scaling
  long ReadoutScalingMaxVal;				// maximum ADC value for 8-bit readout range scaling
} IPMI_readout_scaling_factors_t;

extern front_end_scaling_factors_t ADCscaletbl[ADC_CHANNEL_CNT];
extern IPMI_readout_scaling_factors_t ADCreadouttbl[ADC_CHANNEL_CNT];

void adc_init(void);

void adc_register_convert_complete_callback(ptrDriverISRCallback funcaddr);

void readout_external_ADC(void);

unsigned long get_adc_channel_converstion_value(int chnum);

unsigned char get_adc_channel_sensor_readout_value(long chnum);

long get_full_scale_lsb(int chnum);

long get_full_scale_mV(int chnum);

int get_analog_voltage_string(int chnum, char* pValuestr);


#endif /* ADCREADOUT_H_ */
