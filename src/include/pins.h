/*
 * cmcpins.h
 *
 *  Created on: Oct 5, 2010
 *      Author: tgorski
 */

#ifndef PINS_H_
#define PINS_H_

#include <avr32/io.h>

// CMC pin definitions
// Port A
#define RS232_RCV			AVR32_PIN_PA00
#define RS232_XMT			AVR32_PIN_PA01
#define	IPMI_GA2			AVR32_PIN_PA02
#define IPMI_GA1			AVR32_PIN_PA03
#define IPMI_GA0			AVR32_PIN_PA04
#define IPMI_P1				AVR32_PIN_PA05
#define IPMI_EJTHDL		AVR32_PIN_PA06
#define	_IPMI_BLLED		AVR32_PIN_PA07
#define	_IPMI_LED1		AVR32_PIN_PA08
#define	_IPMI_LED2		AVR32_PIN_PA09
#define _EEP_CS				AVR32_PIN_PA10
#define EEP_MISO			AVR32_PIN_PA11
#define EEP_MOSI			AVR32_PIN_PA12
#define EEP_SCK				AVR32_PIN_PA13
#define	_FSIO_SS0			AVR32_PIN_PA14
#define	FSIO_SCK			AVR32_PIN_PA15
#define	FSIO_MOSI			AVR32_PIN_PA16
#define	FSIO_MISO			AVR32_PIN_PA17
#define	_FSIO_SS1			AVR32_PIN_PA18
#define	_FSIO_SS2			AVR32_PIN_PA19
#define	_FSIO_SS3			AVR32_PIN_PA20
#define	ADC_IN0				AVR32_PIN_PA21
#define	ADC_IN1				AVR32_PIN_PA22
#define	ADC_IN2				AVR32_PIN_PA23
#define	ADC_IN3				AVR32_PIN_PA24
#define	ADC_IN4				AVR32_PIN_PA25
#define	ADC_IN5				AVR32_PIN_PA26
#define	ADC_IN6				AVR32_PIN_PA27
#define	ADC_IN7				AVR32_PIN_PA28
#define	IPMI_SDA			AVR32_PIN_PA29
#define	IPMI_SCL			AVR32_PIN_PA30

// Port B
#define FPGA_DONE			  AVR32_PIN_PB00
#define PWRGOOD				  AVR32_PIN_PB01
#define	_GP_LED0			  AVR32_PIN_PB16
#define	_GP_LED1			  AVR32_PIN_PB17
#define _FPGA_FLASH_LOAD	AVR32_PIN_PB25
#define	UC_PWRENA1			AVR32_PIN_PB29
#define UC_PWRENA2      AVR32_PIN_PB30
#define _FSIO_SCANSLV		AVR32_PIN_PB31


#endif /* PINS_H_ */
