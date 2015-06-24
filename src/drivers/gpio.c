/*
 * gpio.c
 *
 *  Created on: Oct 5, 2010
 *      Author: tgorski
 */


#include "pins.h"
#include "utils.h"
#include "gpio.h"
#include "resetcfg.h"

void gpio_scanslv_mode(void) {
  // set pins in mode for scanning for FPGA SPI slaves
  unsigned long mask;
  gpio_set_gpio_pin(_FSIO_SCANSLV);             // turn off scan slave pin
  mask = (1 <<_FSIO_SS0) | (1<< FSIO_SCK) | (1 << FSIO_MOSI) | (1 << FSIO_MISO) | (1 << _FSIO_SS1) |
    (1 << _FSIO_SS2) | (1 << _FSIO_SS3);

  GPIO.port[0].oderc = mask;          // turn off outputs
  GPIO.port[0].puers = (1 <<_FSIO_SS0) | (1 << _FSIO_SS1) | (1 << _FSIO_SS2) | (1 << _FSIO_SS3);          // turn on pullups
  GPIO.port[0].gpers = mask;          // set to GPIO mode
  gpio_clr_gpio_pin(_FSIO_SCANSLV);   // activate scan slave pin
}


void gpio_spi1_mode(void) {
  // set pins in mode for SPI1 operation
  unsigned long mask;
  gpio_set_gpio_pin(_FSIO_SCANSLV);             // turn off scan slave pin
  mask = (1 <<_FSIO_SS0) | (1<< FSIO_SCK) | (1 << FSIO_MOSI) | (1 << FSIO_MISO) | (1 << _FSIO_SS1) |
    (1 << _FSIO_SS2) | (1 << _FSIO_SS3);
  GPIO.port[0].pmr0s = mask;        // set to mux code 01
  GPIO.port[1].pmr1c = mask;        // set to mux code 01
  GPIO.port[0].gperc = mask;        // set in peripheral mode
  GPIO.port[0].puerc = mask;        // turn off weak pullups
  GPIO.port[0].puers = (1 << FSIO_MISO);      // except for MISO
}


void gpio_spi1_bitbang(void) {
	// puts spi1 in bitbang mode (all GPIOs, all outputs except for MISO)
  unsigned long mask;
  gpio_set_gpio_pin(_FSIO_SCANSLV);             // turn off scan slave pin
  mask = (1 <<_FSIO_SS0) | (1<< FSIO_SCK) | (1 << FSIO_MOSI) | (1 << _FSIO_SS1) | (1 << _FSIO_SS2) | (1 << _FSIO_SS3);
  GPIO.port[0].ovrs = mask;
  GPIO.port[0].oders = mask;
  GPIO.port[0].oderc = (1 << FSIO_MISO);            // turn off MISO driver
  GPIO.port[0].puers = (1 << FSIO_MISO);            // set weak pullup for MISO
	GPIO.port[0].gpers = mask |(1 << FSIO_MISO);      // switch all SPI1 lines to GPIO
}

void gpio_init() {
  unsigned long mask;

  if (AVR32_PM.gplp[0] & RSTCFG_PAYLOAD_HOT_MASK) {
    // starting up into hot payload power

    // set power enable
    if (AVR32_PM.gplp[0] & RSTCFG_BACKEND_HOT_MASK) {
      // get power enable and FPGA load to high states
      GPIO.port[1].ovrs = ((1 << (UC_PWRENA1-32)) | (1 << (_FPGA_FLASH_LOAD-32)));
      GPIO.port[1].oders = ((1 << (UC_PWRENA1-32)) | (1 << (_FPGA_FLASH_LOAD-32)));
    }
    else {
      // back end not hot, clear power enable
      GPIO.port[1].ovrc = ((1 << (UC_PWRENA1-32)) | (1<< (_FPGA_FLASH_LOAD-32)));
      GPIO.port[1].oders = ((1 << (UC_PWRENA1-32)) | (1<< (_FPGA_FLASH_LOAD-32)));
    }
  }
  else {
    // starting up into cold payload power

    mask = ((1 << (UC_PWRENA1-32)) | (1 << (UC_PWRENA2-32)));
    GPIO.port[1].gpers = mask;            // set as GPIO pins
    GPIO.port[1].oderc = mask;            // turn off output drivers
    GPIO.port[1].puerc = mask;            // disable pullups
    // set the enables to outputs, driving logic 0s
    gpio_clr_gpio_pin(UC_PWRENA1);
    gpio_clr_gpio_pin(UC_PWRENA2);


    // zero out payload-connected IOs, to eliminate sneak paths
    gpio_clr_gpio_pin(_FSIO_SCANSLV);
    gpio_clr_gpio_pin(_FSIO_SS0);
    gpio_clr_gpio_pin(FSIO_SCK);
    gpio_clr_gpio_pin(FSIO_MOSI);
    gpio_clr_gpio_pin(FSIO_MISO);
  }

  // finish initialization of non power-related GPIO pins
/* Initialize GPIO pins for their various functions.

The following pins use peripheral mux function A on Port A (0)
	RS232_RCV	RS232_XMT	_EEP_CS		EEP_MISO	EEP_MOSI	EEP_SCK
	ADC_IN0		ADC_IN1		ADC_IN2		ADC_IN3		ADC_IN4		ADC_IN5		ADC_IN6		ADC_IN7
	IPMI_SDA	IPMI_SCL

*/
  mask = (1 << RS232_RCV) | (1 << RS232_XMT) | (1 << _EEP_CS) | (1 << EEP_MISO) | (1 << EEP_MOSI) | (1 << EEP_SCK) |
    (1 << ADC_IN0) | (1 << ADC_IN1) | (1 << ADC_IN2) | (1 << ADC_IN3) | (1 << ADC_IN4) | (1 << ADC_IN5) | (1 << ADC_IN6) | (1 << ADC_IN7) |
    (1 << IPMI_SDA) | (1 << IPMI_SCL);

  GPIO.port[0].pmr0 = 0;
  GPIO.port[0].pmr1 = 0;
  GPIO.port[0].gper = 0xffffffff;
  GPIO.port[0].oder = 0;
  GPIO.port[0].gperc = mask;			// 0 for those bits with peripherals enabled

  // set weak pullups on EEP_MISO and EEP_MOSI
  GPIO.port[0].puers = (1 << EEP_MISO) | (1 << EEP_MOSI);

  GPIO.port[1].gper = 0xffffffff;
  GPIO.port[1].oder = 0;                        // all output drivers disabled for port B
	GPIO.port[1].puerc = ((1<<(PWRGOOD-32)) | (1<<(FPGA_DONE-32)));
	GPIO.port[1].oderc = ((1<<(PWRGOOD-32)) | (1<<(FPGA_DONE-32)));
  
  // All FPGA-connected outputs to logic 0 at start (to minimize sneak paths into FPGA logic)
  gpio_set_gpio_pin(_IPMI_BLLED);	
  gpio_clr_gpio_pin(UC_PWRENA2); 
  gpio_clr_gpio_pin(_FSIO_SCANSLV);  
  gpio_clr_gpio_pin(_FSIO_SS0);
  gpio_clr_gpio_pin(_FSIO_SS1);
  gpio_clr_gpio_pin(_FSIO_SS2);
  gpio_clr_gpio_pin(FSIO_SCK);
  gpio_clr_gpio_pin(FSIO_MOSI);
  gpio_clr_gpio_pin(FSIO_MISO);

  // check reset source to update the reset counter
  TIMESTATREC.lastrsttime = 0;
  if (AVR32_PM.RCAUSE.wdt)
    // bump reset counter
    TIMESTATREC.reset_cnt++;
  else {
    // reset everything
    TIMESTATREC.reset_cnt = 0;
    TIMESTATREC.uptime = 0;
    TIMESTATREC.hottime = 0;
  }
  
}


void gpio_set_gpio_pin(unsigned int pin)
{
  volatile avr32_gpio_port_t *gpio_port = &GPIO.port[pin >> 5];

  gpio_port->ovrs  = 1 << (pin & 0x1F); // Value to be driven on the I/O line: 1.
  gpio_port->oders = 1 << (pin & 0x1F); // The GPIO output driver is enabled for that pin.
  gpio_port->gpers = 1 << (pin & 0x1F); // The GPIO module controls that pin.
}


void gpio_clr_gpio_pin(unsigned int pin)
{
  volatile avr32_gpio_port_t *gpio_port = &GPIO.port[pin >> 5];

  gpio_port->ovrc  = 1 << (pin & 0x1F); // Value to be driven on the I/O line: 0.
  gpio_port->oders = 1 << (pin & 0x1F); // The GPIO output driver is enabled for that pin.
  gpio_port->gpers = 1 << (pin & 0x1F); // The GPIO module controls that pin.
}


void gpio_tgl_gpio_pin(unsigned int pin)
{
  volatile avr32_gpio_port_t *gpio_port = &GPIO.port[pin >> 5];

  gpio_port->ovrt  = 1 << (pin & 0x1F); // Toggle the I/O line.
  gpio_port->oders = 1 << (pin & 0x1F); // The GPIO output driver is enabled for that pin.
  gpio_port->gpers = 1 << (pin & 0x1F); // The GPIO module controls that pin.
}


int gpio_get_pin_value(unsigned int pin)
{
  volatile avr32_gpio_port_t *gpio_port = &GPIO.port[pin >> 5];
  return (gpio_port->pvr >> (pin & 0x1F)) & 1;
}


void gpio_disable_gpio_pin_output(unsigned int pin) {
  // turns off the output driver for a pin.  useful for pseudo-open-drain applications
  volatile avr32_gpio_port_t *gpio_port = &GPIO.port[pin >> 5];
  gpio_port->oderc = 1 << (pin & 0x1F);
}


int gpio_get_pin_oe_setting(unsigned int pin) {
  volatile avr32_gpio_port_t *gpio_port = &GPIO.port[pin >> 5];
  return (gpio_port->oder >> (pin & 0x1F)) & 1;
}


