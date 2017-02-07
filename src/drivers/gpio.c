/*
 * gpio.c
 *
 *  Created on: Oct 5, 2010
 *      Author: tgorski
 */


#include "pins.h"
#include "gpio.h"
#include "resetcfg.h"
#include "utils.h"
#include "pwrmgr.h"

void gpio_scanslv_mode(void) {
  // set pins in mode for scanning for FPGA SPI slaves
  unsigned long mask;
  gpio_set_gpio_pin(_FSIO_SCANSLV);             // turn off scan slave pin
  mask = (1 <<_FSIO_SS0) | (1<< FSIO_SCK) | (1 << FSIO_MOSI) | (1 << FSIO_MISO);

  GPIO.port[0].oderc = mask;          // turn off outputs
  GPIO.port[0].gpers = mask;          // set to GPIO mode
  gpio_clr_gpio_pin(_FSIO_SCANSLV);   // activate scan slave pin
}


void gpio_spi1_mode(void) {
  // set pins in mode for SPI1 operation
  unsigned long mask;
  gpio_set_gpio_pin(_FSIO_SCANSLV);             // turn off scan slave pin
  mask = (1 <<_FSIO_SS0) | (1<< FSIO_SCK) | (1 << FSIO_MOSI) | (1 << FSIO_MISO);
  GPIO.port[0].pmr0s = mask;        // set to mux code 01
  GPIO.port[1].pmr1c = mask;        // set to mux code 01
  GPIO.port[0].gperc = mask;        // set in peripheral mode
  GPIO.port[0].puerc = mask;        // turn off weak pullups
  GPIO.port[0].puers = (1 << FSIO_MISO);      // except for MISO
}

inline int gpio_get_spi1_mode(void) {
	// checks the current pin state for SPI1 port, returns 1 if the port is in
  // spi mode, and 0 if it is in GPIO mode.
  unsigned long mask = (1 <<_FSIO_SS0) | (1<< FSIO_SCK) | (1 << FSIO_MOSI) | (1 << FSIO_MISO);
  unsigned long gperval = GPIO.port[0].gper;
  if ((gperval & mask) == 0x00000000)
    // all bits cleared, therefore all bits in peripheral mode
    return 1;
  return 0;
}

void gpio_spi1_bitbang(void) {
	// puts spi1 in bitbang mode (all GPIOs, all outputs except for MISO)
  unsigned long mask;
  gpio_set_gpio_pin(_FSIO_SCANSLV);             // turn off scan slave pin
  mask = (1 <<_FSIO_SS0) | (1<< FSIO_SCK) | (1 << FSIO_MOSI);
  GPIO.port[0].ovrs = mask;
  GPIO.port[0].oders = mask;
  GPIO.port[0].oderc = (1 << FSIO_MISO);            // turn off MISO driver
  GPIO.port[0].puers = (1 << FSIO_MISO);            // set weak pullup for MISO
	GPIO.port[0].gpers = mask |(1 << FSIO_MISO);      // switch all SPI1 lines to GPIO
}


void gpio_init() {
/* Initialize GPIO pins for their various functions.

The following pins use peripheral mux function A on Port A (0)
	RS232_RCV	RS232_XMT	_EEP_CS		EEP_MISO	EEP_MOSI	EEP_SCK
	ADC_IN0		ADC_IN1		ADC_IN2		ADC_IN3		ADC_IN4		ADC_IN5		ADC_IN6		ADC_IN7
	IPMI_SDA	IPMI_SCL

*/
  unsigned long mask;
  unsigned long snapshot;

  snapshot = AVR32_PM.gplp[0];

  if (AVR32_PM.gplp[0] & RSTCFG_PAYLOAD_HOT_MASK) {
    // starting up into hot payload power
    // have to get the power enables and the aux power I/O pins set to proper value right away

    // set aux power enable pins
    if (AVR32_PM.gplp[0] & RSTCFG_AUX_PWR_IN_ENABLED_MASK) {
      // auxiliary (front panel) input enabled for bus A.  
      GPIO.port[1].ovrs = (1 << (AUX12V_SELECT-32));
      GPIO.port[1].oders = (1 << (AUX12V_SELECT-32)) | (1 << (BP12V_SELECT-32));
    }
    else {
      // backplane input enabled for bus A
      GPIO.port[1].ovrs = (1 << (BP12V_SELECT-32));
      GPIO.port[1].oders = (1 << (AUX12V_SELECT-32)) | (1 << (BP12V_SELECT-32));
    }

    // set power enables
    if (AVR32_PM.gplp[0] & RSTCFG_BACKEND_HOT_MASK) {
      // get power enables active
      GPIO.port[1].ovrs = (1 << (UC_PWRENA1-32)) | (1 << (UC_PWRENA2-32)) | (1 << (UC_PWRENA3-32)) | (1 << (UC_PWRENA4-32));
      GPIO.port[1].oders = (1 << (UC_PWRENA1-32)) | (1 << (UC_PWRENA2-32)) | (1 << (UC_PWRENA3-32)) | (1 << (UC_PWRENA4-32));

      // set MGT trim values per pre-reset values
      // first set all pins to inputs (undriven) this should be post-reset default, but be safe
      GPIO.port[0].oderc = (1 << MGT1P2L_MSEL) | (1 << MGT1P2L_MTOL);
      GPIO.port[1].oderc = ((1 << (MGT1P2R_MTOL-32)) | (1 << (MGT1P2R_MSEL-32)) | (1 << (MGT1P0L_VSET-32)) | (1 << (MGT1P0L_MTOL-32)) | (1 << (MGT1P0L_MSEL-32)) | (1 << (MGT1P0R_VSET-32)) | (1 < (MGT1P0R_MSEL-32)) |
        (1 << (MGT1P0R_MTOL-32)));

      // set 1.0V voltage setpoints
      // 1.0V LEFT SETPOINT
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P0L_VSET_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P0L_VSET_BIT0)
          GPIO.port[1].ovrs = (1 << (MGT1P0L_VSET-32));
        GPIO.port[1].oders = (1 << (MGT1P0L_VSET-32));
      }
      // 1.0V RIGHT SETPOINT
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P0R_VSET_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P0R_VSET_BIT0)
          GPIO.port[1].ovrs = (1 << (MGT1P0R_VSET-32));
        GPIO.port[1].oders = (1 << (MGT1P0R_VSET-32));
      }
      // set 1.0V voltage trim points
      // 1.0V LEFT MTOL
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P0L_MTOL_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P0L_MTOL_BIT0)
          GPIO.port[1].ovrs = (1 << (MGT1P0L_MTOL-32));
        GPIO.port[1].oders = (1 << (MGT1P0L_MTOL-32));
      }
      // 1.0V RIGHT MTOL
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P0R_MTOL_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P0R_MTOL_BIT0)
          GPIO.port[1].ovrs = (1 << (MGT1P0R_MTOL-32));
        GPIO.port[1].oders = (1 << (MGT1P0R_MTOL-32));
      }
      // 1.0V LEFT MSEL
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P0L_MSEL_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P0L_MSEL_BIT0)
          GPIO.port[1].ovrs = (1 << (MGT1P0L_MSEL-32));
        GPIO.port[1].oders = (1 << (MGT1P0L_MSEL-32));
      }
      // 1.0V RIGHT MSEL
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P0R_MSEL_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P0R_MSEL_BIT0)
          GPIO.port[1].ovrs = (1 << (MGT1P0R_MSEL-32));
        GPIO.port[1].oders = (1 << (MGT1P0R_MSEL-32));
      }
      // set 1.2V trim points
      // 1.2V LEFT MTOL
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P2L_MTOL_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P2L_MTOL_BIT0)
          GPIO.port[0].ovrs = (1 << MGT1P2L_MTOL);
        GPIO.port[0].oders = (1 << MGT1P2L_MTOL);
      }
      // 1.2V RIGHT MTOL
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P2R_MTOL_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P2R_MTOL_BIT0)
          GPIO.port[1].ovrs = (1 << (MGT1P2R_MTOL-32));
        GPIO.port[1].oders = (1 << (MGT1P2R_MTOL-32));
      }
      // 1.2V LEFT MSEL
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P2L_MSEL_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P2L_MSEL_BIT0)
          GPIO.port[0].ovrs = (1 << MGT1P2L_MSEL);
        GPIO.port[0].oders = (1 << MGT1P2L_MSEL);
      }
      // 1.2V RIGHT MTOL
      if (!(AVR32_PM.gplp[0] & RSTCFG_1P2R_MSEL_BIT1)) {
        // set bit
        if (AVR32_PM.gplp[0] & RSTCFG_1P2R_MSEL_BIT0)
          GPIO.port[1].ovrs = (1 << (MGT1P2R_MSEL-32));
        GPIO.port[1].oders = (1 << (MGT1P2R_MSEL-32));
      }

    }
    else {
      // back end not hot, can init rest of the way as if a cold init
      mask = ((1 << (UC_PWRENA1-32)) | (1 << (UC_PWRENA2-32)) | (1 << (UC_PWRENA3-32)) | (1 << (UC_PWRENA4-32)) | 
        (1 << (MGT1P2R_MTOL-32)) | (1 << (MGT1P2R_MSEL-32)) | (1 << (MGT1P0L_VSET-32)) | (1 << (MGT1P0L_MTOL-32)) | (1 << (MGT1P0L_MSEL-32)) | (1 << (MGT1P0R_VSET-32)) | (1 < (MGT1P0R_MSEL-32)) |
        (1 << (MGT1P0R_MTOL-32)));
      GPIO.port[1].gpers = mask;            // set as GPIO pins
      GPIO.port[1].oderc = mask;            // turn off output drivers
      GPIO.port[1].puerc = mask;            // disable pullups
      // set the enables to outputs, driving logic 0s
      gpio_clr_gpio_pin(UC_PWRENA1);
      gpio_clr_gpio_pin(UC_PWRENA2);
      gpio_clr_gpio_pin(UC_PWRENA3);
      gpio_clr_gpio_pin(UC_PWRENA4);

      gpio_clr_gpio_pin(_FSIO_SCANSLV);
      gpio_clr_gpio_pin(_FPGA_CPU_RST);
      gpio_clr_gpio_pin(_FPGA_PROGB);
      gpio_clr_gpio_pin(_FPGA_DONE);
      gpio_clr_gpio_pin(_FPGA_REQ_IN);
      gpio_clr_gpio_pin(_FSIO_SS0);
      gpio_clr_gpio_pin(FSIO_SCK);
      gpio_clr_gpio_pin(FSIO_MOSI);
      gpio_clr_gpio_pin(FSIO_MISO);

    }
  }
  else {
    // starting up into cold payload power
    mask = (1 << MGT1P2L_MSEL) | (1 << MGT1P2L_MTOL);
    GPIO.port[0].gpers = mask;            // set as GPIO pins
    GPIO.port[0].oderc = mask;            // turn off output drivers  (let float to midpoint)
    GPIO.port[0].puerc = mask;            // disable pullups

    mask = ((1 << (UC_PWRENA1-32)) | (1 << (UC_PWRENA2-32)) | (1 << (UC_PWRENA3-32)) | (1 << (UC_PWRENA4-32)) | (1 << (AUX12V_SELECT-32)) | (1 << (BP12V_SELECT-32)) |
      (1 << (MGT1P2R_MTOL-32)) | (1 << (MGT1P2R_MSEL-32)) | (1 << (MGT1P0L_VSET-32)) | (1 << (MGT1P0L_MTOL-32)) | (1 << (MGT1P0L_MSEL-32)) | (1 << (MGT1P0R_VSET-32)) | (1 < (MGT1P0R_MSEL-32)) |
      (1 << (MGT1P0R_MTOL-32)));
    GPIO.port[1].gpers = mask;            // set as GPIO pins
    GPIO.port[1].oderc = mask;            // turn off output drivers
    GPIO.port[1].puerc = mask;            // disable pullups
    // set the enables to outputs, driving logic 0s
    gpio_clr_gpio_pin(UC_PWRENA1);
    gpio_clr_gpio_pin(UC_PWRENA2);
    gpio_clr_gpio_pin(UC_PWRENA3);
    gpio_clr_gpio_pin(UC_PWRENA4);

    // clear both enables to VBUSA, leaving it cut off at startup
    gpio_clr_gpio_pin(AUX12V_SELECT);  
    gpio_clr_gpio_pin(BP12V_SELECT);

    // zero out payload-connected IOs, to eliminate sneak paths
    gpio_clr_gpio_pin(_FSIO_SCANSLV);
    gpio_clr_gpio_pin(_FPGA_CPU_RST);
    gpio_clr_gpio_pin(_FPGA_PROGB);
    gpio_clr_gpio_pin(_FPGA_DONE);
    gpio_clr_gpio_pin(_FPGA_REQ_IN);
    gpio_clr_gpio_pin(_FSIO_SS0);
    gpio_clr_gpio_pin(FSIO_SCK);
    gpio_clr_gpio_pin(FSIO_MOSI);
    gpio_clr_gpio_pin(FSIO_MISO);
  }

  // do rest of GPIO pin initialization that doesn't depend on power state coming out of reset
  mask = (1 << RS232_RCV) | (1 << RS232_XMT) | (1 << _EEP_CS) | (1 << EEP_MISO) | (1 << EEP_MOSI) | (1 << EEP_SCK) |
    (1 << ADC_IN0) | (1 << ADC_IN1) | (1 << ADC_IN2) | (1 << ADC_IN3) | (1 << ADC_IN4) | (1 << ADC_IN5) | (1 << ADC_IN6) | (1 << ADC_IN7) |
    (1 << IPMI_SDA) | (1 << IPMI_SCL);

  GPIO.port[0].pmr0 = 0;
  GPIO.port[0].pmr1 = 0;
  GPIO.port[0].gper = 0xffffffff;
  GPIO.port[0].oderc = 0xfff3ffff;      // disable output of all port 0 pins except the 1.2VL trim pins
  GPIO.port[0].gperc = mask;			      // 0 for those bits with peripherals enabled

  // set weak pullups on EEP_MISO and EEP_MOSI
  GPIO.port[0].puers = (1 << EEP_MISO) | (1 << EEP_MOSI);


  mask = ((1 << (PWRGOOD-32)) | (1 << (_FPGA_DONE-32)) | (1 << (_ADC_SS0-32)) | (1 << (ADC_SCK-32)) | (1 << (ADC_MISO-32)) | (1 << (ADC_MOSI-32)) |
    (1 << (_FSIO_SCANSLV-32)) | (1 << (_GP_LED0-32)) | (1 << (_IPMI_LED1Y-32)) | (1 << (_GP_LED2-32)) | (1 << (_FPGA_CPU_RST-32)) | (1 << (_FPGA_PROGB-32)) | (1 < (_FPGA_REQ_IN-32)));
  GPIO.port[1].gpers = mask;            // set as GPIO pins
  GPIO.port[1].oderc = mask;            // turn off output drivers
  GPIO.port[1].puerc = mask;            // disable pullups

  // set initial states on pins not associated with power control or hard peripherals
  gpio_set_gpio_pin(_IPMI_LED1R);        // turn off red IPMI LED1
  gpio_set_gpio_pin(_IPMI_LED1Y);        // turn off yellow IPMI LED1
  gpio_set_gpio_pin(_IPMI_LED2);         // turn off green IPMI LED2
  gpio_set_gpio_pin(_IPMI_BLLED);        // turn off blue IPMI LED
  gpio_set_gpio_pin(_ADC_SS0);
  gpio_clr_gpio_pin(ADC_SCK);
  gpio_clr_gpio_pin(ADC_MOSI);

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


