/*
 * mmcmain.c
 *
 *  Created on: Oct 1, 2010
 *      Author: tgorski
 */

#include <stdio.h>
#include <string.h>
#include "mmc_version.h"
#include "pins.h"
#include "utils.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "sio_usart.h"
#include "intc.h"
#include "twidriver.h"
#include "timer_callback.h"
#include "adc.h"
#include "rtc.h"
#include "eepspi.h"
#include "swevent.h"
#include "ejecthandle.h"
#include "ipmb_svc.h"
#include "LEDdrivers.h"
#include "nonvolatile.h"
#include "ipmi_cmd_parser.h"
#include "fault_log.h"
#include "sensor_svc.h"
#include "payload_mgr.h"
#include "fpga_autoconfig.h"
#include "console_parser.h"


int main(void) {
	
  // low level driver initialization
  Disable_Watchdog();
  Disable_global_interrupt();
  gpio_init();							// set external pin assignments
#ifdef TRACE
  memcpy((void*) RDOUTBUFBASEADDR, (void*) CAPBUFBASEADDR, sizeof(capture_buf_rec_t));        // copy from trace buffer to readout buffer
  set_cap_buf_ena();
#endif
  INTC_init_interrupts();
  osc_init();							  // set up oscillators and PLL
  timer_callback_init();
  sio_init();
  twi_init();
  rtc_init();
  eepspi_init();
  eject_handle_init();
  LED_init();
  Enable_global_interrupt();

  sio_putstr("\r\n\nBoston University AMC13XG Module\n");
  sprintf(spbuf, "MMC Version %s\n", MMC_VERSION_STRING);
  sio_putstr(spbuf);
  sprintf(spbuf, "uTCA Slot=%i  IPMB-L ADDR=%02Xh\n", twi_state.slotid, twi_state.ipmbl_addr);
  sio_putstr(spbuf);

  // service initialization--order of initializations is important!
  nonvol_init();
  swevent_init();						     // software event service
  adc_init();
  sensor_svc_init();
  ipmb_init();							     // ipmb message service
  ipmi_cmd_parser_init();				 // register command parser service events
  pyldmgr_init();                // payload manager init
  fault_log_init();
  start_timers();						     // start timers
  twi_start_slave_listen();			 // put two-wire-interface in listening mode
  fpga_autoconfig_init();        // fpga autoconfiguration service
  program_LED(LED0_TBL_IDX, Local_Control, &LED_1Hz_Blink_Activity);
  Enable_Watchdog();
  
  // main service loop
  while (1) {
    ipmb_service();
    sensor_service();
    pyldmgr_service();
	  console_chk_cmd();
    Service_Watchdog();
  }
}




