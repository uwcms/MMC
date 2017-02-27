/*
 * payload_mgr.c
 *
 *  Created on: Nov 5, 2010
 *      Author: tgorski
 */

#include <compiler.h>
#include <stdio.h>
#include <string.h>
#include "pins.h"
#include "resetcfg.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "utils.h"
#include "CTP7_SPI_addrs.h"
#include "sio_usart.h"
#include "twidriver.h"
#include "ejecthandle.h"
#include "timer_callback.h"
#include "LEDdrivers.h"
#include "swevent.h"
#include "spi1.h"
#include "eepspi.h"
#include "nonvolatile.h"
#include "rtc.h"
#include "ipmb_svc.h"
#include "sensor_svc.h"
#include "ipmi_cmd_parser.h"
#include "payload_mgr.h"
#include "fault_log.h"
#include "fpgaspi.h"


#define COLD_RESET_TICKS              (30)                  // 100ms ticks for cold reset
#define WARM_RESET_TICKS              (10)                  // 100ms ticks for reset pulse
#define FPGA_LOAD_TICKS               (10)                  // 100ms ticks for FPGA load command
#define QUIESCE_MAX_DELAY_TICKS       (180)                 // maximum secondary delay from handle pull to forced quiesced state, 100ms ticks
#define QUIESCE_HOLDOFF_DELAY_TICKS   (20)                  // minimum holdoff from handle pull to reporting quiesced hotswap event, 100ms ticks
#define MONITOR_STARTUP_TICKS         (10)                  // 1 second delay for non-recoverable value monitoring
#define AUTOCONFIG_POLL_TICKS         (15)                  // 1.5 second delay for checking autoconfig port
#define ONESEC_UTIL_TIMER_TICKS       (10)                  // 1 second utility timer ticks

#define PAYLDMGR_PWRON_ISR_DETECT               (1<<0)
#define PAYLDMGR_PWROFF_ISR_DETECT              (1<<1)
#define PAYLDMGR_BKEND_TIMER_EXP_ISR_DETECT     (1<<2)
#define PAYLDMGR_AUTOCONFIG_TIMER_FLAG          (1<<3)
#define PAYLDMGR_BKEND_SEQUENCE_COMPLETE        (1<<4)
#define PAYLDMGR_QUIESCE_TIMER_TICK             (1<<5)
#define PAYLDMGR_WAITAUX_ISR_DETECT             (1<<6)
#define PAYLDMGR_1SEC_UTIL_ISR_DETECT           (1<<7)

#define QUIESCE_SIGNAL_VAL                      (0x01)
#define QUIESCE_CLEAR_VAL                       (0x00)


int payload_power_off_callback(event eventID, void* arg);
int payload_power_on_callback(event eventID, void* arg);


int pyldmgr_100ms_timer_callback(event eventID, void* arg);
int pyldmgr_1sec_timer_callback(event eventID, void* arg);
int fast_output_timer_callback(event eventID, void* arg);
void check_alarm_status(void);
inline void set_MMC_pin_backend_off_state(void);
inline void set_MMC_pin_backend_on_state(void);

void generate_lamp_test_request(void);
int lamp_test_response_callback(void* preq, void* prsp);
void quiesce_monitor(void);

const LED_activity_desc_t LED_Noncritical_Activity = {Blink, LEDOFF, 120, 30};     // 1.5 sec period, on 20% of time
const LED_activity_desc_t LED_Critical_Activity = {Blink, LEDON, 20, 20};     // 0.4 sec period, on 50% of time
  
const physical_power_ena_pin_map_t PWR_ENA_pin_map = {UC_PWRENA1, UC_PWRENA2, UC_PWRENA3, UC_PWRENA4, PWR_ENA_PIN_UNDEFINED, PWR_ENA_PIN_UNDEFINED};

//int autoconfig_timer_flag;
unsigned long isr_event_mask;                     // used to relay events out of the ISR timespace to be processed by pyldmgr_service()
payload_mgr_state_t pyldmgr_state;

unsigned long pwrena2pinmask(unsigned char pwrena_mask);

void pyldmgr_init(void) {
	int i1;

  if (sizeof(nonvolatile_settings_t) > PAYLDMGR_AREA_SIZE) {
    // this is serious error, in that insufficient space has been allocated in nonvolatile memory for
    // the payload manager's record
    sio_putstr("?Error, insufficient memory allocated for Payload Manager nonvolatile storage\n");
    return;
  }

  memset((void *) &pyldmgr_state, 0, sizeof(pyldmgr_state));            // zero out state record

  // get default settings from EEPROM
  while (eepspi_chk_write_in_progress());     // wait for EEPROM to be available
  eepspi_read((unsigned char*) &pyldmgr_state.settings, PAYLDMGR_AREA_BYTE_OFFSET, sizeof(nonvolatile_settings_t));
  if ((pyldmgr_state.settings.sensor_mask_level[0] == 0xff) || (pyldmgr_state.settings.bkend_pwr_cfg.defined_stage_cnt < 1) || (pyldmgr_state.settings.bkend_pwr_cfg.defined_stage_cnt > MAX_PWR_ENA_STAGE_CNT)) {
	  // EEPROM unformatted, need to format with hard-coded defaults
    sio_putstr("Uninitialized or corrupt Payload Manager default area detected.  Writing default image....\n");
    pyldmgr_state.settings.bkend_pwr_ena = 1;
	  pyldmgr_state.settings.bkend_shtdn_evt_ena = 0;               // no hotswap event on backend shutdown
	  pyldmgr_state.settings.autocfg_ena = 0;                       // no auto config from EEPROM
	  pyldmgr_state.settings.global_mask_level = status_OK;         // minimum alarm level
    pyldmgr_state.settings.bkend_pwr_cfg.defined_stage_cnt = 4;
	  pyldmgr_state.settings.bkend_pwr_cfg.stagedelay_ms[0] = 20;
	  pyldmgr_state.settings.bkend_pwr_cfg.stagedelay_ms[1] = 20;
	  pyldmgr_state.settings.bkend_pwr_cfg.stagedelay_ms[2] = 20;
	  pyldmgr_state.settings.bkend_pwr_cfg.enamask[0] = 1 << PWR_ENA_MIN_PIN_OFFSET;      // first stage
	  pyldmgr_state.settings.bkend_pwr_cfg.enamask[1] = 1 << (PWR_ENA_MIN_PIN_OFFSET+1);   // second stage
	  pyldmgr_state.settings.bkend_pwr_cfg.enamask[2] = 1 << (PWR_ENA_MIN_PIN_OFFSET+2);   // third stage
	  pyldmgr_state.settings.bkend_pwr_cfg.enamask[3] = 1 << (PWR_ENA_MIN_PIN_OFFSET+3);   // fourth stage

	  memset((void*) &pyldmgr_state.settings.sensor_mask_level[0], (unsigned char) status_OK, MAX_SENSOR_CNT);                 
    eepspi_write((unsigned char*) &pyldmgr_state.settings, PAYLDMGR_AREA_BYTE_OFFSET, sizeof(nonvolatile_settings_t));
  }
  
  if (AVR32_PM.RCAUSE.wdt)
    sio_putstr("Startup from watchdog timer\n");

  // initialize alarm states
  pyldmgr_state.alarm.cur_alarm_level = status_OK;
  pyldmgr_state.alarm.cur_alarm_source = none;
  
  // initialize state variables
  memset((void*) &pyldmgr_state.ctl, 0, sizeof(state_record_t));
  // convert the logical power enable masks for each stage into a GPIO pin mask
  pyldmgr_state.ctl.pwrena_mask[0] = pwrena2pinmask(pyldmgr_state.settings.bkend_pwr_cfg.enamask[0]);
  i1 = 1;
  while (i1<pyldmgr_state.settings.bkend_pwr_cfg.defined_stage_cnt) {
	  pyldmgr_state.ctl.pwrena_mask[i1] = pyldmgr_state.ctl.pwrena_mask[i1-1] | pwrena2pinmask(pyldmgr_state.settings.bkend_pwr_cfg.enamask[i1]);
	  i1++;
  }

  // initialize on/off comparator thresholds
  pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].lower_thr = SensorData[PAYLOAD_12V_SENSOR].pSDR->nominal_reading/5;
  pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].upper_thr = SensorData[PAYLOAD_12V_SENSOR].pSDR->lower_critical_thr;

  // initialize callbacks
  register_timer_callback(pyldmgr_100ms_timer_callback, TEVENT_100MSEC_4);
  register_timer_callback(fast_output_timer_callback, TEVENT_200USEC);
  register_timer_callback(pyldmgr_1sec_timer_callback, TEVENT_1SEC);

  register_swevent_callback(payload_power_on_callback, PYLDMGREV_PAYLD_PWR_ON_DETECT);
  register_swevent_callback(payload_power_off_callback, PYLDMGREV_PAYLD_PWR_OFF_DETECT);
  isr_event_mask = 0;

  // check if initializing into a hot configuration, if so, adjust state variables to reflect hot state
  if (AVR32_PM.gplp[0] & RSTCFG_PAYLOAD_HOT_MASK) {
    // initialize state variables for the payload power detect state machine to support startup into hot configuration
    sio_putstr("Startup into hot payload power\n");
    pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].cur_on_state = 1;
    pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].prev_on_state = 1;
    SensorData[PAYLOAD_12V_SENSOR].readout_value = SensorData[PAYLOAD_12V_SENSOR].pSDR->nominal_reading;        // see ADC buffer with nominal readout value

    pyldmgr_state.payld_pwr_cur_state = power_on;

    if (AVR32_PM.gplp[0] & RSTCFG_BACKEND_HOT_MASK) {
      // set back end state machine to hot state--power up sequence complete, but with no events that generate callbacks
      sio_putstr("Startup into hot back end\n");
			pyldmgr_state.ctl.backend_cur_stage = 0xff;

    }
  }
  else {
    // payload power was not hot, set power select pins per the current settings
    program_LED(IPMI_BLUELED_TBL_IDX, Local_Control, &LED_On_Activity);

    // initialize comparators to off state
    pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].cur_on_state = 0;
    pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].prev_on_state = 0;
    pyldmgr_state.payld_pwr[PWRCOMP_AUX_12V].cur_on_state = 0;
    pyldmgr_state.payld_pwr[PWRCOMP_AUX_12V].prev_on_state = 0;
  }
} 
   

__inline__ power_status_t pyldmgr_get_payload_power_status(void) {
  // returns current state of payload power.  The payload timer is activated when the power-on transition occurs.  It
  // holds off the return of a power-off condition until the timer has expired
  if (pyldmgr_state.ctl.payload_startup_timer)
    return power_off;
	else
    return pyldmgr_state.payld_pwr_cur_state;
}


power_status_t pyldmgr_get_backend_power_status(void) {
  // returns definitive on/off status of backend power, which depends on payload power status, back end power
  // enable pin state and backend power startup timer
  if ((pyldmgr_state.ctl.backend_cur_stage != 0xff) || (pyldmgr_state.ctl.backend_startup_timer))
    return power_off;
	return power_on;
}


power_status_t pyldmgr_get_backend_power_pin_status(void) {
	// returns the definitive on/off status of the back end power pins
	unsigned long portB;
	unsigned long finalmask;
	
	finalmask = pyldmgr_state.ctl.pwrena_mask[pyldmgr_state.settings.bkend_pwr_cfg.defined_stage_cnt-1];         // mask of Port B bits that should be on in final stage of power enable
	portB = GPIO.port[1].ovr;         // read port B
	if ((portB & finalmask) == finalmask)
	  return power_on;
	else
	  return power_off;
}


void pyldmgr_service(void) {
  CAPBUF1_SAVEPC;
  // check for events from ISRs
  if (isr_event_mask & PAYLDMGR_PWRON_ISR_DETECT) {
    // payload power on detected in ISR
    Disable_global_interrupt();
    isr_event_mask &= ~PAYLDMGR_PWRON_ISR_DETECT;
    Enable_global_interrupt();
    sio_filt_putstr(TXTFILT_INFO, 1, "Payload Power On detected\n");
	  // initialize the startup timer
    pyldmgr_state.ctl.payload_startup_timer = MONITOR_STARTUP_TICKS;
    post_swevent(PYLDMGREV_PAYLD_PWR_ON_DETECT, NULL);
  }

  if (isr_event_mask & PAYLDMGR_PWROFF_ISR_DETECT) {
    // payload power off detected in ISR
    Disable_global_interrupt();
    isr_event_mask &= ~PAYLDMGR_PWROFF_ISR_DETECT;
    Enable_global_interrupt();
    sio_filt_putstr(TXTFILT_INFO, 1, "Payload Power Off detected\n");
    post_swevent(PYLDMGREV_PAYLD_PWR_OFF_DETECT, NULL);
  }

  if (isr_event_mask & PAYLDMGR_BKEND_TIMER_EXP_ISR_DETECT) {
    // backend startup timer expired
    Disable_global_interrupt();
    isr_event_mask &= ~PAYLDMGR_BKEND_TIMER_EXP_ISR_DETECT;
    Enable_global_interrupt();
    sio_filt_putstr(TXTFILT_INFO, 1, "Issuing FPGA Load command\n");
	  //program_LED(LED2_TBL_IDX, Local_Control, &LED_On_Activity);       // turn on LED2 - FPGA load indicator
  }

	if (isr_event_mask & PAYLDMGR_BKEND_SEQUENCE_COMPLETE) {
		// back end sequence complete--set back end interface pins to correct state
    Disable_global_interrupt();
    isr_event_mask &= ~PAYLDMGR_BKEND_SEQUENCE_COMPLETE;
    Enable_global_interrupt();
	  sio_filt_putstr(TXTFILT_INFO, 1, "Back End Power-On Sequence Complete\n");
    set_MMC_pin_backend_on_state();
    // apply current MGT voltage setpoint and trim settings
    post_swevent(PYLDMGREV_BACKEND_PWR_ON_DETECT, NULL);
	}

  if (isr_event_mask & PAYLDMGR_WAITAUX_ISR_DETECT) {
    //backplane payload power came hot but aux power isn't there--signal rapid blink on blue LED as local function
    // but first wait for carrier manager to turn off the blue LED before enabling the rapid blink
    if (LEDstate[IPMI_BLUELED_TBL_IDX].pOvrideDesc == &LED_Off_Activity) {
      Disable_global_interrupt();
      isr_event_mask &= ~PAYLDMGR_WAITAUX_ISR_DETECT;
      Enable_global_interrupt();
      sio_filt_putstr(TXTFILT_INFO, 1, "Waiting on Auxiliary +12V Power\n");
      program_LED(IPMI_BLUELED_TBL_IDX, Local_Control, &LED_3Hz_Blink_Activity);
    }
  }
	
	if (isr_event_mask & PAYLDMGR_QUIESCE_TIMER_TICK) {
		// quiesce timer expired
    Disable_global_interrupt();
    isr_event_mask &= ~PAYLDMGR_QUIESCE_TIMER_TICK;
    Enable_global_interrupt();
    quiesce_monitor();
	}		

  // check one second utility timer
  if (isr_event_mask & PAYLDMGR_1SEC_UTIL_ISR_DETECT) {
    Disable_global_interrupt();
    isr_event_mask &= ~PAYLDMGR_1SEC_UTIL_ISR_DETECT;
    Enable_global_interrupt();
    if (pyldmgr_get_backend_power_status() == power_off)
      // turn off yellow LED.  This is to clear up condition where fault occurred during Linux boot, back end power
      // disabled before Linux can turn off the LED through the SPI path.
      program_LED(IPMI_LED1Y_TBL_IDX, Local_Control, &LED_Off_Activity);
  }
	
  // check/update status of alarms
  check_alarm_status();
 
  // check to see if fpga auto config service needs to be run
  if (pyldmgr_get_backend_power_status() == power_on) {
    if (isr_event_mask & PAYLDMGR_AUTOCONFIG_TIMER_FLAG) {
      Disable_global_interrupt();
      isr_event_mask &= ~PAYLDMGR_AUTOCONFIG_TIMER_FLAG;
      Enable_global_interrupt();
      post_swevent(PYLDMGREV_FPGA_AUTOCONFIG_SVC, NULL);                      // post autoconfig event
    }
    if (!pyldmgr_state.ctl.autoconfig_poll_timer)
      pyldmgr_state.ctl.autoconfig_poll_timer = AUTOCONFIG_POLL_TICKS;
  }
  else {
    // back end power off, no autoconfig poll
    TIMESTATREC.hottime = 0;                                                  // clear hot time counter
    if (pyldmgr_state.ctl.autoconfig_poll_timer) {
      Disable_global_interrupt();
      pyldmgr_state.ctl.autoconfig_poll_timer = 0;
      isr_event_mask &= ~PAYLDMGR_AUTOCONFIG_TIMER_FLAG;
      Enable_global_interrupt();
    }
  }
}


void quiesce_monitor(void) {
  unsigned char spibuf;
  // function is called during payload manager service, on every tick of the quiesce timer (100ms) event produced by the 100ms ISR

  // function holds off quiescing until timer 1 expires, then quiesces if it fails to detect a SPI core in the ZYNQ chip, or sees
  // a 'quiesced' status flag in the SPI memory, set by the ZYNQ.  If this event is not seen by the expiration of timer2 (which starts
  // after timer1 expires), then a quiesce is forced at the end of the timer2 countdown.

  // function checks if timer has expired, sending a quiesced hotswap event if it is, and otherwise
  // checking the SPI interfaced for a quiesced status from the Linux system.  As soon as it sees
  // that status, it sends the hotswap event
  
  if (pyldmgr_state.ctl.quiesce_timer1) {
    Disable_global_interrupt();
    pyldmgr_state.ctl.quiesce_timer1--;
    Enable_global_interrupt();
  }

  if (pyldmgr_state.ctl.quiesce_timer1)
    return;                                   // still not expired

  // timer 1 is done, now check timer2

  if ((!pyldmgr_state.ctl.quiesce_timer2) || (pyldmgr_get_backend_power_status() == power_off) || (pyldmgr_get_payload_power_status() == power_off)) {
    // should not happen that timer 2 is zero when falling into this code
    // payload and/or back end off, say due to non-recoverable fault condition, or aux power not connected, when handle is pulled out
    // first clear the quiesced mask, then set it again, to make sure the quiesced event message gets sent
    update_sensor_discrete_state(HOTSWAP_SENSOR, HOTSWAP_QUIESCED_MASK, 0);  // deassert quiesced state
    update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_QUIESCED_MASK);  // then assert it to force the hotswap event
    Disable_global_interrupt();
    pyldmgr_state.ctl.quiesce_timer2 = 0;       // zero out timer2
    Enable_global_interrupt();
    return;
  }

  Disable_global_interrupt();
  pyldmgr_state.ctl.quiesce_timer2--;
  Enable_global_interrupt();

  if (!pyldmgr_state.ctl.quiesce_timer2) {
    // timer expired, so send hotswap event
    update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_QUIESCED_MASK);
    return;
  }

  // at this point, the only thing that allows this function to fall through without sending a hotswap quiesce event
  // is that the flag byte in the ZYNQ is read to show 
  if (!spi1_lock()) {
    update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_QUIESCED_MASK);
    return;
  }

  if (!(fpgaspi_slave_detect() & FPGA0_SPI_DETECT_MASK)) {
    // FPGA not booted or does not contain interface core
    update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_QUIESCED_MASK);
    spi1_unlock();
    return;
  }

  if (!fpgaspi_data_read(0, &spibuf, PAYLDMGR_MMC_SPI_QUIESCED_FLAG_ADDR, 1)) {
    // read failed
    update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_QUIESCED_MASK);
    spi1_unlock();
    return;
  }

  spi1_unlock();

  if (spibuf == QUIESCE_SIGNAL_VAL)
    // quiesce signaled, okay to send hot swap event to carrier
    update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_QUIESCED_MASK);

  // fall through to bottom without sending hot swap event if Linux not quiesced yet

}


void pyldmgr_ipmicmd_fru_ctrl(const unsigned char ctlcode) {
  unsigned char spibuf;

  // run this function when an AMC 3.10 FRU control command is received.  For this callback,
  // arg points to the IPMB request message, indicating the type of control.  The calling
  // function formats and returns a response.  This function only needs to parse the request
  // and perform the appropriate control
  switch (ctlcode) {
    case FRU_CTLCODE_COLD_RST:   // cold reset
      sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "Cold Reset FRU Command\n");
      if (pyldmgr_get_backend_power_status() == power_on)
        pyldmgr_state.ctl.cold_reset_timer = COLD_RESET_TICKS;            // setting timer will cause power to be disabled while timer counts down
      break;

    case FRU_CTLCODE_WARM_RST:   // warm reset
      sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "Warm Reset FRU Command\n");
      if (pyldmgr_get_backend_power_status() == power_on)
        pyldmgr_state.ctl.warm_reset_timer = WARM_RESET_TICKS;
      break;

    case FRU_CTLCODE_REBOOT:   // graceful reboot
      sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "Graceful Reboot FRU Command\n");
      break;

    case FRU_CTLCODE_QUIESCE:   // quiesce
      sio_filt_putstr(TXTFILT_DBG_SUMMARY, 1, "Quiesce FRU Command\n");
      // attempt to write the quiesce signal to the Linux system
      // if the peripheral is not there or the SPI interface is locked, set a short timeout value
      // if signal is written, then set the long delay ticks
      pyldmgr_state.ctl.quiesce_timer1 = QUIESCE_HOLDOFF_DELAY_TICKS;
      pyldmgr_state.ctl.quiesce_timer2 = QUIESCE_MAX_DELAY_TICKS;

      if (!spi1_lock())
        break;

      if (!(fpgaspi_slave_detect() & FPGA0_SPI_DETECT_MASK)) {
        // FPGA not booted or does not contain interface core
        spi1_unlock();
        break;
      }

      // write the quiesce signal
      spibuf = QUIESCE_SIGNAL_VAL;
      fpgaspi_data_write(0, &spibuf, PAYLDMGR_MMC_SPI_QUIESCE_CMD_FLAG_ADDR, 1);

      spi1_unlock();

	    // experimental code:
      // send Lamp Test command to IPMI LED2 (FP green LED) on the AMC13 card
	  	//generate_lamp_test_request();	
      // idea was to see if it is possible to have AMC13 be told to turn off its FABRIC B drivers to the slot.
      // It is possible, but also unlikely that firmware hooks will be implemented on the AMC13
		
      break;

    case 3:
    default:
      // unknown or unsupported control option
      break;
  }
}


void pyldmgr_ipmicmd_backend_pwr(const ipmb_msg_desc_t* prqmsg) {
  // called when a backend power command is received
  const unsigned char* prqdata = prqmsg->buf + IPMB_RQ_DATA_OFFSET-1;     // request data pointer, offset for 1-base indexing
  switch (prqdata[1] & 0x01) {
    case 0:   // turn off backend power if not already off
      if (pyldmgr_state.settings.bkend_pwr_ena) {
        pyldmgr_state.settings.bkend_pwr_ena = 0;
        // reset timers to zero
        pyldmgr_state.ctl.cold_reset_timer = 0;
        pyldmgr_state.ctl.payload_startup_timer = 0;
        pyldmgr_state.ctl.backend_startup_timer = 0;
        pyldmgr_state.ctl.warm_reset_timer = 0;
        // if configuration calls for it, send a backend power shutdown hotswap event message
        if (pyldmgr_state.settings.bkend_shtdn_evt_ena)
          update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK);
      }
      break;

    case 1:   // turn on backend power bit if not already on
      pyldmgr_state.settings.bkend_pwr_ena = 1;
      break;

    default:
      // unknown command parameter, don't do anything
      break;
  }
}


int payload_power_off_callback(event eventID, void* arg) {
	nonvolatile_settings_t eepsettings;

	program_LED(IPMI_LED1Y_TBL_IDX, Local_Control, &LED_Off_Activity);

  // restore power enable to nonvolatile on/off state
  while (eepspi_chk_write_in_progress()) Service_Watchdog();     // wait for EEPROM to be available
  eepspi_read((unsigned char*) &eepsettings, PAYLDMGR_AREA_BYTE_OFFSET, 4);           // read first few bytes, enough to load setting
  pyldmgr_state.settings.bkend_pwr_ena = eepsettings.bkend_pwr_ena;

  // reset all timers to zero regardless of current state
  pyldmgr_state.ctl.cold_reset_timer = 0;
  pyldmgr_state.ctl.quiesce_timer1 = 0;
  pyldmgr_state.ctl.quiesce_timer2 = 0;
  pyldmgr_state.ctl.payload_startup_timer = 0;
  pyldmgr_state.ctl.backend_startup_timer = 0;
  pyldmgr_state.ctl.warm_reset_timer = 0;
  return 1;
}


int payload_power_on_callback(event eventID, void* arg) {
  // clear the backend failure, shutdown, and quiesced bits in the hotswap sensor
  // this call is made when power is first detected, when the payload startup timer begins
  update_sensor_discrete_state(HOTSWAP_SENSOR, HOTSWAP_QUIESCED_MASK | HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK | HOTSWAP_BACKEND_PWR_FAILURE_MASK, 0);
	update_sensor_discrete_state(ALARM_LEVEL_SENSOR, 0x7fff, ALARMLVLEV_OK_MASK);
  pyldmgr_state.alarm.cur_alarm_level = status_OK;
  pyldmgr_state.alarm.cur_alarm_source = none;
  program_LED(IPMI_BLUELED_TBL_IDX, Local_Control, &LED_Off_Activity);
  program_LED(IPMI_LED1R_TBL_IDX, Local_Control, &LED_Off_Activity);
  program_LED(IPMI_LED1Y_TBL_IDX, Local_Control, &LED_Off_Activity);
  program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_Off_Activity);

  return 1;
}


int fast_output_timer_callback(event eventID, void* arg) {
	// This function provides the fast interlock on the MMC outputs to the back end with the payload power.
	// When payload power is inactive, the outputs should be in a proper state to avoid sneak paths.
	// When payload power is active, and the back end power is enabled, then the phased turn-on of the
	// back end power enables should occur.
	if ((!pyldmgr_state.settings.bkend_pwr_ena) || (pyldmgr_state.alarm.cur_alarm_level == fault) || (pyldmgr_get_payload_power_status() == power_off) ||
	  (pyldmgr_state.ctl.cold_reset_timer)) {
		// power enable and other outputs at should be at logic 0
    set_MMC_pin_backend_off_state();
    AVR32_PM.gplp[0] &= ~RSTCFG_BACKEND_HOT_MASK;           // make sure back end hot mask is clear;
	  pyldmgr_state.ctl.backend_cur_stage = 0;
	  pyldmgr_state.ctl.backend_cur_timer = 0;                // reset back end state machine
	}
	else {
		// back end enables should be on or in process of being turned on
		if (pyldmgr_state.ctl.backend_cur_stage == 0xff) {
		  // back end power-up complete
      AVR32_PM.gplp[0] |= RSTCFG_BACKEND_HOT_MASK;        // mark back end power hot
		  return 1;
    }
		if (pyldmgr_state.ctl.backend_cur_timer) {
			// timer currently running, continue back end power-up sequence
			pyldmgr_state.ctl.backend_cur_timer--;                // decrement timer
			if (!pyldmgr_state.ctl.backend_cur_timer) {
				// timer expired
				pyldmgr_state.ctl.backend_cur_stage++;
				// advance to next mask
				pyldmgr_state.ctl.backend_cur_mask = pyldmgr_state.ctl.pwrena_mask[pyldmgr_state.ctl.backend_cur_stage];
			  GPIO.port[1].ovrs = pyldmgr_state.ctl.backend_cur_mask;                                   
				if (pyldmgr_state.ctl.backend_cur_stage == pyldmgr_state.settings.bkend_pwr_cfg.defined_stage_cnt-1) {
				  // no more timers--sequence complete
				  pyldmgr_state.ctl.backend_cur_stage = 0xff;
				  pyldmgr_state.ctl.backend_startup_timer = MONITOR_STARTUP_TICKS;
				  isr_event_mask |= PAYLDMGR_BKEND_SEQUENCE_COMPLETE;
				}
				else {
					// set timer for next interval
					pyldmgr_state.ctl.backend_cur_timer = 5 * pyldmgr_state.settings.bkend_pwr_cfg.stagedelay_ms[pyldmgr_state.ctl.backend_cur_stage];          // 5 200us ticks per ms
				}				
			}
		}
		else {
			// timer not running and sequence not complete--start back end power-up sequence
			pyldmgr_state.ctl.backend_cur_mask = pyldmgr_state.ctl.pwrena_mask[0];
			pyldmgr_state.ctl.backend_cur_stage = 0;
			pyldmgr_state.ctl.backend_cur_timer = 5 * pyldmgr_state.settings.bkend_pwr_cfg.stagedelay_ms[0];          // 5 200us ticks per ms
			GPIO.port[1].ovrs = pyldmgr_state.ctl.backend_cur_mask;                                   // set enables
		}
	}
	
  return 1;
}


int pyldmgr_100ms_timer_callback(event eventID, void* arg) {
  // this routine runs on the 100ms event timer, and is used to implement the timed functions of the
  // payload manager, including reset timers, power supply startup timers, and the quiesce timer.  It runs
  // at the same rate as the internal ADC and so it is also used to update the payload state machine.
  
  // update payload power detect
  pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].prev_on_state = pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].cur_on_state;
  if (SensorData[PAYLOAD_12V_SENSOR].readout_value >= pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].upper_thr) {
    pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].cur_on_state = 1;
    if (pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].prev_on_state == 0)
      isr_event_mask |= PAYLDMGR_PWRON_ISR_DETECT;
	}
	else {
    if (SensorData[PAYLOAD_12V_SENSOR].readout_value <= pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].lower_thr) {
	    pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].cur_on_state = 0;
		  if (pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].prev_on_state == 1) {
        isr_event_mask |= PAYLDMGR_PWROFF_ISR_DETECT;
		  }		    
		}        
	}	
	// update running state of payload power per comparator state
	if (pyldmgr_state.payld_pwr[PWRCOMP_PAYLD_12V].cur_on_state) {
	  pyldmgr_state.payld_pwr_cur_state = power_on;
    AVR32_PM.gplp[0] |= RSTCFG_PAYLOAD_HOT_MASK;            // mark payload as hot
  }
	else {
	  pyldmgr_state.payld_pwr_cur_state = power_off;
    AVR32_PM.gplp[0] &= ~RSTCFG_PAYLOAD_HOT_MASK;           // mark payload as not hot
  }

  // examine cold reset timer
  if (pyldmgr_state.ctl.cold_reset_timer) {
    pyldmgr_state.ctl.cold_reset_timer--;
  }

  // examine warm reset timer
  if (pyldmgr_state.ctl.warm_reset_timer) {
    pyldmgr_state.ctl.warm_reset_timer--;
    if (!pyldmgr_state.ctl.warm_reset_timer) {
		  // timer expired--turn off active low drive, revert to high-impedance state
      GPIO.port[1].oderc = (1<<(_FPGA_CPU_RST-32));
    }
    else {
      // assert active low output state on CPU reset signal
      GPIO.port[1].ovrc = (1<<(_FPGA_CPU_RST-32));
      GPIO.port[1].oders = (1<<(_FPGA_CPU_RST-32));
    }
  }

  // examine quiesce timer
  if ((pyldmgr_state.ctl.quiesce_timer1 > 0) || (pyldmgr_state.ctl.quiesce_timer2 > 0))
    isr_event_mask |= PAYLDMGR_QUIESCE_TIMER_TICK;

  // examine payload power startup timer
  if (pyldmgr_state.ctl.payload_startup_timer)
    pyldmgr_state.ctl.payload_startup_timer--;

  // examine backend startup timer
  if (pyldmgr_state.ctl.backend_startup_timer) {
    pyldmgr_state.ctl.backend_startup_timer--;
    if (!pyldmgr_state.ctl.backend_startup_timer)
      isr_event_mask |= PAYLDMGR_BKEND_TIMER_EXP_ISR_DETECT;
  }

  // examine autoconfig timer
  if (pyldmgr_state.ctl.autoconfig_poll_timer) {
    pyldmgr_state.ctl.autoconfig_poll_timer--;
    if (!pyldmgr_state.ctl.autoconfig_poll_timer)
      isr_event_mask |= PAYLDMGR_AUTOCONFIG_TIMER_FLAG;
  }

  // examine 1 sec util timer
  if (pyldmgr_state.ctl.sec_util_timer) {
    pyldmgr_state.ctl.sec_util_timer--;
    if (!pyldmgr_state.ctl.sec_util_timer) {
      isr_event_mask |= PAYLDMGR_1SEC_UTIL_ISR_DETECT;
      pyldmgr_state.ctl.sec_util_timer = ONESEC_UTIL_TIMER_TICKS;
    }
  }
  return 1;
}


int pyldmgr_1sec_timer_callback(event eventID, void* arg) {
  // this routine runs on the 1 second event timer, and is used to implement mmc uptime counter
  // and the backend hot time counter
  TIMESTATREC.uptime++;
  TIMESTATREC.lastrsttime++;
  if (pyldmgr_state.ctl.backend_cur_stage == 0xff)
    TIMESTATREC.hottime++;
#ifdef TRACE
	if (get_cap_buf_ena())
	  cap_buf_capture_twi_state();
#endif
  return 1;
}


void check_alarm_status(void) {
  // This function is called during payload manager service.  It checks the state assertions on the threshold sensors
  // and updates the system state as necessary
  
  int i1;
  alarm_level_t working_alarm_level = status_OK;            // working alarm level for current status calculation
  alarm_source_class_t working_alarm_source = none;         // working alarm source for current status calculation
  alarm_level_t sensor_alarm_mask;
  sensor_data_entry_t* pSD;
  unsigned short cur_assert_mask;
  unsigned short cur_alarm_sensor_state_mask;
  fault_log_entry_t fault_log_entry;
  
  if (pyldmgr_state.alarm.cur_alarm_level == fault)
    // in fault state, stay there!
    return;

  // determine working alarm level for this call, from available state inputs
  //  for (i1=FIRST_ANALOG_SENSOR; i1<=LAST_ANALOG_SENSOR; i1++) {
  // change 05jun14, scan through entire list, and process only those sensors which have a event_reading_type of THRESHOLD
  for (i1=FIRST_SENSOR_NUMBER; i1<=LAST_SENSOR_NUMBER; i1++) {
    pSD = &SensorData[i1];
    if (pSD->pSDR->event_reading_type != THRESHOLD_EVENT_READING_TYPE)
      // not a threshold sensor, no need to process here
      continue;

    if (pyldmgr_state.settings.global_mask_level > pyldmgr_state.settings.sensor_mask_level[i1])
      sensor_alarm_mask = pyldmgr_state.settings.global_mask_level;
    else
      sensor_alarm_mask = pyldmgr_state.settings.sensor_mask_level[i1];
    cur_assert_mask = pSD->cur_masked_comp & (pSD->pSDR->assertion_event_mask[LOWBYTE] | ((pSD->pSDR->assertion_event_mask[HIGHBYTE] & 0xf) << 8));

    // assess severity level, starting with most nonrecoverable
    if (cur_assert_mask & (SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK | SENSOREV_LOWER_NONRECOVER_LOW_ASSERT_MASK)) {
      // threshold at nonrecoverable severity
      if ((working_alarm_level < fault) && (sensor_alarm_mask < fault)) {
        working_alarm_source = (pSD->pSDR->sensortype == TEMPERATURE_SENSOR_TYPE) ? temperature : voltage;
        working_alarm_level = fault;
      }
	    // write a fault log entry for storage in EEPROM
		  fault_log_entry.sensor_num = i1;
		  fault_log_entry.sensor_val = pSD->readout_value;
		  fault_log_entry.systime = get_rtc_value();
		  if (cur_assert_mask & SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK) {
			  // upper threshold
			  fault_log_entry.thresh_val = pSD->pSDR->upper_nonrecover_thr;
			  fault_log_entry.event_offset = SENSOREV_UPPER_NONRECOVER_HIGH_OFFSET;
		  }
		  else {
			  // lower threshold
			  fault_log_entry.thresh_val = pSD->pSDR->lower_nonrecover_thr;
			  fault_log_entry.event_offset = SENSOREV_LOWER_NONRECOVER_HIGH_OFFSET;
		  }
      continue;
    }
    if (cur_assert_mask & (SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK | SENSOREV_LOWER_CRITICAL_LOW_ASSERT_MASK)) {
      // threshold at critical
      if ((working_alarm_level < critical) && (sensor_alarm_mask < critical)) {
        working_alarm_source = (pSD->pSDR->sensortype == TEMPERATURE_SENSOR_TYPE) ? temperature : voltage;
        working_alarm_level = critical;
      }
      continue;
    }
    if (cur_assert_mask & (SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK | SENSOREV_LOWER_NONCRITICAL_LOW_ASSERT_MASK)) {
      // threshold at noncritical
      if ((working_alarm_level < noncritical) && (sensor_alarm_mask < noncritical)) {
        working_alarm_source = (pSD->pSDR->sensortype == TEMPERATURE_SENSOR_TYPE) ? temperature : voltage;
        working_alarm_level = noncritical;
      }
    }
  }

  cur_alarm_sensor_state_mask = SensorData[ALARM_LEVEL_SENSOR].cur_masked_comp;
  
  // compare working alarm level to system state, making adjustments as necessary
  if (working_alarm_level != pyldmgr_state.alarm.cur_alarm_level) {
    // update alarm level and associated sensor
    pyldmgr_state.alarm.cur_alarm_level = working_alarm_level;
    pyldmgr_state.alarm.cur_alarm_source = (working_alarm_level == status_OK) ? none : working_alarm_source;
    switch (pyldmgr_state.alarm.cur_alarm_level) {
      case status_OK:
        // things have gone back to normal
				update_sensor_discrete_state(ALARM_LEVEL_SENSOR, 0x7fff, ALARMLVLEV_OK_MASK);
        program_LED(IPMI_LED1R_TBL_IDX, Local_Control, &LED_Off_Activity);
        break;

      case noncritical:
	      if (cur_alarm_sensor_state_mask & ALARMLVLEV_OK_MASK)
			    // things were OK, now non-critical
				  update_sensor_discrete_state(ALARM_LEVEL_SENSOR, 0x7fff, ALARMLVLEV_OK_TO_NONCRIT_MASK);
		  	else if (cur_alarm_sensor_state_mask & (ALARMLVLEV_CRIT_COMBO_MASK | ALARMLVLEV_NONRECOVER_COMBO_MASK))
				  // things were above non-critical
				  update_sensor_discrete_state(ALARM_LEVEL_SENSOR, 0x7fff, ALARMLVLEV_NONCRIT_FROM_HIGHER_MASK);
        program_LED(IPMI_LED1R_TBL_IDX, Local_Control, &LED_Noncritical_Activity);
        break;

      case critical:
	      if (cur_alarm_sensor_state_mask & (ALARMLVLEV_OK_MASK | ALARMLVLEV_NONCRIT_COMBO_MASK))
			    // things were OK, now non-critical
				  update_sensor_discrete_state(ALARM_LEVEL_SENSOR, 0x7fff, ALARMLVLEV_CRIT_FROM_LOWER_MASK);
		  	else if (cur_alarm_sensor_state_mask & ALARMLVLEV_NONRECOVER_COMBO_MASK)
				  // things were above non-critical
				  update_sensor_discrete_state(ALARM_LEVEL_SENSOR, 0x7fff, ALARMLVLEV_CRIT_FROM_NONRECOVER_MASK);
        program_LED(IPMI_LED1R_TBL_IDX, Local_Control, &LED_Critical_Activity);
        break;

      case fault:
        // hit the nonrecoverable fault status--update LED state and send hotswap message bearing the news
				update_sensor_discrete_state(ALARM_LEVEL_SENSOR, 0x7fff, ALARMLVLEV_NONRECOVER_COMBO_MASK);
        program_LED(IPMI_LED1R_TBL_IDX, Local_Control, &LED_On_Activity);
        if (pyldmgr_state.alarm.cur_alarm_source == voltage)
		      update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_BACKEND_PWR_FAILURE_MASK);
        else
		      update_sensor_discrete_state(HOTSWAP_SENSOR, 0, HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK);
			  // write log entry at next location in nonvolatile storage
			  write_fault_log_entry(&fault_log_entry);
        break;

      default:
        // shouldn't come here, do nothing
        break;
    }
  }
}


unsigned long pwrena2pinmask(unsigned char pwrena_mask) {
	int i1;
	unsigned long retval = 0;
	
	for (i1=0; i1<MAX_PWR_ENA_PIN_CNT; i1++) {
		if ((pwrena_mask & (1<<i1)) && (PWR_ENA_pin_map[i1] != PWR_ENA_PIN_UNDEFINED))
		  // set appropriate pin position in mask
			retval |= (1<<(PWR_ENA_pin_map[i1]-32));
	}
	return retval;
}


inline void set_MMC_pin_backend_off_state(void) {
	// sets the MMC/FPGA interface pins to an off state which is intended to prevent sneak paths into the powered-off FPGA logic
  GPIO.port[0].ovrc = ((1<<_FSIO_SS0) | (1<<FSIO_SCK) | (1<<FSIO_MISO) | (1<<FSIO_MOSI));
  GPIO.port[0].oders = ((1<<_FSIO_SS0) | (1<<FSIO_SCK) | (1<<FSIO_MISO) | (1<<FSIO_MOSI));
	GPIO.port[0].gpers = ((1<<_FSIO_SS0) | (1<<FSIO_SCK) | (1<<FSIO_MISO) | (1<<FSIO_MOSI));
	GPIO.port[0].puerc = ((1<<_FSIO_SS0) | (1<<FSIO_SCK) | (1<<FSIO_MISO) | (1<<FSIO_MOSI));

  GPIO.port[1].oderc = (1<<(_FPGA_CPU_RST-32));
	GPIO.port[1].ovrc = ((1<<(PWRGOOD-32)) | (1<<(_FSIO_SCANSLV-32)) | (1<<(UC_PWRENA1-32)) | (1<<(UC_PWRENA2-32)) | (1<<(UC_PWRENA3-32)) | (1<<(UC_PWRENA4-32)));
	GPIO.port[1].oders = ((1<<(PWRGOOD-32)) | (1<<(_FSIO_SCANSLV-32)) | (1<<(UC_PWRENA1-32)) | (1<<(UC_PWRENA2-32)) |
   (1<<(UC_PWRENA3-32)) | (1<<(UC_PWRENA4-32)));
	GPIO.port[1].puerc = ((1<<(PWRGOOD-32)) | (1<<(_FSIO_SCANSLV-32)) | (1<<(UC_PWRENA1-32)) | (1<<(UC_PWRENA2-32)) |
   (1<<(UC_PWRENA3-32)) | (1<<(UC_PWRENA4-32)));
}

inline void set_MMC_pin_backend_on_state(void) {
  // sets the MMC/FPGA interface pins to the proper state
  // called when back end power on event is detected
	GPIO.port[1].oderc = (1<<(PWRGOOD-32));          // turn off drive on input pins and boot mode pins for now
  gpio_spi1_mode();         // set SPI pins
}


// test code to try an inter-MMC message at quiesce time
void generate_lamp_test_request(void) {
  ipmb_msg_desc_t req, sendmsgwrapper;              // request message buffer
  // build LED test command
  ipmb_init_req_hdr(req.buf, 0xa4, NETFN_GRPEXT, 0, 0);
  req.buf[5] = IPMICMD_PICMG_SET_FRU_LED_STATE;
  req.buf[6] = NETFN_PICMG_IDENTIFIER;
  req.buf[7] = 0;     // device ID
  req.buf[8] = 0x02;        // IPMI LED 2
  req.buf[9] = 0xfb;        // LAMP test
  req.buf[10] = 50;         // lamp on for 5 seconds (50 * 100ms)
  req.buf[11] = 0x0f;       // use default color
  req.buf[12] = calc_ipmi_xsum(&req.buf[3], 9);          // message checksum
  req.len = 13;
  encapsulate_message(&sendmsgwrapper, &req);
  ipmb_send_request(&sendmsgwrapper, &lamp_test_response_callback);
}
  

int lamp_test_response_callback(void* preq, void* prsp) {
	//const ipmb_msg_desc_t* prq = preq;
	const ipmb_msg_desc_t* prs = prsp;
	ipmb_msg_desc_t encaprsp;
	
	if (!prs) {
	  sio_filt_putstr(TXTFILT_INFO, 1, "Lamp test command to AMC13 NOT acknowledged\n");
	}
	else {
		sio_filt_putstr(TXTFILT_INFO, 1, "Lamp test command to AMC13 WAS acknowledged\n");
		if (deencapsulate_message(prsp, &encaprsp)) {
			// response had encapsulated message
			sio_filt_putstr(TXTFILT_INFO, 1, "Response had encapsulated message\n");
			ipmb_msg_dump(&encaprsp, TXTFILT_INFO);
		}
		else {
			// response did not have encapsulated message in it
			sio_filt_putstr(TXTFILT_INFO, 1, "No encapsulated message in response\n");
			ipmb_msg_dump(prsp, TXTFILT_INFO);
		}
	}
	return 0; 
}



