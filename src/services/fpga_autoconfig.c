/*
 * fpga_autoconfig.c
 *
 *  Created on: Apr 18, 2011
 *      Author: tgorski
 */

#include <stdio.h>
#include <string.h>
#include "eepspi.h"
#include "spi1.h"
#include "fpgaspi.h"
#include "sio_usart.h"
#include "twidriver.h"
#include "ipmb_svc.h"
#include "timer_callback.h"
#include "LEDdrivers.h"
#include "swevent.h"
#include "nonvolatile.h"
#include "sensor_svc.h"
#include "fpga_autoconfig.h"
#include "payload_mgr.h"

unsigned char autocfg_buf[FPGA_CONFIG_AREA_SIZE];
FPGA_config_area_header_t* pFPGA_config_hdr = (FPGA_config_area_header_t*) &autocfg_buf[0];

typedef struct {
	unsigned char valid;              // nonzero if valid record for transfer to FPGA port
	unsigned char buf_offset;         // offset of data from start of autoconfig buffer
	unsigned char xlen;               // length of config record to transfer
	unsigned short port_daddr;        // destination address for data in transfer to FPGA SPI port
} FPGA_config_record_entry_t;

typedef FPGA_config_record_entry_t FPGA_config_rec_t[MAX_FPGA_ID];

FPGA_config_rec_t FPGA_config_rec;

int fpga_autoconfig_service(event eventID, void* arg);
int init_LED_indicator(event eventID, void* arg);

void fpga_autoconfig_init(void) {
	int i1;
	unsigned char offset;
	int insert_slotID;               // set if config should insert the slot ID at byte 0 of each active config string
	
	FPGA_configrec_header_t* pPort_config_rec_hdr;
	
  // check to see if autoconfig area needs formatting
  memset((void *) &autocfg_buf[0], 0xff, FPGA_CONFIG_AREA_SIZE);
  // get default settings from eeprom
  while (eepspi_chk_write_in_progress());     // wait for EEPROM to be available
  eepspi_read(&autocfg_buf[0], FPGA_CONFIG_AREA_BYTE_OFFSET, FPGA_CONFIG_AREA_SIZE);
  if (pFPGA_config_hdr->formatflag == 0xff) {
    // area unformatted--initialize
    sio_putstr("Uninitialized FPGA auto-config area detected.  Writing default image....\n");
    pFPGA_config_hdr->formatflag = 0;
    pFPGA_config_hdr->flags = 0;
    pFPGA_config_hdr->offset0 = 0xff;
    pFPGA_config_hdr->offset1 = 0xff;
    pFPGA_config_hdr->offset2 = 0xff;
    pFPGA_config_hdr->hdrxsum = calc_ipmi_xsum(&autocfg_buf[0], sizeof(FPGA_config_area_header_t)-1);
    eepspi_write(&autocfg_buf[0], FPGA_CONFIG_AREA_BYTE_OFFSET, FPGA_CONFIG_AREA_SIZE);
  }
  
  insert_slotID = (pFPGA_config_hdr->flags & AUTOCONFIG_ADD_SLOTID) ? 1 : 0;
  // register autoconfig service function to be called as necessary by payload manager
  register_swevent_callback(fpga_autoconfig_service, PYLDMGREV_FPGA_AUTOCONFIG_SVC);
  register_swevent_callback(init_LED_indicator, PYLDMGREV_FPGA_AUTOCONFIG_SVC);
  
  // scan the autoconfig buffer to determine if there are any valid config payloads to be delivered to FPGAs when they start up
  memset((void *) &FPGA_config_rec, 0, sizeof(FPGA_config_rec_t));
  if (pFPGA_config_hdr->hdrxsum != (calc_ipmi_xsum((unsigned char *) pFPGA_config_hdr, sizeof(FPGA_config_area_header_t)-1)))
    // bad checksum in header
	  return;
  for (i1=0; i1<MAX_FPGA_ID; i1++) {
	  if (pFPGA_config_hdr->flags & (1 << i1)) {
		  // flag bit set in header, indicating that there is a valid record for this port
		  offset = *((unsigned char*) &(pFPGA_config_hdr->offset0) + i1);         // get offset for this FPGA in buffer
		  pPort_config_rec_hdr = (FPGA_configrec_header_t*) &autocfg_buf[offset];            // set port config hdr to correct address in buffer;
		  
		  // check buffer to make sure it is OK
		  if (pPort_config_rec_hdr->hdrxsum != calc_ipmi_xsum((unsigned char *) pPort_config_rec_hdr, sizeof(FPGA_configrec_header_t)-1))
		    // bad header on this record, skip it
			  continue;
			if (pPort_config_rec_hdr->reclength > AUTOCONFIG_MAX_XFER_LEN)
			  // record exceeds max length
			  continue;
			  
			FPGA_config_rec[i1].buf_offset = offset + sizeof(FPGA_configrec_header_t);            // starting offset for config payload
			FPGA_config_rec[i1].xlen = pPort_config_rec_hdr->reclength;                           // transfer length
			FPGA_config_rec[i1].port_daddr = pPort_config_rec_hdr->dest_addr_LSB + (((unsigned short) pPort_config_rec_hdr->dest_addr_MSB) << 8);       // 16-bit destination address
			if (pPort_config_rec_hdr->recxsum != calc_ipmi_xsum(&autocfg_buf[FPGA_config_rec[i1].buf_offset], FPGA_config_rec[i1].xlen))
			  // bad checksum, skip record
			  continue;
			  
			// everything checks out, mark entry as valid!!!
			FPGA_config_rec[i1].valid = 1;
			if (insert_slotID)
			  // substitute slotID as the first byte
			  autocfg_buf[FPGA_config_rec[i1].buf_offset] = twi_state.slotid;
	  }
  }
}


int fpga_autoconfig_service(event eventID, void* arg) {
  unsigned char slave_detect_mask;
  unsigned char status_reg;
  int deviceID;
  unsigned short config_sensor_assert_mask, config_sensor_deassert_mask;

  // this routine gets called by the payload manager automatically when the back end power is on.
  // first thing it will do is update the event bits in the FPGA config sensor that are associated with
  // the SPI interface.  After updating those bits, it will check to see if an auto-config operation 
  // should be executed.
  if (!spi1_lock())
    // spi1 interface unavailable, try again next time
    return 1;

  // detect active FPGA slaves on SPI interface
  slave_detect_mask = fpgaspi_slave_detect();

  // update SPI slate detect bits
  config_sensor_assert_mask = 0;
  config_sensor_deassert_mask = 0;
  
  if (slave_detect_mask & FPGA0_SPI_DETECT_MASK)
    config_sensor_assert_mask |= FPGACFGEV_SPI_DETECT0_MASK;
	else
	  config_sensor_deassert_mask |= FPGACFGEV_SPI_DETECT0_MASK;
  if (slave_detect_mask & FPGA1_SPI_DETECT_MASK)
    config_sensor_assert_mask |= FPGACFGEV_SPI_DETECT1_MASK;
	else
	  config_sensor_deassert_mask |= FPGACFGEV_SPI_DETECT1_MASK;
  if (slave_detect_mask & FPGA2_SPI_DETECT_MASK)
    config_sensor_assert_mask |= FPGACFGEV_SPI_DETECT2_MASK;
	else
	  config_sensor_deassert_mask |= FPGACFGEV_SPI_DETECT2_MASK;
	// send events for detected ports
  update_sensor_discrete_state(FPGA_CONFIG_SENSOR, config_sensor_deassert_mask, config_sensor_assert_mask);

  // check if autoconfig enabled in payload manager   
	if (pyldmgr_state.settings.autocfg_ena) {
    // read the config port status register for those SPI ports that are present 
    for (deviceID=0; deviceID<=MAX_FPGA_ID; deviceID++) {
      // check for active FPGA SPI slave interface at this port
      if (!(slave_detect_mask & (1 << deviceID)))
	      continue;
      // read the status register of this port
      status_reg = fpgaspi_status_read(deviceID);
      if ((status_reg & (FPGASPI_SR_REQCFG | FPGASPI_SR_CFGRDY)) != FPGASPI_SR_REQCFG)
		    // looking for REQCFG bit set, CFGRDY clear--did not find this condition
		    continue;
		  // check for valid config record
		  if (FPGA_config_rec[deviceID].valid) {
			  // transfer record to port
			  sprintf(spbuf, "Autoconfiguring FPGA device at port %i\n", deviceID);
			  sio_filt_putstr(TXTFILT_INFO, 1, spbuf);
        if (fpgaspi_data_write(deviceID, &autocfg_buf[FPGA_config_rec[deviceID].buf_offset], FPGA_config_rec[deviceID].port_daddr, FPGA_config_rec[deviceID].xlen)) {
          // transfer successful, set config ready bit in FPGA
          fpgaspi_ctl_write(deviceID, (FPGASPI_HFR_CFGRDY | FPGASPI_HFR_SETFLG));
		      program_LED(IPMI_LED2_TBL_IDX, Bypass, &LED_3sec_Lamp_Test_Activity);                 // 3 seconds light of IPMI LED2 to indicate configuration transferred
        }
		  }
    }		
  }  
  
  // scan through the different sensor ports again to determine the final state of the status registers after all autoconfig actions
  // have been taken, this time to set the config sensor states
  // first reset masks
  config_sensor_assert_mask = 0;
  config_sensor_deassert_mask = 0;
  for (deviceID=0; deviceID<=MAX_FPGA_ID; deviceID++) {
    // check for active FPGA SPI slave interface at this port
    if (!(slave_detect_mask & (1 << deviceID))) {
      // no active FPGA, so deassert the sensor states for the handshake bits
	    config_sensor_deassert_mask |= (FPGACFGEV_REQCFG0_MASK | FPGACFGEV_CFGRDY0_MASK) << (deviceID*3);
      continue;
	  }
	    
    // read the status register of this port
    status_reg = fpgaspi_status_read(deviceID);
	  // update sensor assert bits as appropriate
	  if (status_reg & FPGASPI_SR_REQCFG) 
	    config_sensor_assert_mask |= FPGACFGEV_REQCFG0_MASK << (deviceID*3);
	  else
	    config_sensor_deassert_mask |= FPGACFGEV_REQCFG0_MASK << (deviceID*3);
	  if (status_reg & FPGASPI_SR_CFGRDY) 
	    config_sensor_assert_mask |= FPGACFGEV_CFGRDY0_MASK << (deviceID*3);
	  else
	    config_sensor_deassert_mask |= FPGACFGEV_CFGRDY0_MASK << (deviceID*3);
  }		
 	
	// update config sensor discrete state
  update_sensor_discrete_state(FPGA_CONFIG_SENSOR, config_sensor_deassert_mask, config_sensor_assert_mask);
	  
  spi1_unlock();
  return 1;
}


int init_LED_indicator(event eventID, void* arg) {
  unsigned char slave_detect_mask;
  unsigned char status_reg0;
  unsigned char status_reg1;
  // New at Version 2.3:  Update the green IPMI LED according to the state of the request config and config ready bits at device 0
  
  if (pyldmgr_get_backend_power_status() == power_off) {
    // just in case the back end is off when this callback is made
    program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_Off_Activity);
    return 1;
  }

  if (!spi1_lock())
    // spi1 interface unavailable, try again next time
    return 1;

  // detect active FPGA slaves on SPI interface
  slave_detect_mask = fpgaspi_slave_detect();

  if (slave_detect_mask & FPGA0_SPI_DETECT_MASK) {
    // read status register
    status_reg0 = fpgaspi_status_read(0);
    status_reg1 = fpgaspi_status_read(1);

    if ((!(status_reg0 & FPGASPI_SR_CFGRDY) && (status_reg0 & FPGASPI_SR_REQCFG)) ||
      (!(status_reg1 & FPGASPI_SR_CFGRDY) && (status_reg1 & FPGASPI_SR_REQCFG)))
      program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_1Hz_Blink_Activity_7525);
    else
      program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_Off_Activity);
  }
  else {
    program_LED(IPMI_LED2_TBL_IDX, Local_Control, &LED_Off_Activity);
  }
  spi1_unlock();
  return 1;
}


