/*
 * mmcfrudata.c
 *
 *  Created on: Oct 13, 2010
 *      Author: tgorski
 */
#include <string.h>
#include "sio_usart.h"
#include "ipmb_svc.h"
#include "nonvolatile.h"
#include "mmc_version.h"

GP_param_record_t GPparambuf;               // RAM buffer for GP params


void nonvol_init(void) {
  // checks the first byte of the eeprom.  If it is 0xff, chip is considered erased, and
  // a default image is written to the eeprom
  unsigned char buf[END_OF_FRU_AREA_OFFSET];
  unsigned char buf2[SENSOR_SETTINGS_AREA_SIZE];
  hardware_info_area_t* pHWarea = (hardware_info_area_t*) &buf[HW_HEADER_BYTE_OFFSET];
  App_Device_ID_record_t* pAppDevID = (App_Device_ID_record_t*) &buf[APP_DEV_ID_BYTE_OFFSET];
  FRU_area_common_header_t* pFRUhdr = (FRU_area_common_header_t*) &buf[COMMON_HEADER_BYTE_OFFSET];
  module_current_req_record_t* pcurrec = (module_current_req_record_t*) &buf[MULTIRECORD_AREA_BYTE_OFFSET];
  sensor_settings_record_ID_t* psensrecID = (sensor_settings_record_ID_t*) &buf2[0];
  
  int i;

  while (eepspi_chk_write_in_progress());     // wait for EEPROM to be available
  if (!eepspi_read(&buf[0], HW_HEADER_BYTE_OFFSET, 1))
    // read not successful
    return;						// abort formatting procedure
  if (buf[0] == 0xff) {
    sio_putstr("Blank EEPROM Detected.  Writing default Device ID image....\n");
    // format eeprom image
    for (i=0; i<sizeof(buf); i++)
      buf[i] = 0;					// initialize to zeros

    // configure hardware info area (nonzero fields)
    pHWarea->version = 1;
    pHWarea->mmc_sw_major_vers = MAJOR_MMC_VERSION_NUMBER;
    pHWarea->mmc_sw_minor_vers = MINOR_MMC_VERSION_NUMBER;
    pHWarea->checksum = calc_ipmi_xsum((unsigned char*) pHWarea, sizeof(hardware_info_area_t)-1);

    // configure Application Device ID area (nonzero fields)
    pAppDevID->devicerev = 0x80;					// supports SDRs
    pAppDevID->fwrev1 = 0x00;
    pAppDevID->fwrev2 = 0x03;
    pAppDevID->ipmivers = 0x02;					// IPMI v2.0 support
    pAppDevID->adddevsuppt = 0x3b;				// device supports IPMB events, has SDR, FRU info, sensors

    // configure FRU area common header (nonzero fields)
    pFRUhdr->version = 0x01;						// v1.0 of storage definition
    pFRUhdr->multirecord_area_offset = (MULTIRECORD_AREA_BYTE_OFFSET-COMMON_HEADER_BYTE_OFFSET)/8;
    pFRUhdr->checksum = calc_ipmi_xsum((unsigned char*) pFRUhdr, sizeof(FRU_area_common_header_t)-1);

    // configure FRU multirecord area--default current draw per PICMG
    pcurrec->hdr.rectypeID = 0xc0;				// OEM record type
    pcurrec->hdr.eolflag_recversion = 0x82;		// last record flag + version 02h
    pcurrec->hdr.reclength = sizeof(module_current_req_record_t);
    pcurrec->mfgID_LSB = 0x5a;
    pcurrec->mfgID_MidB = 0x31;
    pcurrec->mfgID_MSB = 0x00;
    pcurrec->PICMG_recID = 0x16;				// current requirements record
    pcurrec->record_fmt_version = 0x00;
    pcurrec->current_draw_100mA = 70;			// say 7A for now
    pcurrec->hdr.recxsum = calc_ipmi_xsum((unsigned char*) &(pcurrec->mfgID_LSB), sizeof(module_current_req_record_t)-
      sizeof(multirecord_header_t));
    pcurrec->hdr.hdrxsum = calc_ipmi_xsum((unsigned char*) &(pcurrec->hdr.rectypeID), sizeof(multirecord_header_t)-1);

    // write image to memory
    eepspi_write(buf, HW_HEADER_BYTE_OFFSET, END_OF_FRU_AREA_OFFSET);
  }  
  
  // initialize general parameter area
  while (eepspi_chk_write_in_progress());     // wait for EEPROM to be available
  if (!eepspi_read((unsigned char*) &GPparambuf, GP_PARAM_AREA_BYTE_OFFSET, sizeof(GP_param_record_t)))
    // read not successful
    return;						// abort formatting procedure
  if (GPparambuf.key == 0xff) {
    sio_putstr("Writing default GP param settings....\n");
    memset((void *) &GPparambuf, 0, sizeof(GPparambuf));            // zero out record
    GPparambuf.sio_filtermask =  TXTFILT_EVENTS | /*TXTFILT_IPMI_STD_REQ |  TXTFILT_IPIM_CUSTOM_REQ | 
      TXTFILT_DBG_SUMMARY | TXTFILT_DBG_DETAIL |*/  TXTFILT_INFO;              // things turned on by default
    eepspi_write((unsigned char*) &GPparambuf, GP_PARAM_AREA_BYTE_OFFSET, sizeof(GP_param_record_t));
  }
  
  // New at 2.3, check the Sensor Settings ID Record area, and initialize if empty
  while (eepspi_chk_write_in_progress());     // wait for EEPROM to be available
  if (!eepspi_read((unsigned char*) buf2, SENSOR_SETTINGS_BYTE_OFFSET, SENSOR_SETTINGS_AREA_SIZE))
    // read not successful
    return;						// abort formatting procedure
  if (buf2[0] == 0xff) {
    // sensor settings record area needs initialization
    sio_putstr("Writing default sensor settings record ID....\n");
    strcpy(psensrecID->recname, "Default Record ID");
    psensrecID->xsum[0] = 0;
    psensrecID->xsum[1] = 0;
    psensrecID->xsum[2] = 0;
    psensrecID->xsum[3] = 0;
    eepspi_write((unsigned char*) psensrecID, SENSOR_SETTINGS_BYTE_OFFSET, SENSOR_SETTINGS_AREA_SIZE);
  }
}



