/*
 * mmcfrudata.h
 *
 *  Created on: Oct 13, 2010
 *      Author: tgorski
 */

#ifndef NONVOLATILE_H_
#define NONVOLATILE_H_

#include "eepspi.h"
//#include "sensor_svc.h"

typedef struct {
  unsigned char EEP_format_flag;
  unsigned char version;                    // version field for hardware info area--this is version 1
  unsigned char cmc_hw_vers;
  unsigned char cmc_hw_ser_num_lbyte;
  unsigned char cmc_hw_ser_num_ubyte;
  unsigned char mmc_sw_major_vers;
  unsigned char mmc_sw_minor_vers;
  unsigned char amc_hw_vers_lbyte;
  unsigned char amc_hw_vers_ubyte;
  unsigned char amc_hw_ser_num_lbyte;
  unsigned char amc_hw_ser_num_lmbyte;
  unsigned char amc_hw_ser_num_umbyte;
  unsigned char amc_hw_ser_num_ubyte;
  unsigned char amc_hw_id_lbyte;
  unsigned char amc_hw_id_ubyte;
  unsigned char amc_fw_id_lbyte;
  unsigned char amc_fw_id_ubyte;
  unsigned char checksum;
} hardware_info_area_t;


typedef struct {
  unsigned char deviceID;
  unsigned char devicerev;
  unsigned char fwrev1;
  unsigned char fwrev2;
  unsigned char ipmivers;
  unsigned char adddevsuppt;
  unsigned char manfIDlbyte;
  unsigned char manfIDmidbyte;
  unsigned char manfIDhbyte;
  unsigned char prodIDlbyte;
  unsigned char prodIDhbyte;
} App_Device_ID_record_t;

typedef struct {
  unsigned char version;
  unsigned char internal_use_area_offset;
  unsigned char chassis_info_area_offset;
  unsigned char board_area_offset;
  unsigned char product_area_offset;
  unsigned char multirecord_area_offset;
  unsigned char pad;
  unsigned char checksum;
} FRU_area_common_header_t;

typedef struct {
  unsigned char version;
  unsigned char length;
  unsigned char language;
  unsigned char filler[8];
  unsigned char endoffields_flag;
} FRU_default_board_info_area_t;

typedef struct {
  unsigned char rectypeID;
  unsigned char eolflag_recversion;
  unsigned char reclength;
  unsigned char recxsum;
  unsigned char hdrxsum;
} multirecord_header_t;

typedef struct {
  multirecord_header_t hdr;
  unsigned char mfgID_LSB;
  unsigned char mfgID_MidB;
  unsigned char mfgID_MSB;
  unsigned char PICMG_recID;
  unsigned char record_fmt_version;
  unsigned char current_draw_100mA;
} module_current_req_record_t;


typedef struct {
	unsigned char key;
	unsigned char sio_filtermask;
	unsigned char unused[30];
} GP_param_record_t;


#define NONVOLATILE_FORMAT_VERSION      (0x01)      // version of format header

// size definitions
// size/location definitions for hardware header information
#define HW_HEADER_BYTE_OFFSET				  (0)
#define HW_HEADER_SIZE						    (32)

// size/location definitions for Application Device ID info
#define APP_DEV_ID_BYTE_OFFSET				(HW_HEADER_BYTE_OFFSET+HW_HEADER_SIZE)
#define APP_DEV_ID_SIZE						    (16)

// size definitions for FRU data area
#define COMMON_HEADER_BYTE_OFFSET			(APP_DEV_ID_BYTE_OFFSET+APP_DEV_ID_SIZE)
#define COMMON_HEADER_SIZE					(8)
#define BOARD_INFO_AREA_BYTE_OFFSET			(COMMON_HEADER_BYTE_OFFSET+COMMON_HEADER_SIZE)
#define BOARD_INFO_AREA_SIZE				(64)
#define MULTIRECORD_AREA_BYTE_OFFSET		(BOARD_INFO_AREA_BYTE_OFFSET+BOARD_INFO_AREA_SIZE)
#define MULTIRECORD_AREA_SIZE				(80)

#define END_OF_FRU_AREA_OFFSET				(MULTIRECORD_AREA_SIZE+MULTIRECORD_AREA_BYTE_OFFSET)

// FPGA config area
#define FPGA_CONFIG_AREA_BYTE_OFFSET		(END_OF_FRU_AREA_OFFSET)
#define FPGA_CONFIG_AREA_SIZE				(256)

// SDR data table
#define SDR_AREA_BYTE_OFFSET				(480)		// define to be on 32-byte boundary
#define SDR_AREA_SIZE						     (32*64)

#if SDR_AREA_BYTE_OFFSET < FPGA_CONFIG_AREA_BYTE_OFFSET+FPGA_CONFIG_AREA_SIZE
#error "SDR area overlaps FPGA config area in EEPROM address allocation"
#endif

// Payload Manager default settings
#define PAYLDMGR_AREA_BYTE_OFFSET   (SDR_AREA_BYTE_OFFSET+SDR_AREA_SIZE)
#define PAYLDMGR_AREA_SIZE          (64)

// ADC Scaling Factors
#define ADC_SCALING_AREA_BYTE_OFFSET    (PAYLDMGR_AREA_BYTE_OFFSET+PAYLDMGR_AREA_SIZE)
#define ADC_SCALING_AREA_SIZE           (192)

// MMC General Purpose Params
#define GP_PARAM_AREA_BYTE_OFFSET   (ADC_SCALING_AREA_BYTE_OFFSET+ADC_SCALING_AREA_SIZE)
#define GP_PARAM_AREA_SIZE          (64)

// Fault Event Log
#define FAULT_LOG_ENTRY_SIZE             (8)
#define FAULT_LOG_ENTRY_CNT              (15)
#define FAULT_LOG_HDR_BYTE_OFFSET        (GP_PARAM_AREA_BYTE_OFFSET+GP_PARAM_AREA_SIZE)
#define FAULT_LOG_ENTRY_BYTE_OFFSET      (FAULT_LOG_HDR_BYTE_OFFSET+FAULT_LOG_ENTRY_SIZE)
#define FAULT_LOG_SIZE                   (FAULT_LOG_ENTRY_SIZE*(FAULT_LOG_ENTRY_CNT+1))

#define EEP_USED_AREA_SIZE					     (FAULT_LOG_HDR_BYTE_OFFSET+FAULT_LOG_SIZE)

#if EEP_USED_AREA_SIZE > EEPSIZE
#error "EEPROM is overallocated"
#endif

#define FRU_AREA_SIZE						(MULTIRECORD_AREA_SIZE+MULTIRECORD_AREA_BYTE_OFFSET-COMMON_HEADER_BYTE_OFFSET)
#define FRU_AREA_SIZE_L						(FRU_AREA_SIZE & 0xff)
#define FRU_AREA_SIZE_H						(FRU_AREA_SIZE >> 8)


void nonvol_init(void);

extern GP_param_record_t GPparambuf;

#endif /* NONVOLATILE_H_ */
