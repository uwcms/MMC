/*
 * fpga_autoconfig.h
 *
 *  Created on: Apr 18, 2011
 *      Author: tgorski
 */

#ifndef FPGA_AUTOCONFIG_H_
#define FPGA_AUTOCONFIG_H_

typedef struct {
  unsigned char formatflag;
  unsigned char flags;
  unsigned char offset0;
  unsigned char offset1;
  unsigned char offset2;
  unsigned char hdrxsum;
} FPGA_config_area_header_t;


// FPGA SPI auto-configuration definition and enable flags
#define FPGA0_AUTOCONFIG_IMG_DEFINED  (0x01)
#define FPGA1_AUTOCONFIG_IMG_DEFINED  (0x02)
#define FPGA2_AUTOCONFIG_IMG_DEFINED  (0x04)
#define AUTOCONFIG_ADD_SLOTID         (0x80)            // set this flag if the MMC slot ID is to dynamically replace the first byte of the config data string
#define AUTOCONFIG_MAX_XFER_LEN       (64)


typedef struct {
  unsigned char dest_addr_LSB;
  unsigned char dest_addr_MSB;
  unsigned char reclength;                // config data record length (not including header)
  unsigned char recxsum;                  // checksum for the config data record
  unsigned char hdrxsum;
} FPGA_configrec_header_t;


void fpga_autoconfig_init(void);


#endif /* FPGA_AUTOCONFIG_H_ */
