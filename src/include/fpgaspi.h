/*
 * fpgaspi.h
 *
 *  Created on: Nov 5, 2010
 *      Author: tgorski
 */

#ifndef FPGASPI_H_
#define FPGASPI_H_


// commands implemented by SPI slave in FPGA
#define READ_DATA_CMD                 (0x03)
#define WRITE_DATA_CMD                (0x02)
#define READ_STATUS_CMD               (0x05)
#define WRITE_CTL_CMD                 (0x07)

#define MAX_FPGA_ID                   (0x02)          // max FPGA device ID (0-3)

#define MAX_FPGA_MEM_SIZE             (4096)          // size of core in CTP7

// FPGA SPI detect mask bits returned by fpgaspi_slave_detect call
#define FPGA3_SPI_DETECT_MASK         (0x08)
#define FPGA2_SPI_DETECT_MASK         (0x04)
#define FPGA1_SPI_DETECT_MASK         (0x02)
#define FPGA0_SPI_DETECT_MASK         (0x01)

// status register bits (read)
#define FPGASPI_SR_UHF                (0x80)
#define FPGASPI_SR_LHF                (0x40)
#define FPGASPI_SR_CFGRDY             (0x20)
#define FPGASPI_SR_REQCFG             (0x10)
#define FPGASPI_SR_LINUX_BOOT_ACTIVE  (0x04)

// control register bits (write)
#define FPGASPI_HFR_USTB              (0x80)
#define FPGASPI_HFR_LSTB              (0x40)
#define FPGASPI_HFR_CFGRDY            (0x20)
#define FPGASPI_HFR_SETFLG            (0x02)
#define FPGASPI_HFR_CLRFLG            (0x01)

extern unsigned long spi_detect_fail2_ctr;

// functions supported by fpga spi driver
unsigned char fpgaspi_slave_detect(void);

int fpgaspi_data_write(const unsigned char deviceID, const unsigned char* pwbuf, unsigned short saddr, unsigned short len);

int fpgaspi_data_read(const unsigned char deviceID, unsigned char* prbuf, unsigned short saddr, unsigned short len);

unsigned char fpgaspi_status_read(const unsigned char deviceID);

int fpgaspi_ctl_write(const unsigned char deviceID, const unsigned char wrval);




#endif /* FPGASPI_H_ */
