/*
 * CTP7_SPI_addrs.h
 *
 * Created: 5/30/2015 11:02:41 AM
 *  Author: tgorski
 */ 

// Collection point for addresses in ZYNQ-based shared 4KB memory, accessed from MMC via SPI interface

#ifndef CTP7_SPI_ADDRS_H_
#define CTP7_SPI_ADDRS_H_

/*
000     000     slotid
001     004     IP
005     008     NM
009     00c     GW
00d     010     Log Server
011     012     bootvec

080     080     sensorsvc: Update Flag
081     081     sensorsvc: Valid Mask
082     082     sensorsvc: ch0 Zynq Die Temp
083     083     sensorsvc: ch1 Zynq 1V Value
084     084     sensorsvc: ch2 V7 Die Temp
085     085     sensorsvc: ch3 V7 1V Value
086     088     sensorsvc: ch4-7 Unallocated

f00     f00     quiesce command (1: quiesce now, 0: quiesce ack)
f01     f01     quiesce confirm (1: quiesce done, 0: quiesce done ack)

f10     f16     MAC Address Area (format: { 0x01, MAC48 })

ffc     fff     Peripheral Validation Area

*/

#define IP_ADDRESS_SPI_ADDR           (0x0001)                            // address in the configuration vector

// payload (ZYNQ) based sensor
#define PBS_RECORD_SPI_ADDRESS        (0x0080)
#define PBS_RECORD_SPI_LENGTH         (10)

// ZYNQ SPI memory addresses for Quiesce Interlock
#define PAYLDMGR_MMC_SPI_QUIESCE_CMD_FLAG_ADDR       (0x0f00)
#define PAYLDMGR_MMC_SPI_QUIESCED_FLAG_ADDR          (0x0f01)

#define MAC_ADDRESS_SPI_ADDR          (0x0f10)

#define PERIPH_VALIDATION_SPI_ADDR    (0x0ffc)
#define PERIPH_VALIDATION_SPI_LEN     (4)


#endif /* CTP7_SPI_ADDRS_H_ */