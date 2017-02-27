/*
 * eepspi.c
 *
 *  Created on: Oct 11, 2010
 *      Author: tgorski
 */

#include "pwrmgr.h"

#include "eepspi.h"
#include "utils.h"
#include "intc.h"
#include "timer_callback.h"


#define EEP_READ_CMD					(0x03)
#define EEP_WRITE_CMD					(0x02)
#define EEP_WREN_CMD					(0x06)
#define EEP_RDSR_CMD					(0x05)

#define EEPSPI_Clear_IRQ_Mask			{AVR32_SPI0.idr = 0xffffffff; }


void eepspi_init(void) {
  AVR32_SPI0.mr = 0;						// preset mode reg to all zeros
  AVR32_SPI0.MR.mstr = 1;					// master mode
  AVR32_SPI0.MR.ps = 0;						// fixed peripheral select
  AVR32_SPI0.MR.pcs = 0;					// should select CS0
  AVR32_SPI0.MR.dlybcs = 1;

  AVR32_SPI0.CSR0.cpol = 0;
  AVR32_SPI0.CSR0.ncpha = 1;
  AVR32_SPI0.CSR0.csnaat = 0;				// chip select rise after every transfer
  AVR32_SPI0.CSR0.csaat = 1;				// chip select rises after after last transfer
  AVR32_SPI0.CSR0.bits = 0;					// 8 bit transfers
  AVR32_SPI0.CSR0.scbr = 3;				// at 12MHz PBA clk, have 4 MHz SPCK
  AVR32_SPI0.CSR0.dlybs = 01;				// something reasonable
  AVR32_SPI0.CSR0.dlybct = 00;				// same here

  EEPSPI_Clear_IRQ_Mask;

  AVR32_SPI0.CR.spien = 1;					// enable spi port
}


int eepspi_chk_write_in_progress(void) {
  // reads the eeprom status register.  Returns 0 if no write is in progress, otherwise 1
  unsigned long rdrreg;
  Disable_global_interrupt();
  AVR32_SPI0.tdr = EEP_RDSR_CMD;
  while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
  AVR32_SPI0.tdr = 0;					// dummy value
  AVR32_SPI0.cr = AVR32_SPI_TDR_LASTXFER_MASK;		// signal last byte
  while (!AVR32_SPI0.SR.txempty);		// wait for transmitter completely empty
  rdrreg = AVR32_SPI0.rdr;				// get read data register
  Enable_global_interrupt();
  return (rdrreg & 0x01) ? 1 : 0;		// return value
}


int eepspi_write(const unsigned char* pwbuf, unsigned short saddr, unsigned short len) {
  // subdivide write operation along block boundaries
  // returns 1 if write transfer occurs, 0 otherwise
  // NOTE:  the routine will exit after the last write has been transmitted, but if
  // multiple blocks are to be written in the operation, then the function will poll
  // the write-in-progress indicator, with interrupts being periodically enabled and
  // disabled
  unsigned short cursaddr;
  unsigned short nextsaddr;
  unsigned short remlen;
  unsigned short wrblkremlen;
  const unsigned char* curwptr;
  unsigned long writehaddr;
  unsigned long writeladdr;

  if (eepspi_chk_write_in_progress())
    return 0;								// chip is busy
  if ((len == 0) || (len > EEPSIZE))
    return 0;				// can't handle length
  cursaddr = saddr;
  remlen = len;
  curwptr = pwbuf;
  if ((saddr & EEPBLKMASK) != ((saddr+len-1) & EEPBLKMASK))
    // write crosses block boundaries, so schedule first transfer up to end of current write block
    wrblkremlen = EEPBLKSIZE-(saddr % EEPBLKSIZE);
  else
    wrblkremlen = len;			// entire transfer fits in this block
  nextsaddr = cursaddr + wrblkremlen;		// next block starting address
  remlen -= wrblkremlen;		// remaining length after current block
  writehaddr = (cursaddr >> 8) & 0xff;
  writeladdr = cursaddr & 0xff;

  while (1) {
    // set write enable latch
    Disable_global_interrupt();
	  AVR32_SPI0.tdr = EEP_WREN_CMD;
	  AVR32_SPI0.cr = AVR32_SPI_TDR_LASTXFER_MASK;
	  while (!AVR32_SPI0.SR.txempty);
	  // send write command and starting address
	  AVR32_SPI0.tdr = EEP_WRITE_CMD;
    while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    AVR32_SPI0.tdr = writehaddr;			// upper address bits
    while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    AVR32_SPI0.tdr = writeladdr;			// lower bits
    while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    while (wrblkremlen > 1) {
      AVR32_SPI0.tdr = *curwptr++;
      wrblkremlen--;
      while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    }
    // perform last transfer
    AVR32_SPI0.tdr = *curwptr++;
    AVR32_SPI0.cr = AVR32_SPI_TDR_LASTXFER_MASK;		// signal last byte
	  while (!AVR32_SPI0.SR.txempty);
    Enable_global_interrupt();
    if (!remlen)
      break;							// no more bytes to write
    // calculate size of next block write
    if (remlen < EEPBLKSIZE)
      wrblkremlen = remlen;
    else
      wrblkremlen = EEPBLKSIZE;
    // update control variables and header values for next block write
    cursaddr = nextsaddr;
    nextsaddr = cursaddr + wrblkremlen;		// next block starting address
    remlen -= wrblkremlen;		// remaining length after current block
    writehaddr = (cursaddr >> 8) & 0xff;
    writeladdr = cursaddr & 0xff;
    while (eepspi_chk_write_in_progress()) Service_Watchdog();				// wait while current write transpires
  }
  return 1;
}


int eepspi_read(unsigned char* prbuf, unsigned short saddr, unsigned short len) {
  // returns 1 if read transfer occurs, 0 otherwise
  // If the length is longer than 32 bytes, the read will be broken up into multiple
  // transfers, allowing interrupts to occur in between the 32-byte reads
  unsigned short cursaddr;
  unsigned short nextsaddr;
  unsigned short remlen;
  unsigned short rdblkremlen;
  unsigned char* currptr;
  unsigned long readhaddr;
  unsigned long readladdr;
  unsigned char rdata;
  unsigned char delayflag;


  if (eepspi_chk_write_in_progress())
    return 0;								// chip is busy
  if ((len == 0) || (len > EEPSIZE))
    return 0;				// can't handle length
  cursaddr = saddr;
  remlen = len;
  currptr = prbuf;
  if (remlen >= EEPBLKSIZE)
    rdblkremlen = EEPBLKSIZE;
  else
    rdblkremlen = remlen;
  nextsaddr = cursaddr + rdblkremlen;		// next block starting address
  remlen -= rdblkremlen;		// remaining length after current block
  readhaddr = (cursaddr >> 8) & 0xff;
  readladdr = cursaddr & 0xff;
  delayflag = 1;
  while (1) {
	Disable_global_interrupt();
	// send read command and starting address
	AVR32_SPI0.tdr = EEP_READ_CMD;
    while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    rdata = AVR32_SPI0.rdr;				// unload read data register
	AVR32_SPI0.tdr = readhaddr;			// upper address bits
    while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    rdata = AVR32_SPI0.rdr;				// unload read data register
    AVR32_SPI0.tdr = readladdr;			// lower bits
    while (!AVR32_SPI0.SR.tdre);		// wait for byte to move
    rdata = AVR32_SPI0.rdr;				// unload read data register
    while (rdblkremlen > 1) {
      AVR32_SPI0.tdr = 0;				// dummy write value
      rdblkremlen--;
      while (!AVR32_SPI0.SR.tdre);			// wait for next byte to be available
      if (delayflag)
        delayflag = 0;
      else {
        rdata = AVR32_SPI0.rdr;
        *currptr++ = rdata;
      }
    }
    // perform last transfer
    AVR32_SPI0.tdr = 0;
    AVR32_SPI0.cr = AVR32_SPI_TDR_LASTXFER_MASK;		// signal last byte
    while (!AVR32_SPI0.SR.tdre);
    if (!delayflag) {
      rdata = AVR32_SPI0.rdr;
      *currptr++ = rdata;
    }
	while (!AVR32_SPI0.SR.txempty);
	rdata = AVR32_SPI0.rdr;
	*currptr++ = rdata;
    Enable_global_interrupt();
    if (!remlen)
      break;							// no more bytes to write
    // calculate size of next block read
    if (remlen < EEPBLKSIZE)
      rdblkremlen = remlen;
    else
      rdblkremlen = EEPBLKSIZE;
    // update control variables and header values for next block write
    cursaddr = nextsaddr;
    nextsaddr = cursaddr + rdblkremlen;		// next block starting address
    remlen -= rdblkremlen;		// remaining length after current block
    readhaddr = (cursaddr >> 8) & 0xff;
    readladdr = cursaddr & 0xff;
    delayflag = 1;
  }
  return 1;
}


int eepspi_chip_erase(void) {
  // returns the chip to the erased (0xff in all values) state
  // returns 0 if erase not attempted, otherwise 1
  unsigned short cursaddr;
  unsigned short nextsaddr;
  unsigned short remlen;
  unsigned short wrblkremlen;
  unsigned long writehaddr;
  unsigned long writeladdr;

  if (eepspi_chk_write_in_progress())
    return 0;								// chip is busy
  cursaddr = 0;
  remlen = EEPSIZE;
  wrblkremlen = EEPBLKSIZE;
  nextsaddr = cursaddr + wrblkremlen;		// next block starting address
  remlen -= wrblkremlen;		// remaining length after current block
  writehaddr = (cursaddr >> 8) & 0xff;
  writeladdr = cursaddr & 0xff;

  while (1) {
    // set write enable latch
    Disable_global_interrupt();
	AVR32_SPI0.tdr = EEP_WREN_CMD;
	AVR32_SPI0.cr = AVR32_SPI_TDR_LASTXFER_MASK;
	while (!AVR32_SPI0.SR.txempty);
	// send write command and starting address
	AVR32_SPI0.tdr = EEP_WRITE_CMD;
    while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    AVR32_SPI0.tdr = writehaddr;			// upper address bits
    while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    AVR32_SPI0.tdr = writeladdr;			// lower bits
    while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    while (wrblkremlen > 1) {
      AVR32_SPI0.tdr = 0xff;
      wrblkremlen--;
      while (!AVR32_SPI0.SR.tdre);			// wait for byte to move
    }
    // perform last transfer
    AVR32_SPI0.tdr = 0xff;
    AVR32_SPI0.cr = AVR32_SPI_TDR_LASTXFER_MASK;		// signal last byte
	while (!AVR32_SPI0.SR.txempty);
    Enable_global_interrupt();
    if (!remlen)
      break;							// no more bytes to write
    // calculate size of next block write
    if (remlen < EEPBLKSIZE)
      wrblkremlen = remlen;
    else
      wrblkremlen = EEPBLKSIZE;
    // update control variables and header values for next block write
    cursaddr = nextsaddr;
    nextsaddr = cursaddr + wrblkremlen;		// next block starting address
    remlen -= wrblkremlen;		// remaining length after current block
    writehaddr = (cursaddr >> 8) & 0xff;
    writeladdr = cursaddr & 0xff;
    while (eepspi_chk_write_in_progress()) Service_Watchdog();				// wait while current write transpires
  }
  return 1;
}


