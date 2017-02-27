/*
 * fault_log.c
 *
 * Created: 11/12/2012 1:34:51 PM
 *  Author: tgorski
 */ 

#include <stdio.h>
#include <string.h>

#include "sio_usart.h"
#include "utils.h"
#include "eepspi.h"
#include "fault_log.h"
#include "sio_usart.h"

fault_log_hdr_t log_hdr;            // static object that contains header information for fault log


void fault_log_init(void) {
  while (eepspi_chk_write_in_progress());			// wait for EEPROM to be available
  eepspi_read((unsigned char*) &log_hdr, FAULT_LOG_HDR_BYTE_OFFSET, sizeof(fault_log_hdr_t));
	if (log_hdr.entry_count == 0xffffffff) {
		// have uninitialized fault log, set entry count and next index both to 0
		log_hdr.entry_count = 0;
		log_hdr.next_entry_idx = 0;
		eepspi_write((unsigned char*) &log_hdr, FAULT_LOG_HDR_BYTE_OFFSET, sizeof(fault_log_hdr_t));
	}
}


void write_fault_log_entry(const fault_log_entry_t* pwrentry) {
	int wraddr = FAULT_LOG_ENTRY_BYTE_OFFSET;
	
	wraddr += log_hdr.next_entry_idx*FAULT_LOG_ENTRY_SIZE;        // advance to proper address
  while (eepspi_chk_write_in_progress()) Service_Watchdog();			// wait for EEPROM to be available	
	eepspi_write((const unsigned char*) pwrentry, wraddr, sizeof(fault_log_entry_t));
	log_hdr.entry_count++;
	if (log_hdr.next_entry_idx >= FAULT_LOG_ENTRY_CNT-1)
	  log_hdr.next_entry_idx = 0;         // wraparound
	else
	  log_hdr.next_entry_idx++;
	// update the header
	while (eepspi_chk_write_in_progress()) Service_Watchdog();
	eepspi_write((unsigned char*) &log_hdr, FAULT_LOG_HDR_BYTE_OFFSET, sizeof(fault_log_hdr_t));
}


int fault_log_get_last_entry_index(void) {
  // returns -1 if there are no entries, otherwise returns the index of the last written fault log entry
  if (!log_hdr.entry_count)
    return -1;
	if (!log_hdr.next_entry_idx)
	  return FAULT_LOG_ENTRY_CNT-1;           // wrap around to end
	return log_hdr.next_entry_idx-1;          // return previous
}


void get_fault_log_entry(const int entry_num, fault_log_entry_t* prdentry) {
	int rdaddr = FAULT_LOG_ENTRY_BYTE_OFFSET;
	
	if ((entry_num < 0) || (entry_num >= FAULT_LOG_ENTRY_CNT))
	  return;     // entry number out of range
	rdaddr += entry_num*FAULT_LOG_ENTRY_SIZE;
  while (eepspi_chk_write_in_progress()) Service_Watchdog();			// wait for EEPROM to be available	
	eepspi_read((unsigned char*) prdentry, rdaddr, sizeof(fault_log_entry_t));
}


void erase_fault_log() {
	unsigned char erase_buf[EEPBLKSIZE];
	int wrlen, remwrlen, wraddr;
	
	remwrlen = FAULT_LOG_SIZE;
	wraddr = FAULT_LOG_HDR_BYTE_OFFSET;
	memset((void*) &erase_buf[0], 0xff, EEPBLKSIZE);

	// write all of the fault log entry bytes to 0xff, except for the header, which points to next index 0x00
  while (eepspi_chk_write_in_progress()) Service_Watchdog();			// wait for EEPROM to be available
  
  while (remwrlen) {
    if (remwrlen > EEPBLKSIZE)
	    wrlen = EEPBLKSIZE;
		else
		  wrlen = remwrlen;
		eepspi_write(&erase_buf[0], wraddr, wrlen);
		wraddr += wrlen;
		remwrlen -= wrlen;
  }
}

