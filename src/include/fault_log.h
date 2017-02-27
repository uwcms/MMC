/*
 * fault_log.h
 *
 * Created: 11/12/2012 1:14:07 PM
 *  Author: tgorski
 */ 


#ifndef FAULT_LOG_H_
#define FAULT_LOG_H_

#include "nonvolatile.h"

typedef struct {
	unsigned long systime;            // system time for fault
	unsigned char sensor_num;          // sensor number
	unsigned char event_offset;       // sensor event offset
	unsigned char sensor_val;         // raw 8-bit sensor value
	unsigned char thresh_val;         // raw 8-bit threshold value
} fault_log_entry_t;


typedef struct {
	unsigned long entry_count;        // total number of entries made
	unsigned char next_entry_idx;     // index for next entry
	unsigned char filler[3];
} fault_log_hdr_t;                  // should have length of 8 bytes


void fault_log_init(void);

void write_fault_log_entry(const fault_log_entry_t* pwrentry);

int fault_log_get_last_entry_index(void);

void get_fault_log_entry(const int entry_num, fault_log_entry_t* prdentry);

void erase_fault_log();

#endif /* FAULT_LOG_H_ */