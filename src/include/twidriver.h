/*
 * twidriver.h
 *
 *  Created on: Jul 8, 2010
 *      Author: tgorski
 */

#ifndef TWIDRIVER_H_
#define TWIDRIVER_H_


#define TWI_IRQ_PRIORITY	  	AVR32_INTC_INT2
#define TWI_IRQ					AVR32_TWI_IRQ

// settings to get 100kHZ master mode from 12000000 master timebase
#define TWI_CWGR_CKDIV			(3)
#define TWI_CWGR_CHDIV			(7)
#define TWI_CWGR_CLDIV			(7)
#define TWI_CWGR_100KHZ			((TWI_CWGR_CKDIV << 16) | (TWI_CWGR_CHDIV << 8) | TWI_CWGR_CLDIV)

enum TWISTATE {disabled = 0, slave_listen, slave_write, slave_read, master_write, failed};
enum GApin {Gpin = 0, Upin, Ppin};

typedef struct {
  enum TWISTATE state;
  //short xfer_cnt;
  unsigned char master_holdoff_cnt;
  unsigned char ipmbl_addr;
  unsigned char slotid;
  enum GApin ga0val;
  enum GApin ga1val;
  enum GApin ga2val;
  unsigned char NACKretrycnt;
} twi_state_record_t;

typedef struct {
  unsigned long rxmsg_cnt;            // received message count
  unsigned long txmsg_cnt;            // transmitted message count
  unsigned long txarblost_cnt;          // loss of arbitration counter
  unsigned long txmsgretried_cnt;     // tx messages retried
  unsigned long txmsgfailed_cnt;      // tx messages failed
} IPMB_stat_rec_t;

extern twi_state_record_t twi_state;

extern IPMB_stat_rec_t IPMB_stats;

extern void twi_init(void);

extern unsigned int get_twi_rcv_msg_cnt(void);

extern short get_twi_rcv_next_msg_len(void);

extern unsigned int get_twi_msg(unsigned char* rbuf, unsigned short* rlen, unsigned short bufsize);

extern unsigned int put_twi_msg(unsigned char* xbuf, unsigned short xlen);

extern unsigned int chk_twi_xbuf_space(void);

extern __inline__ void twi_start_slave_listen(void);

extern void print_twi_driver_stats(void);


#endif /* TWIDRIVER_H_ */
