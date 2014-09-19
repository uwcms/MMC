/*
 * twidriver.c
 *
 *  Created on: Jul 8, 2010
 *      Author: tgorski
 */

#include <avr32/io.h>
#include <stdio.h>
#include "pins.h"
#include "gpio.h"
#include "utils.h"
#include "intc.h"
#include "pwrmgr.h"
#include "timer_callback.h"
#include "twidriver.h"
#include "sio_usart.h"

// putting this here so GPIO code will run--maybe better put somewhere else?
//#define GPIO  AVR32_GPIO


#define MASTER_HOLDOFF_DELAY			(2)		// number of 200usec ticks to hold off master mode on loss-of-arbitration
#define NAK_RETRY_CNT             (3)   // number of times to retry transmission of a NAK'd message before dropping it

#define adv_buf_idx(i,b)				{ i = (b-1-i) ? i+1 : 0; }

twi_state_record_t twi_state;


#define EORBUFSIZE		(8)				// number of elements in end-of-record buffers

// storage for xmt and rcv queue end-of-record buffers
short eor_xbuf[EORBUFSIZE];
short eor_rbuf[EORBUFSIZE];
short eor_xcnt = 0;
short eor_xwidx = 0;
short eor_xridx = 0;
short eor_rcnt = 0;
short eor_rwidx = 0;
short eor_rridx = 0;

#define TWIBUFSIZE	(128)				// number of bytes in TWI transmit and receive buffers

// storage for twi xmt and rcv data buffers
unsigned char twi_xbuf[TWIBUFSIZE];
unsigned char twi_rbuf[TWIBUFSIZE];
short twi_xcnt = 0;
short twi_xwidx = 0;
short twi_xridx = 0;
short twi_rcnt = 0;
short twi_rwidx = 0;
short twi_rridx = 0;
short twi_xfirst, twi_xlast, twi_xcur, twi_xmtcnt;		// index variables for transmitting message off of the queue

typedef struct {
  enum GApin ga2;
  enum GApin ga1;
  enum GApin ga0;
  unsigned char ipmb_addr;
  unsigned char slotid;
} ga_decode_tbl_entry_t;


typedef struct {
	unsigned long rxmsg_cnt;            // received message count
	unsigned long txmsg_cnt;            // transmitted message count
	unsigned long arblost_cnt;          // loss of arbitration counter
	unsigned long txNACK_cnt;           // tx attempts receiving NAK
	unsigned long txmsglost_cnt;        // transmit messages lost due to repeated NAKs
	unsigned long slave_access_cnt;     // number of times bus went into active slave receiver mode
	unsigned long master_access_cnt;    // number of times bus went into active master mode
	unsigned long master_acc_defer;     // number of times master access is deferred (slave busy)
} IPMB_stat_rec_t;

IPMB_stat_rec_t IPMB_stats = {0, 0, 0, 0, 0, 0, 0, 0};

#define GA_DECODE_TBL_SIZE			(27)

const ga_decode_tbl_entry_t ga_decode_tbl[GA_DECODE_TBL_SIZE] = {
		{Gpin, Gpin, Gpin, 0x70, 0xff},
		{Gpin, Gpin, Upin, 0x72, 0x01},
		{Gpin, Upin, Gpin, 0x74, 0x02},
		{Gpin, Upin, Upin, 0x76, 0x03},
		{Upin, Gpin, Gpin, 0x78, 0x04},
		{Upin, Gpin, Upin, 0x7a, 0x05},
		{Upin, Upin, Gpin, 0x7c, 0x06},
		{Upin, Upin, Ppin, 0x7e, 0x07},
		{Upin, Ppin, Upin, 0x80, 0x08},
		{Upin, Ppin, Ppin, 0x82, 0x09},
		{Ppin, Upin, Upin, 0x84, 0x0a},
		{Ppin, Upin, Ppin, 0x86, 0x0b},
		{Ppin, Ppin, Upin, 0x88, 0x0c},
		{Gpin, Gpin, Ppin, 0x8a, 0xff},
		{Gpin, Upin, Ppin, 0x8c, 0xff},
		{Gpin, Ppin, Gpin, 0x8e, 0xff},
		{Gpin, Ppin, Upin, 0x90, 0xff},
		{Gpin, Ppin, Ppin, 0x92, 0xff},
		{Upin, Gpin, Ppin, 0x94, 0xff},
		{Upin, Ppin, Gpin, 0x96, 0xff},
		{Ppin, Gpin, Gpin, 0x98, 0xff},
		{Ppin, Gpin, Upin, 0x9a, 0xff},
		{Ppin, Gpin, Ppin, 0x9c, 0xff},
		{Ppin, Upin, Gpin, 0x9e, 0xff},
		{Ppin, Ppin, Gpin, 0xa0, 0xff},
		{Upin, Upin, Upin, 0xa2, 25},
		{Ppin, Ppin, Ppin, 0xa4, 26} };


__inline__ void put_xmt_eor(short eor_loc);
__inline__ short peek_xmt_next_eor(void);
__inline__ void delete_xmt_eor(void);
__inline__ void put_rcv_eor(short eor_loc);
__inline__ short peek_rcv_next_eor(void);
__inline__ void delete_rcv_eor(void);


__inline__ void put_xmt_byte(unsigned char val);
__inline__ unsigned char get_xmt_byte(void);
__inline__ void put_rcv_byte(unsigned char val);
__inline__ unsigned char get_rcv_byte(void);

void unload_xmt_msg(void);
void twi_start_master_mode_write(void);
//int OK_to_enter_master_mode(void);
void check_twi_state(int new_enqueue);


#define TWI				(AVR32_TWI)							// structure mapped to twi in address space


unsigned int chk_twi_rbuf_space(void);
void chk_xmt_start(void);
short get_twi_xmt_next_msg_len(void);

int twi200uscallback(event evID, void* arg);
void get_ipmb_address(unsigned char* ipmbl_addr, unsigned char* slotid);

#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void twi_int_handler(void) {
  short tempsh;
  unsigned char curbyte;
  unsigned int status, mask;
  unsigned int databyte;
  status = TWI.sr;
  mask = TWI.imr;

  // check each interrupt type by bit

  // START OF SLAVE ACCESS
  if ((status & AVR32_TWI_SR_SVACC_MASK) && (mask & AVR32_TWI_SR_SVACC_MASK)) {
	  // start of slave access
	  IPMB_stats.slave_access_cnt++;
	  TWI.idr = AVR32_TWI_SR_SVACC_MASK;				// turn off start-of-slave-access irq
	  if (status & AVR32_TWI_SR_SVREAD_MASK) {
	    // invalid read access from master--send all one's back
	    twi_state.state = slave_read;
	    TWI.thr = 0xff;
      TWI.ier = AVR32_TWI_SR_TXRDY_MASK | AVR32_TWI_SR_EOSACC_MASK;
												  // will need TXRDY irqs going forward to service this slave read
	  }
	  else {
	    // start of write access, arm rxrdy interrupt
	    twi_state.state = slave_write;
	    TWI.ier = AVR32_TWI_SR_RXRDY_MASK | AVR32_TWI_SR_EOSACC_MASK;
	    put_rcv_byte(twi_state.ipmbl_addr);		// put slave address at front of message as rsSA field of message
	    //trigger_LED_pulse(_GP_LED0);
	  }
	  return;
  }

  // SLAVE ACCESS, CONTINUING WRITE ACCESS
  if ((status & (AVR32_TWI_SR_RXRDY_MASK | AVR32_TWI_SR_SVACC_MASK)) ==
	  (AVR32_TWI_SR_RXRDY_MASK | AVR32_TWI_SR_SVACC_MASK)) {
    databyte = TWI.rhr;
    put_rcv_byte(databyte);						// put received byte in queue
    return;
  }

  // SLAVE ACCESS, BOGUS READ MODE, THR REGISTER EMPTY
  if ((status & (AVR32_TWI_SR_TXRDY_MASK | AVR32_TWI_SR_SVACC_MASK)) ==
	  (AVR32_TWI_SR_TXRDY_MASK | AVR32_TWI_SR_SVACC_MASK)) {
	  // slave access with reads!
	  if (!(status & AVR32_TWI_SR_NACK_MASK))
	    // master sent ACK, so provide another dummy byte
	    TWI.thr = 0xff;
	  return;
  }

  // END-OF-SLAVE-ACCESS
  if ((status & AVR32_TWI_SR_EOSACC_MASK) && (mask & AVR32_TWI_SR_EOSACC_MASK)) {
	  TWI.idr = AVR32_TWI_SR_EOSACC_MASK;				// disable end-of-slave-access irq
	  if (twi_state.state == slave_write) {
	    // add end-of-record entry, marking end of received message (back up one spot to get last byte index)
	    if (twi_rwidx)
		    tempsh = twi_rwidx-1;
	    else
		    tempsh = TWIBUFSIZE-1;
	    put_rcv_eor(tempsh);							// put index of last message byte in end-of-record queue
		  IPMB_stats.rxmsg_cnt++;
	  }
	  TWI.ier = AVR32_TWI_SR_TXCOMP_MASK;				// enable transmit complete interrupt
    return;
  }

  // NACK received, master mode only
  if ((status & AVR32_TWI_SR_NACK_MASK) && (mask & AVR32_TWI_SR_NACK_MASK)) {
	  // transmitted byte was NACK'ed at far end.  Shut down the transmit, and check the retry count
	  // to determine if the same message should be retried or flushed.  If it is to be retried, then it
	  // is left on the front of the queue for the next time the driver cycles into master mode
	  IPMB_stats.txNACK_cnt++;
	  twi_state.NACKretrycnt++;
	  if (twi_state.NACKretrycnt == NAK_RETRY_CNT) {
		  // flush message
	    unload_xmt_msg();								  // removed failed message from transmit queue
		  IPMB_stats.txmsglost_cnt++;
		  twi_state.NACKretrycnt = 0;       // reset retry count
	  }		
    twi_start_slave_listen();						// switch back to slave mode for awhile
	  return;
  }


  // transmit complete, either master or slave mode
  if ((status & AVR32_TWI_SR_TXCOMP_MASK) && (mask & AVR32_TWI_SR_TXCOMP_MASK)) {
	  TWI.idr = AVR32_TWI_SR_TXCOMP_MASK;				// disable tx complete interrupt
	  if (twi_state.state == master_write) {
	    unload_xmt_msg();								      // removed completed message from transmit queue
		  IPMB_stats.txmsg_cnt++;
	  }
	  // at this point it may be advantageous to include some idle time between the completion of a slave access
	  // and a switch to master mode write operations.  So set the holdoff delay if the device was in slave mode
	  // and there is a message pending in the transmit queue.  If the device was already in master mode, immediately
	  // start the next master mode write operation
/*	  if (eor_xcnt) {
		  if (twi_state.state == master_write) {
		    twi_start_master_mode_write();
			  return;
		  }			
			else
			  twi_state.master_holdoff_cnt = MASTER_HOLDOFF_DELAY;
	  }
	  twi_start_slave_listen();		*/
    if (eor_xcnt && (twi_state.state == master_write)) 
			twi_start_master_mode_write();
		else
			twi_start_slave_listen();
    return;
  }

  // LOSS-OF-ARBITRATION, MASTER MODE WRITE ACCESS
  if (status & AVR32_TWI_SR_ARBLST_MASK) {
	  // master loss of arbitration interrupt
	  IPMB_stats.arblost_cnt++;
    twi_state.master_holdoff_cnt = MASTER_HOLDOFF_DELAY;
    twi_start_slave_listen();						// switch back to slave mode for awhile
    return;
  }

  // MASTER MODE WRITE ACCESS, TRANSMITTING REGISTER EMPTY
  if ((status & AVR32_TWI_SR_TXRDY_MASK) && (~status & AVR32_TWI_SR_SVACC_MASK)) {
    // continue sending message, unload when complete
    if (twi_xmtcnt) {
      // still some bytes to transmit
   	  curbyte = twi_xbuf[twi_xcur];				// get first data byte
   	  adv_buf_idx(twi_xcur, TWIBUFSIZE);
   	  twi_xmtcnt--;								// decrement transmit count
   	  TWI.thr = curbyte;						// write first data byte--this should start TWI transfer
    }
    else {
      // nothing more to send, arm the transmit complete interrupt
      TWI.ier = AVR32_TWI_SR_TXCOMP_MASK;
    }
    return;
  }

  // if we get to this point it is because the state machine and interrupt masks are significantly corrupted
  twi_state.state = failed;						// mark state machine state
  TWI.cr = 0x28;								// master and slave modes disabled
  TWI.idr = 0xffff;								// turn off all interrupts
}


extern void twi_init(void) {
  twi_state.master_holdoff_cnt = 0;

  //twi_state.xfer_cnt = 0;
  get_ipmb_address(&(twi_state.ipmbl_addr), &(twi_state.slotid));

  // do general initialization of twi, but leave in disabled state
  twi_state.state = disabled;
  TWI.cwgr = TWI_CWGR_100KHZ;					// set master clock frequency
  TWI.cr = 0x28;								// master and slave modes disabled
  twi_state.NACKretrycnt = 0;   // initialize retry count

  register_timer_callback(twi200uscallback, TEVENT_200USEC);			// register callback for 200usec service routine
  INTC_register_interrupt(&twi_int_handler, TWI_IRQ, TWI_IRQ_PRIORITY);
}


extern unsigned int get_twi_rcv_msg_cnt(void) {
  return (unsigned int) eor_rcnt;
}


extern short get_twi_rcv_next_msg_len(void) {
  // examines next message on rcv queue and determines its length
  // returns 0 if there are no complete messages on queue
  short eoridx;
  if (!eor_rcnt)
	return 0;
  eoridx = eor_rbuf[eor_rridx];
  // calculate length of msg on queue
  if (twi_rridx > eoridx)
     return TWIBUFSIZE + 1 + eoridx - twi_rridx;			// message wraps around in twi buffer
  else
	return eoridx + 1 - twi_rridx;
}


short get_twi_xmt_next_msg_len(void) {
  // examines next message on xmt queue and determines its length
  // returns 0 if there are no complete messages on queue
  short eoridx;
  short retval;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  if (!eor_xcnt)
	retval = 0;
  else {
    eoridx = eor_xbuf[eor_xridx];
    // calculate length of msg on queue
    if (twi_xridx > eoridx)
      retval = TWIBUFSIZE + 1 + eoridx - twi_xridx;			// message wraps around in twi buffer
    else
	  retval = eoridx + 1 - twi_xridx;
  }
  Int_Restore(giflag);
  return retval;
}


extern unsigned int get_twi_msg(unsigned char* rbuf, unsigned short* rlen, unsigned short bufsize) {
  // check receive message queue, and copy oldest message to the supplied buffer
  // returns 1 if message is dequeued and copied, and 0 if there is no message
  // NOTE:  if the message is longer than the buffer, then function dequeues the
  //   *entire* message from the buffer, but only copies as many bytes as indicated
  //   by the 'bufsize' argument.  Remaining bytes are discarded.  This should cause
  //   checksum errors in the message
  short msglen, getcnt, copycnt;
  unsigned char* bptr;

  check_twi_state(0);                              // chance to launch any pending outgoing transmit messages
  
  msglen = get_twi_rcv_next_msg_len();						// get length of next message in buffer
  
  if (!msglen)
    // nothing to get
	  return 0;
    
  getcnt = 0;
  bptr = rbuf;
  // figure number of bytes to copy to buffer
  if (msglen > bufsize)
	  copycnt = bufsize;
  else
	  copycnt = msglen;
  while (getcnt < copycnt) {
	  // copy characters to buffer
	  *bptr++ = get_rcv_byte();
	  getcnt++;
  }
  *rlen = getcnt;
  while (getcnt < msglen)
	  // flush excess characters if any
    get_rcv_byte();

  delete_rcv_eor();											// remove entry from eor queue--this drops the rcv msg cnt
  return 1;
}

void unload_xmt_msg(void) {
  // this function unloads the next transmit message from the front of the transmit queue
  // this is done by updating the index variables and count
  short eor_val;
  Bool giflag = Is_global_interrupt_enabled();
  if (!eor_xcnt)
	  // no complete message to unload
	  return;
  Int_Disable(giflag);
  twi_xcnt -= get_twi_xmt_next_msg_len();	// decrement count by length of message
  eor_val = peek_xmt_next_eor();			// get end-of-record mark from xmt queue
  delete_xmt_eor();							// remove end-of-record mark from queue
  twi_xridx = eor_val;
  adv_buf_idx(twi_xridx, TWIBUFSIZE);
  Int_Restore(giflag);
}

unsigned int put_twi_msg(unsigned char* xbuf, unsigned short xlen) {
  // function returns 1 if message is copied to end of transmit queue,
  // otherwise returns 0
  short putcnt;
  unsigned char* bptr;
  Bool giflag = Is_global_interrupt_enabled();
  
  // check for space in transmit queue
  if (xlen > chk_twi_xbuf_space())
	// not enough space, abort put
	return 0;

  Int_Disable(giflag);
  bptr = xbuf;
  putcnt = 0;
  while (putcnt < xlen-1) {
	  // put all but last character on queue
	  put_xmt_byte(*bptr);
	  bptr++;
	  putcnt++;
  }
  // write current value of xmt queue write index to transmit eor queue, then put the last character
  put_xmt_eor(twi_xwidx);			// this is index of last byte to be written to queue
  put_xmt_byte(*bptr);

  check_twi_state(1);          // transmit will start if bus is available

  Int_Restore(giflag);
  return 1;
}


unsigned int chk_twi_xbuf_space(void) {
  // function returns the amount of space in bytes currently available in transmit buffer
  unsigned int retval;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  if (twi_xwidx >= twi_xridx)
	  retval = TWIBUFSIZE-(twi_xwidx-twi_xridx);
  else
	  retval = twi_xridx-twi_xwidx;
  Int_Restore(giflag);
  return retval;
}


unsigned int chk_twi_rbuf_space(void) {
  // function returns the amount of space in bytes currently available in receive buffer
  unsigned int retval;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  if (twi_rwidx >= twi_rridx)
	  retval = TWIBUFSIZE-(twi_rwidx-twi_rridx);
  else
	  retval = twi_rridx-twi_rwidx;
  Int_Restore(giflag);
  return retval;
}


__inline__ void put_xmt_eor(short eor_loc) {
  Bool giflag;
  if (eor_xcnt >= EORBUFSIZE-1)
	// no room on queue
	return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  eor_xbuf[eor_xwidx] = eor_loc;
  eor_xcnt++;
  adv_buf_idx(eor_xwidx, EORBUFSIZE);
  Int_Restore(giflag);
}


__inline__ short peek_xmt_next_eor(void) {
  short retval;
  Bool giflag;
  if (!eor_xcnt)
	// nothing in queue
	return -1;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  retval = eor_xbuf[eor_xridx];
  Int_Restore(giflag);
  return retval;

}


__inline__ void delete_xmt_eor(void) {
  Bool giflag;
  if (!eor_xcnt)
	// nothing in queue
	return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  eor_xcnt--;
  adv_buf_idx(eor_xridx, EORBUFSIZE);
  Int_Restore(giflag);
}


__inline__ void put_rcv_eor(short eor_loc) {
  Bool giflag;
  if (eor_rcnt >= EORBUFSIZE-1)
	// no room on queue
	return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  eor_rbuf[eor_rwidx] = eor_loc;
  eor_rcnt++;
  adv_buf_idx(eor_rwidx, EORBUFSIZE);
  Int_Restore(giflag);
}


__inline__ short peek_rcv_next_eor(void) {
  if (!eor_rcnt)
	// nothing in queue
	return -1;
  return eor_rbuf[eor_rridx];
}


__inline__ void delete_rcv_eor(void) {
  Bool giflag;
  if (!eor_rcnt)
	// nothing in queue
	return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  eor_rcnt--;
  adv_buf_idx(eor_rridx, EORBUFSIZE);
  Int_Restore(giflag);
}


__inline__ void put_xmt_byte(unsigned char val) {
  Bool giflag;
  if (twi_xcnt == TWIBUFSIZE)
	// no room on queue
	return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  twi_xbuf[twi_xwidx] = val;
  twi_xcnt++;
  adv_buf_idx(twi_xwidx, TWIBUFSIZE);
  Int_Restore(giflag);
}


__inline__ unsigned char get_xmt_byte(void) {
  unsigned char retval;
  Bool giflag;
  if (!twi_xcnt)
	// nothing in queue
	return -1;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  retval = twi_xbuf[twi_xridx];
  twi_xcnt--;
  adv_buf_idx(twi_xridx, TWIBUFSIZE);
  Int_Restore(giflag);
  return retval;
}


__inline__ void put_rcv_byte(unsigned char val) {
  Bool giflag;
  if (twi_rcnt == TWIBUFSIZE)
	// no room on queue
	return;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  twi_rbuf[twi_rwidx] = val;
  twi_rcnt++;
  adv_buf_idx(twi_rwidx, TWIBUFSIZE);
  Int_Restore(giflag);
}


__inline__ unsigned char get_rcv_byte(void) {
  unsigned char retval;
  Bool giflag;
  if (!twi_rcnt)
	// nothing in queue
	return -1;
  giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  retval = twi_rbuf[twi_rridx];
  twi_rcnt--;
  adv_buf_idx(twi_rridx, TWIBUFSIZE);
  Int_Restore(giflag);
  return retval;
}


void twi_start_slave_listen(void) {
  // configure TWI for slave mode listen
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  twi_state.state = slave_listen;

  //TWI.cr = AVR32_TWI_CR_SWRST_MASK;
  //TWI.sr;
  //TWI.cwgr = TWI_CWGR_100KHZ;					// set master clock frequency
  TWI.cr = (AVR32_TWI_CR_MSDIS_MASK | AVR32_TWI_CR_SVDIS_MASK);					// disable slave mode, disable master mode
  TWI.idr = 0xffff;							// disable all interrupts in mask
  TWI.sr;									// dummy status register read
  TWI.smr = twi_state.ipmbl_addr << (AVR32_TWI_SMR_SADR_OFFSET-1);

  TWI.cr = AVR32_TWI_CR_SVEN_MASK | AVR32_TWI_CR_MSDIS_MASK;					// enable slave mode
  TWI.ier = AVR32_TWI_SR_SVACC_MASK;											// enable slave access interrupt

  Int_Restore(giflag);
}


void check_twi_state(int new_enqueue) {
  // function intended to be called from outside the TWI service ISR.  It checks the current state of the TWI state machine
  // and the transmit queue, starting a master mode write if the bus is available and there is a pending message in the queue
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  if ((eor_xcnt > 0) && (twi_state.master_holdoff_cnt == 0)) {
	  if (twi_state.state == slave_listen)
	    twi_start_master_mode_write();
	  else {
		  if (new_enqueue)
	      IPMB_stats.master_acc_defer++;
	  }		  
  }	
  Int_Restore(giflag);
}


/*int OK_to_enter_master_mode(void) {
  // function returns 1 if it is OK to switch into master mode and begin a write operation
  // returns 0 otherwise
  int retval;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);

  if ((twi_state.state == slave_listen) && (twi_state.master_holdoff_cnt == 0) && (eor_xcnt > 0))
	  retval = 1;
  else
	  retval = 0;
  Int_Restore(giflag);
  return retval;
}
*/

void twi_start_master_mode_write(void) {
  // function starts a master mode write
  // IMPORTANT--function assumes all necessary checks (twi state, xmt queue, holdoff ctr) have been
  // performed as necessary *before* making this call
  unsigned char curbyte;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);

  if (TWI.sr & AVR32_TWI_SR_SVACC_MASK) {
    // slave access started, do not attempt switch to master mode
		IPMB_stats.master_acc_defer++;
	  return;
  }	
	  
  TWI.cr = (AVR32_TWI_CR_MSDIS_MASK | AVR32_TWI_CR_SVDIS_MASK);					// disable master mode
  IPMB_stats.master_access_cnt++;
  twi_state.state = master_write;					// update state machine state

  twi_xfirst = twi_xridx;					// set first index of message at tail of queue
  twi_xmtcnt = get_twi_xmt_next_msg_len();	// set count to message length
  twi_xlast = peek_xmt_next_eor();			// set last index at the end-of-record
  twi_xcur = twi_xfirst;					// set current index variable

  TWI.sr;									// dummy status register read
  curbyte = twi_xbuf[twi_xcur];				// get address byte
  adv_buf_idx(twi_xcur, TWIBUFSIZE);
  twi_xmtcnt--;								// decrement transmit count

  TWI.mmr = curbyte << (AVR32_TWI_MMR_DADR_OFFSET-1);			// set destination address (write operation)
  TWI.cr = AVR32_TWI_CR_MSEN_MASK;			// enable master mode
  TWI.idr = 0xffff;							// disable all interrupts in mask
  TWI.ier = AVR32_TWI_SR_ARBLST_MASK | AVR32_TWI_SR_TXRDY_MASK | AVR32_TWI_SR_NACK_MASK;
											// enable loss of arbitration and txrdy interrupts
  curbyte = twi_xbuf[twi_xcur];				// get first data byte
  adv_buf_idx(twi_xcur, TWIBUFSIZE);
  twi_xmtcnt--;								// decrement transmit count
  TWI.thr = curbyte;						// write first data byte--this should start TWI transfer
  Int_Restore(giflag);
}


int twi200uscallback(event evID, void* arg) {
  // 200usec interval callback for twi driver
  // This callback is used to provide a holdoff time in which the twi interface
  // remains in slave mode, after a master mode loss-of-arbitration event.
  // The switch back to master mode for a retransmit will be initiated the next time the TWI queue is checked for Rx messages
  if (twi_state.master_holdoff_cnt) {
	  twi_state.master_holdoff_cnt--;
  }
  return 1;
}

#define P1DELAYCNT		(1000)				// for loop interations waiting for P1 assertion to take effect

void get_ipmb_address(unsigned char* ipmbl_addr, unsigned char* slotid) {
  int i1;
  unsigned char ga0t0, ga1t0, ga2t0, ga0t1, ga1t1, ga2t1;
  //enum GApin ga0val, ga1val, ga2val;
  unsigned long readpvr;
  const ga_decode_tbl_entry_t* pdectbl;
  volatile avr32_gpio_port_t *gpio_port = &GPIO.port[0];

  // enable P1 line for output
  gpio_port->oders = 1 << (IPMI_P1 & 0x1F); // The GPIO output driver is enabled for that pin.
  gpio_port->gpers = 1 << (IPMI_P1 & 0x1F); // The GPIO module controls that pin.

  // set P1 to 0
  gpio_port->ovrc  = 1 << (IPMI_P1 & 0x1F); // Value to be driven on the I/O line: 0.
  for (i1=0; i1<P1DELAYCNT; i1++)
    AVR32_INTC.ipr[0];  // Dummy read
  readpvr = gpio_port->pvr;
  ga0t0 = (readpvr >> (IPMI_GA0 & 0x1F) & 1);
  ga1t0 = (readpvr >> (IPMI_GA1 & 0x1F) & 1);
  ga2t0 = (readpvr >> (IPMI_GA2 & 0x1F) & 1);


  // set P1 to 1
  gpio_port->ovrs  = 1 << (IPMI_P1 & 0x1F); // Value to be driven on the I/O line: 0.
  for (i1=0; i1<P1DELAYCNT; i1++)
    AVR32_INTC.ipr[0];  // Dummy read
  readpvr = gpio_port->pvr;
  ga0t1 = (readpvr >> (IPMI_GA0 & 0x1F) & 1);
  ga1t1 = (readpvr >> (IPMI_GA1 & 0x1F) & 1);
  ga2t1 = (readpvr >> (IPMI_GA2 & 0x1F) & 1);

  // compare test results to compute IPMI  and slot addresses
  if ((ga0t0 == 0) && (ga0t1 == 1))
	  // ga0 unconnected
	  twi_state.ga0val = Upin;
  else if ((ga0t0 == 1) && (ga0t1 == 1))
	  // ga0 pulled up to power
	  twi_state.ga0val = Ppin;
  else
	  twi_state.ga0val = Gpin;
  if ((ga1t0 == 0) && (ga1t1 == 1))
	  // ga1 unconnected
	  twi_state.ga1val = Upin;
  else if ((ga1t0 == 1) && (ga1t1 == 1))
	  // ga1 pulled up to power
	  twi_state.ga1val = Ppin;
  else
	  twi_state.ga1val = Gpin;
  if ((ga2t0 == 0) && (ga2t1 == 1))
	  // ga2 unconnected
	  twi_state.ga2val = Upin;
  else if ((ga2t0 == 1) && (ga2t1 == 1))
	  // ga2 pulled up to power
	  twi_state.ga2val = Ppin;
  else
	  twi_state.ga2val = Gpin;
  // scan table to get addresses
  for (i1=0; i1<GA_DECODE_TBL_SIZE; i1++) {
	  pdectbl = &ga_decode_tbl[i1];
	  if ((twi_state.ga0val == pdectbl->ga0) && (twi_state.ga1val == pdectbl->ga1) && (twi_state.ga2val == pdectbl->ga2)) {
	    // have match
	    *ipmbl_addr = pdectbl->ipmb_addr;
	    *slotid = pdectbl->slotid;
	    return;
	  }
  }
  // because the table fully defines the possible search space, we should never get here.  But if we do,
  // return some consistent (if distressing) values
  *ipmbl_addr = 0xff;
  *slotid = 0xff;
}


extern void print_twi_driver_stats(void) {
	unsigned long TWIsr, TWIimr;
	sio_putstr("IPMB-L Driver Stats:\n");
	sprintf(spbuf, "  Rx Msg Cnt:  %lu\n", IPMB_stats.rxmsg_cnt);
	sio_putstr(spbuf);
	sprintf(spbuf, "  Tx Msg Cnt:  %lu\n", IPMB_stats.txmsg_cnt);
	sio_putstr(spbuf);
	sprintf(spbuf, "  Tx NACK Cnt:  %lu\n", IPMB_stats.txNACK_cnt);
	sio_putstr(spbuf);
	sprintf(spbuf, "  Tx Msg Lost Cnt:  %lu\n", IPMB_stats.txmsglost_cnt);
	sio_putstr(spbuf);
	sprintf(spbuf, "  Slave Access Cnt:  %lu\n", IPMB_stats.slave_access_cnt);
	sio_putstr(spbuf);
	sprintf(spbuf, "  Master Access Cnt:  %lu\n", IPMB_stats.master_access_cnt);
	sio_putstr(spbuf);
	sprintf(spbuf, "  Master Loss-of-Arbitration Cnt:  %lu\n", IPMB_stats.arblost_cnt);
	sio_putstr(spbuf);
	sprintf(spbuf, "  Master Access Deferred Cnt:  %lu\n", IPMB_stats.master_acc_defer);
	sio_putstr(spbuf);
	sprintf(spbuf, "  Xmt Queue Cnt:  %u   Rcv Queue Cnt:  %u\n", eor_xcnt, eor_rcnt);
	sio_putstr(spbuf);
	sio_putstr("  Current Driver State:  ");
	switch (twi_state.state) {
		case disabled:
		  sio_putstr("disabled\n");
		  break;
		case slave_listen:
		  sio_putstr("listening (idle)\n");
		  break;
		case slave_write:
		  sio_putstr("slave (write op)\n");
		  break;
		case slave_read:
		  sio_putstr("slave (read op)\n");
		  break;
		case master_write:
		  sio_putstr("master write\n");
		  break;
		case failed:
		  sio_putstr("FAILED\n");
		  break;
    default:
			sio_putstr("UNKNOWN\n");
		  break;
	}
  Disable_global_interrupt();
  TWIsr = TWI.sr;
  TWIimr = TWI.imr;
  Enable_global_interrupt();
  sio_putstr("  Driver Regs:\n");
  sprintf(spbuf, "    SR:  0x%08lx\n", TWIsr);
  sio_putstr(spbuf);
  sprintf(spbuf, "    IMR:  0x%08lx\n", TWIimr);
  sio_putstr(spbuf);
}


