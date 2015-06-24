/*
 * sio_usart.c
 *
 *  Created on: Oct 5, 2010
 *      Author: tgorski
 */

#include <avr32/io.h>
#include "nonvolatile.h"
#include "sio_usart.h"
#include "utils.h"
#include "intc.h"
#include "pwrmgr.h"

#define TX_PUTC_BUF_SIZE	(TX_BUFFER_SIZE-4)		// leave some of tx buffer free for local rx char echo

#define txisempty(usart)	(usart->csr & AVR32_USART_CSR_TXEMPTY_MASK ? 1 : 0)
#define txisready(usart)  (usart->csr & AVR32_USART_CSR_TXRDY_MASK ? 1 : 0)

unsigned char rx_buffer[RX_BUFFER_SIZE];
unsigned char tx_buffer[TX_BUFFER_SIZE];
unsigned short rx_wr_idx, rx_rd_idx, rx_ctr;
unsigned short tx_wr_idx, tx_rd_idx, tx_ctr;
unsigned char echo_rx_char;
unsigned int sio_msg_ctr;

char spbuf[SPRINTBUFSIZE];


/*! \name Return Values
 */
//! @{
#define USART_SUCCESS                 0 //!< Successful completion.
#define USART_FAILURE                -1 //!< Failure because of some unspecified reason.
#define USART_INVALID_INPUT           1 //!< Input value out of range.
#define USART_INVALID_ARGUMENT       -1 //!< Argument value out of range.
#define USART_TX_BUSY                 2 //!< Transmitter was busy.
#define USART_RX_EMPTY                3 //!< Nothing was received.
#define USART_RX_ERROR                4 //!< Transmission error occurred.
#define USART_MODE_FAULT              5 //!< USART not in the appropriate mode.
//! @}
/*! \name Parity Settings
 */
//! @{
#define USART_EVEN_PARITY             AVR32_USART_MR_PAR_EVEN   //!< Use even parity on character transmission.
#define USART_ODD_PARITY              AVR32_USART_MR_PAR_ODD    //!< Use odd parity on character transmission.
#define USART_SPACE_PARITY            AVR32_USART_MR_PAR_SPACE  //!< Use a space as parity bit.
#define USART_MARK_PARITY             AVR32_USART_MR_PAR_MARK   //!< Use a mark as parity bit.
#define USART_NO_PARITY               AVR32_USART_MR_PAR_NONE   //!< Don't use a parity bit.
#define USART_MULTIDROP_PARITY        AVR32_USART_MR_PAR_MULTI  //!< Parity bit is used to flag address characters.
//! @}

/*! \name Stop Bits Settings
 */
//! @{
#define USART_1_STOPBIT               AVR32_USART_MR_NBSTOP_1   //!< Use 1 stop bit.
#define USART_1_5_STOPBITS            AVR32_USART_MR_NBSTOP_1_5 //!< Use 1.5 stop bits.
#define USART_2_STOPBITS              AVR32_USART_MR_NBSTOP_2   //!< Use 2 stop bits (for more, just give the number of bits).

//! Input parameters when initializing RS232 and similar modes.
typedef struct
{
  //! Set baud rate of the USART (unused in slave modes).
  unsigned long baudrate;

  //! Number of bits to transmit as a character (5 to 9).
  unsigned char charlength;

  //! How to calculate the parity bit: \ref USART_EVEN_PARITY, \ref USART_ODD_PARITY,
  //! \ref USART_SPACE_PARITY, \ref USART_MARK_PARITY, \ref USART_NO_PARITY or
  //! \ref USART_MULTIDROP_PARITY.
  unsigned char paritytype;

  //! Number of stop bits between two characters: \ref USART_1_STOPBIT,
  //! \ref USART_1_5_STOPBITS, \ref USART_2_STOPBITS or any number from 3 to 257
  //! which will result in a time guard period of that length between characters.
  //! \note \ref USART_1_5_STOPBITS is supported in asynchronous modes only.
  unsigned short stopbits;

  //! Run the channel in testmode: \ref USART_NORMAL_CHMODE, \ref USART_AUTO_ECHO,
  //! \ref USART_LOCAL_LOOPBACK or \ref USART_REMOTE_LOOPBACK.
  unsigned char channelmode;
} usart_options_t;

static int usart_set_async_baudrate(volatile avr32_usart_t *usart, unsigned int baudrate, unsigned long pba_hz);
void usart_reset(volatile avr32_usart_t *usart);
int usart_init_rs232(volatile avr32_usart_t *usart, const usart_options_t *opt, long pba_hz);
int usart_read_char(volatile avr32_usart_t *usart, int *c);

__inline__ void enqueue_tx_char(char c) {
  tx_ctr++;
  tx_buffer[tx_wr_idx] = c;
    if (++tx_wr_idx >= TX_BUFFER_SIZE)
	tx_wr_idx = 0;	// wraparound
}


__inline__ void enqueue_rx_char(char c) {
  rx_ctr++;
  rx_buffer[rx_wr_idx] = c;
  if (++rx_wr_idx >= RX_BUFFER_SIZE)
	  rx_wr_idx = 0;	// wraparound
}


#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif
static void usart_int_handler(void)
{
  int rxchar;
  int txchar;
  int tmp_wr_idx;
  int usart_read_status;


  CAPBUF2_SAVEPC;

  usart_read_status = usart_read_char(USART, &rxchar);
  if (usart_read_status == USART_SUCCESS) {
  	// have character to enqueue, check for room
	  if (rx_ctr >= RX_BUFFER_SIZE-2) {
	    // buffer overrun--reset rx buffer back to empty (no great alternative, but this at least allows the rx channel
	    // to keep operating)
	    rx_wr_idx = 0;
	    rx_rd_idx = 0;
	    rx_ctr = 0;
      sio_msg_ctr = 0;
	  }
    switch (rxchar) {
      case '\n':
      case '\r':
        // newline character
        enqueue_rx_char('\n');
        sio_msg_ctr++;
        if (echo_rx_char) {
          enqueue_tx_char('\r');		// add a return
          enqueue_tx_char('\n');
          USART->ier = AVR32_USART_IER_TXRDY_MASK;
        }
        break;

      case BACKSPACE:
      case DELETE:
        // process backspace character
        if (!rx_ctr) {
          // rx queue empty, nothing to erase
          if (echo_rx_char)
            enqueue_tx_char(BELL);
          USART->ier = AVR32_USART_IER_TXRDY_MASK;
        }
        else {
          // rx queue not empty, but only erase up to \n char
          tmp_wr_idx = rx_wr_idx ? rx_wr_idx-1 : RX_BUFFER_SIZE-1;
          if (rx_buffer[tmp_wr_idx] == '\n') {
            // last enqueued  char was newline
            if (echo_rx_char) {
              enqueue_tx_char(BELL);
              USART->ier = AVR32_USART_IER_TXRDY_MASK;
            }
          }
          else {
            // erase last character
            rx_ctr--;
            rx_wr_idx = tmp_wr_idx;
            if (echo_rx_char) {
              enqueue_tx_char(BACKSPACE);
              enqueue_tx_char(' ');
              enqueue_tx_char(BACKSPACE);
              USART->ier = AVR32_USART_IER_TXRDY_MASK;
            }
          }
        }
        break;

      case CTRLR:
        // reset sio rx buffers
  	    rx_wr_idx = 0;
	      rx_rd_idx = 0;
	      rx_ctr = 0;
	      sio_msg_ctr = 0;
        break;

      default:
        enqueue_rx_char((char) rxchar);
        if (echo_rx_char) {
          // echo char by placing in tx queue
          enqueue_tx_char((char) rxchar);
          USART->ier = AVR32_USART_IER_TXRDY_MASK;
        }
    }

  }
  else if (usart_read_status == USART_RX_ERROR) {
    // usart read error--clear error and reset rx receiver
    rxchar = USART->rhr;
    USART->cr = AVR32_USART_CR_RXDIS_MASK;      // disable rx
    USART->cr = AVR32_USART_CR_RSTRX_MASK;      // reset rx
    USART->cr = AVR32_USART_CR_RSTSTA_MASK;     // clear rx status
    USART->cr = AVR32_USART_CR_RXEN_MASK;       // enable rx status
    USART->ier = AVR32_USART_IER_RXRDY_MASK;    // enable rx interrupt
  }

  // transmit interrupt service
  if (txisready(USART) && (USART->imr & AVR32_USART_IMR_TXRDY_MASK)) {
    // transmitter is ready and the txready interrupt is enabled
    if (tx_ctr) {
      // there is another character to be sent
      // dequeue and transmit character
      txchar = tx_buffer[tx_rd_idx];
      tx_ctr--;
      if (++tx_rd_idx >= TX_BUFFER_SIZE)
        tx_rd_idx = 0; // wraparound
      USART->thr = (txchar << AVR32_USART_THR_TXCHR_OFFSET) & AVR32_USART_THR_TXCHR_MASK;
    }
    else {
      // there are no more characters to send
      USART->idr = AVR32_USART_IDR_TXRDY_MASK;
    }
  }
}


unsigned int get_sio_msg_cnt(void) {
  unsigned int temp_msg_ctr;
  Disable_global_interrupt();
  temp_msg_ctr = sio_msg_ctr;
  Enable_global_interrupt();
  return temp_msg_ctr;
}


void dec_sio_msg_cnt(void) {
  Disable_global_interrupt();
  if (sio_msg_ctr)
    sio_msg_ctr--;			// don't wrap below zero!!!
  Enable_global_interrupt();
}


void sio_init(void) {
  static const usart_options_t USART_OPTIONS = {
 	    .baudrate     = USART_BAUDRATE,
 	    .charlength   = 8,
 	    .paritytype   = USART_NO_PARITY,
 	    .stopbits     = USART_1_STOPBIT,
 	    .channelmode  = USART_NORMAL_CHMODE };

  // initialize sio port 0, rcv and xmt queues, isrs
  rx_wr_idx = 0;
  rx_rd_idx = 0;
  rx_ctr = 0;
  tx_wr_idx = 0;
  tx_rd_idx = 0;
  tx_ctr = 0;
  sio_msg_ctr = 0;

  echo_rx_char = CHAR_ECHO_SETTING;

  // Initialize USART in RS232 mode.
  usart_init_rs232(USART, &USART_OPTIONS, PBAFREQ);

  // install interrupt handler
  INTC_register_interrupt(&usart_int_handler, USART_IRQ, USART_IRQ_PRIORITY);

  // Enable USART Rx interrupt.
  USART->ier = AVR32_USART_IER_RXRDY_MASK;
}


void sio_buf_reset(void) {
  // function to reset sio buffers if they get garbled for some reason
  Disable_global_interrupt();
  rx_wr_idx = 0;
  rx_rd_idx = 0;
  rx_ctr = 0;
  tx_wr_idx = 0;
  tx_rd_idx = 0;
  tx_ctr = 0;
  sio_msg_ctr = 0;
  Enable_global_interrupt();
}


void sio_putc(char c) {
  if (tx_ctr >= TX_PUTC_BUF_SIZE) {
    // have to wait for transmit queue to free up
    while (tx_ctr >= TX_PUTC_BUF_SIZE) Service_Watchdog();
  }
  Disable_global_interrupt();
  if (txisready(USART) && (tx_ctr == 0)) {
	  // write to hardware, enable interrupts
	  USART->thr = (c << AVR32_USART_THR_TXCHR_OFFSET) & AVR32_USART_THR_TXCHR_MASK;
  }
  else {
	  // put on transmit queue
	  enqueue_tx_char(c);
  }
  USART->ier = AVR32_USART_IER_TXRDY_MASK;   // enable transmit interrupt

  Enable_global_interrupt();
}


char sio_getc(void) {
   char rdchar;
   if (!rx_ctr) {
     // stuck here waiting for character, which can happen on a handful of console commands
     while (!rx_ctr) Service_Watchdog();             // SIO ISR will eventually put a character here
   }
   rdchar = rx_buffer[rx_rd_idx];
   Disable_global_interrupt();
   rx_rd_idx++;
   if (rx_rd_idx >= RX_BUFFER_SIZE)
     rx_rd_idx = 0;
   rx_ctr--;
   Enable_global_interrupt();
   return rdchar;
 }

int sio_putstr(char *cptr) {
  // transmit characters until a null is observed
  char *xptr;

  xptr = cptr;
  while (*xptr) {
	// put next character
	if (*xptr == '\n')
    sio_putc('\r');
	sio_putc(*xptr);
	xptr++;
  }
  return 0;
}

int sio_getln(char *cptr, int maxlen) {
  // get string of characters until newline ('\n') or return ('\r') or null character is observed,
  // or until max length is reached.  Funciton returns the number of non-null characters returned,
  // up to maxlen-1
  int curlen;
  char *curcptr;
  char curchar;

  curlen = 0;
  curcptr = cptr;

  while (curlen < maxlen) {
    curchar = sio_getc();
    *curcptr = curchar;
    if (!curchar) {
      // null character, return now
      return curlen;
    }
    // bump length and pointer
    curcptr++;
    curlen++;
    if (curchar == '\n' || curchar == '\r') {
      dec_sio_msg_cnt();
      break;		// time to null terminate and return
    }
  }
  // reached end of string--null terminate and return
  *curcptr = 0;
  return curlen;
}


static int usart_set_async_baudrate(volatile avr32_usart_t *usart, unsigned int baudrate, unsigned long pba_hz)
{
  unsigned int over = (pba_hz >= 16 * baudrate) ? 16 : 8;
  unsigned int cd_fp = ((1 << AVR32_USART_BRGR_FP_SIZE) * pba_hz + (over * baudrate) / 2) / (over * baudrate);
  unsigned int cd = cd_fp >> AVR32_USART_BRGR_FP_SIZE;
  unsigned int fp = cd_fp & ((1 << AVR32_USART_BRGR_FP_SIZE) - 1);

  if (cd < 1 || cd > (1 << AVR32_USART_BRGR_CD_SIZE) - 1)
    return USART_INVALID_INPUT;

  usart->mr = (usart->mr & ~(AVR32_USART_MR_USCLKS_MASK |
                             AVR32_USART_MR_SYNC_MASK |
                             AVR32_USART_MR_OVER_MASK)) |
              AVR32_USART_MR_USCLKS_MCK << AVR32_USART_MR_USCLKS_OFFSET |
              ((over == 16) ? AVR32_USART_MR_OVER_X16 : AVR32_USART_MR_OVER_X8) << AVR32_USART_MR_OVER_OFFSET;

  usart->brgr = cd << AVR32_USART_BRGR_CD_OFFSET |
                fp << AVR32_USART_BRGR_FP_OFFSET;

  return USART_SUCCESS;
}


int usart_init_rs232(volatile avr32_usart_t *usart, const usart_options_t *opt, long pba_hz)
{
  // Reset the USART and shutdown TX and RX.
  usart_reset(usart);

  // Check input values.
  if (!opt || // Null pointer.
      opt->charlength < 5 || opt->charlength > 9 ||
      opt->paritytype > 7 ||
      opt->stopbits > 2 + 255 ||
      opt->channelmode > 3 ||
      usart_set_async_baudrate(usart, opt->baudrate, pba_hz) == USART_INVALID_INPUT)
    return USART_INVALID_INPUT;

  if (opt->charlength == 9)
  {
    // Character length set to 9 bits. MODE9 dominates CHRL.
    usart->mr |= AVR32_USART_MR_MODE9_MASK;
  }
  else
  {
    // CHRL gives the character length (- 5) when MODE9 = 0.
    usart->mr |= (opt->charlength - 5) << AVR32_USART_MR_CHRL_OFFSET;
  }

  usart->mr |= opt->paritytype << AVR32_USART_MR_PAR_OFFSET |
               opt->channelmode << AVR32_USART_MR_CHMODE_OFFSET;

  if (opt->stopbits > USART_2_STOPBITS)
  {
    // Set two stop bits
    usart->mr |= AVR32_USART_MR_NBSTOP_2 << AVR32_USART_MR_NBSTOP_OFFSET;
    // and a timeguard period gives the rest.
    usart->ttgr = opt->stopbits - USART_2_STOPBITS;
  }
  else
    // Insert 1, 1.5 or 2 stop bits.
    usart->mr |= opt->stopbits << AVR32_USART_MR_NBSTOP_OFFSET;

  // Set normal mode.
  usart->mr = (usart->mr & ~AVR32_USART_MR_MODE_MASK) |
              AVR32_USART_MR_MODE_NORMAL << AVR32_USART_MR_MODE_OFFSET;

  // Setup complete; enable communication.
  // Enable input and output.
  usart->cr = AVR32_USART_CR_RXEN_MASK |
              AVR32_USART_CR_TXEN_MASK;

  return USART_SUCCESS;
}


void usart_reset(volatile avr32_usart_t *usart)
{
  Bool global_interrupt_enabled = Is_global_interrupt_enabled();

  // Disable all USART interrupts.
  // Interrupts needed should be set explicitly on every reset.
  if (global_interrupt_enabled) Disable_global_interrupt();
  usart->idr = 0xFFFFFFFF;
  usart->csr;
  if (global_interrupt_enabled) Enable_global_interrupt();

  // Reset mode and other registers that could cause unpredictable behavior after reset.
  usart->mr = 0;
  usart->rtor = 0;
  usart->ttgr = 0;

  // Shutdown TX and RX (will be re-enabled when setup has successfully completed),
  // reset status bits and turn off DTR and RTS.
  usart->cr = AVR32_USART_CR_RSTRX_MASK   |
              AVR32_USART_CR_RSTTX_MASK   |
              AVR32_USART_CR_RSTSTA_MASK  |
              AVR32_USART_CR_RSTIT_MASK   |
              AVR32_USART_CR_RSTNACK_MASK |
#ifndef AVR32_USART_440_H_INCLUDED
// Note: Modem Signal Management DTR-DSR-DCD-RI are not included in USART rev.440.
              AVR32_USART_CR_DTRDIS_MASK  |
#endif
              AVR32_USART_CR_RTSDIS_MASK;
}


int usart_read_char(volatile avr32_usart_t *usart, int *c)
{
  // Check for errors: frame, parity and overrun. In RS485 mode, a parity error
  // would mean that an address char has been received.
  if (usart->csr & (AVR32_USART_CSR_OVRE_MASK |
                    AVR32_USART_CSR_FRAME_MASK |
                    AVR32_USART_CSR_PARE_MASK))
    return USART_RX_ERROR;

  // No error; if we really did receive a char, read it and return SUCCESS.
  if ((usart->csr & AVR32_USART_CSR_RXRDY_MASK) != 0) {
    *c = (usart->rhr & AVR32_USART_RHR_RXCHR_MASK) >> AVR32_USART_RHR_RXCHR_OFFSET;
    return USART_SUCCESS;
  }
  else
    return USART_RX_EMPTY;
}

void sio_filt_putstr(const unsigned char filtcode, int addtag, char *cptr) {
  // fiters the string put operation, per the filtcode argument and the current mask settings.
  // The string put occurs only if the argument is in the active mask.  If the addtag argment is
  // nonzero, then a prefix string indicating the message type is added.
  unsigned char mask = filtcode & GPparambuf.sio_filtermask;
  if (!mask)
    return;
  if (addtag) {
    if (mask & TXTFILT_EVENTS)
      sio_putstr("[IPMIEVT] ");
    if (mask & TXTFILT_IPMI_STD_REQ)
      sio_putstr("[IPMISTD] ");
    if (mask & TXTFILT_IPMI_CUSTOM_REQ)
      sio_putstr("[IPMICSTM] ");
    if (mask & TXTFILT_DBG_SUMMARY)
      sio_putstr("[DBGSUM] ");
    if (mask & TXTFILT_DBG_DETAIL)
      sio_putstr("[DBGDTL] ");
    if (mask & TXTFILT_INFO)
      sio_putstr("[INFO] ");
  }
  sio_putstr(cptr);
}


void sio_set_txt_filter(const unsigned char filterval) {
	GPparambuf.sio_filtermask |= filterval;
  while (eepspi_chk_write_in_progress()) Service_Watchdog();     // wait for EEPROM to be available
  eepspi_write((unsigned char*) &GPparambuf.sio_filtermask, GP_PARAM_AREA_BYTE_OFFSET + ((int) &GPparambuf.sio_filtermask - (int) &GPparambuf), 1);
}


void sio_clr_txt_filter(const unsigned char filterval) {
	GPparambuf.sio_filtermask &= ~filterval;
  while (eepspi_chk_write_in_progress()) Service_Watchdog();     // wait for EEPROM to be available
  eepspi_write((unsigned char*) &GPparambuf.sio_filtermask, GP_PARAM_AREA_BYTE_OFFSET + ((int) &GPparambuf.sio_filtermask - (int) &GPparambuf), 1);
}


unsigned char sio_get_txt_filter(void) {
  return GPparambuf.sio_filtermask;
}


