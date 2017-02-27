/*
 * sio_usart.h
 *
 *  Created on: Oct 5, 2010
 *      Author: tgorski
 */

#ifndef SIO_USART_H_
#define SIO_USART_H_

#define USART                 (&AVR32_USART0)
#define USART_IRQ             AVR32_USART0_IRQ
#define USART_IRQ_PRIORITY	  AVR32_INTC_INT2
#define USART_BAUDRATE        19200
#define USART_NO_PARITY       AVR32_USART_MR_PAR_NONE   //!< Don't use a parity bit.
#define USART_1_STOPBIT       AVR32_USART_MR_NBSTOP_1   //!< Use 1 stop bit.
#define USART_NORMAL_CHMODE   AVR32_USART_MR_CHMODE_NORMAL      //!< Normal communication.

#define BACKSPACE			(0x8)
#define DELETE				(0x7f)
#define BELL			  	(0x7)
#define CTRLR         (0x12)

#define RX_BUFFER_SIZE		(128)
#define TX_BUFFER_SIZE		(1024)

#define CHAR_ECHO_ENA		(1)
#define CHAR_ECHO_DIS		(0)
#define CHAR_ECHO_SETTING	CHAR_ECHO_ENA

#define SPRINTBUFSIZE			(256)

// console text filter flags
#define TXTFILT_EVENTS            (0x01)      // outgoing IPMI event messages
#define TXTFILT_IPMI_STD_REQ      (0x02)      // incoming IPMI standard requests
#define TXTFILT_IPMI_CUSTOM_REQ   (0x04)      // incoming IPMI custom requests
#define TXTFILT_DBG_SUMMARY       (0x40)      // debug level 1 messages (summary)
#define TXTFILT_DBG_DETAIL        (0x80)      // debug level 2 messages (detail)
#define TXTFILT_INFO              (0x08)      // general notifications

extern char spbuf[SPRINTBUFSIZE];

extern unsigned int get_sio_msg_cnt(void);

extern void dec_sio_msg_cnt(void);

extern void sio_init(void);

extern void sio_putc(char c);

extern char sio_getc(void);

extern int sio_putstr(char *cptr);

extern int sio_getln(char *cptr, int maxlen);

extern void sio_filt_putstr(const unsigned char filtcode, int addtag, char *cptr);

extern void sio_set_txt_filter(const unsigned char filterval);

extern void sio_clr_txt_filter(const unsigned char filterval);

extern unsigned char sio_get_txt_filter(void);

extern void sio_buf_reset(void);

#endif /* SIO_USART_H_ */
