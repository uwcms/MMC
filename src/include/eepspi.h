/*
 * eepspi.h
 *
 *  Created on: Oct 11, 2010
 *      Author: tgorski
 */

#ifndef EEPSPI_H_
#define EEPSPI_H_

//#include "events.h"

#define EEPSIZE			(8192)					// size of EEPROM in bytes
#define EEPBLKSIZE		(32)					// size of EEPROM block in bytes
#define EEPBLKMASK		(0xffe0)				// mask for even block boundary

void eepspi_init(void);

int eepspi_chk_write_in_progress(void);

int eepspi_write(const unsigned char* pwbuf, unsigned short saddr, unsigned short len);

int eepspi_read(unsigned char* prbuf, unsigned short saddr, unsigned short len);

int eepspi_chip_erase(void);

#endif /* EEPSPI_H_ */
