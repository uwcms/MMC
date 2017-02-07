/*
 * spi1.c
 *
 *  Created on: Apr 11, 2011
 *      Author: tgorski
 */

#include "compiler.h"
#include "spi1.h"
#include "utils.h"

#define SPI1_LOCKED_VAL       (1)
#define SPI1_UNLOCKED_VAL     (0)

int spi1_lock_flag = SPI1_UNLOCKED_VAL;

void spi1_init(void) {
  spi1_lock_flag = SPI1_UNLOCKED_VAL;
}


int spi1_is_locked(void) {
	return spi1_lock_flag;
}


int spi1_lock(void) {
  // returns 1 if SPI1 is locked by this call.  If SPI1 already locked, returns 0
  int retval;
  Bool giflag = Is_global_interrupt_enabled();
  Int_Disable(giflag);
  Disable_global_interrupt();
  if (spi1_lock_flag==SPI1_LOCKED_VAL)
    retval=0;
  else {
    spi1_lock_flag = SPI1_LOCKED_VAL;
    retval = 1;
  }
  Int_Restore(giflag);
  return retval;
}


void spi1_unlock(void) {
  // sets the SPI1 flag to unlocked
  spi1_lock_flag = SPI1_UNLOCKED_VAL;
}
