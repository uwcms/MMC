/*
 * pwrmgr.c
 *
 *  Created on: Oct 5, 2010
 *      Author: tgorski
 */
#include "pwrmgr.h"


void osc_init() {
  unsigned long mask;

  // initalize all timebases on the system
  AVR32_PM.OSCCTRL0.mode = AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G3;
  AVR32_PM.OSCCTRL0.startup = AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC;
  AVR32_PM.MCCTRL.osc0en = 1;										// enable oscillator 0
  while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_OSC0RDY_MASK));			// wait for osc0 to be stable and ready

  // initialize the 32 kHz oscillator
  AVR32_PM.OSCCTRL32.mode = AVR32_PM_OSCCTRL32_MODE_CRYSTAL;
  AVR32_PM.OSCCTRL32.startup = AVR32_PM_OSCCTRL32_STARTUP_8192_RCOSC;
  AVR32_PM.OSCCTRL32.osc32en = 1;

  //DEBUG:  uncomment these next two lines to bypass the PLL and run everything
  //  (CPU, PBA, PBB, HSB) off of the 12 MHz CPU clock
  //AVR32_PM.MCCTRL.mcsel = 1;
  //return;

  // initialize PLL0 to 4x multiplication (8x multiplication at VCO)
  volatile avr32_pm_pll_t* pPLL = &(AVR32_PM.PLL[0]);
  pPLL->pllosc = 0;											// select oscillator 0
  pPLL->pllcount = 0x10;
  pPLL->plldiv = 0;

  pPLL->pllmul = 3;										// fVCO = 2*(PLLMULT+1)*fOSC
  pPLL->pllopt = 3;				// wide bandwidth mode, fPLL = fVCO/2, lower VCO freq range
  pPLL->pllen = 1;											// enable PLL
  while (!(AVR32_PM.poscsr & AVR32_PM_POSCSR_LOCK0_MASK));		// wait for PLL to lock

  // set the PBA/PBB clocks at 1/4 the main clock
  mask = (1 << AVR32_PM_CKSEL_PBBDIV_OFFSET) |
    (1 << AVR32_PM_CKSEL_PBADIV_OFFSET) |
    (1 << AVR32_PM_CKSEL_PBBSEL_OFFSET) |
    (1 << AVR32_PM_CKSEL_PBASEL_OFFSET);
  AVR32_PM.cksel = mask;

  AVR32_PM.MCCTRL.mcsel = 2;								// set PLL0 to be the main clock frequency

  // turn off clocks to USB
  AVR32_PM.hsbmask &= ~(0x8);								// USB on HSB
  AVR32_PM.pbbmask &= ~(0x2);
  

  // initialize brown-out detection
  AVR32_PM.bod = 0x55000000;        // write key
  AVR32_PM.bod = 0xaa000000;        // disable brown-out detect
  AVR32_PM.bod = 0x55000000;        // write key
  AVR32_PM.bod = 0xaa00001f;        // set level to 1.55V - 1.65V for brownout
  AVR32_PM.bod = 0x55000000;        // write key
  AVR32_PM.bod = 0xaa00011f;        // enable brown-out detect to reset the chip at the previously-written level

}
