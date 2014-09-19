/*
 * resetcfg.h
 *
 * Created: 7/25/2014 2:41:59 PM
 *  Author: tgorski
 */ 


#ifndef RESETCFG_H_
#define RESETCFG_H_

// LPGP Reg 0 mask bits
#define RSTCFG_AUX_PWR_IN_ENABLED_MASK              (1<<0)
#define RSTCFG_PAYLOAD_HOT_MASK                     (1<<1)
#define RSTCFG_BACKEND_HOT_MASK                     (1<<2)

#define RSTCFG_1P2L_MTOL_BIT0                       (1<<8)
#define RSTCFG_1P2L_MTOL_BIT1                       (1<<9)
#define RSTCFG_1P2L_MSEL_BIT0                       (1<<10)
#define RSTCFG_1P2L_MSEL_BIT1                       (1<<11)
#define RSTCFG_1P2R_MTOL_BIT0                       (1<<12)
#define RSTCFG_1P2R_MTOL_BIT1                       (1<<13)
#define RSTCFG_1P2R_MSEL_BIT0                       (1<<14)
#define RSTCFG_1P2R_MSEL_BIT1                       (1<<15)
#define RSTCFG_1P0L_MTOL_BIT0                       (1<<16)
#define RSTCFG_1P0L_MTOL_BIT1                       (1<<17)
#define RSTCFG_1P0L_MSEL_BIT0                       (1<<18)
#define RSTCFG_1P0L_MSEL_BIT1                       (1<<19)
#define RSTCFG_1P0R_MTOL_BIT0                       (1<<20)
#define RSTCFG_1P0R_MTOL_BIT1                       (1<<21)
#define RSTCFG_1P0R_MSEL_BIT0                       (1<<22)
#define RSTCFG_1P0R_MSEL_BIT1                       (1<<23)
#define RSTCFG_1P0L_VSET_BIT0                       (1<<24)
#define RSTCFG_1P0L_VSET_BIT1                       (1<<25)
#define RSTCFG_1P0R_VSET_BIT0                       (1<<26)
#define RSTCFG_1P0R_VSET_BIT1                       (1<<27)





#endif /* RESETCFG_H_ */