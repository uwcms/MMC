/*
 * console_parser.c
 *
 * Created: 8/12/2011 2:40:44 PM
 *  Author: tgorski
 */ 

#include <avr32/io.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "mmc_version.h"
#include "utils.h"
#include "swevent.h"
#include "timer_callback.h"
#include "ejecthandle.h"
#include "fpgaspi.h"
#include "adc.h"
#include "rtc.h"
#include "twidriver.h"
#include "sio_usart.h"
#include "console_parser.h"
#include "payload_mgr.h"
#include "ipmb_svc.h"
#include "ipmi_cmd_parser.h"
#include "sensor_svc.h"
#include "nonvolatile.h"
#include "fault_log.h"

// debug 2.3
#include "ejecthandle.h"

#define iswhite(c)        ((c <= 0x20) || (c>0x7f))
#define CONSOLEWDMAXLEN       (20)

typedef struct {
  int wordID;
  char word[CONSOLEWDMAXLEN];
} WordListEntrytype;

// Word IDs for console commands
#define WD_EOLIST                     (0)
#define WD_CMD_HELP                   (1)
#define WD_CMD_SETMSGMASK             (2)
#define WD_CMD_CLRMSGMASK             (3)
#define WD_CMD_GETMSGMASK             (4)
#define WD_CMD_QMARK                  (11)
#define WD_CMD_WRESET                 (13)
#define WD_CMD_CRESET                 (14)
#define WD_CMD_MRESET                 (15)
#define WD_CMD_EEPERASE               (16)
#define WD_CMD_SENSREAD               (17)
#define WD_CMD_IPMBSTATS              (18)
#define WD_CMD_SLOTID                 (19)
#define WD_CMD_PINSTATE               (20)
#define WD_CMD_TESTEVT                (21)
#define WD_CMD_DISPFL                 (22)
#define WD_CMD_CLRFL                  (23)
#define WD_CMD_SETTRIM                (24)
#define WD_CMD_GETTRIM                (25)
#define WD_CMD_SETV                   (26)
#define WD_CMD_GETV                   (27)
#define WD_CMD_SIORST                 (37)
#define WD_CMD_GETPAUX                (38)
#define WD_CMD_ENAPAUX                (39)
#define WD_CMD_DISPAUX                (40)
#define WD_CMD_TIMESTATS              (41)
#define WD_CMD_DISPCB                 (42)
#define WD_CMD_VERSION                (43)
#define WD_CMD_EVTUPDATE              (44)
#define WD_CMD_SPIF2CNT               (45)
#define WD_CMD_OHO                    (46)
#define WD_CMD_OHI                    (47)
#define WD_CMD_OHR                    (48)
#define WD_CMD_DISPSENSREC            (49)

// Word IDs for filter options
#define WD_FILT_IPMIEVT               (5)
#define WD_FILT_IPMISTD               (6)
#define WD_FILT_IPMICUSTOM            (7)
#define WD_FILT_DBGSUMMARY            (8)
#define WD_FILT_DBGDETAIL             (9)
#define WD_FILT_INFO                  (12)
#define WD_FILT_ALL                   (10)

// Word IDs for trim/voltage options
#define WD_SETTRIM_CUR                (28)
#define WD_SETTRIM_EEP                (29)
#define WD_SETTRIM_1P0VL              (30)
#define WD_SETTRIM_1P0VR              (31)
#define WD_SETTRIM_1P2VL              (32)
#define WD_SETTRIM_1P2VR              (33) 
#define WD_SETTRIM_OFF                (34)  
#define WD_SETVOLT_HIGH               (35)
#define WD_SETVOLT_LOW                (36)


const WordListEntrytype Cmd_Wd_List[] = {
  {WD_CMD_HELP, "help"},
  {WD_CMD_QMARK, "?"},
  {WD_CMD_SETMSGMASK, "setmmask"},
  {WD_CMD_CLRMSGMASK, "clrmmask"},
  {WD_CMD_GETMSGMASK, "getmmask"},
  {WD_CMD_WRESET, "wreset"},
  {WD_CMD_CRESET, "creset"},
  {WD_CMD_MRESET, "mreset"},
  {WD_CMD_EEPERASE, "eeperase"},
  {WD_CMD_SENSREAD, "sensread"},	
  {WD_CMD_IPMBSTATS, "ipmbstats"},	
  {WD_CMD_SLOTID, "slotid"},	
  {WD_CMD_PINSTATE, "pinstate"},	
  {WD_CMD_TESTEVT, "testevt"},
  {WD_CMD_DISPFL, "dispfl"},  
  {WD_CMD_CLRFL, "clrfl"}, 
  {WD_CMD_SETTRIM, "settrim"},
  {WD_CMD_GETTRIM, "gettrim"},
  {WD_CMD_SETV, "setv"},
  {WD_CMD_GETV, "getv"},
  {WD_CMD_SIORST, "siorst"},
  {WD_CMD_GETPAUX, "getpaux"},
  {WD_CMD_ENAPAUX, "enapaux"},
  {WD_CMD_DISPAUX, "dispaux"},
  {WD_CMD_TIMESTATS, "timestats"},
  {WD_CMD_DISPCB, "dispcb"},
  {WD_CMD_VERSION, "version"},
  {WD_CMD_EVTUPDATE, "evtrefresh"},
  {WD_CMD_SPIF2CNT, "spif2c"},
  {WD_CMD_OHO, "ohout"},
  {WD_CMD_OHI, "ohin"},
  {WD_CMD_OHR, "ohrel"},
	{WD_CMD_DISPSENSREC, "sensrec"},  
  {WD_EOLIST, ""} };

const WordListEntrytype Filteropt_Wd_List[] = {
  {WD_FILT_IPMIEVT, "ipmievt"},
  {WD_FILT_IPMISTD, "ipmistd"},
  {WD_FILT_IPMICUSTOM, "ipmicstm"},
  {WD_FILT_DBGSUMMARY, "dbgsum"},
  {WD_FILT_DBGDETAIL, "dbgdtl"},
  {WD_FILT_INFO, "info"},
  {WD_FILT_ALL, "all"},
  {WD_EOLIST, ""} };

const WordListEntrytype SetTrim_Wd_List[] = {
  {WD_SETTRIM_EEP, "-e"},
  {WD_SETTRIM_CUR, "-c"	},
  {WD_SETTRIM_1P0VL, "1.0VL"},
  {WD_SETTRIM_1P0VR, "1.0VR"},
  {WD_SETTRIM_1P2VL, "1.2VL"},
  {WD_SETTRIM_1P2VR, "1.2VR"},
  {WD_SETTRIM_OFF, "off"},
  {WD_SETVOLT_HIGH, "high"},
  {WD_SETVOLT_LOW, "low"},
  {WD_EOLIST, ""} };

const char* TrimCodeStr[] = {"off", "-5\%", "-3\%", "-1\%", "+1\%", "+3\%", "+5\%"};

char cmd_buf[RX_BUFFER_SIZE];

int get_next_word(const char*const plinebuf, const char** plbpos, char* pwdbuf);
int match_word(const WordListEntrytype *pWdList, const char* pWd);
void parse_cmd(const char*const plinebuf, const char** plbpos, const char* pcmdwd);
void change_filter_mask(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void display_sensor_value(const char*const plinebuf, const char** plbpos);
void format_sensor_value_str(const int SensorNum, char* pbuf);
void display_pin_val(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void sensor_test_evt(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void display_fault_log(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void set_mgt_vtrim(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void get_mgt_vtrim(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void set_mgt_vout(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void get_mgt_vout(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void display_mgt_trim_settings(const const MGT_Vreg_settings_t*const pvrs);
void change_paux(const char*const plinebuf, const char** plbpos, const int cmdwdID, const int paux_enable);
void etimestr(char* sbuf, unsigned int timesec);
void display_sensor_settings_rec(void);


void console_chk_cmd(void) {
  const char* posptr = &cmd_buf[0];
  char cmdwd[CONSOLEWDMAXLEN];
  
  CAPBUF1_SAVEPC;
  if (!get_sio_msg_cnt())
    // no messages
	  return;
  sio_getln(cmd_buf, RX_BUFFER_SIZE-1);
  dec_sio_msg_cnt();
  
  // pull off command word
  if (get_next_word(cmd_buf, &posptr, cmdwd))
    parse_cmd(cmd_buf, &posptr, cmdwd);
}


int get_next_word(const char*const plinebuf, const char** plbpos, char* pwdbuf) {
  // finds and returns next occurring word in a text line buffer.  Returns a value of 1 if a word is found, 0 otherwise.
  // Word is copied to buffer pointed to by arg pwdbuf, and that word is set to a null-terminated string if no word is
  // returned.
  const char* plbend = plinebuf + strlen(plinebuf);				// pointer to end of buffer
  char* pcurwd = pwdbuf;
  const char* pcurpos = *plbpos;
  int foundnonwhite = 0;											// flag marking start of first nonwhite chars

  // make sure plbpos is in range of buffer
  if ((*plbpos < plinebuf) || (*plbpos >= plbend)) {
	  // out of range starting position
	  *pwdbuf = 0;
	  return 0;  
  }
  while (pcurpos < plbend) {
    if (iswhite(*pcurpos)) {
	    if (!foundnonwhite) {
        // leading white space--ignore
        pcurpos++;
        continue;
      }
      // this is trailing white space--terminate the string
      pcurpos++;
      *pcurwd = 0;
      *plbpos = pcurpos;
      return 1;
	  }    
	  // copy nonwhite character to word buffer
    *pcurwd++ = *pcurpos++;
    foundnonwhite = 1;
  }
  // at end of line
  *plbpos = pcurpos;				// update line buffer position pointer
  *pcurwd = 0;						// null terminate word buffer
  if (pcurwd > pwdbuf)
    return 1;						// non-zero contents of word buffer
  return 0;
}


int match_word(const WordListEntrytype *pWdList, const char* pWd) {
  // searches a word list for a specific word.  Returns associated ID of word if matched,
  // otherwise null
  const WordListEntrytype* pCurEntry = pWdList;
  
  if (!pWdList || !pWd)
    return 0;

  while (pCurEntry->wordID != WD_EOLIST) {
    // check for match
    if (!strcmp(pWd, pCurEntry->word))
      // have match
      return pCurEntry->wordID;
    pCurEntry++; 
  }
  return 0;
}


void parse_cmd(const char*const plinebuf, const char** plbpos, const char* pcmdwd) {
  int cmdwdID;
  unsigned char filter;
  char reply_buf[20];

  // compare command word against command list and go from there
  cmdwdID = match_word(Cmd_Wd_List, pcmdwd);
  if (!cmdwdID) {
    // no match
    sprintf(spbuf, "?Unknown command \"%s\"\n", pcmdwd);
    sio_putstr(spbuf);
    return;
  }
  
  switch(cmdwdID) {
    case WD_CMD_HELP:
    case WD_CMD_QMARK:
      sio_putstr("Console Commands:\n");
      sio_putstr("  help or ? - display this message\n");
	  sio_putstr("  clrfl - clears the fault log\n");
      sio_putstr("  clrmmask <arglist> - disables console output for specified message types\n");
      sio_putstr("  creset - issues cold reset to payload (cycles back end power)\n");
      sio_putstr("  dispaux - disable 12V auxiliary power front panel input\n");
	  sio_putstr("  dispfl - displays current contents of fault log\n");
      sio_putstr("  dispcb - displays capture buffer\n");
	  sio_putstr("  eeperase - erases EEPROM (will auto-format at next MMC startup)\n");
      sio_putstr("  enapaux - enable 12V auxiliary power front panel input\n");
      sio_putstr("  evtrefresh - retransmit asserted events to IPMI event receiver\n");
      sio_putstr("  getmmask - displays the current message filter setting\n");
      sio_putstr("  getpaux - displays the current auxilliary power enable/disable setting\n");
	  sio_putstr("  gettrim - returns the trim level for MGT voltage regulators\n");
	  sio_putstr("  getv - returns the output level for MGT 1.0V regulators\n");
	  sio_putstr("  ipmbstats - display IPMB statistics\n");
      sio_putstr("  mreset - resets MMC controller\n");
	  sio_putstr("  pinstate - displays the current state on a microcontroller pin\n");
	  sio_putstr("  sensread - returns the value of a sensor\n");
    sio_putstr("  sensrec - displays the sensor record/checksum area\n");
      sio_putstr("  setmmask <arglist> - enables console output for specified message types\n");
	  sio_putstr("  settrim - sets the trim level for MGT voltage regulators\n");
	  sio_putstr("  setv - sets the output level for MGT 1.0V regulators\n");
      sio_putstr("  siorst - reset USART buffers\n");
	  sio_putstr("  slotid - displays Slot ID and IPMB-L address\n");
	  sio_putstr("  testevt - generate IPMI sensor test event\n");
      sio_putstr("  timestats - display the MMC time statistics\n");
      sio_putstr("  version - display the MMC version\n");
      sio_putstr("  wreset - issues warm (CPU) reset to payload\n");
      sio_putstr("\n");
      break; 
    
    case WD_CMD_SETMSGMASK:
    case WD_CMD_CLRMSGMASK:
      // call function to change filter settings
      change_filter_mask(plinebuf, plbpos, cmdwdID);
      break;
    
    case WD_CMD_GETMSGMASK:
      filter = sio_get_txt_filter();
      sio_putstr("Current Console Message Mask Settings:\n");
      sio_putstr("  IPMI event msgs:  ");
      if (filter & TXTFILT_EVENTS)
        sio_putstr("enabled\n");
      else
        sio_putstr("disabled\n");
      sio_putstr("  Standard IPMI cmds:  ");
      if (filter & TXTFILT_IPMI_STD_REQ)
        sio_putstr("enabled\n");
      else
        sio_putstr("disabled\n");
      sio_putstr("  Custom IPMI cmds:  ");
      if (filter & TXTFILT_IPMI_CUSTOM_REQ)
        sio_putstr("enabled\n");
      else
        sio_putstr("disabled\n");
      sio_putstr("  Error/Debug summary msgs:  ");
      if (filter & TXTFILT_DBG_SUMMARY)
        sio_putstr("enabled\n");
      else
        sio_putstr("disabled\n");
      sio_putstr("  Error/Debug detail msgs:  ");
      if (filter & TXTFILT_DBG_DETAIL)
        sio_putstr("enabled\n");
      else
        sio_putstr("disabled\n");
      sio_putstr("  General Informational msgs:  ");
      if (filter & TXTFILT_INFO)
        sio_putstr("enabled\n");
      else
        sio_putstr("disabled\n");

      break; 
    case WD_CMD_WRESET:
      sio_putstr("Issuing Warm Reset command\n");
	    pyldmgr_ipmicmd_fru_ctrl(FRU_CTLCODE_WARM_RST);
		  break;
	
    case WD_CMD_CRESET:
	    sio_putstr("Issuing Cold Reset command\n");
	    pyldmgr_ipmicmd_fru_ctrl(FRU_CTLCODE_COLD_RST);
		  break;
		  
    case WD_CMD_MRESET:
	    sio_putstr("Issuing MMC Reset\n");
      Force_Watchdog_Reset();
	    break;
	
    case WD_CMD_EEPERASE:
	    sio_putstr("Executing this command will cause the entire contents of the EEPROM to be erased.\n");
	    sio_putstr("Are you sure you want to do this (type 'yes'):  ");
		  while (!get_sio_msg_cnt()) Service_Watchdog();
      sio_getln(reply_buf, 18);
      dec_sio_msg_cnt();
	    if (!strcmp(reply_buf, "yes\n")) {
			  sio_putstr("Erasing EEPROM....");
        eepspi_chip_erase();
		    sio_putstr("Erase complete!\n");
		  }			
		  else
		    sio_putstr("Erase aborted\n");
		  break;
		  
		case WD_CMD_SENSREAD:
		  display_sensor_value(plinebuf, plbpos);
		  break;

    case WD_CMD_IPMBSTATS:
	    print_twi_driver_stats();
		  break;
		  
		case WD_CMD_SLOTID:
		  sprintf(spbuf, "uTCA Slot=%i  IPMB-L ADDR=%02Xh\n", twi_state.slotid, twi_state.ipmbl_addr);
      sio_putstr(spbuf);
	    break;
		  
    case WD_CMD_PINSTATE:
      display_pin_val(plinebuf, plbpos, cmdwdID);
	    break;
	
	  case WD_CMD_TESTEVT:
      sensor_test_evt(plinebuf, plbpos, cmdwdID);
      break;
	  
	  case WD_CMD_DISPFL:
      display_fault_log(plinebuf, plbpos, cmdwdID);
	    break;
		
		case WD_CMD_CLRFL:
	    sio_putstr("Executing this command will cause the contents of the fault log to be erased.\n");
	    sio_putstr("Are you sure you want to do this (type 'yes'):  ");
		  while (!get_sio_msg_cnt()) Service_Watchdog();
      sio_getln(reply_buf, 18);
      dec_sio_msg_cnt();
	    if (!strcmp(reply_buf, "yes\n")) {
			  sio_putstr("Erasing Fault Log....");
        erase_fault_log();
		    fault_log_init();
		    sio_putstr("Erase complete!\n");
		  }			
		  else
		    sio_putstr("Erase aborted\n");
		  break;
		case WD_CMD_SETTRIM:
      set_mgt_vtrim(plinebuf, plbpos, cmdwdID);
      break;

		case WD_CMD_GETTRIM:
      get_mgt_vtrim(plinebuf, plbpos, cmdwdID);
      break;

		case WD_CMD_SETV:
      set_mgt_vout(plinebuf, plbpos, cmdwdID);
      break;

		case WD_CMD_GETV:
      get_mgt_vout(plinebuf, plbpos, cmdwdID);
      break;

    case WD_CMD_SIORST:
      sio_buf_reset();
      sio_putstr("SIO Buffer reset complete\r\n");
      break;

    case WD_CMD_GETPAUX:
      if (pyldmgr_state.settings.aux_12V_pwr_select) 
        sio_putstr("Aux 12V input is ENABLED\n");
      else
        sio_putstr("Aux 12V input is DISABLED\n");
      break;

    case WD_CMD_ENAPAUX:
      sio_putstr("Enabling Auxiliary 12V input.\n");
      change_paux(plinebuf, plbpos, cmdwdID, 1);
      break;

    case WD_CMD_DISPAUX:
      sio_putstr("Disabling Auxiliary 12V input.\n");
      change_paux(plinebuf, plbpos, cmdwdID, 0);
      break;

    case WD_CMD_TIMESTATS:
      // display time statistics
      etimestr(reply_buf, TIMESTATREC.uptime);
      sprintf(spbuf, "Up Time:  %s\n", reply_buf);
      sio_putstr(spbuf);
      etimestr(reply_buf, TIMESTATREC.hottime);
      sprintf(spbuf, "Hot Time:  %s\n", reply_buf);
      sio_putstr(spbuf);
      sprintf(spbuf, "Reset Counter:  %li\n", TIMESTATREC.reset_cnt);
      sio_putstr(spbuf);
      etimestr(reply_buf, TIMESTATREC.lastrsttime);
      sprintf(spbuf, "Elapsed Time Since Last Reset:  %s\n", reply_buf);
      sio_putstr(spbuf);
      sprintf(spbuf, "Current MMC Time:  %s\n", systimestr());
      sio_putstr(spbuf);
      break;

    case WD_CMD_DISPCB:
      print_cap_buf();
      break;

    case WD_CMD_VERSION:
      sprintf(spbuf, "MMC Version:  %s\n", MMC_VERSION_STRING);
      sio_putstr(spbuf);
      break;

    case WD_CMD_EVTUPDATE:
      sio_putstr("Retransmitting asserted events\n");
      update_sensor_events();
      break;

    case WD_CMD_SPIF2CNT:
      sprintf(spbuf, "SPI detect secondary check fail count:  %li\n", spi_detect_fail2_ctr);
      sio_putstr(spbuf);
      break;



    case WD_CMD_OHO:
	  sio_putstr("handle override OUT\n");
	  set_handle_position_override(out_open, 0);
	  break;
	  
	case WD_CMD_OHI:
	  sio_putstr("handle override IN\n");
	  set_handle_position_override(in_closed, 0);
	  break;
	
	case WD_CMD_OHR:
	  sio_putstr("handle override released\n");
	  clear_handle_position_override();
	  break;

  case WD_CMD_DISPSENSREC:
    display_sensor_settings_rec();
    break;

    default:
      // should never come here since unrecognized words caught in match phase
      sprintf(spbuf, "?Parser Error, unknown Word ID %i\n", cmdwdID);
      sio_putstr(spbuf);
  }
  
}


void change_filter_mask(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  unsigned char chgmsk = 0;
  char argwd[CONSOLEWDMAXLEN];
  int argwdID;
  
  // parse rest of command line for arguments indicating which mask settings should be changed
  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no arguments on line--display help
    sio_putstr("Valid arguments for message filter set/clear operations:\n");
    sio_putstr("  all - select all filter types\n");
    sio_putstr("  ipmievt - select IPMI event msg filter\n");
    sio_putstr("  ipmistd - select standard IPMI cmd filter\n");
    sio_putstr("  ipmicstm - select custom IPMI cmd filter\n");
    sio_putstr("  dbgsum - select debug summary msg filter\n");
    sio_putstr("  dbgdtl - select debug detail msg filter\n");
    sio_putstr("  info - select eneral informational messages\n");
    return;
  }  
  
  do {
    // parse argument

    argwdID = match_word(Filteropt_Wd_List, argwd);
    if (!argwdID) {
      // no match
      sprintf(spbuf, "?Unknown argument \"%s\"\n", argwd);
      sio_putstr(spbuf);
    }
  
    switch (argwdID) {
      case WD_FILT_IPMIEVT:
        chgmsk |= TXTFILT_EVENTS;
        break;
      case WD_FILT_IPMISTD:
        chgmsk |= TXTFILT_IPMI_STD_REQ;
        break;
      case WD_FILT_IPMICUSTOM:
        chgmsk |= TXTFILT_IPMI_CUSTOM_REQ;
        break;
      case WD_FILT_DBGSUMMARY:
        chgmsk |= TXTFILT_DBG_SUMMARY;
        break;
      case WD_FILT_DBGDETAIL:
        chgmsk |= TXTFILT_DBG_DETAIL;
        break;
      case WD_FILT_INFO:
        chgmsk |= TXTFILT_INFO;
        break;
      case WD_FILT_ALL:
        chgmsk |= 0xff;
        break;
      default:
        // shouldn't come here
        break;
    }    
  } while (get_next_word(plinebuf, plbpos, argwd));
  // check filter mask state
  if (!chgmsk)
    // nothing to do
    return;
    
  if (cmdwdID == WD_CMD_SETMSGMASK)
    sio_set_txt_filter(chgmsk);
  else
    sio_clr_txt_filter(chgmsk);
}  


void display_sensor_value(const char*const plinebuf, const char** plbpos) {
  char argwd[CONSOLEWDMAXLEN];
  
  int sensornum;
  int allflag = 0;
  
  // parse rest of command line for arguments indicating which mask settings should be changed
  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no arguments on line--display help
    sio_putstr("Valid arguments for reading sensors:\n");
    sio_putstr("  all - display all sensor values\n");
    sio_putstr("  <n> - display value for sensor <n>\n");
    return;
  }  
	
	if (!strcmp(argwd, "all")) {
		allflag = 1;
	}
	else {
		// convert arg to sensor number
		sensornum = atoi(argwd);
		if ((sensornum < 1) || (sensornum >= SDRstate.sensor_cnt)) {
		  sscanf(spbuf, "Illegal argument '%s'\n", argwd);
		  sio_putstr(spbuf);
		  return;
		}
	}

  if (allflag) {
	  // display all sensors
	  for (sensornum=0; sensornum<SDRstate.sensor_cnt; sensornum++) {
	    format_sensor_value_str(sensornum, spbuf);
	    sio_putstr(spbuf);
	  }
  }
  else {
	  // display one sensor
	  format_sensor_value_str(sensornum, spbuf);
	  sio_putstr(spbuf);
  }	
}


void format_sensor_value_str(const int SensorNum, char* pbuf) {
	SDR_type_01h_t* pSDR;
	sensor_data_entry_t* pSD;
	char SensorNameBuf[20];         // buffer for sensor name from SDR
	char SensorValBuf[20];
	unsigned short eventmask;
  unsigned char rawvalue;
	
	if ((SensorNum < 0) || (SensorNum >= SDRstate.sensor_cnt)) {
	  // out of range
	  sprintf(pbuf, "(Illegal/Unknown Sensor Index)\n");
		return;
	}
	pSD = &SensorData[SensorNum];
	pSDR = SensorData[SensorNum].pSDR;
	if (!get_sensor_name_str(SensorNum, SensorNameBuf)) {
		sprintf(pbuf, "(Sensor Definition not found)\n");
		return;
	}
	
	// examine SDR to determine sensor type
	switch (pSDR->event_reading_type) {
		case THRESHOLD_EVENT_READING_TYPE:
      if ((SensorNum >= FIRST_ANALOG_SENSOR) && (SensorNum <= LAST_ANALOG_SENSOR))
		    get_analog_voltage_string(pSD->readout_func_arg, SensorValBuf);
      else
        // payload based sensor--use SDR vaues
        get_payload_sensor_string(SensorNum, SensorValBuf);

		  if (pSDR->sensortype == 1)
	      sprintf(pbuf, "%2i  %s:  %s degC\n", SensorNum, SensorNameBuf, SensorValBuf);
		  else
	      sprintf(pbuf, "%2i  %s:  %s V\n", SensorNum, SensorNameBuf, SensorValBuf);
      break;
		
		case DIGITAL_EVENT_READING_TYPE:
		  if ((*(pSD->readout_function))(pSD->readout_func_arg))
	      sprintf(pbuf, "%2i  %s:  active\n", SensorNum, SensorNameBuf);
			else
	      sprintf(pbuf, "%2i  %s:  inactive\n", SensorNum, SensorNameBuf);
      break;
		
		case SEVERITY_EVENT_READING_TYPE:
      rawvalue = pSD->readout_value;
      switch (rawvalue) {
        case 0:
          sprintf(SensorValBuf, "OK");
          break;
        case 1:
          sprintf(SensorValBuf, "Non-Critical");
          break;
        case 2:
          sprintf(SensorValBuf, "Critical");
          break;
        case 3:
          sprintf(SensorValBuf, "Non-Recoverable");
          break;
        default:
          sprintf(SensorValBuf, "Unknown Code 0x%02x", rawvalue);
      }
	    sprintf(pbuf, "%2i  %s:  %s\n", SensorNum, SensorNameBuf, SensorValBuf);
			break;
		
		case SENSOR_SPECIFIC_READING_TYPE:
		  switch(pSDR->sensortype) {
			  case FPGA_CONFIG_SENSOR_TYPE:
			  case MODULE_BOARD_SENSOR_TYPE:
			  case HOTSWAP_SENSOR_TYPE:
			    // read out the event mask
				  eventmask = pSD->cur_masked_comp;
	        sprintf(pbuf, "%2i  %s:  0x%04x\n", SensorNum, SensorNameBuf, eventmask);
				  break;
			  
			  default:
	        sprintf(pbuf, "%2i  %s:  unknown\n", SensorNum, SensorNameBuf);
		      break;
			  
		  }
		  break;
		  
		default:
	    sprintf(pbuf, "%2i  %s:  unknown\n", SensorNum, SensorNameBuf);
		  break;
		
	}
}


void display_pin_val(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  char argwd[CONSOLEWDMAXLEN];
  int pinid;
  int pinstate;
  
  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no arguments on line--display help
	  sio_putstr("Usage:  pinstate <n>, where <n> is a pin number in the range 0-31 for PA00-PA31, and 32-63 for PB00-PB31\n");
	  return;
  }	
	pinid = atoi(argwd);
	if ((pinid < 0) || (pinid > 63)) {
		sio_putstr("? Invalid pin number\n");
		return;
	}
	pinstate = gpio_get_pin_value(pinid);
	sprintf(spbuf, "Pin %i Value = %i ", pinid, pinstate);
	sio_putstr(spbuf);
	if (gpio_get_pin_oe_setting(pinid))
    sio_putstr("(output)\n");
	else
	  sio_putstr("(input)\n");
}


void sensor_test_evt(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  char argwd[CONSOLEWDMAXLEN];
  int sensornum;
  int offsetnum;
  event_assertion_type_t evtype;
  ipmb_msg_desc_t evtmsg;
  
  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no arguments on line--display help
	  sio_putstr("Usage:  testevt <sensor num> <offset val> <a/d>\n");
	  return;
  }	

  sensornum = atoi(argwd);
  if ((sensornum < 1) || (sensornum >= SDRstate.sensor_cnt)) {
	  sio_putstr("? Sensor number out of range\n");
	  return;
  }

  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no argument--missing offset val
	  sio_putstr("? Missing event offset value\n");
	  return;
  }	

  offsetnum = atoi(argwd);
  if ((offsetnum < 0) || (offsetnum >= 15)) {
	  sio_putstr("? Offset number out of range\n");
	  return;
  }

  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no argument--assume assertion
	  evtype = assert;
  }	
  else {
	  switch (argwd[0]) {
	    case 'a':
		    evtype = assert;
			  break;
			case 'd':
			  evtype = deassert;
			  break;
			default:
			  sio_putstr("? Unknown event type\n");
			  return;
		 }
  }
  
  if (!SensorData[sensornum].pSDR) {
	  sio_putstr("? Cannot find SDR entry, unable to send sensor event\n");
	  return;
  }

	ipmb_init_req_hdr(evtmsg.buf, ipmb_get_event_rcvr_ipmb_addr(), NETFN_SE, ipmb_get_event_rcvr_lun(), 0);
  evtmsg.buf[5] = IPMICMD_SE_PLATFORM_EVENT;
  evtmsg.buf[6] = 0x04;           // event message revision
  evtmsg.buf[7] = SensorData[sensornum].pSDR->sensortype;      // sensor type
  evtmsg.buf[8] = sensornum & 0xff;
  if (evtype == assert)
    evtmsg.buf[9] = 0x7f & SensorData[sensornum].pSDR->event_reading_type;   // bit 7=0 for assertion, event reading type should be 0x01 (threshold)
  else
    evtmsg.buf[9] = 0x80 | SensorData[sensornum].pSDR->event_reading_type;   // bit 7=1 for deassertion, event reading type should be 0x01 (threshold)
  evtmsg.buf[10] = ((unsigned char) offsetnum & 0xf) | 0x50;     // trigger reading in byte 2, threshold in byte 3, event offset bits 0-3
  evtmsg.buf[11] = 0xff;
  evtmsg.buf[12] = 0xff;
  evtmsg.buf[13] = calc_ipmi_xsum(&evtmsg.buf[3], 10);          // message checksum
  evtmsg.len = 14;
  ipmb_send_request(&evtmsg, NULL);
}


void display_fault_log(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  int curentry, startentry;
  fault_log_entry_t fault_log;

  
  startentry = fault_log_get_last_entry_index();
  if (startentry == -1) {
	  sio_putstr("Fault log is empty\n");
	  return;
  }
  curentry = startentry;
  sio_putstr("Fault Log Entries: (most recent first)\n");
  do {
	  get_fault_log_entry(curentry, &fault_log); 
	  sprintf(spbuf, "Sensor=%02i  Offset=0x%02x  Value=0x%02x  Thr=0x%02x  Time=%s", fault_log.sensor_num, fault_log.event_offset, fault_log.sensor_val,
	    fault_log.thresh_val, ctime((const time_t*) &fault_log.systime));
	  sio_putstr(spbuf);
	  if (!curentry)
	    curentry = FAULT_LOG_ENTRY_CNT-1;      // wrap
		else
		  curentry--;                           // decrement
  } while (curentry != startentry);		
}

void set_mgt_vtrim(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  char argwd[CONSOLEWDMAXLEN];
  int eepflag, curflag, trimflag, trimarg, regflag, argerr;
  int argwdID;
  int getflag;
  unsigned int LDO_ID;
  MGT_Vreg_settings_t Vreg_settings;
  unsigned short trimcode;
  
  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no arguments on line--display help
    sio_putstr("Usage:  settrim <VregID> {-c} {-e} <trimval>, where:\n");
    sio_putstr("  <VregID> - Voltage regulator ID, one of the following:  \"1.0VL\", \"1.0VR\", \"1.2VL\", or \"1.2VR\"\n");
    sio_putstr("  -c - optional flag to update current settings\n");
    sio_putstr("  -e - optional flag to update eeprom settings\n");
    sio_putstr("  <trimval> - required trim value, either -5, -3, -1, off, 1, 3, or 5\n");
    sio_putstr("Example:  \"settrim 1.0VR -c 3\" to set the trim value on the right 1.0V regulator to +3%\n");
    return;
  }

  eepflag = 0;
  curflag = 0;
  regflag = 0;
  trimflag = 0;
  argerr = 0;

  do {
    // parse argument
    argwdID = match_word(SetTrim_Wd_List, argwd);
    if (!argwdID) {
      // no match--see if it is a numeric value
      trimarg = atoi(argwd);
      switch (trimarg) {
        case -5:
          trimcode = MGTTRIM_m5pct;
          trimflag = 1;
          break;
        case -3:
          trimcode = MGTTRIM_m3pct;
          trimflag = 1;
          break;
        case -1:
          trimcode = MGTTRIM_m1pct;
          trimflag = 1;
          break;
        case 1:
          trimcode = MGTTRIM_p1pct;
          trimflag = 1;
          break;
        case 3:
          trimcode = MGTTRIM_p3pct;
          trimflag = 1;
          break;
        case 5:
          trimcode = MGTTRIM_p5pct;
          trimflag = 1;
          break;
        default:
        // unknown value
        sprintf(spbuf, "?Unknown argument \"%s\"\n", argwd);
        sio_putstr(spbuf);
        argerr = 1;
      }
    }
    else {
      switch (argwdID) {
        case WD_SETTRIM_EEP:
          eepflag = 1;
          break;
        case WD_SETTRIM_CUR:
          curflag = 1;
          break;

        case WD_SETTRIM_OFF:
          trimcode = MGTTRIM_0pct;
          trimflag = 1;
          break;

        case WD_SETTRIM_1P0VL:
          regflag = 1;
          LDO_ID = MGTLDO_1p0VL;
          break;
 
        case WD_SETTRIM_1P0VR:
           regflag = 1;
           LDO_ID = MGTLDO_1p0VR;
           break;

        case WD_SETTRIM_1P2VL:
          regflag = 1;
          LDO_ID = MGTLDO_1p2VL;
          break;
 
       case WD_SETTRIM_1P2VR:
          regflag = 1;
          LDO_ID = MGTLDO_1p2VR;
          break;

        default:
          // argument not correct for this command
          sprintf(spbuf, "?Invalid argument \"%s\"\n", argwd);
          sio_putstr(spbuf);
          argerr = 1;
          break;
      }
    }
    getflag = get_next_word(plinebuf, plbpos, argwd);
  } while (!argerr && getflag);                        // continue loop as long as there are words being fetched from command line and no errors
  if (argerr) {
    sio_putstr("Command aborted\n");
    return;
  }
  if (!trimflag || !regflag || (!eepflag && !curflag)) {
    sio_putstr("?error, missing argument\n");
    return;
  }

  if (curflag) {
    // update current regulator setting
    apply_trim_settings_to_reg(trimcode, LDO_ID, &(pyldmgr_state.settings.MGT_Vreg));
    apply_trim_settings_to_pins(trimcode, LDO_ID);
  }

  if (eepflag) {
    // update setting in EEPROM (takes effect at next MMC startup)
    while (eepspi_chk_write_in_progress()) Service_Watchdog();
    eepspi_read((unsigned char*) &(Vreg_settings.uint16), PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void*) (&pyldmgr_state.settings.MGT_Vreg.uint16) - (void *) &pyldmgr_state.settings)), 2);
    apply_trim_settings_to_reg(trimcode, LDO_ID, &Vreg_settings);
    eepspi_write((unsigned char*) &(Vreg_settings.uint16), PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void*) (&pyldmgr_state.settings.MGT_Vreg.uint16) - (void *) &pyldmgr_state.settings)), 2);
  }
}


void get_mgt_vtrim(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  MGT_Vreg_settings_t Vreg_settings;
  // current and eeprom trim settings
  sio_putstr("Current MGT Voltage Regulator Trim Settings:\n");
  display_mgt_trim_settings(&(pyldmgr_state.settings.MGT_Vreg));
  sio_putstr("EEPROM MGT Voltage Regulator Trim Settings:\n");
  while (eepspi_chk_write_in_progress()) Service_Watchdog();
  eepspi_read((unsigned char*) &(Vreg_settings.uint16), PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void*) (&pyldmgr_state.settings.MGT_Vreg.uint16) - (void *) &pyldmgr_state.settings)), 2);
  display_mgt_trim_settings(&Vreg_settings);
}


void set_mgt_vout(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  char argwd[CONSOLEWDMAXLEN];
  int eepflag, curflag, levelflag, regflag;
  int argwdID;
  unsigned int LDO_ID;
  MGT_Vreg_settings_t Vreg_settings;
  unsigned short levelcode;
  
  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no arguments on line--display help
    sio_putstr("Usage:  setv <VregID> {-c} {-e} <vlevel>, where:\n");
    sio_putstr("  <VregID> - Voltage regulator ID, either \"1.0VL\" or \"1.0VR\"\n");
    sio_putstr("  -c - optional flag to update current settings\n");
    sio_putstr("  -e - optional flag to update eeprom settings\n");
    sio_putstr("  <vlevel> - \"low\" for 1.00V, and \"high\" for a 1.05V setpoint\n");
    sio_putstr("Example:  \"setv 1.0VR -c high\" to set the output level on the right MGT 1.0V regulator to 1.05V\n");
    return;
  }

  eepflag = 0;
  curflag = 0;
  regflag = 0;
  levelflag = 0;

  do {
    // parse argument
    argwdID = match_word(SetTrim_Wd_List, argwd);
    if (!argwdID) {
      // no match
      sprintf(spbuf, "?Unknown argument \"%s\", command aborted\n", argwd);
      sio_putstr(spbuf);
      return;
    }
    else {
      switch (argwdID) {
        case WD_SETTRIM_EEP:
          eepflag = 1;
          break;

        case WD_SETTRIM_CUR:
          curflag = 1;
          break;

        case WD_SETVOLT_LOW:
          levelcode = MGTVSET_low;
          levelflag = 1;
          break;

        case WD_SETVOLT_HIGH:
          levelcode = MGTVSET_high;
          levelflag = 1;
          break;

        case WD_SETTRIM_1P0VL:
          regflag = 1;
          LDO_ID = MGTLDO_1p0VL;
          break;
        
        case WD_SETTRIM_1P0VR:
          regflag = 1;
          LDO_ID = MGTLDO_1p0VR;
          break;

        default:
          // argument not correct for this command
          sprintf(spbuf, "?Invalid argument \"%s\", command aborted \n", argwd);
          sio_putstr(spbuf);
          break;
      }
    }
  } while (get_next_word(plinebuf, plbpos, argwd));

  if (!levelflag || !regflag || (!eepflag && !curflag)) {
    sio_putstr("?error, missing argument\n");
    return;
  }

  if (curflag) {
    // update current regulator setting
    apply_voltage_settings_to_reg(levelcode, LDO_ID, &(pyldmgr_state.settings.MGT_Vreg));
    apply_voltage_settings_to_pins(levelcode, LDO_ID);
  }

  if (eepflag) {
    // update setting in EEPROM (takes effect at next MMC startup)
    while (eepspi_chk_write_in_progress()) Service_Watchdog();
    eepspi_read((unsigned char*) &(Vreg_settings.uint16), PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void*) (&pyldmgr_state.settings.MGT_Vreg.uint16) - (void *) &pyldmgr_state.settings)), 2);
    apply_voltage_settings_to_reg(levelcode, LDO_ID, &Vreg_settings);
    eepspi_write((unsigned char*) &(Vreg_settings.uint16), PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void*) (&pyldmgr_state.settings.MGT_Vreg.uint16) - (void *) &pyldmgr_state.settings)), 2);
  }
}


void get_mgt_vout(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  MGT_Vreg_settings_t Vreg_settings;
  // current and eeprom trim settings
  sio_putstr("Current MGT 1.0 Voltage Regulator Setpoints:\n");
  if (pyldmgr_state.settings.MGT_Vreg.bits.left1p0_set == MGTVSET_high)
    sio_putstr("  MGT Left:  1.05V\n");
  else
    sio_putstr("  MGT Left:  1.00V\n");
  if (pyldmgr_state.settings.MGT_Vreg.bits.right1p0_set == MGTVSET_high)
    sio_putstr("  MGT Right:  1.05V\n");
  else
    sio_putstr("  MGT Right:  1.00V\n");
  while (eepspi_chk_write_in_progress()) Service_Watchdog();
  eepspi_read((unsigned char*) &(Vreg_settings.uint16), PAYLDMGR_AREA_BYTE_OFFSET + ((long) ((void*) (&pyldmgr_state.settings.MGT_Vreg.uint16) - (void *) &pyldmgr_state.settings)), 2);
  sio_putstr("EEPROM MGT 1.0 Voltage Regulator Setpoints:\n");
  if (Vreg_settings.bits.left1p0_set == MGTVSET_high)
    sio_putstr("  MGT Left:  1.05V\n");
  else
    sio_putstr("  MGT Left:  1.00V\n");
  if (Vreg_settings.bits.right1p0_set == MGTVSET_high)
    sio_putstr("  MGT Right:  1.05V\n");
  else
    sio_putstr("  MGT Right:  1.00V\n");
}


void display_mgt_trim_settings(const MGT_Vreg_settings_t*const pvrs) {
  // displays settings
  sprintf(spbuf, "  MGT Left 1.0V:  %s\n", TrimCodeStr[pvrs->bits.left1p0_margin]);
  sio_putstr(spbuf);
  sprintf(spbuf, "  MGT Right 1.0V:  %s\n", TrimCodeStr[pvrs->bits.right1p0_margin]);
  sio_putstr(spbuf);
  sprintf(spbuf, "  MGT Left 1.2V:  %s\n", TrimCodeStr[pvrs->bits.left1p2_margin]);
  sio_putstr(spbuf);
  sprintf(spbuf, "  MGT Right 1.2V:  %s\n", TrimCodeStr[pvrs->bits.right1p2_margin]);
  sio_putstr(spbuf);
}


void change_paux(const char*const plinebuf, const char** plbpos, const int cmdwdID, const int paux_enable) {
  nonvolatile_settings_t eepsettings;

  while (eepspi_chk_write_in_progress()) Service_Watchdog();
  eepspi_read((unsigned char*) &eepsettings, PAYLDMGR_AREA_BYTE_OFFSET, sizeof(nonvolatile_settings_t));
  eepsettings.aux_12V_pwr_select = paux_enable ? 1 : 0;
  eepspi_write((unsigned char*) &eepsettings, PAYLDMGR_AREA_BYTE_OFFSET, sizeof(nonvolatile_settings_t));
  sio_putstr("Setting updated in EEPROM.  It will take effect the next time the MMC restarts.\n");
}


void etimestr(char* sbuf, unsigned int timesec) {
  // formats elapsed time in seconds into string of type 'DD days HH:MM:SS'
  unsigned long days, hours, minutes, seconds, remainder;

  days = timesec / (24 * 3600);
  remainder = timesec - days*24*3600;
  hours = remainder / 3600;
  remainder -= hours*3600;
  minutes = remainder / 60;
  seconds = remainder - minutes*60;

  sprintf(sbuf, "%li days %02li:%02li:%02li", days, hours, minutes, seconds); 
}

void display_sensor_settings_rec(void) {
  // read and display the sensor settings record ID area
  int i;
  sensor_settings_record_ID_t settingsrec;

  // fill record from EEPROM
  while (eepspi_chk_write_in_progress()) Service_Watchdog();
  eepspi_read((unsigned char*) &(settingsrec), SENSOR_SETTINGS_BYTE_OFFSET, sizeof(sensor_settings_record_ID_t));
  sio_putstr("Record Name Field:\n  Hex:  ");
  for (i=0; i<SENSOR_SETTING_NAMELEN; i++) {
    sprintf(spbuf,"%02X ", (unsigned char) settingsrec.recname[i]);
    sio_putstr(spbuf);
  }
  sio_putstr("\n  String:  \"");
  for (i=0; i<SENSOR_SETTING_NAMELEN; i++) {
    if (isprint(settingsrec.recname[i]))
      sio_putc(settingsrec.recname[i]);
    else
      break;
  } 
  sio_putstr("\"\nChecksum Field (Hex):  ");
  for (i=0; i<4; i++) {
    sprintf(spbuf, "%02X ", settingsrec.xsum[i]);
    sio_putstr(spbuf);
  } 
  sio_putstr("\n\n");
}

