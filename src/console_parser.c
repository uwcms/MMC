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
#define WD_CMD_TIMESTATS              (31)
#define WD_CMD_DISPCB                 (32)
#define WD_CMD_VERSION                (33)
#define WD_CMD_EVTUPDATE              (34)
#define WD_CMD_BKENDPWR               (29)
#define WD_CMD_FWLOAD                 (30)
#define WD_CMD_OHO                    (46)
#define WD_CMD_OHI                    (47)
#define WD_CMD_OHR                    (48)

// Word IDs for filter options
#define WD_FILT_IPMIEVT               (5)
#define WD_FILT_IPMISTD               (6)
#define WD_FILT_IPMICUSTOM            (7)
#define WD_FILT_DBGSUMMARY            (8)
#define WD_FILT_DBGDETAIL             (9)
#define WD_FILT_INFO                  (12)
#define WD_FILT_ALL                   (10)

/*
// Word IDs for setting boot mode options
#define WD_SETBTMODE_EEP              (26)
#define WD_SETBTMODE_CUR              (27)
#define WD_SETBTMODE_MMODE            (28)
*/

const WordListEntrytype Cmd_Wd_List[] = {
  {WD_CMD_HELP, "help"},
  {WD_CMD_QMARK, "?"},
  {WD_CMD_SETMSGMASK, "setmmask"},
  {WD_CMD_CLRMSGMASK, "clrmmask"},
  {WD_CMD_GETMSGMASK, "getmmask"},
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
  {WD_CMD_BKENDPWR, "bkendpwr" },
  {WD_CMD_FWLOAD, "fwload" },
  {WD_CMD_TIMESTATS, "timestats"},
  {WD_CMD_DISPCB, "dispcb"},
  {WD_CMD_VERSION, "version"},
  {WD_CMD_EVTUPDATE, "evtrefresh"},
  {WD_CMD_OHO, "ohout"},
  {WD_CMD_OHI, "ohin"},
  {WD_CMD_OHR, "ohrel"},
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

/*
const WordListEntrytype Set_BtMode_Wd_List[] = {
  {WD_SETBTMODE_EEP, "-e"},
  {WD_SETBTMODE_CUR, "-c"	},
  {WD_SETBTMODE_MMODE, "-m"} };
*/

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
void bkendpwr_cmd(const char*const plinebuf, const char** plbpos, const int cmdwdID);
void etimestr(char* sbuf, unsigned int timesec);
void eventmask2str(int sensortype, unsigned short eventmask, char* pbuf);


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

// The list below really needs to be sorted and cleaned up.
//CTP7 version
      sio_putstr("Console Commands:\n");
      sio_putstr("  help or ? - display this message\n");
      sio_putstr("  bkendpwr - enable or disable back end power\n");
      sio_putstr("  clrfl - clears the fault log\n");
      sio_putstr("  clrmmask <arglist> - disables console output for specified message types\n");
      sio_putstr("  creset - issues cold reset to payload (cycles back end power)\n");
      sio_putstr("  dispfl - displays current contents of fault log\n");
      sio_putstr("  dispcb - displays capture buffer\n");
      sio_putstr("  eeperase - erases EEPROM (will auto-format at next MMC startup)\n");
      sio_putstr("  evtrefresh - retransmit asserted events to IPMI event receiver\n");
      sio_putstr("  fwload - signal FPGA to reload firmware image\n");
      sio_putstr("  getmmask - displays the current message filter setting\n");
      sio_putstr("  ipmbstats - display IPMB statistics\n");
      sio_putstr("  mreset - resets MMC controller\n");
      sio_putstr("  pinstate - displays the current state on a microcontroller pin\n");
      sio_putstr("  sensread - returns the value of a sensor\n");
      sio_putstr("  slotid - displays Slot ID and IPMB-L address\n");
      sio_putstr("  testevt - generate IPMI sensor test event\n");
      sio_putstr("  timestats - display the MMC time statistics\n");
      sio_putstr("  version - display the MMC version\n");
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
      sprintf(spbuf, "Request Messages deleted as undeliverable:  %u\n", ipmb_get_req_delete_count());
      sio_putstr(spbuf);
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
		  
    case WD_CMD_BKENDPWR:
      bkendpwr_cmd(plinebuf, plbpos, cmdwdID);
      break;

    case WD_CMD_FWLOAD:
      sio_putstr("Initiating FPGA Load operation.\n");
      pyldmgr_ipmicmd_fru_ctrl(FRU_CTLCODE_REBOOT);
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
  char MaskbitDecodeBuf[120];
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
		  get_analog_voltage_string(pSD->readout_func_arg, SensorValBuf);
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
      // get a string containing the decoded mask bits by sensor type
      eventmask = pSD->cur_masked_comp;
      eventmask2str(pSDR->sensortype, eventmask, MaskbitDecodeBuf);
      switch(pSDR->sensortype) {
        case FPGA_CONFIG_SENSOR_TYPE:
        case MODULE_BOARD_SENSOR_TYPE:
        case HOTSWAP_SENSOR_TYPE:
        // read out the event mask
        sprintf(pbuf, "%2i  %s:  0x%04x %s\n", SensorNum, SensorNameBuf, eventmask, MaskbitDecodeBuf);
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



void bkendpwr_cmd(const char*const plinebuf, const char** plbpos, const int cmdwdID) {
  char argwd[CONSOLEWDMAXLEN];
  ipmb_msg_desc_t evtmsg;
  int onflag = 0;
  int offflag = 0;
  
  // parse rest of command line for arguments indicating which mask settings should be changed
  if (!get_next_word(plinebuf, plbpos, argwd)) {
    // no arguments on line--display help
    sio_putstr("Usage:  bkendpwr [on|off]\n");
    sio_putstr("Commands changes the current on/off setting of the backend power without altering the\n");
    sio_putstr("setting stored in EEPROM, which is applied when payload power (+12V) is detected.\n");
    return;
  }

	if (!strcmp(argwd, "on"))
  	onflag = 1;
  else
    if (!strcmp(argwd, "off")) 
      offflag = 1;

  if (!onflag && !offflag) {
    sprintf(spbuf, "?Error, unknown argument '%s'\n", argwd);
    sio_putstr(spbuf);
    return;
  }

  if (onflag) {
    evtmsg.buf[6] = 0x01;           // set the bit in the command that turns on backend power
    printf("Enabling back end power\n");
  }
  else {
    evtmsg.buf[6] = 0x00;
    printf("Disabling back end power\n");
  }
  pyldmgr_ipmicmd_backend_pwr(&evtmsg);
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

void eventmask2str(int sensortype, unsigned short eventmask, char* pbuf) {
  // decodes the asserted events to a readable string buffer for certain discrete sensor types
  const char* HotSwapEventStr[] = {
    "[HdlClsd] ",
    "[HdlOpen] ",
    "[Quiesced] ",
    "[BkndFail] ",
    "[BkndShtdn] "
  };
  
  const char* FPGACfgEventStr[] = {
    "[LdDone] ",
    "[FWEvnt] ",
    "[SPIdet0] ",
    "[ReqCfg0] ",
    "[CfgRdy0] ",
    "[SPIdet1] ",
    "[ReqCfg1] ",
    "[CfgRdy1] ",
    "[SPIdet2] ",
    "[ReqCfg2] ",
    "[CfgRdy2] "
  };
  
  char* pcur = pbuf;
  int slen, i1;
  
  *pcur = 0;          // initialize string as null terminated
  
  switch (sensortype) {
    case FPGA_CONFIG_SENSOR_TYPE:
    for (i1 = FPGACFGEV_CFGRDY2; i1 >= FPGACFGEV_LOAD_DONE; i1--)
    if (eventmask & (1<<i1)) {
      // append string onto the buffer
      slen = strlen(FPGACfgEventStr[i1]);
      strncpy(pcur, FPGACfgEventStr[i1], slen);
      pcur += slen;
    }
    *pcur = 0;  // null terminate
    break;
    
    case HOTSWAP_SENSOR_TYPE:
    for (i1 = HOTSWAP_EVENT_BACKEND_SHUTDOWN; i1 >= HOTSWAP_EVENT_HANDLE_CLOSED; i1--)
    if (eventmask & (1<<i1)) {
      // append string onto the buffer
      slen = strlen(HotSwapEventStr[i1]);
      strncpy(pcur, HotSwapEventStr[i1], slen);
      pcur += slen;
    }
    *pcur = 0;  // null terminate
    break;
    
    default:
    // noting to encode
    return;
  }
}


