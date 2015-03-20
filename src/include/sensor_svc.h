/*
 * sdr.h
 *
 *  Created on: Oct 26, 2010
 *      Author: tgorski
 */

#ifndef SENSOR_SVC_H_
#define SENSOR_SVC_H_

#include "nonvolatile.h"
#include "ipmb_svc.h"

#define MAX_SENSOR_CNT			    (31)
#define MAX_SDR_REC_CNT         (MAX_SENSOR_CNT+1)
#define SDR_RECORD_LENGTH				(64)
#define SDR_SIZE						    (MAX_SDR_REC_CNT*SDR_RECORD_LENGTH)

#if SDR_AREA_SIZE < SDR_SIZE
#error "SDR area too small in nonvolatile storage"
#endif

#define LOWBYTE             (0)
#define HIGHBYTE            (1)

typedef struct {
  unsigned char recID_LSB;
  unsigned char recID_MSB;
  unsigned char SDRversion;
  unsigned char rectype;
  unsigned char reclength;
} SDR_entry_hdr_t;

typedef struct {
  SDR_entry_hdr_t hdr;
  unsigned char ownerID;										        // 6
  unsigned char ownerLUN;										        // 7
  unsigned char sensornum;										      // 8
  unsigned char entityID;										        // 9
  unsigned char entityinstance;									    // 10
  unsigned char sensorinit;										      // 11
  unsigned char sensorcap;										      // 12
  unsigned char sensortype;										      // 13
  unsigned char event_reading_type;								  // 14
  unsigned char assertion_event_mask[2];            // 15-16
  unsigned char deassertion_event_mask[2];          // 17-18
  unsigned char readable_threshold_mask;            // 19
  unsigned char settable_threshold_mask;            // 20
  unsigned char sensor_units_1;									    // 21
  unsigned char sensor_units_2;									    // 22
  unsigned char sensor_units_3;									    // 23
  unsigned char linearization;									    // 24
  unsigned char M;												          // 25
  unsigned char M_tol;											        // 26
  unsigned char B;												          // 27
  unsigned char B_accuracy;										      // 28
  unsigned char acc_exp_sensor_dir;								  // 29
  unsigned char Rexp_Bexp;										      // 30
  unsigned char analog_flags;									      // 31
  unsigned char nominal_reading;								    // 32
  unsigned char normal_max;										      // 33
  unsigned char normal_min;										      // 34
  unsigned char sensor_max_reading;								  // 35
  unsigned char sensor_min_reading;								  // 36
  unsigned char upper_nonrecover_thr;							  // 37
  unsigned char upper_critical_thr;								  // 38
  unsigned char upper_noncritical_thr;							// 39
  unsigned char lower_nonrecover_thr;							  // 40
  unsigned char lower_critical_thr;								  // 41
  unsigned char lower_noncritical_thr;							// 42
  unsigned char pos_thr_hysteresis;								  // 43
  unsigned char neg_thr_hysteresis;								  // 44
  unsigned char reserved1;										      // 45
  unsigned char reserved2;										      // 46
  unsigned char OEM;											          // 47
  unsigned char IDtypelen;										      // 48
  char IDstring[16];								                // 49-64 (0x40 length max)
} SDR_type_01h_t;


typedef struct {
  SDR_entry_hdr_t hdr;
  unsigned char slaveaddr;
  unsigned char chnum;
  unsigned char power_notification_global_init;
  unsigned char device_cap;
  unsigned char reserved[3];
  unsigned char entityID;
  unsigned char entityinstance;
  unsigned char OEM;
  unsigned char IDtypelen;
  char IDstring[16];
} SDR_type_12h_t;


typedef struct {
	SDR_entry_hdr_t hdr;
	unsigned char body[59];
} SDR_generic_type_t;

typedef SDR_generic_type_t SDR_table_t[MAX_SDR_REC_CNT];

// sensor definitions
#define HOTSWAP_SENSOR						      (0)
#define LOWER_TEMP_SENSOR						    (1)
#define UPPER_TEMP_SENSOR		  	 	      (2)
#define PAYLOAD_12V_SENSOR					    (3)
#define BKEND_3p3V_SENSOR					      (4)
#define BKEND_2p5V_SENSOR               (5)
#define BKEND_2p0V_SENSOR               (6)
#define BKEND_1p8V_SENSOR               (7)
#define BKEND_1p5V_SENSOR               (8)
#define BKEND_1p0VDD_SENSOR             (9)
#define BKEND_1p5VLDO_SENSOR            (10)
#define BKEND_1p3VLDO_SENSOR            (11)
#define BKEND_1p0LMGT_SENSOR            (12)
#define BKEND_1p2LMGT_SENSOR            (13)
#define BKEND_1p0RMGT_SENSOR            (14)
#define BKEND_1p2RMGT_SENSOR            (15)
#define AUX_12V_SENSOR                  (16)
#define PGOOD_SENSOR                    (17)
#define ALARM_LEVEL_SENSOR              (18)
#define FPGA_CONFIG_SENSOR              (19)
// payload-based sensors (ZYNQ and V7 on CTP7)
#define ZYNQ_DIE_TEMP_SENSOR            (20)
#define V7_DIE_TEMP_SENSOR              (21)
#define ZYNQ_1P0VDD_SENSOR              (22)
#define V7_1P0VDD_SENSOR                (23)

#define FIRST_ANALOG_SENSOR             LOWER_TEMP_SENSOR               // first sensor that requires scaling from ADC reading to 8-bit IPMI sensor value
#define LAST_ANALOG_SENSOR              AUX_12V_SENSOR                  // last sensor that requires scaling from ADC reading to 8-bit IPMI sensor value
#define FIRST_SENSOR_NUMBER             HOTSWAP_SENSOR                  // first sensor in list
#define LAST_SENSOR_NUMBER              V7_1P0VDD_SENSOR                // last sensor in list--NOTE, sensor numbers should be contiguous with NO GAPS so that for loops work to scan through them

// payload based sensor offsets for readout
#define PBS_OFFSET_ZYNQ_DIE             (0)
#define PBS_OFFSET_ZYNQ_1P0VDD          (1)
#define PBS_OFFSET_V7_DIE               (2)
#define PBS_OFFSET_V7_1P0VDD            (3)
#define PBS_OFFSET_MAX_VAL              (7)

#define SDR_MMC								          (0)
#define SDR_HOTSWAP_SENSOR					    (1+HOTSWAP_SENSOR)
#define SDR_LOWER_TEMP_SENSOR					  (1+LOWER_TEMP_SENSOR)
#define SDR_UPPER_TEMP_SENSOR					  (1+UPPER_TEMP_SENSOR)
#define SDR_PAYLOAD_12V_SENSOR				  (1+PAYLOAD_12V_SENSOR)
#define SDR_BKEND_3p3V_SENSOR					  (1+BKEND_3p3V_SENSOR)
#define SDR_BKEND_2p5V_SENSOR           (1+BKEND_2p5V_SENSOR)
#define SDR_BKEND_2p0V_SENSOR           (1+BKEND_2p0V_SENSOR)
#define SDR_BKEND_1p8V_SENSOR           (1+BKEND_1p8V_SENSOR)
#define SDR_BKEND_1p5V_SENSOR           (1+BKEND_1p5V_SENSOR)
#define SDR_BKEND_1p0VDD_SENSOR         (1+BKEND_1p0VDD_SENSOR)
#define SDR_BKEND_1p5VLDO_SENSOR        (1+BKEND_1p5VLDO_SENSOR)
#define SDR_BKEND_1p3VLDO_SENSOR        (1+BKEND_1p3VLDO_SENSOR)
#define SDR_BKEND_1p0LMGT_SENSOR        (1+BKEND_1p0LMGT_SENSOR)
#define SDR_BKEND_1p2LMGT_SENSOR        (1+BKEND_1p2LMGT_SENSOR)
#define SDR_BKEND_1p0RMGT_SENSOR        (1+BKEND_1p0RMGT_SENSOR)
#define SDR_BKEND_1p2RMGT_SENSOR        (1+BKEND_1p2RMGT_SENSOR)
#define SDR_AUX_12V_SENSOR              (1+AUX_12V_SENSOR)
#define SDR_PGOOD_SENSOR                (1+PGOOD_SENSOR)
#define SDR_ALARM_LEVEL_SENSOR          (1+ALARM_LEVEL_SENSOR)
#define SDR_FPGA_CONFIG_SENSOR          (1+FPGA_CONFIG_SENSOR)
#define SDR_ZYNQ_DIE_TEMP_SENSOR        (1+ZYNQ_DIE_TEMP_SENSOR)
#define SDR_V7_DIE_TEMP_SENSOR          (1+V7_DIE_TEMP_SENSOR)
#define SDR_ZYNQ_1P0VDD_SENSOR          (1+ZYNQ_1P0VDD_SENSOR)
#define SDR_V7_1P0VDD_SENSOR            (1+V7_1P0VDD_SENSOR)

// SDR OEM field mask and codes
#define SDR_OEM_FIELD_DEV_SENSOR_MASK   (0x1f)
#define SDR_OEM_FIELD_SYSMGR_CODE_MASK  (0x07)
#define SDR_OEM_FIELD_SYSMGR_CODE_POS   (5)

#define SDR_SYSMGR_CODE_NONE            (0)
#define SDR_SYSMGR_CODE_CONFIG          (1)
#define SDR_SYSMGR_CODE_ALARM_LVL       (2)
#define SDR_SYSMGR_CODE_HOTSWAP         (3)

#define END_OF_SDR_LIST						      (0xFFFF)

// common constants found in SDR records
#define SDR_VERSION							        (0x51)
#define ENTITY_ID							          (0xc1)
#define ENTITY_INSTANCE_BASE				    (0x60)

#define THRESHOLD_EVENT_READING_TYPE    (0x01)
#define DIGITAL_EVENT_READING_TYPE      (0x03)
#define SEVERITY_EVENT_READING_TYPE     (0x07)
#define SENSOR_SPECIFIC_READING_TYPE    (0x6f)

#define TEMPERATURE_SENSOR_TYPE         (0x01)
#define VOLTAGE_SENSOR_TYPE             (0x02)
#define POWER_SUPPLY_SENSOR_TYPE        (0x08)
#define MODULE_BOARD_SENSOR_TYPE        (0x15)
#define FPGA_CONFIG_SENSOR_TYPE         (0xc0)
#define HOTSWAP_SENSOR_TYPE             (0xf2)

// context codes for when sensors are considered active
#define SENSORACTV_ALWAYS                   (0)                 // active whenever MMC is alive
#define SENSORACTV_PAYLOADPWRON             (1)                 // active when payload power is on
#define SENSORACTV_BACKENDPWRON             (2)                 // active when backend power is on
#define SENSORACTV_NEVER                    (3)                 // never active for MMC event generation purposes
#define SENSORACTV_PAYLOADSOURCED           (4)                 // active when sourced from payload
#define SENSORACTV_AUXPAYLOADON             (5)                 // code for auxiliary power input

typedef enum {assert=0, deassert} event_assertion_type_t;
typedef unsigned char(*pGetReadoutVal)(long);		// type definition for function that gets a sensor readout value
typedef void(*pSendEventMsg)(int, int, event_assertion_type_t);


// sensor threshold event definitions
#define SENSOREV_MAX_OFFSET                       (14)          // maximum offset for an IPMI sensor

#define SENSOREV_UPPER_NONRECOVER_HIGH_OFFSET     (11)
#define SENSOREV_UPPER_NONRECOVER_LOW_OFFSET      (10)
#define SENSOREV_UPPER_CRITICAL_HIGH_OFFSET       (9)
#define SENSOREV_UPPER_CRITICAL_LOW_OFFSET        (8)
#define SENSOREV_UPPER_NONCRITICAL_HIGH_OFFSET    (7)
#define SENSOREV_UPPER_NONCRITICAL_LOW_OFFSET     (6)
#define SENSOREV_LOWER_NONRECOVER_HIGH_OFFSET     (5)
#define SENSOREV_LOWER_NONRECOVER_LOW_OFFSET      (4)
#define SENSOREV_LOWER_CRITICAL_HIGH_OFFSET       (3)
#define SENSOREV_LOWER_CRITICAL_LOW_OFFSET        (2)
#define SENSOREV_LOWER_NONCRITICAL_HIGH_OFFSET    (1)
#define SENSOREV_LOWER_NONCRITICAL_LOW_OFFSET     (0)

#define SENSOREV_UPPER_NONRECOVER_HIGH_ASSERT_MASK     (0x800)
#define SENSOREV_UPPER_NONRECOVER_LOW_ASSERT_MASK      (0x400)
#define SENSOREV_UPPER_CRITICAL_HIGH_ASSERT_MASK       (0x200)
#define SENSOREV_UPPER_CRITICAL_LOW_ASSERT_MASK        (0x100)
#define SENSOREV_UPPER_NONCRITICAL_HIGH_ASSERT_MASK    (0x80)
#define SENSOREV_UPPER_NONCRITICAL_LOW_ASSERT_MASK     (0x40)
#define SENSOREV_LOWER_NONRECOVER_HIGH_ASSERT_MASK     (0x20)
#define SENSOREV_LOWER_NONRECOVER_LOW_ASSERT_MASK      (0x10)
#define SENSOREV_LOWER_CRITICAL_HIGH_ASSERT_MASK       (0x08)
#define SENSOREV_LOWER_CRITICAL_LOW_ASSERT_MASK        (0x04)
#define SENSOREV_LOWER_NONCRITICAL_HIGH_ASSERT_MASK    (0x02)
#define SENSOREV_LOWER_NONCRITICAL_LOW_ASSERT_MASK     (0x01)

#define SENSOREV_UPPER_NONRECOVER_THR_SETREAD_MASK     (0x20)
#define SENSOREV_UPPER_CRITICAL_THR_SETREAD_MASK       (0x10)
#define SENSOREV_UPPER_NONCRITICAL_THR_SETREAD_MASK    (0x08)
#define SENSOREV_LOWER_NONRECOVER_THR_SETREAD_MASK     (0x04)
#define SENSOREV_LOWER_CRITICAL_THR_SETREAD_MASK       (0x02)
#define SENSOREV_LOWER_NONCRITICAL_THR_SETREAD_MASK    (0x01)

// digital sensor (type 3) event offsets
#define DIGITALEV_STATE_DEASSERTED                     (0x0)
#define DIGITALEV_STATE_ASSERTED                       (0x1)

// alarm level sensor (type 7) event offsets
#define ALARMLVLEV_OK                                  (0x0)
#define ALARMLVLEV_OK_TO_NONCRIT                       (0x1)
#define ALARMLVLEV_CRIT_FROM_LOWER                     (0x2)
#define ALARMLVLEV_NONRECOVER_FROM_LOWER               (0x3)
#define ALARMLVLEV_NONCRIT_FROM_HIGHER                 (0x4)
#define ALARMLVLEV_CRIT_FROM_NONRECOVER                (0x5)
#define ALARMLVLEV_NONRECOVER                          (0x6)
#define ALARMLVLEV_MONITOR                             (0x7)
#define ALARMLVLEV_INFORMATIONAL                       (0x8)

// alarm level masks
#define ALARMLVLEV_OK_MASK                             (1 << ALARMLVLEV_OK)
#define ALARMLVLEV_OK_TO_NONCRIT_MASK                  (1 << ALARMLVLEV_OK_TO_NONCRIT)
#define ALARMLVLEV_CRIT_FROM_LOWER_MASK                (1 << ALARMLVLEV_CRIT_FROM_LOWER)
#define ALARMLVLEV_NONRECOVER_FROM_LOWER_MASK          (1 << ALARMLVLEV_NONRECOVER_FROM_LOWER)
#define ALARMLVLEV_NONCRIT_FROM_HIGHER_MASK            (1 << ALARMLVLEV_NONCRIT_FROM_HIGHER)
#define ALARMLVLEV_CRIT_FROM_NONRECOVER_MASK           (1 << ALARMLVLEV_CRIT_FROM_NONRECOVER)
#define ALARMLVLEV_NONRECOVER_MASK                     (1 << ALARMLVLEV_NONRECOVER)
#define ALARMLVLEV_MONITOR_MASK                        (1 << ALARMLVLEV_MONITOR)
#define ALARMLVLEV_INFORMATIONAL_MASK                  (1 << ALARMLVLEV_INFORMATIONAL)

#define ALARMLVLEV_NONCRIT_COMBO_MASK                  (ALARMLVLEV_OK_TO_NONCRIT_MASK | ALARMLVLEV_NONCRIT_FROM_HIGHER_MASK)
#define ALARMLVLEV_CRIT_COMBO_MASK                     (ALARMLVLEV_CRIT_FROM_LOWER_MASK | ALARMLVLEV_CRIT_FROM_NONRECOVER_MASK)
#define ALARMLVLEV_NONRECOVER_COMBO_MASK               (ALARMLVLEV_NONRECOVER_FROM_LOWER_MASK | ALARMLVLEV_NONRECOVER_MASK)


// sensor-specific (6f), hot swap sensor type (OEM sensor type 0xf2) offsets
#define HOTSWAP_EVENT_HANDLE_CLOSED         (0)
#define HOTSWAP_EVENT_HANDLE_OPENED         (1)
#define HOTSWAP_EVENT_QUIESCED              (2)
#define HOTSWAP_EVENT_BACKEND_FAILURE       (3)
#define HOTSWAP_EVENT_BACKEND_SHUTDOWN      (4)

// hot swap sensor state mask bits
#define HOTSWAP_MODULE_HANDLE_CLOSED_MASK   (1 << HOTSWAP_EVENT_HANDLE_CLOSED)
#define HOTSWAP_MODULE_HANDLE_OPEN_MASK     (1 << HOTSWAP_EVENT_HANDLE_OPENED)
#define HOTSWAP_QUIESCED_MASK               (1 << HOTSWAP_EVENT_QUIESCED)
#define HOTSWAP_BACKEND_PWR_FAILURE_MASK    (1 << HOTSWAP_EVENT_BACKEND_FAILURE)
#define HOTSWAP_BACKEND_PWR_SHUTDOWN_MASK   (1 << HOTSWAP_EVENT_BACKEND_SHUTDOWN)

// sensor-specific (6f), FPGA config sensor (OEM sensor type 0xc0) offsets
#define FPGACFGEV_UNUSED_RESERVED1                     (0)
#define FPGACFGEV_UNUSED_RESERVED2                     (1)
#define FPGACFGEV_SPI_DETECT0                          (2)
#define FPGACFGEV_REQCFG0                              (3)
#define FPGACFGEV_CFGRDY0                              (4)
#define FPGACFGEV_SPI_DETECT1                          (5)
#define FPGACFGEV_REQCFG1                              (6)
#define FPGACFGEV_CFGRDY1                              (7)
#define FPGACFGEV_SPI_DETECT2                          (8)
#define FPGACFGEV_REQCFG2                              (9)
#define FPGACFGEV_CFGRDY2                              (10)

#define FPGACFGEV_LOAD_DONE_MASK                       (1 << FPGACFGEV_LOAD_DONE)
#define FPGACFGEV_FPGA_FIRMWARE_MASK                   (1 << FPGACFGEV_FPGA_FIRMWARE)
#define FPGACFGEV_SPI_DETECT0_MASK                     (1 << FPGACFGEV_SPI_DETECT0)
#define FPGACFGEV_REQCFG0_MASK                         (1 << FPGACFGEV_REQCFG0)
#define FPGACFGEV_CFGRDY0_MASK                         (1 << FPGACFGEV_CFGRDY0)
#define FPGACFGEV_SPI_DETECT1_MASK                     (1 << FPGACFGEV_SPI_DETECT1)
#define FPGACFGEV_REQCFG1_MASK                         (1 << FPGACFGEV_REQCFG1)
#define FPGACFGEV_CFGRDY1_MASK                         (1 << FPGACFGEV_CFGRDY1)
#define FPGACFGEV_SPI_DETECT2_MASK                     (1 << FPGACFGEV_SPI_DETECT2)
#define FPGACFGEV_REQCFG2_MASK                         (1 << FPGACFGEV_REQCFG2)
#define FPGACFGEV_CFGRDY2_MASK                         (1 << FPGACFGEV_CFGRDY2)

// bit masks for sensor event control message 
#define SENSOREV_MSG_CTL_ENABLE_ALL_EVENTS_MASK        (0x80)
#define SENSOREV_MSG_CTL_SELECTED_EVENT_ACTION_MASK    (0x30)


typedef struct {
  SDR_type_01h_t* pSDR;
  unsigned char event_msg_ctl;
  unsigned short cur_masked_comp;               // for threshold sensors, current state of masked event comparators for discrete sensors, state assertion mask
  unsigned short prev_masked_comp;              // for threshold sensors, previous state of masked event comparators
  unsigned char comparator_status;              // for IPMI comparator readout for get sensor reading command
  unsigned char readout_value;                  // for threshold sensors
  pGetReadoutVal readout_function;
  pSendEventMsg send_event_function;            // function used to send events for this sensor type
  long readout_func_arg;
  unsigned char active_context_code;            // context code for when sensor is active
} sensor_data_entry_t;

typedef sensor_data_entry_t sensor_data_tbl_t[MAX_SENSOR_CNT];

typedef struct {
  unsigned char SDR_cnt;                        // number of active SDR records, including MMC SDR record
  unsigned char sensor_cnt;                     // number of active sensors, including hot swap sensor
  unsigned short SDRreservationID;
} SDR_state_record_t;

#define PBS_RECORD_SPI_ADDRESS        (0x0080)
#define PBS_RECORD_SPI_LENGTH         (10)
typedef struct {
  unsigned char updateflag;           // set to 1 by ZYNQ after new values are available for reading, cleared to zero by MMC after reading them
  unsigned char validmask;            // bit mask for whether the sensor readings in the subsequent bytes are valid or not.  Bit x to assert val[x] is valid
  unsigned char pbsval[PBS_OFFSET_MAX_VAL+1];             // array of 8 payload-based-sensor (pbs) values retrieved from the payload through the MMC/SPI interface.  A value should be considered
                                                          // valid only if its corresponding mask bit is set to a logic 1.
} PBS_sensor_record_t;


extern SDR_state_record_t SDRstate;
extern SDR_table_t SDRtbl;
extern sensor_data_tbl_t SensorData;


void sensor_svc_init(void);

unsigned char* get_SDR_entry_addr(int SDRrecordID);

SDR_type_01h_t* get_sensor_SDR_addr(int SensorNum);

int get_SDR_next_recID(SDR_entry_hdr_t* pCurSDRHdr);

void update_sensor_events(void);

void sensor_service(void);

//void update_hotswap_state_mask(int deassert_mask, int assert_mask);
void update_sensor_discrete_state(int sensornum, int deassert_mask, int assert_mask);

int get_payload_sensor_string(int SensorNum, char* pValuestr);

int get_sensor_name_str(int SensorNum, char* pnamestr);


#endif /* SDR_H_ */
