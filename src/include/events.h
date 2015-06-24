/*
 * events.h
 *
 *  Created on: Oct 11, 2010
 *      Author: tgorski
 */

#ifndef EVENTS_H_
#define EVENTS_H_

// timer event types
#define TEVENT_200USEC				            (0)			    // 200 usec event ID
#define TEVENT_2MSEC				              (1)			    // 2 msec event ID
#define TEVENT_10MSEC				              (2)         // 10 msec eventID
#define TEVENT_100MSEC_0			            (3)			    // 100 msec slot 0 event ID 
#define TEVENT_100MSEC_1                  (21)        // 100 msec slot 1 event ID
#define TEVENT_100MSEC_2                  (22)        // 100 msec slot 2 event ID
#define TEVENT_100MSEC_3                  (23)        // 100 msec slot 3 event ID
#define TEVENT_100MSEC_4                  (24)        // 100 msec slot 4 event ID
#define TEVENT_1SEC					              (4)			    // 1 second event type (timer ISR)

// IPMB service
#define IPMBEV_REQ_RCVD				            (7)			    // request received
#define IPMBEV_UNACK_REQ_MSG		          (8)			    // request message transmitted max # of times w/o acknowledge
#define IPMBEV_UNMATCHED_RSP_MSG	        (9)			    // cannot find match in request tbl for rcv'd response msg

// Payload Manager Events
#define PYLDMGREV_PAYLD_PWR_ON_DETECT     (11)        // Payload Power (+12) on detected (above lower non-critical)
#define PYLDMGREV_PAYLD_PWR_OFF_DETECT    (12)        // Payload Power (+12) off detected (below 10% of nominal)
#define PYLDMGREV_FPGA_AUTOCONFIG_SVC     (14)        // Autoconfig service event
#define PYLDMGREV_BACKEND_PWR_ON_DETECT   (15)        // Backend power-up cycle completed
#define PYLDMGREV_FPGA_LOAD_DONE          (16)
// RTC
#define RTC_1SEC_EVENT                    (13)        // Value change detected on RTC


typedef unsigned short event;

typedef int(*ptrCallback)(event, void*);

typedef void(*ptrDriverISRCallback)(void);

typedef struct {
  unsigned char active;
  event eventID;
  ptrCallback ptr;
} CallbackTblEntry;


#endif /* EVENTS_H_ */
