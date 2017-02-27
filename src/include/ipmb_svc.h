/*
 * ipmb_svc.h
 *
 *  Created on: Oct 12, 2010
 *      Author: tgorski
 */

#ifndef IPMB_SVC_H_
#define IPMB_SVC_H_

// ipmb header macros
#define IPMB_RQ_rsSA(p)				(*(p+0))
#define IPMB_RQ_netFN_GET(p)		(((*(p+1)) >> 2) & 0x3f)
#define IPMB_RQ_netFN_SET(p, v)		{*(p+1) = (*(p+1) & 0x3) | ((v & 0x3f) << 2); }
#define IPMB_RQ_rsLUN_GET(p)		(*(p+1) & 0x3)
#define IPMB_RQ_rsLUN_SET(p, v)		{*(p+1) = (*(p+1) & 0xfc) | (v & 0x3); }
#define IPMB_RQ_HXSUM(p)			(*(p+2))
#define IPMB_RQ_rqSA(p)				(*(p+3))
#define IPMB_RQ_rqSEQ_SET(p, v)		{*(p+4) = (*(p+4) & 0x3) | ((v & 0x3f) << 2); }
#define IPMB_RQ_rqSEQ_GET(p)		(((*(p+4)) >> 2) & 0x3f)
#define IPMB_RQ_rqLUN_GET(p)		(*(p+4) & 0x3)
#define IPMB_RQ_rqLUN_SET(p, v)		{*(p+4) = (*(p+4) & 0xfc) | (v & 0x3); }
#define IPMB_RQ_CMD(p)				(*(p+5))
#define IPMB_RQ_DATA_OFFSET			(6)

#define IPMB_RS_rqSA(p)				(*(p+0))
#define IPMB_RS_netFN_GET(p)		(((*(p+1)) >> 2) & 0x3f)
#define IPMB_RS_netFN_SET(p, v)		{*(p+1) = (*(p+1) & 0x3) | ((v & 0x3f) << 2); }
#define IPMB_RS_rqLUN_GET(p)		(*(p+1) & 0x3)
#define IPMB_RS_rqLUN_SET(p, v)		{*(p+1) = (*(p+1) & 0xfc) | (v & 0x3); }
#define IPMB_RS_HXSUM(p)			(*(p+2))
#define IPMB_RS_rsSA(p)				(*(p+3))
#define IPMB_RS_rqSEQ_SET(p, v)		{*(p+4) = (*(p+4) & 0x3) | ((v & 0x3f) << 2); }
#define IPMB_RS_rqSEQ_GET(p)		(((*(p+4)) >> 2) & 0x3f)
#define IPMB_RS_rsLUN_GET(p)		(*(p+4) & 0x3)
#define IPMB_RS_rsLUN_SET(p, v)		{*(p+4) = (*(p+4) & 0xfc) | (v & 0x3); }
#define IPMB_RS_CMD(p)				(*(p+5))
#define IPMB_RS_CCODE(p)			(*(p+6))
#define IPMB_RS_DATA_OFFSET			(6)

#define IPMIMINMSGLEN			(7)			// minimum size for valid ipmi message
#define IPMIMAXMSGLEN			(32)		// max IPMI msg length on IPMB
#define IPMBREQOVHEAD			(7)			// bytes of IPMB overhead in request message
#define IPMBRSPOVHEAD			(8)			// bytes of IPMB overhead in response message

#define DEFAULT_EVT_RCVR_IPMBL_ADDR			(0x20)			// default ipmb-l address for event receiver

typedef struct {
  unsigned char buf[IPMIMAXMSGLEN];
  unsigned short len;
} ipmb_msg_desc_t;

typedef int(*ptrMsgCallback)(void*, void*);

unsigned char calc_ipmi_xsum(const unsigned char* pbuf, unsigned short len);

void ipmb_init_req_hdr(unsigned char* pmsg, unsigned char rsSA, unsigned char netFn, unsigned char rsLUN,
  unsigned char rqLUN);

void ipmb_init_rsp_hdr(unsigned char* pmsg, const unsigned char *preq);

int ipmb_send_request(ipmb_msg_desc_t* pmsg, ptrMsgCallback rspCallback);

int ipmb_send_response(ipmb_msg_desc_t* pmsg);

void ipmb_init(void);

void ipmb_service(void);

void ipmb_msg_dump(const ipmb_msg_desc_t* pmsg, int filtcode);

void ipmb_set_event_rcvr_ipmb_addr(unsigned char ipmbl_addr);

unsigned char ipmb_get_event_rcvr_ipmb_addr(void);

void ipmb_set_event_rcvr_lun(unsigned char lun);

unsigned char ipmb_get_event_rcvr_lun(void);

unsigned int ipmb_get_req_delete_count();




#endif /* IPMB_SVC_H_ */
