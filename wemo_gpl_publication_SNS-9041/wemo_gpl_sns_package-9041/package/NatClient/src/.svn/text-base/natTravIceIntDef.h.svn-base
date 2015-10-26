/***************************************************************************
*
*
* natTravIceIntDef.h
*
* Copyright (c) 2012-2014 Belkin International, Inc. and/or its affiliates.
* All rights reserved.
*
* Permission to use, copy, modify, and/or distribute this software for any 
* purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* 
*
* THE SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
* INCIDENTAL, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER 
* RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
* NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH
* THE USE OR PERFORMANCE OF THIS SOFTWARE.
*
*
***************************************************************************/

#ifndef __NAT_TRAV_ICE_INT_DEF_H
#define __NAT_TRAV_ICE_INT_DEF_H


#include <stdio.h>
#include <stdlib.h>
#include <pjlib.h>
#include <pjlib-util.h>
#include <pjnath.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "types.h"
#include "defines.h"
#include "natTravIceWrapper.h"

#define NAT_TRAV_LIB_LOG_FILE   "./nat_trav_lib_log"

/*  STUN keep-alive time */
#define KA_INTERVAL 300

#ifdef DLR_SAMPLE
#define DLR_PORT 40000
#define DLR_IP    "125.23.218.144" 
#define DLR_OP_SET 1
#define DLR_OP_GET 2
#endif

#define NAT_TRAV_MAX_HOST_CAND 1
#define NAT_TRAV_MAX_COMP_CNT  1


#define NAT_TRAV_ICE_MODE_REGULAR                1
#define NAT_TRAV_ICE_MODE_AGGRESSIVE             2



#define NAT_TRAV_CANDIDATES_GATERED        1
#define NAT_TRAV_NEGOTIATION_COMPLETE      2

#define NAT_TRAV_MAX_NAT_TRAV_LIB_INSTNC   5


#define NAT_TRAV_AUTH_FAIL                    0x100
#define NAT_TRAV_AUTH_DB_CONNECTIVITY_FAIL    0x101
#define NAT_TRAV_AUTH_SUCCESS                 0x200

/* global variables */
typedef struct sGlbInfo
{
    pj_caching_pool	 cp;
    pj_pool_t		*pool;
    pj_thread_t		*thread;
    pj_bool_t		thread_quit_flag;	   
    FILE		*log_fhnd;
    unsigned int        IceInstCnt;
    unsigned int        libinitflag;
    pj_ice_strans_cfg	ice_cfg; // stun and turn server config
}sGlbInfo;

typedef struct sIceInstanceInfo
{
    pj_ice_strans	*icest;
    unsigned int NatTravFlag;
    ice_config_info sNatLibConfInfo;
}sIceInstanceInfo;


typedef struct sAuthInfo
{
	char mac_addr[MAX_MAC_LEN];
	char serial_no[SIZE_20B];
	char passwd[SIZE_64B];
	char login [SIZE_64B];
	char realm[SIZE_32B];
	unsigned int expires;
	unsigned int method_id;
}sAuthInfo;

typedef struct sAuthResult
{
      unsigned int response_code;
}sAuthResult;

#ifdef DLR_SAMPLE

typedef struct sOpSetInfo
{
   unsigned char op;
   char peer_id[SIZE_20B];
   nattrav_cand_offer_Info sCandInfo;
}sOpSetInfo;

typedef struct sOpGetInfo
{
   unsigned char op;
   char peer_id[SIZE_20B];
}sOpGetInfo;

#endif


/* global configuration and other info */
extern sGlbInfo gNatTravGlbInfo;
/* Ice instance for every ICE session*/
extern sIceInstanceInfo  gNatTravList[NAT_TRAV_MAX_NAT_TRAV_LIB_INSTNC];
/* to protect the global flag*/
extern pthread_mutex_t g_NatTrav_lock;


int nattrav_create_ice_instance(sIceInstanceInfo *psInstInfo);
int nattrav_init_ice_session(unsigned short ice_role,sIceInstanceInfo *psInstInfo);
void nattrav_destroy_instance(void *hndl);
void nattrav_get_local_ice_cand_info(sIceInstanceInfo *psInstInfo,nattrav_rest_field_Info *psLocalCandInfo);
int  nattrav_convert_addr_to_pjformat(char *ipaddr,unsigned int port,pj_sockaddr *pj_addr);
int nattrav_send_authinfo_turnserver(ice_config_info *psConfigInfo);
int  nattrav_lib_init(ice_config_info *psConfigInfo);
void nattrav_perror(const char *title, pj_status_t status);
void log_func(int level, const char *data, int len);
void err_exit(const char *title, pj_status_t status);
int nattrav_worker_thread(void *unused);

void cb_on_rx_data(pj_ice_strans *ice_st,
			  unsigned comp_id, 
			  void *pkt, pj_size_t size,
			  const pj_sockaddr_t *src_addr,
			  unsigned src_addr_len);

void cb_on_ice_complete(pj_ice_strans *ice_st, pj_ice_strans_op op,pj_status_t status);
int getPeerLocation(nattrav_rest_field_Info *candInfo,ice_config_info *psConfInfo);
int postPeerLocation(nattrav_rest_field_Info *candInfo,ice_config_info *psConfInfo);
int delPeerLocation(nattrav_rest_field_Info *candInfo,ice_config_info *psConfInfo);
int  nattrav_get_icelite_remote_address_info(void *hndl, nattrav_cand_offer_Info *psRemotePeerInfo);
#endif
