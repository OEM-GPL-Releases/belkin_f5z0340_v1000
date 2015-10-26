/***************************************************************************
*
*
* natClient.h
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

#ifndef __NATCLIENT_H
#define __NATCLIENT_H

// Must TODO read for enabled TURN_DNS_RESOLVE and CLOUD_DNS_NAME
// Point your router DNS server configuration to 173.196.160.174 for CI and
// 173.196.160.170 for QA; If no DNS setting in router is done then by default
// it will point to production platform
#define TRANSACTION_ID_LEN            30+1
#define TURN_SERVER_IP   "nat.xbcs.net"
#define TURN_LB_IP   "nat.xbcs.net"

#define TURN_SERVER_PORT 3478
#define TURN_PUBSERVER_PORT 3475
#define REALM "belkin.org"
#define USE_TCP 1
#define USE_UDP 0

#define AUTH_PORT 49150

#define BL_DOMAIN_NM "api.xbcs.net"

//Nat initialization status
#define NATCL_NOT_INITIALIZED 0
#define NATCL_INIT_INPROGRESS 2
#define NATCL_INITIALIZED 1
#define NATCL_INIT_TRIGGER	3

//Nat Initialization wait status
#define NATCL_INIT_NOWAIT	2
#define NATCL_INIT_INWAIT	1
#define NATCL_INIT_OUTWAIT	0

//PJNATH turn on state flags
#define NAT_ONSTATE_IDLE 0
#define NAT_ONSTATE_INPROGRESS 1

//NAT Re-init in progreaa states
#define NAT_REINIT_IDLE 0
#define NAT_REINIT_INPROGRESS 1

//NAT Re-init 24 hourly states
#define NAT_REINIT_24HOURLY_IDLE 0
#define NAT_REINIT_24HOURLY_INPROGRESS 1

//NAT Re-init counts
#define MIN_NATREINIT_COUNT 0
#define MAX_NATREINIT_COUNT 3

//Miscllaneous
#define MIN_DLR_RETRY_COUNT 1
#define MAX_DLR_RETRY_COUNT 3

#define MIN_RETRY_COUNT MIN_DLR_RETRY_COUNT
#define MAX_RETRY_COUNT MAX_DLR_RETRY_COUNT
#define MAX_LOG_ACOUNT 200

//Sleep multiples
#define NATCL_SLEEP_ONE 1
#define MIN_NATCL_SLEEP 2
#define NATCL_CUSTOM_METHOD_ID	10
#define MIN_NATCL_RNDSLEEP 30
#define MIN_NATCL_INITSLEEP 140
#define MAX_NATCL_BACKOFF 60
#define MAX_NATCL_RELAYCHK_CNT 10

//NAT REINIT Configuration
/* Upper limit for randomizing periodic reinit interval (in seconds) */
#define CONFIG_NAT_REINIT_INTERVAL	(6*60)
/* Upper limit for back-off algorithm (in minutes) */
#define CONFIG_NAT_REINIT_TIMEOUT	30

//Auth Retries variables
#define MIN_NATAUTH_SLEEP MIN_NATCL_SLEEP*15
#define AUTH_1ST_ITER 5
#define SUBS_AUTH_INTERVAL 30
#define MAX_AUTH_RETRY	12

#define NATCL_DISCON_SLEEP_INTERVAL	60

#define NATCL_TURE	1
#define NATCL_FALSE	0

#define NATCL_HEALTH_NOTGOOD	-1
#define NATCL_HEALTH_PARTIALGOOD	-2
#define NATCL_HEALTH_GOOD	0

#ifndef __ALLOC_NO_UNAUTH__
#define REMOTE_ALLOC_AUTH_ERROR 111
#endif

#define ICE_GETNODE_MAX_COUNT 5
#define MAX_MSG_CNT   4
#define ICE_PEER_LB_NAME "ice.xbcs.net"    //ice lite peer name for getNode
#define MAX_IP_LEN                    16
#define REMOTE_PEER_MAC   "PL17Aug4002" //To be removed
#define PLG_SIGNATURE_EXP 3600
#define PLG_PEERIP "12.2.2.2"
#define PLG_PEER_PORT 23432

#define NAT_REINIT 0
#define NAT_FORCE_REINIT 1

#define DEVICE_STATE_DISCONNECTED		0	
#define DEVICE_STATE_CONNECTED			1
#define DEVICE_STATE_INTERNET_NOT_CONNECTED	3

enum E_COMMAND{
        CMD_ICE=1,
        CMD_RELAY,
        CMD_REGISTER,
        CMD_NAT,
        CMD_SYSTEM,
        CMD_CONSOLELOGS,
        CMD_LOGLEVEL,
        CMD_LOGUPLOAD,
        CMD_NONE
};

struct arg_struct{
                void                                    *hndl;
                void                                    *pkt;
                unsigned int  pkt_len;
                void                              *remoteInfo;
                char                              *transaction_id;
};

#endif 
