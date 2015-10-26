/***************************************************************************
*
*
* natTravIceWrapper.h
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

#ifndef __NAT_TRAV_ICE_WRAPPER_H
#define __NAT_TRAV_ICE_WRAPPER_H


#include <stdio.h>
#include <stdlib.h>
#include <pjlib.h>
#include <pjlib-util.h>
#include <pjnath.h>
#include "defines.h"


/* ICE capabilities of a peer*/

#define NAT_TRAV_ICE_TYPE_NONE              1
#define NAT_TRAV_ICE_TYPE_FULL              2
#define NAT_TRAV_ICE_TYPE_LITE              3 



/* which transport to use to exchange data over turn relay*/
#define NAT_TRAV_USE_TRANSPORT_UDP          17
#define NAT_TRAV_USE_TRANSPORT_TCP          6

#define NAT_TRAV_FAMILY_IPV4                4
#define NAT_TRAV_FAMILY_IPV6                6

#define NAT_TRAV_ICE_ROLE_CONTROLLING       1
#define NAT_TRAV_ICE_ROLE_CONTROLLED        2


/* type of authentication method used by the auth service on the cloud*/
#define NATTRAV_AUTH_METHOD_MAC_BASED        1
#define NATTRAV_AUTH_METHOD_USERNAME_BASED   2



#define NAT_TRAV_INET_ADDRSTRLEN             16


/* maximum ICE candidates supported by the NatTrav library*/
#define NAT_TRAV_MAX_CAND_CNT                 4


#define NAT_TRAV_TURN_AUTH_LISTEN_PORT       49150

#define NAT_TRAV_TURN_LISTEN_PORT            34789

#define SUCCESS    0
#define FAIL       1

typedef void (*nattrav_on_rx_data_handler)(void *hndl,
	void *pkt,
	unsigned pkt_len,
	char* sender_addr); // callback func ptr



typedef struct ice_config_info
{
    char user_name[SIZE_64B]; 
    char realm[SIZE_32B];	// realm to which user belong
    char signaure[SIZE_64B];	// password in encrypted format
    char auth_key[SIZE_50B];	// password in encrypted format
    char mac_addr[SIZE_64B] ; //mac address of device, if it needs to be sent across
    char serial_num[SIZE_32B]; //serial no. of device, if it needs to be sent across    
    char turn_server_ip[SIZE_16B];
    char dlr_url[SIZE_128B]; //cloud server DLR service url
    char peerIp[SIZE_16B]; //Peer ip if it is present in the conf file
    unsigned short peerPort; // Peer port if it is present in the conf file
    unsigned int method_id;
    unsigned int signaure_expires; 
    unsigned short transport;     // udp/tcp  transport 
    unsigned short turn_srvr_port;
    unsigned short turn_auth_port;
    unsigned short turn_tcp_data_port;
    unsigned char ice_capabilty;
    unsigned char ice_role;     // controlling/controlled agent
    nattrav_on_rx_data_handler  appl_cb_on_rx_data;
}ice_config_info;

typedef struct nattrav_selected_pair
{        
    char      local_addr[NAT_TRAV_INET_ADDRSTRLEN];
    int       local_port;
    char      remote_addr[NAT_TRAV_INET_ADDRSTRLEN];
    int       remote_port;
} nattrav_selected_pair;


typedef pj_ice_sess_cand nat_trav_cand_detail;

typedef struct nattrav_def_addr_info
{
    char                   def_ipaddr[NAT_TRAV_INET_ADDRSTRLEN];
    int                      def_port;
    unsigned char      def_transport;
    unsigned char      ip_family;
}nattrav_def_addr_info;

typedef struct nattrav_cand_offer_Info
{
    char                    peer_id[SIZE_20B];  
    unsigned char      ice_type;
    nattrav_def_addr_info def_addr_info;
    struct 
    {
	char	   ufrag[SIZE_16B];
	char	   pwd[SIZE_100B];
	unsigned   cand_cnt;
	nat_trav_cand_detail  cand[NAT_TRAV_MAX_CAND_CNT];
    }ice_info;
}nattrav_cand_offer_Info;


typedef struct nattrav_rest_field_Info
{ 
    char                    peer_id[SIZE_20B];    
    unsigned char      ice_type;  //   1 - none, 2- full , 3- lite
    nattrav_def_addr_info   def_addr_info;     
    struct
    {
	char            username[SIZE_10B];
	char            passwd[SIZE_10B];       
	unsigned     cand_cnt;
	struct
	{
	    unsigned char    cand_type; // 0 : host, 1 : server reflexive, 2
	    unsigned char    transport; // 1 : udp, 2: tcp
	    unsigned char    compid;     
	    unsigned short   port;
	    unsigned int        priority;         
	    char                   foundation[36];
	    char                   ipaddr[SIZE_16B];                 
	}candidate[NAT_TRAV_MAX_CAND_CNT];
    }ice_info;
}nattrav_rest_field_Info;


int nattrav_read_conf_info(ice_config_info *psConfigInfo);

void * nattrav_init(ice_config_info *psConfigInfo);

int  nattrav_update_local_address_info(void *hndl, nattrav_cand_offer_Info *psLocalPeernfo);

int  nattrav_get_remote_address_info(void *hndl, nattrav_cand_offer_Info *psRemotePeerInfo);

int nattrav_start_session(void *hndl, nattrav_cand_offer_Info *psRemotePeerInfo);

int nattrav_send_data(void *hndl, const char *data, int data_len,nattrav_cand_offer_Info *psRemotePeerInfo);

int nattrav_ice_destroy(void *hndl, nattrav_cand_offer_Info *psLocalInfo);



/*
 * EMS Application global configuration structure which gets populated by reading
 * the configuration parameters from global.conf file at EMS Application startup.
 */
struct glbConfigApp {
    char    peerIp[SIZE_16B]; //Storage file locations
    UINT32    peerPort; // Number of threads in thread pool
    char    userName[SIZE_64B]; //Storage file locations
    char    realm[SIZE_32B]; //Storage file locations
    char    macAddr[SIZE_64B]; //Storage file locations
    char    serialNum[SIZE_32B]; //Storage file locations
    UINT32    transport; // Number of threads in thread pool
    UINT32    signatureExpires; // Number of threads in thread pool
    char    turnServerIp[SIZE_16B]; //Storage file locations
    UINT32    turnServerPort; // Number of threads in thread pool
    UINT32    turnServerAuthPort; //port on turn server to read Authentication info
    char    dlrUrl[SIZE_128B]; //DLR service URL
    char    iceCapability[SIZE_10B]; //ice capability : none, full
    char    iceRole[15]; //ice role : answerer, offerer
    UINT32  log_level;    // Debug messages log level
    UINT32  log_options;  // log destination, currently console or file
    char    authKey[SIZE_32B]; //authentication key
};
typedef struct glbConfigApp GlbConfigApp, *pGlbConfigApp;

/*
 * This interface will be used to read the configuration from global conf file
 * and populates it to the AppGlobalConf structure.
 */
int glbConfigReadFile(char* fileName, GlbConfigApp **pGlobalConfig, int options);


#endif
