/***************************************************************************
*
*
* turn_wrapper.h
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

#ifndef __TURN_WRAPPER_H
#define __TURN_WRAPPER_H


#define TRX_UDP 1
#define TRX_TCP 0

#define CLIENT_AUTH_FAIL                    0x100
#define CLIENT_AUTH_DB_CONNECTIVITY_FAIL    0x101
#define CLIENT_AUTH_SUCCESS                 0x200


#define NAT_INIT_SUCCESS    0

#define TS_AUTH_FAIL -1
#define AUTH_SIGN_FAIL -2
#define POST_RELAY_FAIL -3
#define NAT_FAILURE TS_AUTH_FAIL
#define  PERM_FAIL -1
#define  NO_REALY_ERR -2
#define  NAT_SUCCESS 0

struct peer
{
    pj_stun_sock   *stun_sock;
    pj_sockaddr     mapped_addr;
};

typedef struct options
{
    pj_bool_t	 use_tcp;
    char	*srv_addr;	//turn server addr
    char	*srv_port;	// turn server port
    char	*realm;
    char	*user_name;
    char	*password;
    pj_bool_t	 use_fingerprint;
    char	*stun_server;
    char	*nameserver;
} opt;


typedef struct nattrav_config_info
{
    char user_name[SIZE_64B]; 
    char realm[SIZE_32B];	// realm to which user belong
    char signaure[SIZE_64B];	// password in encrypted format
    unsigned int signaure_expires;	// 
    char mac_addr[SIZE_64B] ; //mac address of device, if it needs to be sent across
    char serial_num[SIZE_32B]; //serial no. of device, if it needs to be sent across    
    char key[MAX_PKEY_LEN]; //serial no. of device, if it needs to be sent across    
    unsigned short transport;     // udp/tcp  transport 
    char turn_server_ip[SIZE_16B];     
    unsigned short turn_srvr_port;
    unsigned short data_port;
    char server_url[SIZE_64B]; //cloud server relay url in char array
    unsigned int method_id;
    void (*nattrav_on_rx_data_handler)(void *turn_sock,
	    void *pkt,
	    unsigned pkt_len,
	    const pj_sockaddr_t *peer_addr,
	    unsigned addr_len, void *data_sock);
    char dlrUrl[SIZE_128B]; // callback func ptr
    unsigned short authPort;
}nattrav_config_info;

typedef struct global
{
    pj_caching_pool	 cp;
    pj_pool_t		*pool;
    pj_stun_config	 stun_config;
    pj_thread_t		*thread;
    pj_bool_t		 quit;

    pj_dns_resolver	*resolver;

    pj_turn_sock	*relay;
    pj_sockaddr		 relay_addr;

    struct peer		 peer[MAX_MIN_LEN];
    int handle;
    nattrav_config_info *pInfo;
    int data_sockfd;
} glb;

typedef struct b_sockaddr_in
{
    unsigned short transport;     /**< udp/tcp.                */
    unsigned short sin_port;       /**< Transport layer port number.   */
    //unsigned long sin_addr;       /**< IP address.                    */
    struct in_addr sin_addr;       /**< IP address.                    */
    char        sin_zero[MAX_OCT_LEN];    /**< Padding.                       */
}b_sockaddr_in;

typedef union b_sockaddr
{   
    //addr_hdr     addr;       /**< Generic transport address.     */
    b_sockaddr_in  ipv4;       /**< IPv4 transport address.        */
    //b_sockaddr_in6 ipv6;       /**< IPv6 transport address.        */
} sockaddr;

typedef struct addr_hdr
{   
    pj_uint16_t sa_family;      /**< Common data: address family.   */
} addr_hdr;

typedef struct authInfo
{
    char my_mac_add[MAX_MAC_LEN];  // device mac id
    char serno[MAX_MAC_LEN];           //sn information
    char passWd[SIZE_64B];	// password in encrypted format or signature
    char login [SIZE_64B];	
    char realm[SIZE_32B];	// realm to which user belong
    unsigned int expires;
    unsigned int method_id;
}authInfo;



typedef struct nattrav_device_location
{
    char device_identifier[SIZE_64B]; 
    char login_name[SIZE_64B];   //username for which location info is stored, right now not used
    char relay_ip_addr[SIZE_16B]; // relay ip address where data can be sent for the user   
    unsigned short  port ;
    unsigned short transport;     // udp/tcp  transport 
} nattrav_device_location; 

typedef struct nattrav_peer_info
{
    char ip_addr[SIZE_16B]; // peer address    
    unsigned short  port ;
} nattrav_peer_info;

typedef struct clientauthresult
{
    unsigned int response_code;
}ClientAuthResult;

int sendAuthInfTurnServer(authInfo *auth_info, char *turnServerIp, unsigned short authPort);
int nat_trav_init(nattrav_config_info *pInfo, nattrav_device_location *location_info);
void nat_trav_destroy(void *handle);
void nat_set_log_level();
int nattrav_set_permission(void * nathandl,  nattrav_peer_info *peer_info);
int  nattrav_send_device_location(void * nathandl, nattrav_device_location *location_info, void *url);
int nattrav_destroy(void *handle);
void* nat_trav_reinit(void *handle);
void initTurnDataMutex(void);
#endif
