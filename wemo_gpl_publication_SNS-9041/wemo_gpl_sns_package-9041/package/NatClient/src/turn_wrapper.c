/***************************************************************************
*
*
* turn_wrapper.c
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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <pjnath.h>
#include <pjlib-util.h>
#include <pjlib.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <syslog.h>
#include <ithread.h>

#include "defines.h"
#include "turn_wrapper.h"

#include "curl/curl.h"
#include "httpsWrapper.h"

#include "types.h"
#include "osUtils.h"
#include "sigGen.h"
#include "logger.h"

#include "utils.h"
#include "natTravIceIntDef.h"
#include "natClient.h"
#include "thready_utils.h"

pj_turn_session * pj_turn_sock_get_turn_session(pj_turn_sock *turn_sock);
PJ_DEF(pj_int32_t) pj_turn_session_get_alloc_error( pj_turn_session *sess);
PJ_DEF(void) pj_turn_session_set_alloc_error( pj_turn_session *sess, int val);
int gInetSleepInterval = 1800; //default value 30 minutes
extern int gIceRunning; 
extern sGlbInfo gNatTravGlbInfo;
extern char gClientType[SIZE_128B];

#define THIS_FILE	"turn_wrapper.c"
#define PERM_REFRESH_INTERVAL	600		    /* -1 to disable */

#define OPTIONS		0

static pthread_mutex_t nattrav_init_mutex = PTHREAD_MUTEX_INITIALIZER;

extern char gturn_serverip[SIZE_32B];
extern char gturn_lbip[SIZE_32B];

static int postRelayDataDlr(nattrav_device_location  *relayData, char *restUrl);
static pj_status_t create_relay(nattrav_config_info *pInfo);

int gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
int gNatReinitInProgress = NAT_REINIT_IDLE;
int gNatOnStateInProgress = NAT_ONSTATE_IDLE;
int gNatAuthRetryTime = NATCL_NOT_INITIALIZED;

int gHandle=1111;
int gShutdownNat = PLUGIN_SUCCESS;

opt o;
glb g;


static int worker_thread(void *unused);
static void turn_on_rx_data(pj_turn_sock *relay,
				void *pkt,
				unsigned pkt_len,
				const pj_sockaddr_t *peer_addr,
				unsigned addr_len);
static void turn_on_state(pj_turn_sock *relay, pj_turn_state_t old_state,
				pj_turn_state_t new_state);
static pj_bool_t stun_sock_on_status(pj_stun_sock *stun_sock, 
				pj_stun_sock_op op,
				pj_status_t status);
static pj_bool_t stun_sock_on_rx_data(pj_stun_sock *stun_sock,
				void *pkt,
				unsigned pkt_len,
				const pj_sockaddr_t *src_addr,
				unsigned addr_len);

static int nat_pjshutdown();
extern unsigned short gpluginRemAccessEnable;
extern int gpluginNatInitialized;
extern char gPluginPrivatekey[MAX_PKEY_LEN];
extern int gInitInProgressCounter;
extern int gWiFiClientDeviceCurrState;

pthread_mutex_t turndatamutex;
void initTurnDataMutex();
void 	LockTurnDataMutex();
void	UnlockTurnDataMutex();

extern PJ_DEF(void) pj_turn_sock_set_data_port( pj_turn_sock *turn_sock, int data_port);

void initTurnDataMutex()
{
		osUtilsCreateLock(&turndatamutex);
}

void 	LockTurnDataMutex()
{
		osUtilsGetLock(&turndatamutex);
}

void UnlockTurnDataMutex()
{
		osUtilsReleaseLock(&turndatamutex);
}

static void my_perror(const char *title, pj_status_t status)
{
		char errmsg[PJ_ERR_MSG_SIZE];
		pj_strerror(status, errmsg, sizeof(errmsg));

		PJ_LOG(3,(THIS_FILE, "%s: %s", title, errmsg));
}

#define CHECK(expr)	status=expr; \
													 if (status!=PJ_SUCCESS) { \
															 my_perror(#expr, status); \
															 return status; \
													 }

void nat_set_log_level() {
		int log_level = PLUGIN_SUCCESS;
		log_level = loggerGetLogLevel();
		if (log_level <= LOG_ERR) {
				pj_log_set_level(1); 
		}else if (log_level <= LOG_INFO) {
				pj_log_set_level(3); 
		}else if (log_level > LOG_INFO) {
				pj_log_set_level(6); 
		}else {
				pj_log_set_level(5); 
		}
		return;
}

int nat_trav_init(nattrav_config_info *pInfo, nattrav_device_location *location_info) //TODO change to void *
{
		int i, retval=PLUGIN_SUCCESS; 	// *handle=NULL;
		pj_status_t status;
		int handle=PLUGIN_SUCCESS, data_sock=PLUGIN_SUCCESS, auth_retry = MIN_RETRY_COUNT;
		authSign *assign = NULL;
		authInfo *auth_info = NULL;
		int try_relay_cnt = 0;

		o.user_name = NULL; 
		o.password = NULL;
		o.realm = NULL;
		o.srv_port = NULL;
		o.srv_addr = NULL;

		gpluginNatInitialized = NATCL_INIT_INPROGRESS;
		gInitInProgressCounter++;
		if(!pInfo)
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "Error: pInfo is NULL\n");
				return PLUGIN_FAILURE;
		}

		memset(&o,0,sizeof(opt));
		memset(&g,0,sizeof(glb));

		nat_set_log_level();

		CHECK( pj_init() );
		CHECK( pjlib_util_init() );
		CHECK( pjnath_init() );

		auth_info = (authInfo *)malloc(sizeof(authInfo));
		if (!auth_info) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "Error: auth_info is NULL\n");
				return PLUGIN_FAILURE;
		}
		g.pInfo = pInfo;

		retval = PLUGIN_SUCCESS;
		auth_retry = MIN_RETRY_COUNT;

		while(1)
		{
				gInitInProgressCounter++;
				if (strcmp(gPluginPrivatekey, pInfo->key) != PLUGIN_SUCCESS) {
						APP_LOG("REMOTEACCESS", LOG_HIDE, " update pvt key with =%s:\n",gPluginPrivatekey);
						memset(pInfo->key, 0, sizeof(pInfo->key));
						strncpy(pInfo->key, gPluginPrivatekey, sizeof(pInfo->key)-1);
				}
				assign = createAuthSignatureNoExp(pInfo->mac_addr, pInfo->serial_num, pInfo->key);
				if (!assign) {
						APP_LOG("REMOTEACCESS", LOG_ERR, "\n Signature Structure returned NULL\n");
						pluginUsleep(MIN_NATCL_SLEEP*1000000);
						continue;
				}
				APP_LOG("REMOTEACCESS", LOG_HIDE, ": macid: =%s serno =%s key=%s\n",pInfo->mac_addr, pInfo->serial_num, pInfo->key);
				strncpy(pInfo->signaure, assign->ssign, sizeof(pInfo->signaure)-1);
				pInfo->signaure_expires = assign->expiry;

				strncpy(auth_info->my_mac_add, pInfo->mac_addr, sizeof(auth_info->my_mac_add)-1);
				strncpy(auth_info->serno, pInfo->serial_num, sizeof(auth_info->serno)-1);
				strncpy(auth_info->passWd, pInfo->signaure, sizeof(auth_info->passWd)-1);
				strncpy(auth_info->login, pInfo->user_name, sizeof(auth_info->login)-1);
				strncpy(auth_info->realm, pInfo->realm, sizeof(auth_info->realm)-1);
				auth_info->expires = pInfo->signaure_expires;
				auth_info->method_id = pInfo->method_id;
				APP_LOG("REMOTEACCESS", LOG_HIDE, "\n mac =%s ser =%s pass=%s login=%s realm=%s expires=%d \n",auth_info->my_mac_add, auth_info->serno, auth_info->passWd, auth_info->login, auth_info->realm, auth_info->expires);
				retval = CLIENT_AUTH_SUCCESS;
				if(retval == CLIENT_AUTH_SUCCESS) {
						APP_LOG("REMOTEACESS", LOG_DEBUG, "auth success after retry no:%d\n",auth_retry);
						free(assign);
						break;
				}	
#if 0
				int inum = PLUGIN_SUCCESS;
#endif
#if 0
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "b4 lb dns lookup\n");
				while (inum <= PLUGIN_SUCCESS) {
						pluginUsleep(MIN_NATCL_SLEEP*3*1000000);
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server lb dns lookup--%d\n", inum);
						inum = remoteLBDomainResolve(TURN_LB_IP);
						gInitInProgressCounter++;
				}
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "turn server lb ip=%s\n", gturn_lbip);
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "after dns lb lookup\n");
				//LB IP resolved, now get TS public IP
#endif
#if 0
				char *tspubIP = NULL;
				while (1) {
						inum = PLUGIN_SUCCESS;
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "b4 lb dns lookup\n");
						while (inum <= PLUGIN_SUCCESS) {
								pluginUsleep(MIN_NATCL_SLEEP*3*1000000);
								APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server lb dns lookup--%d\n", inum);
								inum = remoteLBDomainResolve(TURN_LB_IP);
								gInitInProgressCounter++;
						}
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "turn server lb ip=%s\n", gturn_lbip);
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "after dns lb lookup\n");
						//LB IP resolved, now get TS public IP

						pluginUsleep(MIN_NATCL_SLEEP*3*1000000);
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server public IPlookup");
						tspubIP = getTSPublicIP(gturn_lbip);
						if (!tspubIP) {
								APP_LOG("REMOTEACCESS", LOG_DEBUG, "turn server public IPlookup returned NULL");
								gInitInProgressCounter++;
								continue;
						}
						memset(gturn_serverip, 0x0, SIZE_32B);
						snprintf(gturn_serverip, sizeof(gturn_serverip), "%s", tspubIP);
						free(tspubIP);
						tspubIP = NULL;
						break;
				}
				strncpy(pInfo->turn_server_ip, gturn_serverip, sizeof(pInfo->turn_server_ip)-1);
#endif

				gInitInProgressCounter++;
				if (!pInfo) {
						APP_LOG("REMOTEACCESS", LOG_ERR, "pInfo in auth loop is null......... =%d \n", retval);
						break;
				}
		}

		if(retval != CLIENT_AUTH_SUCCESS)
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "auth failure ...retry failed =%d \n", retval);
				free(auth_info);
				return TS_AUTH_FAIL;
		}else {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "auth success\n");
		}

		free(auth_info);

		pj_caching_pool_init(&g.cp, &pj_pool_factory_default_policy, 0);

		g.pool = pj_pool_create(&g.cp.factory, "main", 1000, 1000, NULL);

		/* Init global STUN config */
		pj_stun_config_init(&g.stun_config, &g.cp.factory, 0, NULL, NULL);

		/* Create global timer heap */
		CHECK( pj_timer_heap_create(g.pool, 1000, &g.stun_config.timer_heap) );

		/* Create global ioqueue */
		CHECK( pj_ioqueue_create(g.pool, 16, &g.stun_config.ioqueue) );

		/* 
		 * Create peers
		 */
		for (i=0; i<(int)PJ_ARRAY_SIZE(g.peer); ++i) {
				gInitInProgressCounter++;
				pj_stun_sock_cb stun_sock_cb;
				char name[] = "peer0";
				pj_uint16_t port=0;
				pj_stun_sock_cfg ss_cfg;
				pj_str_t server;

				pj_bzero(&stun_sock_cb, sizeof(stun_sock_cb));
				stun_sock_cb.on_rx_data = &stun_sock_on_rx_data;
				stun_sock_cb.on_status = &stun_sock_on_status;

				g.peer[i].mapped_addr.addr.sa_family = pj_AF_INET();

				pj_stun_sock_cfg_default(&ss_cfg);
				/* make reading the log easier */
				ss_cfg.ka_interval = PERM_REFRESH_INTERVAL;

				name[strlen(name)-1] = '0'+i;
				o.user_name = (char *)malloc(SIZE_64B);
				o.password = (char *)malloc(SIZE_64B);
				o.realm = (char *)malloc(SIZE_32B);
				o.srv_port = (char *)malloc(MAX_OCT_LEN);

				port = (pj_uint16_t)(pInfo->turn_srvr_port);
				memset(o.realm, 0, SIZE_32B);
				strncpy(o.realm, pInfo->realm, SIZE_32B-1);
				memset(o.user_name, 0, SIZE_64B);
				strncpy(o.user_name, pInfo->user_name, SIZE_64B-1);
				memset(o.srv_port, 0, MAX_OCT_LEN);
				snprintf(o.srv_port, MAX_OCT_LEN, "%d",pInfo->turn_srvr_port);
				memset(o.password, 0, SIZE_64B);
				strncpy(o.password, pInfo->signaure, SIZE_64B-1);
				o.use_tcp = pInfo->transport;	//tcp =1, udp=0
				if (o.stun_server) {
						server = pj_str(o.stun_server);
						port = PJ_STUN_PORT;
				} 
				else 
				{
						o.srv_addr = (char *)malloc(SIZE_16B);
						memset(o.srv_addr, 0, SIZE_16B);
						strncpy(o.srv_addr, pInfo->turn_server_ip, SIZE_16B-1);

						APP_LOG("REMOTEACCESS", LOG_DEBUG, "%s %s turn server ip addr=%s, port=%d, (u16)port=%d domain=%s \n",__FILE__, __func__,  pInfo->turn_server_ip, pInfo->turn_srvr_port, port, o.realm);
						server = pj_str(o.srv_addr);
				}

		}
		g.data_sockfd = data_sock;
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "g.data_sockfd=%d\n",g.data_sockfd);

		/* Start the worker thread */
		CHECK( pj_thread_create(g.pool, "stun", &worker_thread, NULL, 0, 0, &g.thread) );
		g.relay_addr.ipv4.sin_addr.s_addr = 0;
		g.relay = NULL;
		gShutdownNat = 0;
		create_relay(g.pInfo);
		while((g.relay_addr.ipv4.sin_addr.s_addr == 0) && (gShutdownNat == 0))
		{
				gInitInProgressCounter++;
				pluginUsleep(MIN_NATCL_SLEEP*1000000);
				try_relay_cnt++;
				APP_LOG("REMOTEACCESS", LOG_NOTICE, "trying to get relay %d\n", try_relay_cnt);
				if ((try_relay_cnt >= MAX_NATCL_RELAYCHK_CNT)) {
						gShutdownNat = 1;
						APP_LOG("REMOTEACCESS", LOG_NOTICE, "stopping relay check need to re-init nat again %d\n", try_relay_cnt);
						try_relay_cnt = 0;
						gNatAuthRetryTime = NATCL_INIT_TRIGGER;
						break;
				}
		}

		if(gShutdownNat==1)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "gshutdown =1.. nattrav already destroyed try destroying again and return error\n");
				return NO_REALY_ERR;
		}

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "after relay succ\n");
		strncpy(location_info->relay_ip_addr, pj_inet_ntoa(g.relay_addr.ipv4.sin_addr), sizeof(location_info->relay_ip_addr)-1); 
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "after relay succ relay addr=%s\n", location_info->relay_ip_addr);
		location_info->port = pj_htons(g.relay_addr.ipv4.sin_port);

		if ((location_info->port == 0) || (strcmp(location_info->relay_ip_addr, "0.0.0.0") == 0)) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "relay informations are not correct...something wrong with pjnath\n");
				return NO_REALY_ERR;
		}

		handle = ++gHandle;
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "exiting nat_trav_init");
		return NAT_SUCCESS; // handle to session map //TODO return void *

}

int nat_pjshutdown()
{
		unsigned i;

		if (g.thread) {
				g.quit = 1;
				pj_thread_join(g.thread);
				pj_thread_destroy(g.thread);
				g.thread = NULL;
		}
		for (i=0; i<PJ_ARRAY_SIZE(g.peer); ++i) {
				if (g.peer[i].stun_sock) {
						pj_stun_sock_destroy(g.peer[i].stun_sock);
						g.peer[i].stun_sock = NULL;
				}
		}
		if (g.stun_config.timer_heap) {
				pj_timer_heap_destroy(g.stun_config.timer_heap);
				g.stun_config.timer_heap = NULL;
		}
		if (g.stun_config.ioqueue) {
				pj_ioqueue_destroy(g.stun_config.ioqueue);
				g.stun_config.ioqueue = NULL;
		}
		if (g.pool) {
				pj_pool_release(g.pool);
				g.pool = NULL;
				pj_caching_pool_destroy(&g.cp);
				pj_shutdown();
		}
#if 0
		pj_pool_factory_dump(&g.cp.factory, PJ_TRUE);
		pj_caching_pool_destroy(&g.cp);
		pj_shutdown();
#endif
		memset(&o,0,sizeof(opt));
		memset(&g,0,sizeof(glb));
		return PJ_SUCCESS;
}

static int worker_thread(void *unused)
{
		PJ_UNUSED_ARG(unused);
		while (!g.quit) {
				const pj_time_val delay = {0, 10};
				/* Poll ioqueue for the TURN client */
				if(g.quit!=1)
				{
						pj_ioqueue_poll(g.stun_config.ioqueue, &delay);
						/* Poll the timer heap */
						pj_timer_heap_poll(g.stun_config.timer_heap, NULL);
				}
		}
		return 0;
}

static pj_status_t create_relay(nattrav_config_info *pInfo)
{
		pj_turn_sock_cb rel_cb;
		pj_stun_auth_cred cred;
		pj_str_t srv;
		pj_status_t status;

		if (g.relay) {
				PJ_LOG(1,(THIS_FILE, "Relay already created"));
				return -1;
		}

		/* Create DNS resolver if configured */
		if (o.nameserver) {
				pj_str_t ns = pj_str(o.nameserver);

				status = pj_dns_resolver_create(&g.cp.factory, "resolver", 0, 
								g.stun_config.timer_heap, 
								g.stun_config.ioqueue, &g.resolver);
				if (status != PJ_SUCCESS) {
						PJ_LOG(1,(THIS_FILE, "Error creating resolver (err=%d)", status));
						return status;
				}

				status = pj_dns_resolver_set_ns(g.resolver, 1, &ns, NULL);
				if (status != PJ_SUCCESS) {
						PJ_LOG(1,(THIS_FILE, "Error configuring nameserver (err=%d)", status));
						return status;
				}
		}

		pj_bzero(&rel_cb, sizeof(rel_cb));
		rel_cb.on_rx_data = &turn_on_rx_data;
		rel_cb.on_state = &turn_on_state;

		/* create turn and stun session stun.sess=turnsession; */
		CHECK( pj_turn_sock_create(&g.stun_config, pj_AF_INET(), 
								(o.use_tcp? PJ_TURN_TP_TCP : PJ_TURN_TP_UDP),
								&rel_cb, 0,
								NULL, &g.relay) );

		if (o.user_name) {
				pj_bzero(&cred, sizeof(cred));
				cred.type = PJ_STUN_AUTH_CRED_STATIC;
				cred.data.static_cred.realm = pj_str(o.realm);
				cred.data.static_cred.username = pj_str(o.user_name);
				cred.data.static_cred.data_type = PJ_STUN_PASSWD_PLAIN;
				cred.data.static_cred.data = pj_str(o.password);
		} else {
				PJ_LOG(2,(THIS_FILE, "Warning: no credential is set"));
		}

		srv = pj_str(o.srv_addr);
		CHECK(pj_turn_sock_alloc(g.relay,				 /* the relay */
								&srv,				 /* srv addr */
								(o.srv_port?atoi(o.srv_port):PJ_STUN_PORT),/* def port */
								g.resolver,				 /* resolver */
								(o.user_name?&cred:NULL),		 /* credential */
								NULL)				 /* alloc param */
				 );
		if(g.relay)
		{
				pj_turn_sock_set_data_sock(g.relay, g.data_sockfd);
				pj_turn_sock_set_data_port(g.relay,0);
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "g.data_sockfd =%d data_port=%d\n", g.data_sockfd, pInfo->data_port);
		}

		return PJ_SUCCESS;
}

#if 0
static void destroy_relay(void)
{
		APP_LOG("REMOTEACESS", LOG_DEBUG, "\n In destroy_relay %d:%d:%d:%d:%d\n", gpluginNatInitialized, \
						gNatReinitInProgress, gNatOnStateInProgress, gShutdownNat, gNatClReInit24);
		if (g.relay) {
				APP_LOG("REMOTEACESS", LOG_DEBUG, "\n destroy g.relay and rest will be handled by turn_on_state\n");
				pj_turn_sock_destroy(g.relay);
#if 1
				g.relay = NULL;
#endif
		}
}
#endif

static void turn_on_rx_data(pj_turn_sock *relay,
				void *pkt,
				unsigned pkt_len,
				const pj_sockaddr_t *peer_addr,
				unsigned addr_len)
{
		int data_sock = 0;

		LockTurnDataMutex();

#ifdef __CHECKDATASOCK__
		PJ_LOG(3,(THIS_FILE, "***Client received %d bytes data*** ", pkt_len));
		pj_turn_session  *t_sess = NULL;
		t_sess =(pj_turn_session *) pj_turn_sock_get_turn_session(relay);
		data_sock = pj_turn_session_get_data_sock( t_sess);
		PJ_LOG(3,(THIS_FILE, "***Client received %d bytes data*** latest data_sock=%d", pkt_len, data_sock));
#else
		data_sock = addr_len;
		PJ_LOG(3,(THIS_FILE, "***Client received %d bytes data*** actual data_sock=%d", pkt_len, data_sock));
#endif

		if (g.pInfo) {
				g.pInfo->nattrav_on_rx_data_handler(relay, pkt, pkt_len, peer_addr, addr_len, &data_sock);
		}else {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "g.pInfo looks to be null, this should not happen");
		}
		UnlockTurnDataMutex();
}

static void turn_on_state(pj_turn_sock *relay, pj_turn_state_t old_state, pj_turn_state_t new_state)
{
		PJ_LOG(3,(THIS_FILE, "State %s --> %s", pj_turn_state_name(old_state), pj_turn_state_name(new_state)));

#ifndef __ALLOC_NO_UNAUTH__
		int alloc_unauth = PLUGIN_SUCCESS;	
		pj_turn_session  *t_sess = NULL;
		t_sess =(pj_turn_session *) pj_turn_sock_get_turn_session(relay);

		if ((old_state == PJ_TURN_STATE_ALLOCATING) && (new_state == PJ_TURN_STATE_DEALLOCATED)) {
				alloc_unauth = pj_turn_session_get_alloc_error(t_sess);
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "is alloc_error unauthorized................%d........", alloc_unauth);
				if (alloc_unauth == REMOTE_ALLOC_AUTH_ERROR) {
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "yes alloc_error unauthorized................%d........", alloc_unauth);
						pj_turn_session_set_alloc_error(t_sess, 0);
						alloc_unauth = pj_turn_session_get_alloc_error(t_sess);
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "set alloc_error unauthorized to................%d........", alloc_unauth);
						gNatAuthRetryTime = NATCL_INIT_TRIGGER;
				}
		}
#endif

		if (new_state == PJ_TURN_STATE_READY) {
				pj_turn_session_info info;
				pj_turn_sock_get_info(relay, &info);
				pj_memcpy(&g.relay_addr, &info.relay_addr, sizeof(pj_sockaddr));
		} else if ((old_state == PJ_TURN_STATE_ALLOCATING) && (new_state == PJ_TURN_STATE_DEALLOCATED)) {
				PJ_LOG(3,(THIS_FILE, "old state allocating new state deallocated setting g.relay to NULL"));
				g.relay_addr.ipv4.sin_addr.s_addr = 0;
				g.relay = NULL;
				gShutdownNat = 1;
		} else if ((old_state == PJ_TURN_STATE_RESOLVED) && (new_state == PJ_TURN_STATE_DEALLOCATED)) {
				PJ_LOG(3,(THIS_FILE, "old state resolved new state deallocated setting g.relay to NULL"));
				g.relay_addr.ipv4.sin_addr.s_addr = 0;
				g.relay = NULL;
				gShutdownNat = 1;
		} else if ((new_state > PJ_TURN_STATE_READY) && (g.relay) && (new_state != PJ_TURN_STATE_DESTROYING)) {
				PJ_LOG(3,(THIS_FILE, "Relay shutting down.."));
				PJ_LOG(3,(THIS_FILE, "Relay shutting down..status %d\n", pj_turn_sock_get_status(relay)));
				gShutdownNat = 1;
				if(gWiFiClientDeviceCurrState ==  DEVICE_STATE_CONNECTED) {
						gInetSleepInterval = NATCL_DISCON_SLEEP_INTERVAL;
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"Changed gInetSleepInterval to %d\n", gInetSleepInterval);
						//gWiFiClientDeviceCurrState = DEVICE_STATE_INTERNET_NOT_CONNECTED;
				}
		} else if ((new_state > PJ_TURN_STATE_READY) && (new_state == PJ_TURN_STATE_DESTROYING)) {
				PJ_LOG(3,(THIS_FILE, "Destroying..status %d\n", pj_turn_sock_get_status(relay)));
				gShutdownNat = 1;
				if(pj_turn_sock_get_status(relay) != 37004)
				{
						PJ_LOG(3,(THIS_FILE, "Destroying..status == ..set g.relay=NULL"));
						g.relay_addr.ipv4.sin_addr.s_addr = 0;
						g.relay = NULL;
				}
				g.relay_addr.ipv4.sin_addr.s_addr = 0;
				if(gWiFiClientDeviceCurrState ==  DEVICE_STATE_CONNECTED) {
						gInetSleepInterval = NATCL_DISCON_SLEEP_INTERVAL;
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"Changed gInetSleepInterval to %d\n", gInetSleepInterval);
						//gWiFiClientDeviceCurrState = DEVICE_STATE_INTERNET_NOT_CONNECTED;
				}
				if (gNatAuthRetryTime == NATCL_INIT_TRIGGER) {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"Out from NATCL_INIT_TRIGGER with %d:%d:%d:%d\n",\
										gNatClReInit24, gNatReinitInProgress, gShutdownNat, gNatOnStateInProgress);
						return;
				}
				if (gNatClReInit24 != NAT_REINIT_24HOURLY_INPROGRESS) {
						if (gNatReinitInProgress != NAT_REINIT_INPROGRESS) {
								gNatOnStateInProgress = NAT_ONSTATE_INPROGRESS;
								APP_LOG("REMOTEACCESS", LOG_DEBUG,"Going to re-init from %d:%d:%d:%d\n",\
												gNatClReInit24, gNatReinitInProgress, gShutdownNat, gNatOnStateInProgress);
								gpluginNatInitialized = NATCL_NOT_INITIALIZED;
								ithread_t remoteinit_thread;
								ithread_create(&remoteinit_thread, NULL, nat_trav_reinit, NULL);
								ithread_detach(remoteinit_thread);
						}
				}
				//pluginUsleep(MIN_NATCL_SLEEP*1000000);
		}
}

static pj_bool_t stun_sock_on_status(pj_stun_sock *stun_sock, 
				pj_stun_sock_op op,
				pj_status_t status)
{
		struct peer *peer = (struct peer*) pj_stun_sock_get_user_data(stun_sock);

		if (status == PJ_SUCCESS) {
				PJ_LOG(4,(THIS_FILE, "peer%d: %s success", peer-g.peer,
										pj_stun_sock_op_name(op)));
		} else {
				char errmsg[PJ_ERR_MSG_SIZE];
				pj_strerror(status, errmsg, sizeof(errmsg));
				PJ_LOG(1,(THIS_FILE, "peer%d: %s error: %s", peer-g.peer,
										pj_stun_sock_op_name(op), errmsg));
				return PJ_FALSE;
		}

		if (op==PJ_STUN_SOCK_BINDING_OP || op==PJ_STUN_SOCK_KEEP_ALIVE_OP) {
				pj_stun_sock_info info;
				int cmp;

				pj_stun_sock_get_info(stun_sock, &info);
				cmp = pj_sockaddr_cmp(&info.mapped_addr, &peer->mapped_addr);

				if (cmp) {
						char straddr[PJ_INET6_ADDRSTRLEN+10];

						pj_sockaddr_cp(&peer->mapped_addr, &info.mapped_addr);
						pj_sockaddr_print(&peer->mapped_addr, straddr, sizeof(straddr), 3);
						PJ_LOG(3,(THIS_FILE, "peer%d: STUN mapped address is %s",
												peer-g.peer, straddr));
				}
		}

		return PJ_TRUE;
}

static pj_bool_t stun_sock_on_rx_data(pj_stun_sock *stun_sock,
				void *pkt,
				unsigned pkt_len,
				const pj_sockaddr_t *src_addr,
				unsigned addr_len)
{
		struct peer *peer = (struct peer*) pj_stun_sock_get_user_data(stun_sock);
		char straddr[PJ_INET6_ADDRSTRLEN+10];

		((char*)pkt)[pkt_len] = '\0';

		pj_sockaddr_print(src_addr, straddr, sizeof(straddr), 3);
		PJ_LOG(3,(THIS_FILE, "peer%d: received %d bytes data from %s: %s",
								peer-g.peer, pkt_len, straddr, (char*)pkt));

		return PJ_TRUE;
}

int sendAuthInfTurnServer(authInfo *auth_info, char *turnServerIp, unsigned short authPort)
{

		int sock, bytes_recieved;
		char recv_data[MAX_BUF_LEN];
		struct hostent *host;
		struct sockaddr_in server_addr;
		APP_LOG("REMOTEACCESS", LOG_HIDE, "in sendAuthInfTurnServer mac%s, ser=%s passwd=%s realm=%s login %s\n", auth_info->my_mac_add, auth_info->serno, auth_info->passWd, auth_info->realm, auth_info->login);

		host = gethostbyname(turnServerIp);
		if(!host)
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "Unable to resolve host name: %s", turnServerIp);
				return NAT_FAILURE;
		}

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "creeate socket \n");
		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
				perror("Socket");
				return NAT_FAILURE;
		}
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "%srv=s\n",turnServerIp);
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(authPort);
		server_addr.sin_addr = *((struct in_addr *)host->h_addr);
		bzero(&(server_addr.sin_zero), MAX_OCT_LEN);

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "b4 connect\n");
		if (connect(sock, (struct sockaddr *)&server_addr,
								sizeof(struct sockaddr)) == -1)
		{
				perror("Connect");
				APP_LOG("REMOTEACCESS", LOG_ERR, "Connect to authentication server failed\n");
				close(sock);
				return NAT_FAILURE;
		}

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "b4 send \n");
		send(sock,auth_info,sizeof(authInfo), 0);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "after send \n");
		while(1)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "waiting to recv drespo from TS \n");
				bytes_recieved=recv(sock,recv_data,MAX_BUF_LEN,0);
				if (bytes_recieved > 0) {
						recv_data[bytes_recieved] = '\0';
						ClientAuthResult *caResutl = (ClientAuthResult *)recv_data;
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n resp code = %d \n" , caResutl->response_code);
						close(sock);
						return(caResutl->response_code);
				}else {
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n nothing received code = %d \n" , bytes_recieved);
						close(sock);
						return NAT_FAILURE;
				}
		}
		close(sock);
		return CLIENT_AUTH_SUCCESS;
}

int nattrav_set_permission(void * nathandl,  nattrav_peer_info *peer_info)
{
		int status;
		struct peer *peer;
		peer = &g.peer[0];
		peer->mapped_addr.addr.sa_family = AF_INET;
		peer->mapped_addr.ipv4.sin_port= pj_htons(peer_info->port);
		peer->mapped_addr.ipv4.sin_addr = pj_inet_addr2(peer_info->ip_addr);
		if(g.relay == NULL)
		{	
				APP_LOG("REMOTEACCESS", LOG_ERR, "turn sock is NULL dont proceed with set_permission\n");
				return -1;

		}
#if CLOUD_PERM_HANDLING
		status = pj_turn_sock_set_perm(g.relay, 1, &peer->mapped_addr, 0);
#else
		status = pj_turn_sock_set_perm(g.relay, 1, &peer->mapped_addr, 1);
#endif

		if (status != PJ_SUCCESS)
		{
				my_perror("pj_turn_sock_set_perm() failed", status);
				return PERM_FAIL;
		}
		return NAT_SUCCESS;
}

int  nattrav_send_device_location(void * nathandl, nattrav_device_location *location_info, void *url)
{
		int retval=PLUGIN_SUCCESS;
		{
				/* this is required in case pInfo is passed as NULL at init time and config param are picked from file*/
				strncpy(location_info->device_identifier, g.pInfo->user_name, sizeof(location_info->device_identifier)-1);
				strncpy(location_info->login_name, g.pInfo->user_name, sizeof(location_info->login_name)-1);
				location_info->transport = g.pInfo->transport;	//tcp =1

				APP_LOG("REMOTEACCESS", LOG_DEBUG, ": macid: =%s serno =%s transport=%d url=%s purl=%s\n",g.pInfo->mac_addr, g.pInfo->serial_num, g.pInfo->transport, url,g.pInfo->dlrUrl);
		}
		retval = postRelayDataDlr(location_info, url);	
		APP_LOG("REMOTEACCESS", LOG_DEBUG, ": *****NC*****  after postrelay\n");
		return retval;	
}

int postRelayDataDlr(nattrav_device_location  *relayData, char *restUrl)
{
		UserAppSessionData *pUsrAppSsnData = NULL;
		UserAppData *pUsrAppData = NULL;
		char httpRelayData[MAX_BUF_LEN];
		int retVal = NAT_SUCCESS;
		int ip_family = 4;
		authSign *assign = NULL;

		memset(httpRelayData, 0x0, MAX_BUF_LEN);
		snprintf(httpRelayData, sizeof(httpRelayData), "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?><peerAddresses><iceType>1</iceType>"
						"<defaultAddress>%s</defaultAddress><defaultPort>%d</defaultPort><defaultTransport>%d</defaultTransport>"
						"<ipFamily>%d</ipFamily></peerAddresses>", relayData->relay_ip_addr,relayData->port,relayData->transport,ip_family);


		assign = createAuthSignature(g.pInfo->mac_addr, g.pInfo->serial_num, g.pInfo->key);
		if (!assign) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "\n Signature Structure returned NULL\n");
				return AUTH_SIGN_FAIL;
		}



		APP_LOG("REMOTEACCESS", LOG_HIDE, "httpRelayData=%s len=%d\n",httpRelayData, strlen(httpRelayData));

		pUsrAppData = (UserAppData *)malloc(sizeof(UserAppData));
		if (!pUsrAppData) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "\n Malloc Structure returned NULL\n");
				retVal = NAT_FAILURE;
				goto on_return;
		}
		memset( pUsrAppData, 0x0, sizeof(UserAppData));

		pUsrAppSsnData = webAppCreateSession(0);
		if (!pUsrAppSsnData) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "\n Signature Structure returned NULL\n");
				retVal = NAT_FAILURE;
				goto on_return;
		}

		strncpy(pUsrAppData->url, restUrl, sizeof(pUsrAppData->url)-1);
		strncat(pUsrAppData->url,"peerAddresses", sizeof(pUsrAppData->url)-strlen(pUsrAppData->url)-1);
		strncpy( pUsrAppData->keyVal[0].key, "Content-Type", sizeof(pUsrAppData->keyVal[0].key)-1);   
		strncpy( pUsrAppData->keyVal[0].value, "application/xml", sizeof(pUsrAppData->keyVal[0].value)-1);   
		strncpy( pUsrAppData->keyVal[1].key, "Accept", sizeof(pUsrAppData->keyVal[1].key)-1);   
		strncpy( pUsrAppData->keyVal[1].value, "application/xml", sizeof(pUsrAppData->keyVal[1].value)-1);   

		strncpy( pUsrAppData->keyVal[2].key, "Authorization", sizeof(pUsrAppData->keyVal[2].key)-1);   
		strncpy( pUsrAppData->keyVal[2].value, assign->signature, sizeof(pUsrAppData->keyVal[2].value)-1);   

		strncpy( pUsrAppData->keyVal[3].key, "X-Belkin-Client-Type-Id", sizeof(pUsrAppData->keyVal[3].key)-1);   
		strncpy( pUsrAppData->keyVal[3].value, gClientType, sizeof(pUsrAppData->keyVal[3].value)-1);   
		pUsrAppData->keyValLen = 4;

		strncpy( pUsrAppData->inData, httpRelayData, sizeof(pUsrAppData->inData)-1);
		pUsrAppData->inDataLength = strlen(httpRelayData);
		char *check = strstr(restUrl, "https://");
		if (check) {
				pUsrAppData->httpsFlag = 1;
		}
		pUsrAppData->disableFlag = 1;
		retVal = webAppSendData( pUsrAppSsnData, pUsrAppData, 1);  
		if (retVal)
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "\n Some error encountered in adding info at dlr  , errorCode %d \n", retVal);
				APP_LOG("REMOTEACCESS", LOG_CRIT, "\n Some error encountered in adding info at dlr  , errorCode %d respCode %d\n", retVal, pUsrAppData->outResp);
				retVal = POST_RELAY_FAIL;
				goto on_return;
		} else {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n Response rcvd in adding info at dlr  , retVal %d respCode %d\n", retVal, pUsrAppData->outResp);
		}

		if(!strstr(pUsrAppData->outHeader, "200 OK"))
		{
				APP_LOG("REMOTEACESS", LOG_DEBUG, "\n Some error encountered in adding info at dlr , not 200 OK\n");
				if(strstr(pUsrAppData->outHeader, "403"))
						if (strstr(pUsrAppData->outData, "ERR_002")) {
							APP_LOG("REMOTEACCESS", LOG_DEBUG,"error encountered: ERR_002");
						}
				retVal = POST_RELAY_FAIL;
		}
on_return:
		if (pUsrAppData) {
				if (pUsrAppData->outData) {free(pUsrAppData->outData); pUsrAppData->outData = NULL;}
				free(pUsrAppData); pUsrAppData = NULL;
		}
		if (pUsrAppSsnData) {webAppDestroySession ( pUsrAppSsnData ); pUsrAppSsnData = NULL;}
		if (assign) {free(assign);assign = NULL;}
		return retVal;

}

//This interface is specific to TURN-RELAY.
//DO NOT USE FOR ICE
int nattrav_destroy(void *handle)
{
		int retval = PLUGIN_FAILURE;
		pj_thread_desc nat_destroy_thread_desc;
		pj_thread_t         *nat_destroy_thread;

		pj_thread_register("nattrav_destroy", nat_destroy_thread_desc, &nat_destroy_thread);

		gpluginNatInitialized = NATCL_NOT_INITIALIZED;

		APP_LOG("REMOTEACESS", LOG_DEBUG, "nattrav_destroy invoked either from nat_trav_init or remoteAccessInit %d\n", \
						gpluginNatInitialized);
		APP_LOG("REMOTEACESS", LOG_DEBUG, "In destroy_relay %d:%d:%d:%d:%d\n", gpluginNatInitialized, \
						gNatReinitInProgress, gNatOnStateInProgress, gShutdownNat, gNatClReInit24);
		if (g.relay) {
				APP_LOG("REMOTEACESS", LOG_DEBUG, "destroy g.relay and rest will be handled by turn_on_state\n");
				pj_turn_sock_destroy(g.relay);
				g.relay = NULL;
				if (gNatAuthRetryTime == NATCL_INIT_TRIGGER) {
						retval = NO_REALY_ERR;
				}else {

						retval = PLUGIN_SUCCESS;
				}
		}
		return retval;
}

#if 0
void nat_trav_destroy(void *handle)
{
		pj_thread_desc nat_trav_destroy_thread_desc;
		pj_thread_t         *nat_trav_destroy_thread;

		pj_thread_register("nat_trav_destroy", nat_trav_destroy_thread_desc, &nat_trav_destroy_thread);

		APP_LOG("REMOTEACCESS",LOG_DEBUG, "Entry nat_trav_destroy...........................");
		APP_LOG("REMOTEACESS", LOG_DEBUG, "\n In destroy_relay %d:%d:%d:%d:%d\n", gpluginNatInitialized, \
						gNatReinitInProgress, gNatOnStateInProgress, gShutdownNat, gNatClReInit24);
		if (g.relay) {
				APP_LOG("REMOTEACESS", LOG_DEBUG, "\n destroy g.relay and rest will be handled by turn_on_state\n");
				pj_turn_sock_destroy(g.relay);
				g.relay = NULL;
		}
#if 0
		destroy_relay();
#endif
		if (g.pInfo) {free(g.pInfo);g.pInfo = NULL;}
		pluginUsleep(MIN_NATCL_SLEEP*1000000);
		if (g.thread) {
				g.quit = 1;
				pj_thread_join(g.thread);
				pj_thread_destroy(g.thread);
				g.thread = NULL;
		}
		nat_pjshutdown();
		gpluginNatInitialized = NATCL_NOT_INITIALIZED;
		gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
		gNatReinitInProgress = NAT_REINIT_IDLE;
		gNatOnStateInProgress = NAT_ONSTATE_IDLE;
		APP_LOG("REMOTEACCESS",LOG_DEBUG, "Exiting nat_trav_destroy.......................");
}
#endif

void* nat_trav_reinit(void *handle)
{
		pj_thread_desc nat_trav_reinit_thread_desc;
		pj_thread_t         *nat_trav_reinit_thread;
		int retVal;

    //            tu_set_my_thread_name( __FUNCTION__ );

		APP_LOG("REMOTEACCESS",LOG_DEBUG, "Entry nat_trav_reinit...........................");
		if (gpluginNatInitialized == NATCL_INIT_INPROGRESS) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n remoteAccessInit....is in-progress....%d-%d", \
								gInitInProgressCounter, gpluginNatInitialized);
				return PLUGIN_SUCCESS;
		}

		retVal = osUtilsTryLock(&nattrav_init_mutex);
		if (retVal == EBUSY) {
			APP_LOG("REMOTEACCESS",LOG_ERR, "nat_trav_reinit, busy, returning");
			return NULL;
		}

		pj_thread_register("nat_trav_reinit", nat_trav_reinit_thread_desc, &nat_trav_reinit_thread);
		if(gIceRunning)			// if turn is running and not ICE then only do following
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "gng to destroy ICE\n");
				// destroy ICE
				nattrav_ice_destroy((void *)gNatTravGlbInfo.IceInstCnt, NULL);	
		}
		else
		{
				if (g.pInfo) {free(g.pInfo);g.pInfo = NULL;}
				//pluginUsleep(MIN_NATCL_SLEEP*1000000);
				if (g.thread) {
						g.quit = 1;
						pj_thread_join(g.thread);
						pj_thread_destroy(g.thread);
						g.thread = NULL;
				}
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "gng to destroy relay nat_pjshutdown");
				nat_pjshutdown();
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "Connection status set to %d", gWiFiClientDeviceCurrState);
				memset(&o,0,sizeof(opt));
				memset(&g,0,sizeof(glb));
		}

		gpluginNatInitialized = NATCL_NOT_INITIALIZED;
		gInitInProgressCounter = NATCL_NOT_INITIALIZED;

#if 0
		remoteAccessInitThdAg(NULL);
#endif

		remoteAccessInitThd(NULL);
		gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
		gNatReinitInProgress = NAT_REINIT_IDLE;
		gNatOnStateInProgress = NAT_ONSTATE_IDLE;
		retVal = osUtilsReleaseLock(&nattrav_init_mutex);
		APP_LOG("REMOTEACCESS",LOG_DEBUG, "Exiting nat_trav_reinit, release log: %d...", retVal);
		return NULL;
}

void* nat_trav_force_reinit(void *handle)
{
		APP_LOG("REMOTEACCESS",LOG_DEBUG, "Entry nat_trav_force_reinit...........................");

		pj_thread_desc nat_trav_force_reinit_thread_desc;
		pj_thread_t         *nat_trav_force_reinit_thread;
		int retVal;

		retVal = osUtilsTryLock(&nattrav_init_mutex);
		if (retVal == EBUSY) {
			APP_LOG("REMOTEACCESS",LOG_ERR, "nat_trav_force_reinit, busy, returning");
			return NULL;
		}

		pj_thread_register("nat_trav_force_reinit", nat_trav_force_reinit_thread_desc, &nat_trav_force_reinit_thread);

		if(gIceRunning)			// if turn is running and not ICE then only do following
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "gng to destroy ICE\n");
				// destroy ICE
				nattrav_ice_destroy((void *)gNatTravGlbInfo.IceInstCnt, NULL);	
		}
		else
		{
				if (g.relay)
				{
						PJ_LOG(3,(THIS_FILE, "Relay shutting down.."));
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"In invokeNatReInit et g.relay!=NULL");
						nattrav_destroy(0);
						g.relay = NULL;
						g.relay_addr.ipv4.sin_addr.s_addr = 0;
				}
				if (g.pInfo) {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"In g.pInfo et g.pInfo!=NULL");
						free(g.pInfo);
						g.pInfo = NULL;
				}
				if (g.thread) {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"In g.thread et g.thread!=NULL");
						g.quit = 1;
						pj_thread_join(g.thread);
						pj_thread_destroy(g.thread);
						g.thread = NULL;
				}
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "gng to destroy relay nat_pjshutdown");
				nat_pjshutdown();
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "Connection status set to %d", gWiFiClientDeviceCurrState);
		}
		memset(&o,0,sizeof(opt));
		memset(&g,0,sizeof(glb));

		gpluginNatInitialized = NATCL_NOT_INITIALIZED;
		gInitInProgressCounter = NATCL_NOT_INITIALIZED;
		gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
		gNatReinitInProgress = NAT_REINIT_IDLE;
		gNatOnStateInProgress = NAT_ONSTATE_IDLE;
		gNatAuthRetryTime = NATCL_NOT_INITIALIZED;

		remoteAccessInitThd(NULL);

		gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
		gNatReinitInProgress = NAT_REINIT_IDLE;
		gNatOnStateInProgress = NAT_ONSTATE_IDLE;
		retVal = osUtilsReleaseLock(&nattrav_init_mutex);
		APP_LOG("REMOTEACCESS",LOG_DEBUG, "Exiting nat_trav_force_reinit, release lock: %d...", retVal);
		return NULL;
}

#define NAT_SLEEP_24HR 24*60*60
unsigned int gNatReinitInterval=0;

void* remoteInitNatClient24(void *args) {
		unsigned int random_delay = rand()%CONFIG_NAT_REINIT_INTERVAL;
		unsigned int hours=0, mins=0, secs=0;
		gNatReinitInterval = NAT_SLEEP_24HR + random_delay;
                tu_set_my_thread_name( __FUNCTION__ );

		APP_LOG("REMOTEACCESS",LOG_DEBUG, "Entry Re-Initialize nat client every 24+ hours");
		while(1) {
				hours = (random_delay/(60*60));
				mins = (random_delay - (hours*3600))/60;
				secs = (random_delay - (hours*3600) - (mins*60));

				APP_LOG("REMOTEACCESS",LOG_CRIT, "Re-Initialize nat client after gNatReinitInterval: %u hours %u minutes %u secs", 
								24 + hours, mins, secs);

				sleep(gNatReinitInterval);
				/* following computation is kept intentionally after sleep so that 
					 this value could be changed while reinit is in progress from cloud message
				 */
				random_delay = rand()%CONFIG_NAT_REINIT_INTERVAL;
				gNatReinitInterval = NAT_SLEEP_24HR + random_delay;

				APP_LOG("REMOTEACCESS",LOG_DEBUG, "Inside while Re-Initialize nat client every 24+ hours-nat_24_init set");
				if (gpluginRemAccessEnable) {
						APP_LOG("REMOTEACCESS",LOG_DEBUG, "going to Re-Initialize nat 24 hours");
						gNatClReInit24 = NAT_REINIT_24HOURLY_INPROGRESS;
						if (g.relay)
						{
								APP_LOG("REMOTEACCESS", LOG_DEBUG,"NC..in remoteInitNatClient24 et g.relay!=NULL");
								nattrav_destroy(0);
								g.relay = NULL;
								g.relay_addr.ipv4.sin_addr.s_addr = 0;
						}
						ithread_t remoteinit_thread;
						ithread_create(&remoteinit_thread, NULL, nat_trav_reinit, NULL);
						ithread_detach(remoteinit_thread);
				}
		}
		APP_LOG("REMOTEACCESS",LOG_DEBUG, "Exit Re-Initialize nat client every 24 hours");
		return NULL;
}

int lastLifeTime = PLUGIN_FAILURE;
void invokeNatReInit(int flag)
{
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"Entry invokeNatReInit..................\n");
		if (flag <= NAT_REINIT) {
				if ((gpluginNatInitialized == NATCL_INIT_INPROGRESS) || (gNatOnStateInProgress == NAT_ONSTATE_INPROGRESS) \
								|| (gNatAuthRetryTime == NATCL_INIT_TRIGGER)) {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"Re-init from turn_on_state in progress....%d.....%d.....%d....\n", \
										gpluginNatInitialized, gNatOnStateInProgress, gNatAuthRetryTime);
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"Exit invokeNatReInit..................\n");
						return;
				}
		}

		pj_thread_desc inv_thread_desc;
		pj_thread_t         *inv_thread;
		pj_thread_register("invokeNatReInit", inv_thread_desc, &inv_thread);

		gNatReinitInProgress = NAT_REINIT_INPROGRESS;
		if (flag <= NAT_REINIT) {
				if (g.relay)
				{
						PJ_LOG(3,(THIS_FILE, "Relay shutting down.."));
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"NC..in invokeNatReInit et g.relay!=NULL");
						nattrav_destroy(0);
						g.relay = NULL;
						g.relay_addr.ipv4.sin_addr.s_addr = 0;
				}
		}

		if(gWiFiClientDeviceCurrState ==  DEVICE_STATE_CONNECTED) {
				gInetSleepInterval = NATCL_DISCON_SLEEP_INTERVAL;
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Changed gInetSleepInterval to %d\n", gInetSleepInterval);
		}

		ithread_t remoteinit_thread;
		if (flag <= NAT_REINIT) {
				ithread_create(&remoteinit_thread, NULL, nat_trav_reinit, NULL);
				ithread_detach(remoteinit_thread);
		} else {
				ithread_create(&remoteinit_thread, NULL, nat_trav_force_reinit, NULL);
				ithread_detach(remoteinit_thread);
		}

		pluginUsleep(MIN_NATCL_SLEEP*1000000);
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"Exit invokeNatReInit..................\n");
}

void* invokeNatReInitThd(void *arg)
{
	int *dataPtr = NULL;
	int natReInitType = -1;

	dataPtr = (int*)arg;
	if(NULL == dataPtr)
	{
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"Null argument received!");
		return NULL;
	}
	natReInitType = *dataPtr;

	APP_LOG("REMOTEACCESS", LOG_DEBUG,"Entry invokeNatReInitThd, natReInitType:%d", natReInitType);

	invokeNatReInit(natReInitType);

	free(dataPtr);
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"Exit invokeNatReInitThd");
	return NULL;
}

int glt_same = PLUGIN_SUCCESS;
int lastInProgress = PLUGIN_SUCCESS;
int gip_same = PLUGIN_SUCCESS;
int monitorNATCLStatus(void *arg)
{
		pj_turn_session_info info;

		APP_LOG("REMOTEACCESS",LOG_DEBUG,"Entry NAT Monitor Task ...");
		if (gpluginNatInitialized == NATCL_INITIALIZED) {
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"Both RemoteAccess and NAT Client is ready...%d", gpluginNatInitialized);
				gip_same = PLUGIN_SUCCESS;
				lastInProgress = PLUGIN_SUCCESS;
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"Nat Client is initialized....lastInProgress %d- \
								gInitInProgressCounter %d", lastInProgress, gInitInProgressCounter);
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"Nat Client is initialized....lastInProgress %d- \
								gInitInProgressCounter %d - gip_same %d", lastInProgress, gInitInProgressCounter, gip_same);
		}else {
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"NAT Client is not yet in running state...%d", gpluginNatInitialized);
				if (gpluginNatInitialized == NATCL_NOT_INITIALIZED) {
						APP_LOG("REMOTEACCESS",LOG_DEBUG,"RemoteAccess is enabled, nat client should be running here...\n");
						gip_same = PLUGIN_SUCCESS;
						lastInProgress = PLUGIN_SUCCESS;
						APP_LOG("REMOTEACCESS",LOG_DEBUG,"Nat Client is not yet initialized....lastInProgress %d- \
										gInitInProgressCounter %d", lastInProgress, gInitInProgressCounter);
						APP_LOG("REMOTEACCESS",LOG_DEBUG,"Nat Client is not yet initialized....lastInProgress %d- \
										gInitInProgressCounter %d - gip_same %d\n", lastInProgress, gInitInProgressCounter, gip_same);
						return NATCL_HEALTH_NOTGOOD;
				} else if (gpluginNatInitialized == NATCL_INIT_INPROGRESS) {
						APP_LOG("REMOTEACCESS",LOG_DEBUG,"RemoteAccess is enabled, nat client is still in progress...lastInProgress %d- \
										gInitInProgressCounter %d\n", lastInProgress, gInitInProgressCounter);
						if (lastInProgress != gInitInProgressCounter) {
								APP_LOG("REMOTEACCESS",LOG_DEBUG,"RemoteAccess is enabled, nat client is still in progress...lastInProgress %d- \
												gInitInProgressCounter %d - gip_same %d\n", lastInProgress, gInitInProgressCounter, gip_same);
								lastInProgress = gInitInProgressCounter;
								gip_same = PLUGIN_SUCCESS;
								APP_LOG("REMOTEACCESS",LOG_DEBUG,"RemoteAccess is enabled, nat client is still in progress...lastInProgress %d- \
												gInitInProgressCounter %d - gip_same %d\n", lastInProgress, gInitInProgressCounter, gip_same);
								return NATCL_HEALTH_GOOD;
						}else {
								++gip_same;
								if (gip_same >= MAX_RETRY_COUNT) {
										APP_LOG("REMOTEACCESS",LOG_DEBUG,"RemoteAccess is enabled, nat client is still in progress...lastInProgress %d- \
														gInitInProgressCounter %d - gip_same %d\n", lastInProgress, gInitInProgressCounter, gip_same);
										return NATCL_HEALTH_NOTGOOD;
								}else {
										APP_LOG("REMOTEACCESS",LOG_DEBUG,"RemoteAccess is enabled, nat client is still in progress...lastInProgress %d- \
														gInitInProgressCounter %d - gip_same %d\n", lastInProgress, gInitInProgressCounter, gip_same);
										return NATCL_HEALTH_GOOD;
								}
						}
				}
		}
		if((gIceRunning) ||(!(g.relay))){
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"Exit: NAT Monitor Task ...\n");
				return NATCL_HEALTH_GOOD;
		}

		if ((gNatClReInit24 != NAT_REINIT_24HOURLY_INPROGRESS) && (gNatReinitInProgress != NAT_REINIT_INPROGRESS) && \
						(gNatOnStateInProgress != NAT_ONSTATE_INPROGRESS)) {
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"Nat...get session info -%d:%d:%d-%d\n",\
								gNatClReInit24, gNatReinitInProgress, gNatOnStateInProgress, gpluginNatInitialized);
				pj_turn_sock_get_info(g.relay, &info);
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"In Session info attributes info.state-%d info.last_status-%d \
								info.conn_type-%d info.lifetime-%d lastLifeTime-%d\n", info.state, info.last_status, info.conn_type, \
								info.lifetime, lastLifeTime);
		}else {
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"Nat...re-init in progress-%d:%d:%d-%d\n",\
								gNatClReInit24, gNatReinitInProgress, gNatOnStateInProgress, gpluginNatInitialized);
				return NATCL_HEALTH_GOOD;
		}
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"Session info attributes info.state-%d info.last_status-%d \
						info.conn_type-%d info.lifetime-%d lastLifeTime-%d\n", info.state, info.last_status, info.conn_type, \
						info.lifetime, lastLifeTime);
		if (info.state == PJ_TURN_STATE_READY) {
				if (lastLifeTime != info.lifetime) {
						APP_LOG("REMOTEACCESS",LOG_ERR, "NAT session info lastLifeTime-%d info.lifetime-%d glt_same-%d", \
										lastLifeTime, info.lifetime, glt_same);
						lastLifeTime = info.lifetime;
						glt_same = PLUGIN_SUCCESS;
						return NATCL_HEALTH_GOOD;
				}else {
						++glt_same;
						if (glt_same >= MAX_RETRY_COUNT) {
								APP_LOG("REMOTEACCESS",LOG_ERR, "NAT session info lastLifeTime-%d info.lifetime-%d glt_same-%d", \
												lastLifeTime, info.lifetime, glt_same);
								return NATCL_HEALTH_NOTGOOD;
						}else {
								APP_LOG("REMOTEACCESS",LOG_ERR, "NAT session info lastLifeTime-%d info.lifetime-%d glt_same-%d", \
												lastLifeTime, info.lifetime, glt_same);
								return NATCL_HEALTH_GOOD;
						}
				}
		}else {
				APP_LOG("REMOTEACCESS",LOG_ERR, "NAT state not READY will be handled by turn_on_state");
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Session info attributes info.state-%d info.last_status-%d \
								info.conn_type-%d info.lifetime-%d lastLifeTime-%d\n", info.state, info.last_status, info.conn_type, \
								info.lifetime, lastLifeTime);
				return NATCL_HEALTH_NOTGOOD;
		}
		APP_LOG("REMOTEACCESS",LOG_DEBUG,"Exit NAT Monitor Task ...\n");
		return NATCL_HEALTH_GOOD;
}

//Destroy NAT in device reboot or app exit case or any other case
void invokeNatDestroy()
{
		pj_thread_desc inv_thread_desc;
		pj_thread_t         *inv_thread;

                tu_set_my_thread_name( __FUNCTION__ );
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"Entry invokeNatDestroy..................\n");
		if (gpluginNatInitialized != NATCL_INITIALIZED) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Exit invokeNatDestroy..................\n");
				pluginUsleep(1000000);
				return;
		}
		APP_LOG("REMOTEACESS", LOG_DEBUG, "\n In destroy_relay %d:%d:%d:%d:%d\n", gpluginNatInitialized, \
						gNatReinitInProgress, gNatOnStateInProgress, gShutdownNat, gNatClReInit24);
		pj_thread_register("invokeNatDestroy", inv_thread_desc, &inv_thread);
		gNatReinitInProgress = NAT_REINIT_INPROGRESS;

		if(gIceRunning)			// if turn is running and not ICE then only do following
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "gng to destroy ICE\n");
				// destroy ICE
				nattrav_ice_destroy((void *)gNatTravGlbInfo.IceInstCnt, NULL);	
		}
		else
		{
				PJ_LOG(3,(THIS_FILE, "Relay shutting down.."));
				if (g.relay)
				{
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"..in invokeNatDestroy et g.relay!=NULL");
						nattrav_destroy(0);
						g.relay = NULL;
						g.relay_addr.ipv4.sin_addr.s_addr = 0;
				}
				if (g.pInfo) {free(g.pInfo);g.pInfo = NULL;}
				pluginUsleep(MIN_NATCL_SLEEP*1000000);
				if (g.thread) {
						g.quit = 1;
						pj_thread_join(g.thread);
						pj_thread_destroy(g.thread);
						g.thread = NULL;
				}
				nat_pjshutdown();
				memset(&o,0,sizeof(opt));
				memset(&g,0,sizeof(glb));
		}
		gpluginNatInitialized = NATCL_NOT_INITIALIZED;
		gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
		gNatReinitInProgress = NAT_REINIT_IDLE;
		gNatOnStateInProgress = NAT_ONSTATE_IDLE;
		gpluginNatInitialized = NATCL_NOT_INITIALIZED;
		pluginUsleep(MIN_NATCL_SLEEP*2*1000000);
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"Exit invokeNatDestroy..................\n");
}
