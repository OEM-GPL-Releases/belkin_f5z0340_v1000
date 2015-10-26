/***************************************************************************
*
*
* remoteInit.c
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

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <syslog.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "sigGen.h"
#include "defines.h"
#include "curl/curl.h"
#include "httpsWrapper.h"
#include "logger.h"

#include "pjnath.h"
#include "pjlib-util.h"
#include "pjlib.h"

#include "turn_wrapper.h"
#include "natClient.h"
#include "global.h"
#include "utils.h"
#include "asyncDNS.h"
#include "ithread.h"
#include "thready_utils.h"
#include "ares.h"
#include "osUtils.h"

#ifdef __RINIT_M__
static pthread_mutex_t remote_init_mutex;
#endif

#ifdef WeMo_SMART_SETUP_V2
extern int g_customizedState;
#endif
unsigned short gpluginRemAccessEnable;
extern char gRestoreState[MAX_RES_LEN];
extern int gNTPTimeSet;
//extern int gDataInProgress;
extern float g_lastTimeZone;
extern int gDstEnable;
int gpluginNatInitialized = NATCL_NOT_INITIALIZED;
int gInitInProgressCounter = NATCL_NOT_INITIALIZED;
int gIceRunning = NATCL_NOT_INITIALIZED;
extern int g_cmd_type; 
int gIceAddrChangeFreq=0;
#define ICE_FREQ_THRESHOLD    5
#define MAX_ICE_IND_RETRIES  9 
extern nattrav_config_info gConfigInfo;

extern opt o;
extern glb g;

char gdlr_url[SIZE_256B];
char gdlr_url_wos[SIZE_256B];

char gturn_serverip[SIZE_32B];
char gturn_lbip[SIZE_32B];

//--plugin registration globals to avoid frequent flash access----
#ifdef WeMo_INSTACONNECT
char gBridgeNodeList[SIZE_768B];
#endif
char g_turnServerEnvIPaddr[SIZE_32B];

extern int gNatAuthRetryTime;
extern char gSerialNo[SIZE_64B];
extern char gWiFiMacAddress[SIZE_64B];

unsigned int gBackOffInterval=0;
int gIcePeerDown=0;
void trigger_nat(void );

extern void pj_set_mapped_addr_status(int value);
extern int pj_unreplied_send_ind_count();
extern void pj_set_unreplied_send_ind_count(int value);
extern int pj_is_mapped_addr_change();

extern int gNatClReInit24;
extern int gNatReinitInProgress;
extern int gNatOnStateInProgress;

int g_icethread_blocked=0;
int s_prev_ind_count=0;

char *getTSPublicIP(char *turnserver_ip);
char *getTSPublicIP_N(char *turnlb_ip);
char *getTSUdpPublicIP(char *turnlb_ip);

extern int gWiFiClientDeviceCurrState;
extern char gPluginPrivatekey[MAX_PKEY_LEN];
extern int UDSrelayReqestHandler(void *relay,void *pkt,unsigned pkt_len,const void* peer_addr,unsigned addr_len,void *data_sock);
extern void* remoteInitNatClient24(void *args);

void initNatClient24Hrs()
{
	int retVal = -1;
	ithread_t remotereinit_thread;
	retVal = ithread_create(&remotereinit_thread, NULL, remoteInitNatClient24, NULL);
	if(retVal != 0)
	{
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"Failed to create remote Init NatClient24 hours thread");
		resetSystem();
	}
}

int geticethreadstatus(void)
{
		int retval=0;
#if !defined(PRODUCT_WeMo_Baby)
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"unreplied count %d s_prev_ind_count = %d \n",pj_unreplied_send_ind_count(),s_prev_ind_count);
		if ((pj_unreplied_send_ind_count() == s_prev_ind_count ))
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Entered geticethreadstatus \n");
				g_icethread_blocked++;
		}
		else
				g_icethread_blocked=0;
		if(g_icethread_blocked >=MAX_ICE_IND_RETRIES)
		{	
				retval= 1;
		}
		if(!g_icethread_blocked){s_prev_ind_count=pj_unreplied_send_ind_count();}
#endif
		return retval;
}
void* iceIndMonThread(void *arg)
{
		int retVal = PLUGIN_SUCCESS;
                tu_set_my_thread_name( __FUNCTION__ );

#if !defined(PRODUCT_WeMo_Baby)
		pj_set_mapped_addr_status(0);
		pj_set_unreplied_send_ind_count(0);
		g_icethread_blocked=0;
		s_prev_ind_count=0;
		while(1)
		{
				if (gIceRunning != 1) {
						break;
				}
				//if( (!gDataInProgress) && ((geticethreadstatus())||(pj_is_mapped_addr_change())) )
				if((geticethreadstatus()) || (pj_is_mapped_addr_change()))
				{
						APP_LOG("REMOTEACCESS", LOG_CRIT,"Send Indication Unreplied Count %d and mappee-addr change status %d\n",pj_unreplied_send_ind_count(),pj_is_mapped_addr_change());
						APP_LOG("REMOTEACCESS", LOG_ALERT,"g_icethread_blocked %d\n",g_icethread_blocked);
						gIcePeerDown = 1;
						if((pj_unreplied_send_ind_count() ==1)&&(pj_is_mapped_addr_change()))
						{
								gIceAddrChangeFreq++;
						}
						pj_set_mapped_addr_status(0);
						trigger_nat();
						pj_set_unreplied_send_ind_count(0);
						break;
				}
				else
				{
						sleep(20);
				}
		}
#endif
		pthread_exit(&retVal);
}

int remoteAccessInit() {
		int retVal = PLUGIN_SUCCESS;
		int nat_type_index = 0;
		
		char *nat_type=GetBelkinParameter(NAT_TYPE);

		if ((0x00 != nat_type) && 0x00 != (strlen(nat_type)))
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "nat_type is read as %s ",nat_type);
				nat_type_index=atoi(nat_type);
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "nat_type_index is set as %d ",nat_type_index);
		}

		if(gIceAddrChangeFreq >=ICE_FREQ_THRESHOLD)
		{
				APP_LOG("REMOTEACCESS", LOG_CRIT, "gIceAddrChangeFreq value is %d ",gIceAddrChangeFreq);
				nat_type_index=CMD_RELAY;
				gIceAddrChangeFreq=0;
		}
		if(( nat_type_index == CMD_ICE)) {
				retVal = remoteAccessInitNatICE();
				APP_LOG("REMOTEACCESS", LOG_ALERT, "ICE: INIT %d-%d\n",nat_type_index, retVal);
				if (retVal == PLUGIN_SUCCESS) {
					setUDSIceRunningstatus(gIceRunning);
					initNatClient24Hrs();
					return retVal;
				}
		}

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "Going for RELAY: INIT %d-%d\n",nat_type_index, retVal);
		retVal = remoteAccessInitNatTURN();
		APP_LOG("REMOTEACCESS", LOG_CRIT, "RELAY: INIT %d-%d\n",nat_type_index, retVal);

		return retVal;
}

int remoteAccessInitNatICE() {
		int  status =PLUGIN_SUCCESS;

#if defined(PRODUCT_WeMo_Baby) || defined(PRODUCT_WeMo_Streaming)
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n In remoteAccessInit........");
		return PLUGIN_SUCCESS;
#endif

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n Still In remoteAccessInit........");
		if (gpluginNatInitialized == NATCL_INIT_INPROGRESS) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n remoteAccessInit....is in-progress....%d-%d", \
								gInitInProgressCounter, gpluginNatInitialized);
				return PLUGIN_SUCCESS;
		}

		memset(&o,0,sizeof(opt));
		memset(&g,0,sizeof(glb));

		gpluginNatInitialized = NATCL_INIT_INPROGRESS;
		gInitInProgressCounter++;

		/* Wait for the back-off time interval */
		APP_LOG("REMOTEACCESS", LOG_CRIT, "Sleeping for Back-off timeout: %d", gBackOffInterval);
		pluginUsleep(gBackOffInterval*1000000);
		int btime = (MIN_NATCL_SLEEP*2) + (rand()%((MAX_NATCL_BACKOFF)-(MIN_NATCL_SLEEP*2)));
		if (!gBackOffInterval) {
				gBackOffInterval = 1;
		}else {
				gBackOffInterval = btime;
		}

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "Back-off timeout for next iteration: %d", gBackOffInterval);

		int inum = PLUGIN_SUCCESS;
		char *tspubIP = NULL;
		int wktime = PLUGIN_SUCCESS;
		int pubfail = PLUGIN_SUCCESS; 
		while (1) {
				inum = PLUGIN_SUCCESS;
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "b4 lb dns lookup\n");
				while (inum <= PLUGIN_SUCCESS) {
						if(gWiFiClientDeviceCurrState != DEVICE_STATE_CONNECTED)
						{
								APP_LOG("REMOTEACCESS", LOG_DEBUG, "Internet  state is not connected");
								gInitInProgressCounter++;
								pluginUsleep(MIN_NATCL_SLEEP*1000000);
								continue;
						}
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server lb dns lookup--%d\n", inum);
						inum = remoteLBDomainResolve(TURN_LB_IP);
						gInitInProgressCounter++;
						if (inum <= PLUGIN_SUCCESS) {
								pluginUsleep(MIN_NATCL_SLEEP*1000000);
						}
				}
				APP_LOG("REMOTEACCESS", LOG_CRIT, "turn server lb ip=%s", gturn_lbip);
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "turn server lb ip=%s", gturn_lbip);
				memset(gturn_serverip, 0x0, SIZE_32B);
#if 0
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "after dns lb lookup\n");
				//LB IP resolved, now get TS public IP

				wktime = NATCL_SLEEP_ONE;
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server public IPlookup - sleep %d\n", wktime);
				gInitInProgressCounter++;
				pluginUsleep(wktime*1000000);
				gInitInProgressCounter++;
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server public IPlookup");
				tspubIP = getTSPublicIP_N(gturn_lbip); //if TCP
				if (!tspubIP) {
						++pubfail;
						if (pubfail >= SIZE_2B) {
								tspubIP = getTSPublicIP(gturn_lbip); //if TCP
								pubfail = PLUGIN_SUCCESS;
						}
				}
				if (!tspubIP) {
						APP_LOG("REMOTEACCESS", LOG_CRIT, "turn server public IPlookup returned NULL");
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "turn server public IPlookup returned NULL");
						gInitInProgressCounter++;
						pluginUsleep(MIN_NATCL_SLEEP*2*1000000);
						continue;
				}
				memset(gturn_serverip, 0x0, SIZE_32B);
				snprintf(gturn_serverip, sizeof(gturn_serverip), "%s", tspubIP);
				free(tspubIP);
				tspubIP = NULL;
#endif
				break;
		}
#if 0
		APP_LOG("REMOTEACCESS", LOG_CRIT, "turn server public ip after getTSPublicIP=%s",gturn_serverip);

		memset(&gConfigInfo, 0, sizeof(nattrav_config_info));
		//strncpy(gConfigInfo.turn_server_ip, gturn_serverip, strlen(gturn_serverip));
		strncpy(gConfigInfo.turn_server_ip, gturn_serverip, sizeof(gConfigInfo.turn_server_ip)-1);
		gConfigInfo.turn_srvr_port = TURN_SERVER_PORT;
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "ICE: TS Ip=%s, strlen=%d", gturn_serverip, strlen(gturn_serverip));
#endif
		gIcePeerDown = 0;
		if((status = remoteAccessInitIce())== SUCCESS )
		{
#if !defined(PRODUCT_WeMo_Baby)
				pj_set_mapped_addr_status(0);
				pj_set_unreplied_send_ind_count(0);
#endif
				gIceRunning = 1;
				APP_LOG("REMOTEACCESS", LOG_CRIT, "ICE Initialization sucessful");
				{
				pluginUsleep(MIN_NATCL_SLEEP*2*1000000);
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "waiting for data on ICE candidates");
				}
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "*************init ICE after 3 secs****************");
				pluginUsleep(MIN_NATCL_SLEEP*1000000);
				pthread_t iceIndMon_thread;
				pthread_create(&iceIndMon_thread, NULL, iceIndMonThread, NULL);
				pthread_detach (iceIndMon_thread);
				gpluginNatInitialized = NATCL_INITIALIZED;
				gInitInProgressCounter = NATCL_NOT_INITIALIZED;
				gNatAuthRetryTime = NATCL_NOT_INITIALIZED;
				/* reset back off interval and bypass cnt in case of success */
				gBackOffInterval = 0;
				gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
				gNatReinitInProgress = NAT_REINIT_IDLE;
				gNatOnStateInProgress = NAT_ONSTATE_IDLE;
				return PLUGIN_SUCCESS;
		}else {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "ICE Initialization Failed");
				gpluginNatInitialized = NATCL_NOT_INITIALIZED;
				gInitInProgressCounter = NATCL_NOT_INITIALIZED;
				gNatAuthRetryTime = NATCL_NOT_INITIALIZED;
				gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
				gNatReinitInProgress = NAT_REINIT_IDLE;
				gNatOnStateInProgress = NAT_ONSTATE_IDLE;
				return PLUGIN_FAILURE;
		}
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "ICE Initialization Failed");
		gpluginNatInitialized = NATCL_NOT_INITIALIZED;
		gInitInProgressCounter = NATCL_NOT_INITIALIZED;
		gNatAuthRetryTime = NATCL_NOT_INITIALIZED;
		gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
		gNatReinitInProgress = NAT_REINIT_IDLE;
		gNatOnStateInProgress = NAT_ONSTATE_IDLE;
		return PLUGIN_FAILURE;
}

int remoteAccessInitNatTURN()
{
		int  status =PLUGIN_SUCCESS, sdloc_retry;
#if defined(PRODUCT_WeMo_Baby) || defined(PRODUCT_WeMo_Streaming)
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n In remoteAccessInit........");
		return PLUGIN_SUCCESS;
#endif

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n Still In remoteAccessInit........");
		if (gpluginNatInitialized == NATCL_INIT_INPROGRESS) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n remoteAccessInit....is in-progress....%d-%d", \
								gInitInProgressCounter, gpluginNatInitialized);
				return PLUGIN_SUCCESS;
		}

		char url[SIZE_64B], sKey[MAX_PKEY_LEN];
		nattrav_config_info *pInfo= NULL;
		int *handle=NULL;
		int ret = PLUGIN_SUCCESS, data_port;
		authInfo *authInf = NULL; //remove this from here - let it be its not used though
		authSign *assign = NULL;
		nattrav_device_location *location_info = NULL;
		gpluginNatInitialized = NATCL_INIT_INPROGRESS;
		gInitInProgressCounter++;

		/* Wait for the back-off time interval */
		APP_LOG("REMOTEACCESS", LOG_CRIT, "Sleeping for Back-off timeout: %d", gBackOffInterval);
		pluginUsleep(gBackOffInterval*1000000);
		int btime = (MIN_NATCL_SLEEP*2) + (rand()%((MAX_NATCL_BACKOFF)-(MIN_NATCL_SLEEP*2)));
		if (!gBackOffInterval) {
				gBackOffInterval = 1;
		}else {
				gBackOffInterval = btime;
		}

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "Back-off timeout for next iteration: %d", gBackOffInterval);

		//initTurnDataMutex();
		authInf = (authInfo *)malloc(sizeof(authInfo));
		if (!authInf) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "\n Malloc Failed returned NULL\n");
				ret = NAT_FAILURE;
				goto on_return;
		}

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n In remoteAccessInit........");
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n MAC Address of plug-in device is %s", gWiFiMacAddress);
		strncpy(authInf->my_mac_add, gWiFiMacAddress, sizeof(authInf->my_mac_add)-1);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n Serial Number of plug-in device is %s", gSerialNo);
		strncpy(authInf->serno, gSerialNo, sizeof(authInf->serno)-1);
		APP_LOG("REMOTEACCESS", LOG_HIDE, "\n Key for plug-in device is %s", gPluginPrivatekey);
		memset(sKey, 0x0, MAX_PKEY_LEN);
		strncpy(sKey , gPluginPrivatekey, sizeof(sKey)-1);
		data_port = PLUGIN_SUCCESS;
		assign = createAuthSignatureNoExp(authInf->my_mac_add, authInf->serno, sKey);
		if (!assign) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "\n Signature Structure returned NULL\n");
				ret = CLIENT_AUTH_FAIL;
				goto on_return;
		}
		pInfo = (nattrav_config_info *)malloc(sizeof(nattrav_config_info));
		if (!pInfo) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "\n Malloc Failed returned NULL\n");
				ret = NAT_FAILURE;
				goto on_return;
		}
		memset(pInfo, 0, sizeof(nattrav_config_info));

		strncpy(pInfo->user_name, authInf->my_mac_add, sizeof(pInfo->user_name)-1);
		strncpy(pInfo->realm, REALM, sizeof(pInfo->realm)-1);
		strncpy(pInfo->signaure, assign->ssign, sizeof(pInfo->signaure)-1);
		pInfo->signaure_expires = assign->expiry;
		strncpy(pInfo->mac_addr, authInf->my_mac_add, sizeof(pInfo->mac_addr)-1);
		strncpy(pInfo->serial_num, authInf->serno, sizeof(pInfo->serial_num)-1);
		strncpy(pInfo->key, sKey, sizeof(pInfo->key)-1);
		APP_LOG("REMOTEACCESS", LOG_HIDE, "\n gatewayapp mac =%s ser =%s key=%s\n",pInfo->mac_addr, pInfo->serial_num, pInfo->key);

		pInfo->transport= USE_TCP;

		int inum = PLUGIN_SUCCESS;
		char *tspubIP = NULL;
		int wktime = PLUGIN_SUCCESS;
		int pubfail = PLUGIN_SUCCESS; 
		while (1) {
				inum = PLUGIN_SUCCESS;
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "b4 lb dns lookup\n");
				while (inum <= PLUGIN_SUCCESS) {
						if(gWiFiClientDeviceCurrState != DEVICE_STATE_CONNECTED)
						{
								APP_LOG("REMOTEACCESS", LOG_DEBUG, "Internet  state is not connected");
								gInitInProgressCounter++;
								pluginUsleep(MIN_NATCL_SLEEP*1000000);
								continue;
						}
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server lb dns lookup--%d\n", inum);
						inum = remoteLBDomainResolve(TURN_LB_IP);
						gInitInProgressCounter++;
						if (inum <= PLUGIN_SUCCESS) {
								pluginUsleep(MIN_NATCL_SLEEP*1000000);
						}
				}
				APP_LOG("REMOTEACCESS", LOG_CRIT, "turn server lb ip=%s", gturn_lbip);
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "turn server lb ip=%s", gturn_lbip);
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "after dns lb lookup\n");
				//LB IP resolved, now get TS public IP

				wktime = NATCL_SLEEP_ONE;
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server public IPlookup - sleep %d\n", wktime);
				gInitInProgressCounter++;
				pluginUsleep(wktime*1000000);
				gInitInProgressCounter++;
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "trying to resolve turn server public IPlookup");
				tspubIP = getTSPublicIP_N(gturn_lbip); //if TCP
				if (!tspubIP) {
						++pubfail;
						if (pubfail >= SIZE_2B) {
								tspubIP = getTSPublicIP(gturn_lbip); //if TCP
								pubfail = PLUGIN_SUCCESS;
						}
				}
				if (!tspubIP) {
						APP_LOG("REMOTEACCESS", LOG_CRIT, "turn server public IPlookup returned NULL");
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "turn server public IPlookup returned NULL");
						gInitInProgressCounter++;
						pluginUsleep(MIN_NATCL_SLEEP*1000000);
						continue;
				}
				memset(gturn_serverip, 0x0, SIZE_32B);
				snprintf(gturn_serverip, sizeof(gturn_serverip), "%s", tspubIP);
				free(tspubIP);
				tspubIP = NULL;
				break;
		}
		strncpy(pInfo->turn_server_ip, gturn_serverip, sizeof(pInfo->turn_server_ip)-1);
		APP_LOG("REMOTEACCESS", LOG_CRIT, "turn server public ip after getTSPublicIP=%s",pInfo->turn_server_ip);


		pInfo->turn_srvr_port = TURN_SERVER_PORT;
		memset(url, 0x0, sizeof(url));
		snprintf(url, sizeof(url), "https://%s:8443/apis/http/dlr/", BL_DOMAIN_NM);
		strncpy(pInfo->server_url, url, sizeof(pInfo->server_url)-1);

		int stNatTime = 0;
		stNatTime = NATCL_SLEEP_ONE;
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n In remoteAccessInit.....%d....before alloc..", stNatTime);
		gInitInProgressCounter++;
		pluginUsleep(stNatTime*1000000);
		gInitInProgressCounter++;

		pInfo->method_id = NATCL_CUSTOM_METHOD_ID;
		pInfo->nattrav_on_rx_data_handler = (void *)&UDSrelayReqestHandler;
		pInfo->data_port = data_port;
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "data_port at app=%d\n", pInfo->data_port);
		pInfo->authPort = AUTH_PORT;
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "auth_port at app=%d\n", pInfo->authPort);

		location_info = (nattrav_device_location *)malloc(sizeof(nattrav_device_location));
		if (!location_info) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "\n Malloc Failed returned NULL\n");
				ret = NAT_FAILURE;
				goto on_return;
		}
		memset(location_info, 0x0, sizeof(nattrav_device_location));
		strncpy(location_info->device_identifier, authInf->my_mac_add, sizeof(location_info->device_identifier)-1);
		strncpy(location_info->login_name, authInf->my_mac_add, sizeof(location_info->login_name)-1);
		location_info->transport = USE_TCP;

		if((status=nat_trav_init(pInfo, location_info)) != NAT_SUCCESS){
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "status=%d\n", status);
				{
						APP_LOG("REMOTEACCESS", LOG_ERR, "ERR: authentication failed or create relay failed =%d", status);
						ret = NO_REALY_ERR;
						goto on_return;
				}
		}else { ret = NAT_SUCCESS; }

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "loc_info: relay_ip_addr=%s port=%d transport=%d\n",location_info->relay_ip_addr, location_info->port, location_info->transport);
		memset(url, 0x0, sizeof(url));
		snprintf(url, sizeof(url), "https://%s:8443/apis/http/dlr/", BL_DOMAIN_NM);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "url in gatewayd:%s\n",url);
		sdloc_retry = MIN_DLR_RETRY_COUNT;
		while(sdloc_retry <= MAX_DLR_RETRY_COUNT)
		{
				gInitInProgressCounter++;
				ret = nattrav_send_device_location(handle, location_info, url);
				if(ret < PLUGIN_SUCCESS)
				{
						APP_LOG("REMOTEACCESS", LOG_ERR, "erro code returned by natlib:%d\n", ret);
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "DLR peer info update failed");
						ret = POST_RELAY_FAIL;
						sdloc_retry++;
						pluginUsleep(MIN_NATCL_SLEEP*1000000);
				}else {
						ret = NAT_SUCCESS;
						break;
				}
		}
		if (ret != NAT_SUCCESS)
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "unrecoverable error finally giving up:%d \n", ret);
				APP_LOG("REMOTEACCESS",LOG_ALERT, "DLR peer info update failed, unrecoverable %d\n", ret);
				ret = POST_RELAY_FAIL;
				goto on_return;
		}

		setUDSIceRunningstatus(gIceRunning);
		initNatClient24Hrs();
		gpluginNatInitialized = NATCL_INITIALIZED;
		gInitInProgressCounter = NATCL_NOT_INITIALIZED;
		gNatAuthRetryTime = NATCL_NOT_INITIALIZED;
		/* reset back off interval and bypass cnt in case of success */
		gBackOffInterval = 0;

		gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
		gNatReinitInProgress = NAT_REINIT_IDLE;
		gNatOnStateInProgress = NAT_ONSTATE_IDLE;

on_return:
		if (ret != NAT_SUCCESS) {
				gpluginNatInitialized = NATCL_NOT_INITIALIZED;
				gInitInProgressCounter = NATCL_NOT_INITIALIZED;
				gNatAuthRetryTime = NATCL_NOT_INITIALIZED;
				gNatClReInit24 = NAT_REINIT_24HOURLY_IDLE;
				gNatReinitInProgress = NAT_REINIT_IDLE;
				gNatOnStateInProgress = NAT_ONSTATE_IDLE;
		}
		if (assign) {free(assign); assign = NULL;}
		if (authInf) {free(authInf); authInf = NULL;}
		if (location_info) {free(location_info); location_info = NULL;}
		if (o.user_name) {free(o.user_name); o.user_name = NULL;}
		if (o.password) {free(o.password); o.password = NULL;}
		if (o.realm) {free(o.realm); o.realm = NULL;}
		if (o.srv_port) {free(o.srv_port); o.srv_port = NULL;}
		if (o.srv_addr) {free(o.srv_addr); o.srv_addr = NULL;}

		if ((ret == NO_REALY_ERR) || (ret == POST_RELAY_FAIL)) {
				APP_LOG("REMOTEACCESS",LOG_DEBUG, "Going to destroy NAT if any.....................");
				if (ret == NO_REALY_ERR) {
						gNatAuthRetryTime = NATCL_INIT_TRIGGER;
				}
				ret = nattrav_destroy(handle);
				if ((ret == NO_REALY_ERR) && (gNatOnStateInProgress != NAT_ONSTATE_INPROGRESS)) {
						trigger_nat();
				}
				gNatAuthRetryTime = NATCL_NOT_INITIALIZED;
				ret = NAT_FAILURE;
		}
		return ret;
}

void* remoteAccessInitThd(void *args) {
		int retVal, i = PLUGIN_SUCCESS;
		char szRstPrm[MAX_RES_LEN];
    tu_set_my_thread_name( __FUNCTION__ );

		APP_LOG("REMOTEACCESS",LOG_DEBUG, "gpluginRemAccessEnable flag value is %d ---remoteAccessInitThd, gWiFiClientDeviceCurrState:%d", gpluginRemAccessEnable,gWiFiClientDeviceCurrState);
#ifdef __RINIT_M__
		retVal = osUtilsTryLock(&remote_init_mutex);
		if (retVal == EBUSY) {
				APP_LOG("REMOTEACCESS",LOG_ERR, "remoteAccessInitThd, busy, not scheduling");

				return NULL;
		}
#endif
		if ((gpluginRemAccessEnable) && (gpluginNatInitialized == NATCL_NOT_INITIALIZED)) {
				while ((gWiFiClientDeviceCurrState !=  DEVICE_STATE_CONNECTED)) {
						pluginUsleep(MIN_NATCL_SLEEP*1000000);
						if (i < MAX_RETRY_COUNT) {
								APP_LOG("REMOTEACCESS",LOG_DEBUG, "remoteAccessInit in loop waiting for net connection %d,gWiFiClientDeviceCurrState:%d", i,gWiFiClientDeviceCurrState);
						}else if (i > MAX_LOG_ACOUNT) {
								APP_LOG("REMOTEACCESS",LOG_DEBUG, "remoteAccessInit in loop waiting for net connection %d,gWiFiClientDeviceCurrState:%d", i,gWiFiClientDeviceCurrState);
								i = PLUGIN_SUCCESS;
						}
						i++;
						continue;
				}
				i = 0;
				while ((gNTPTimeSet != (PLUGIN_SUCCESS+1))) {
						pluginUsleep(MIN_NATCL_SLEEP*1000000);
						if (i < MAX_RETRY_COUNT) {
								APP_LOG("REMOTEACCESS",LOG_DEBUG, "remoteAccessInit in loop waiting for time syn %d,gNTPTimeSet:%d", i,gNTPTimeSet);
						}else if (i > MAX_LOG_ACOUNT) {
								APP_LOG("REMOTEACCESS",LOG_DEBUG, "remoteAccessInit in loop waiting for time syn %d,gNTPTimeSet:%d", i,gNTPTimeSet);
								i = PLUGIN_SUCCESS;
						}
						i++;
						continue;
				}
				memset(szRstPrm, 0x0, sizeof(szRstPrm));
				strncpy(szRstPrm, gRestoreState, sizeof(szRstPrm)-1);
				APP_LOG("REMOTEACCESS",LOG_DEBUG, "restore state: %s", szRstPrm);
				if (atoi(szRstPrm) == 0) {
						retVal = remoteAccessInit();
				} else {
						while (1) {
								if (atoi(szRstPrm) == 0x0) {
										retVal = remoteAccessInit();
										break;
								}else {
										pluginUsleep(MIN_NATCL_SLEEP*1000000);
										memset(szRstPrm, 0x0, sizeof(szRstPrm));
										strncpy(szRstPrm, gRestoreState, sizeof(szRstPrm)-1);
								}
						}
				}
				APP_LOG("REMOTEACCESS",LOG_DEBUG, "remoteAccessInit returned %d", retVal);
				/*if(retVal)
						enqueueBugsense("REMOTE_ACCESS_INIT_FAILURE");
				else
						enqueueBugsense("REMOTE_ACCESS_INIT_SUCCESS");*/
		}

#ifdef __RINIT_M__
		retVal = osUtilsReleaseLock(&remote_init_mutex);
		APP_LOG("REMOTEACCESS",LOG_ERR, "remoteAccessInitThd, release lock: %d",
				retVal);
#endif
		return NULL;
}

static void callback_natdns(void *arg, int status, int timeouts, struct hostent *host)
{
	const char *retVal = NULL;
	int i = 0;
	char ip1[INET6_ADDRSTRLEN];
	
	if(!host || status != PLUGIN_SUCCESS){
		APP_LOG("ASYNCDNS", LOG_ERR, "Failed to lookup %s", ares_strerror(status));
		return;
	}

  APP_LOG("ASYNCDNS", LOG_DEBUG, "Found address name %s", host->h_name);

	memset(ip1, 0x0, INET6_ADDRSTRLEN);
	memset(gturn_lbip, 0x0, sizeof(gturn_lbip));
	for (i = 0; host->h_addr_list[i]; ++i) {
		retVal = inet_ntop(host->h_addrtype, host->h_addr_list[i], ip1, sizeof(ip1));
		if (retVal) {
			APP_LOG("ASYNCDNS", LOG_DEBUG, "Found resolve IP %d - %s", i, ip1);
			if (i == 0) {
				snprintf(gturn_lbip, sizeof(gturn_lbip), "%s", ip1);
				APP_LOG("ASYNCDNS", LOG_DEBUG, "Using resolved IP %d-%s-%s", i, ip1, gturn_lbip);
			}
		} else {
			continue;
		}
	}

  APP_LOG("ASYNCDNS", LOG_DEBUG, "Going out of callback");
	return;
}

int remoteLBDomainResolve(char* turn_lb_name) {
		int num = PLUGIN_SUCCESS;
		int retVal = PLUGIN_FAILURE;

#ifdef __SYNC_DNS__
		char ipArr[MAX_IP_COUNT][SIZE_64B];
		int i = PLUGIN_SUCCESS;
		memset(ipArr, 0x0, (sizeof(char)*((MAX_IP_COUNT)*(SIZE_64B))));
		memset(gturn_lbip, 0x0, SIZE_32B);

		if(0x00 != strlen(g_turnServerEnvIPaddr))       /* turn server environment set from App */
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "nemoteTSDomainResolve g_turnServerEnvIPaddr%s\n", g_turnServerEnvIPaddr);
				snprintf(gturn_lbip, sizeof(gturn_lbip), "%s", g_turnServerEnvIPaddr);
				num = 1;
		}
		else
		{
				remoteParseDomainLkup(turn_lb_name, ipArr, &num);
				for (i=PLUGIN_SUCCESS; i < num; i++) {
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "TURN SERVER LB IP-%d-%d-%s-\n", num, i, *(ipArr+i));
				}
				if (num > PLUGIN_SUCCESS) {
						snprintf(gturn_lbip, sizeof(gturn_lbip), "%s", *(ipArr));
				}
		}
#else
		if(0x00 != strlen(g_turnServerEnvIPaddr))       /* turn server environment set from App */
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "nemoteTSDomainResolve g_turnServerEnvIPaddr%s\n", g_turnServerEnvIPaddr);
				snprintf(gturn_lbip, sizeof(gturn_lbip), "%s", g_turnServerEnvIPaddr);
				num = 1;
		}
		else
		{
				retVal = resolveDNSToIP(turn_lb_name, 1, NULL, callback_natdns);
				if ((0x00 != strlen(gturn_lbip))) {
						num = 1;
				}
		}
#endif
		return num;
}

#define TSPUBDATA "msggetTurnserverIpgetTSPublicIPforclientsgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIP" \
		"getTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIP" \
"getTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIPgetTS-PublicIP"

char *getTSPublicIP_N(char *turnlb_ip)
{
		int sock, flag, sndbuf;
		char send_data[MAX_BUF_LEN];
		char *recv_data = NULL;
		char rdata[MAX_BUF_LEN];
		struct sockaddr_in server_addr;
		int recv_len = PLUGIN_SUCCESS, send_len = PLUGIN_SUCCESS;
		int serv_port = PLUGIN_SUCCESS;
		struct timeval tv;
		uint16_t recv_timeout=8;
		int numsecs = MIN_NATCL_SLEEP*10;

		serv_port = TURN_PUBSERVER_PORT;

		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == PLUGIN_FAILURE) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket creation failed in getTSPublicIP\n");
				pluginUsleep(numsecs*1000000);
				return NULL;
		}

		flag = 1;
		setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));
		sndbuf = 0;  /* Send buffer size */
		setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "LB ip in getTSPublicIP=%s\n", turnlb_ip);
		bzero(&server_addr, sizeof(server_addr));
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = ntohs(serv_port);
		server_addr.sin_addr.s_addr = inet_addr(turnlb_ip);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "before connecting to %d\n", serv_port);

		int flags1 = fcntl(sock, F_GETFL, 0);
		int flags = (flags1|O_NONBLOCK);
		fcntl(sock, F_SETFL, flags);

		int connstatus = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
		int reterr = errno;
		APP_LOG("REMOTEACCESS", LOG_ERR, "local socket trying to connected to %d with connstatus %d in getTSPublicIP %d-%d\n", serv_port, connstatus, errno, reterr);
		if (connstatus < PLUGIN_SUCCESS) {
		  APP_LOG("REMOTEACCESS", LOG_ERR, "local socket after connect attempt to %d with connstatus %d in getTSPublicIP %d\n", serv_port, connstatus, errno, reterr);
			if (reterr != EINPROGRESS) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket connect to %d failed in getTSPublicIP\n", serv_port);
				//close(sock);
				//shutdown(sock, SHUT_RDWR);
				//pluginUsleep(numsecs*1000000);
				//return NULL;
			}
		}
		if (connstatus == PLUGIN_SUCCESS) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket connected to %d immediately in getTSPublicIP\n", serv_port);
				goto success_conn;
		}

		fd_set reads, writes;
		struct timeval	tval;
		int error; unsigned int len;
		FD_ZERO(&reads);
		FD_SET(sock, &reads);
		writes = reads;
		tval.tv_sec = numsecs;
		tval.tv_usec = 0;

		if ( (connstatus = select(sock+1, &reads, &writes, NULL, numsecs ? &tval : NULL)) == 0) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket connection to %d timed out after %d in getTSPublicIP\n", serv_port, numsecs);
				close(sock);			/* timedout */
				shutdown(sock, SHUT_RDWR);
				pluginUsleep(numsecs*1000000);
		  	return NULL;
		}

		if (FD_ISSET(sock, &reads) || FD_ISSET(sock, &writes)) {
				len = sizeof(error);
				if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
						APP_LOG("REMOTEACCESS", LOG_ERR, "local socket connection to %d returned error %d in getTSPublicIP\n", serv_port, error);
						close(sock);			/* timedout */
						shutdown(sock, SHUT_RDWR);
						pluginUsleep(numsecs*1000000);
						return NULL;
				}
		}

		if (error) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket connection to %d returned error %d in getTSPublicIP\n", serv_port, error);
				close(sock);			/* timedout */
				shutdown(sock, SHUT_RDWR);
				pluginUsleep(numsecs*1000000);
				return NULL;
		}

success_conn:
		fcntl(sock, F_SETFL, flags1);	/* restore file status flags */
		memset(send_data, 0, MAX_BUF_LEN);
		snprintf(send_data, sizeof(send_data), "%s", TSPUBDATA);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "sending data bytes :%d to turn server=%s on port:%d\n", strlen(send_data), \
						inet_ntoa(server_addr.sin_addr),(server_addr.sin_port));
		send_len = send(sock,send_data,strlen(send_data), 0);
		if (send_len <= PLUGIN_SUCCESS) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket send to %d failed in getTSPublicIP\n", serv_port);
				close(sock);
				shutdown(sock, SHUT_RDWR);
				pluginUsleep(numsecs*1000000);
				return NULL;
		}
		tv.tv_sec = recv_timeout;
		tv.tv_usec = 0;
		setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));

		memset(rdata, 0x0, MAX_BUF_LEN);
		recv_len = recv(sock, rdata, MAX_BUF_LEN,0);
		if (recv_len <= PLUGIN_SUCCESS) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket recv from %d failed in getTSPublicIP\n", serv_port);
				close(sock);
				shutdown(sock, SHUT_RDWR);
				pluginUsleep(numsecs*1000000);
				return NULL;
		}
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "data received=%s recv_len=%d\n", rdata, recv_len);
		close(sock);
		shutdown(sock, SHUT_RDWR);

		//recv_data = (char *)malloc(sizeof(char) * SIZE_32B);
		recv_data = (char *)malloc(sizeof(char) * (recv_len+1));
		if(!recv_data )
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "malloc failed for recv_data in getTSPublicIP\n");
				pluginUsleep(numsecs*1000000);
				return NULL;
		}
		if (strstr(rdata, "0.0.0.0")) {
				rdata[recv_len] = '\0';
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "NC data recv_data=%s\n", rdata);
				free(recv_data);
				recv_data = NULL;
				pluginUsleep(numsecs*1000000);
				return NULL;
		}else {
				//memset(recv_data, 0x00, (sizeof(char) * SIZE_32B));
				memset(recv_data, 0x00, recv_len+1);
				//strncpy(recv_data,rdata,(sizeof(char) * SIZE_32B));
				strncpy(recv_data,rdata,recv_len);
				recv_data[recv_len] = '\0';
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "NC data recv_data=%s\n", recv_data);
		}

		return recv_data;
}

char *getTSPublicIP(char *turnlb_ip)
{
		int sock, flag, sndbuf;
		char send_data[MAX_BUF_LEN];
		char *recv_data = NULL;
		char rdata[MAX_BUF_LEN];
		struct sockaddr_in server_addr;
		int recv_len = PLUGIN_SUCCESS, send_len = PLUGIN_SUCCESS;
		int serv_port = PLUGIN_SUCCESS;
		struct timeval tv;
		uint16_t recv_timeout=8;

		serv_port = TURN_PUBSERVER_PORT;

		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == PLUGIN_FAILURE) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket creation failed in getTSPublicIP\n");
				return NULL;
		}

		flag = 1;
		setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(flag));
		sndbuf = 0;  /* Send buffer size */
		setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "LB ip in getTSPublicIP=%s\n", turnlb_ip);
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = ntohs(serv_port);
		server_addr.sin_addr.s_addr = inet_addr(turnlb_ip);
		bzero(&(server_addr.sin_zero),8);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "before connecting to %d\n", serv_port);
		if (connect(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == PLUGIN_FAILURE)
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket connect to %d failed in getTSPublicIP\n", serv_port);
				close(sock);
				shutdown(sock, SHUT_RDWR);
				return NULL;
		}
		memset(send_data, 0, MAX_BUF_LEN);
		snprintf(send_data, sizeof(send_data), "%s", TSPUBDATA);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "sending data bytes :%d to turn server=%s on port:%d\n", strlen(send_data), \
						inet_ntoa(server_addr.sin_addr),(server_addr.sin_port));
		send_len = send(sock,send_data,strlen(send_data), 0);
		if (send_len <= PLUGIN_SUCCESS) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket send to %d failed in getTSPublicIP\n", serv_port);
				close(sock);
				shutdown(sock, SHUT_RDWR);
				return NULL;
		}
		tv.tv_sec = recv_timeout;
		tv.tv_usec = 0;
		setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));

		memset(rdata, 0x0, MAX_BUF_LEN);
		recv_len = recv(sock, rdata, MAX_BUF_LEN,0);
		if (recv_len <= PLUGIN_SUCCESS) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket recv from %d failed in getTSPublicIP\n", serv_port);
				close(sock);
				shutdown(sock, SHUT_RDWR);
				return NULL;
		}
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "data received=%s recv_len=%d\n", rdata, recv_len);
		close(sock);
		shutdown(sock, SHUT_RDWR);

		recv_data = (char *)malloc(sizeof(char) * (recv_len+1));
		if(!recv_data )
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "malloc failed for recv_data in getTSPublicIP\n");
				return NULL;
		}
		if (strstr(rdata, "0.0.0.0")) {
				rdata[recv_len] = '\0';
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "NC data recv_data=%s\n", rdata);
				free(recv_data);
				recv_data = NULL;
				return NULL;
		}else {
				memset(recv_data, 0x00, sizeof(char) * (recv_len+1));
				strncpy(recv_data,rdata,recv_len);
				recv_data[recv_len] = '\0';
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "NC data recv_data=%s\n", recv_data);
		}

		return recv_data;
}

char *getTSUdpPublicIP(char *turnlb_ip)
{
		int sock, cliLen;
		char send_data[MAX_BUF_LEN];
		char *recv_data;
		char rdata[MAX_BUF_LEN];
		struct sockaddr_in server_addr;
		struct sockaddr_in cliAddr;
		int recv_len = 0, send_len = 0;
		int serv_port = 0;
		struct timeval tv;
		uint16_t recv_timeout=5;

		serv_port = TURN_PUBSERVER_PORT;

		if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1){
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket creation failed in getTSUdpPublicIP\n");
				return NULL;
		}

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "LB ip in getTSUdpPublicIP=%s\n", turnlb_ip);
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = ntohs(serv_port);
		server_addr.sin_addr.s_addr = inet_addr(turnlb_ip);
		bzero(&(server_addr.sin_zero),8);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "before connecting to %d\n", serv_port);
		memset(send_data, 0, MAX_BUF_LEN);
		snprintf(send_data, sizeof(send_data), "%s", TSPUBDATA);
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "sending data bytes :%d to turn server=%s on port:%d\n", strlen(send_data), \
						inet_ntoa(server_addr.sin_addr),(server_addr.sin_port));
		send_len  = sendto(sock, send_data, strlen(send_data), 0,
						(struct sockaddr *)&server_addr, sizeof(struct sockaddr));
		if (send_len <= PLUGIN_SUCCESS) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket send to %d failed in getTSUdpPublicIP\n", serv_port);
				close(sock);
				shutdown(sock, SHUT_RDWR);
				return NULL;
		}
		tv.tv_sec = recv_timeout;
		tv.tv_usec = 0;
		setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));

		memset(rdata, 0x0, MAX_BUF_LEN);
		cliLen = sizeof(cliAddr);
		recv_len = recvfrom(sock, rdata, MAX_BUF_LEN, 0,
						(struct sockaddr *) &cliAddr, (socklen_t*)&cliLen);
		if (recv_len <= 0) {
				APP_LOG("REMOTEACCESS", LOG_ERR, "local socket recv from %d failed in getTSUdpPublicIP\n", serv_port);
				close(sock);
				shutdown(sock, SHUT_RDWR);
				return NULL;
		}
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "data received=%s recv_len=%d\n", rdata, recv_len);
		close(sock);
		shutdown(sock, SHUT_RDWR);

		//recv_data = (char *)malloc(sizeof(char) * SIZE_32B);
		recv_data = (char *)malloc(sizeof(char) * (recv_len+1));
		if(!recv_data )
		{
				APP_LOG("REMOTEACCESS", LOG_ERR, "malloc failed for recv_data in getTSUdpPublicIP\n");
				return NULL;
		}
		memset(recv_data, 0x00, sizeof(char) * (recv_len+1));
		strncpy(recv_data,rdata,recv_len);
		recv_data[recv_len] = '\0';
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "NC data recv_data=%s\n", recv_data);
		return recv_data;
}

void trigger_nat(void )
{
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"reached In trigger_nat() \n");
		pthread_t relay_thread;
		pthread_create(&relay_thread, NULL, nat_trav_reinit, NULL);
		pthread_detach(relay_thread);

}

int initNATCore() {
		memset(&o,0,sizeof(opt));
		memset(&g,0,sizeof(glb));
		initTurnDataMutex();
#ifdef __RINIT_M__
		osUtilsCreateLock(&remote_init_mutex);
#endif
		return PLUGIN_SUCCESS;
}
