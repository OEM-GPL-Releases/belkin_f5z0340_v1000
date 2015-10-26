/***************************************************************************
*
*
* iceInit.c
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
#include "logger.h"

#include "types.h"
#include "sigGen.h"
#include "natTravIceWrapper.h"
#include "natTravIceIntDef.h"
/******identify header file for debug logs and remove rest - TODO *****/
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <syslog.h>

#include "sigGen.h"
#include "defines.h"
#include "curl/curl.h"
#include "httpsWrapper.h"
#include "logger.h"

#include "pjnath.h"
#include "pjlib-util.h"
#include "pjlib.h"
#include "natTravIceWrapper.h"

#include "natClient.h"
#include "mxml.h"
#include "global.h"

#define THIS_FILE   "natTravIceWrapper.c"

extern int gpluginNatInitialized;
extern int gInitInProgressCounter;
extern int gNatAuthRetryTime;

extern char gturn_serverip[SIZE_32B];

char g_auth_key[SIZE_50B];
char g_nattrav_local_peer_id[SIZE_20B];
char g_nattrav_remote_peer_id[SIZE_20B];
ice_config_info gConfigInfo;
nattrav_cand_offer_Info gLocalnfo;
nattrav_cand_offer_Info gRemoteInfo;
int g_count;
char gIcePeerIp[SIZE_32B];
char gIcePeerPort[SIZE_32B];
char gIcePeerInternalIp[SIZE_32B];
char gServersLBIP[SIZE_64B];

void dev_appl_on_rx_data_handler(void *hndl,  void *pkt, unsigned int pkt_len, char* sender_addr);
char *checkICEPeerPublicIP(char* srvName, char *mac, char* key, char *serial);
char* getICEpeerPublicIP(char* srvName, char *mac, char* key, char *serial);
char *getSIPSrvPublicIPRespParser(char* sipIPResp);
char* checkDomainNameResolved(char* name);
pthread_mutex_t g_tsx_id_list_lock = PTHREAD_MUTEX_INITIALIZER;

extern int gWiFiClientDeviceCurrState;
extern void* UDSiceReqHandlThread(void *arg);
extern char gPluginPrivatekey[MAX_PKEY_LEN];
extern char gClientType[SIZE_128B];
extern char gWiFiMacAddress[SIZE_64B];
extern char gSerialNo[SIZE_64B];
int remoteAccessInitIce()
{
		int status;
		void *hndl = NULL;
		int use_dlr_for_remote = 0;

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "entering remoteAccessInitIce\n");
		memset(g_auth_key, 0, SIZE_50B);
		strncpy(g_auth_key, gPluginPrivatekey, sizeof(g_auth_key)-1);
		strcpy(g_nattrav_remote_peer_id,REMOTE_PEER_MAC);

		APP_LOG("REMOTEACCESS", LOG_DEBUG, "remote peer's mac: %s\n", g_nattrav_remote_peer_id);
		APP_LOG("REMOTEACCESS", LOG_HIDE, "use dlr to get remote peer's address info:%d,g_auth_key:%s,gPluginPrivatekey:%s",use_dlr_for_remote, g_auth_key,gPluginPrivatekey);

		memset(&gConfigInfo, 0, sizeof(ice_config_info));
		g_count = 1;

		status = nattrav_read_conf_info(&gConfigInfo);
		if(status != SUCCESS)
				return -1;

		memset(gConfigInfo.auth_key, 0, SIZE_50B);
		strncpy(gConfigInfo.auth_key, gPluginPrivatekey, sizeof(gConfigInfo.auth_key)-1);
		APP_LOG("REMOTEACCESS", LOG_HIDE, "NC pvt key=%s , ser=%s TSIP=%s \n", gConfigInfo.auth_key, gConfigInfo.serial_num, gConfigInfo.turn_server_ip);

		gConfigInfo.appl_cb_on_rx_data = dev_appl_on_rx_data_handler;
		gConfigInfo.turn_tcp_data_port = 0;

#ifndef __STBTOTS__
		char *IcePeerIp=NULL;
		/* get Ice lite peer IP */
		IcePeerIp = checkICEPeerPublicIP(NULL,gConfigInfo.mac_addr,gConfigInfo.auth_key,gConfigInfo.serial_num);
		if(!IcePeerIp)
		{
				APP_LOG("REMOTEACCESS", LOG_CRIT, "getNode  failed IcePeerIp NULL\n");
				return -1;
		}
		strcpy(gConfigInfo.turn_server_ip, IcePeerIp);
		PJ_LOG(2,(THIS_FILE, "ICE:: PLG_SIGNATURE_EXP\n"));
		gConfigInfo.turn_srvr_port =atoi(gIcePeerPort);
		PJ_LOG(2,(THIS_FILE, "ICE:: PLG_SIGNATURE_EXP\n"));
#endif

		memset(&gLocalnfo,0,sizeof(gLocalnfo));
		strcpy(g_nattrav_local_peer_id,gConfigInfo.mac_addr);
		strcpy(gLocalnfo.peer_id, g_nattrav_local_peer_id);

		hndl  = nattrav_init(&gConfigInfo);
		if(hndl == NULL)
		{
				APP_LOG("REMOTEACCESS", LOG_CRIT, "device appl : nat library init failed\n");
				nattrav_ice_destroy(hndl,&gLocalnfo);
				return -1;
		}

		memset(&gRemoteInfo,0,sizeof(gRemoteInfo));
		use_dlr_for_remote=0;
		if(use_dlr_for_remote)
		{
				strcpy(gRemoteInfo.peer_id, g_nattrav_remote_peer_id);
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "local peer id:%s, remote peer id:%s\n",g_nattrav_local_peer_id,g_nattrav_remote_peer_id);
				while(1)
				{
						status = nattrav_get_remote_address_info(hndl,&gRemoteInfo);

						if(status != SUCCESS)
						{
								APP_LOG("REMOTEACCESS", LOG_ERR, "peer:%s info not available\n",g_nattrav_remote_peer_id);
								sleep(DELAY_5SEC);
								continue;
						}
						else
						{
								APP_LOG("REMOTEACCESS", LOG_DEBUG,"remote peer:%s, def port:%u,def ip :%s, ice type :%d\n", gRemoteInfo.peer_id,
												gRemoteInfo.def_addr_info.def_port,gRemoteInfo.def_addr_info.def_ipaddr,gRemoteInfo.ice_type);
								break;
						}
				}
		}
		else
		{
#ifdef __STBTOTS__
				char *IcePeerIp=NULL;
				/* get Ice lite peer IP */
				IcePeerIp = checkICEPeerPublicIP(NULL,gConfigInfo.mac_addr,gConfigInfo.auth_key,gConfigInfo.serial_num);
				if(!IcePeerIp)
				{
						nattrav_ice_destroy(hndl,&gLocalnfo);
						return -1;
				}
#endif
				/* for a peer which does not add its record in dlr, mainly for cloud */
				gRemoteInfo.ice_type = NAT_TRAV_ICE_TYPE_FULL;
				strcpy(gRemoteInfo.def_addr_info.def_ipaddr,gIcePeerIp);
				gRemoteInfo.def_addr_info.def_port = atoi(gIcePeerPort) /*8035*/;
				gRemoteInfo.def_addr_info.def_transport = gConfigInfo.transport;
				gRemoteInfo.def_addr_info.ip_family = NAT_TRAV_FAMILY_IPV4;
				gRemoteInfo.ice_info.cand_cnt= 1;
				status = nattrav_get_icelite_remote_address_info(hndl, &gRemoteInfo);
		}

		status = nattrav_start_session(hndl,&gRemoteInfo);
		if(status != SUCCESS)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"ice session can not be finished\n");
				nattrav_ice_destroy(hndl,&gLocalnfo);
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"exiting peer:%s\n",g_nattrav_local_peer_id);
				return -1;
		}

		gpluginNatInitialized = NATCL_INITIALIZED;
		gInitInProgressCounter = NATCL_NOT_INITIALIZED;
		gNatAuthRetryTime = NATCL_NOT_INITIALIZED;

		return SUCCESS;
}

int nattrav_read_conf_info(ice_config_info *psConfigInfo)
{
		/* copy authentication related info*/
		strcpy(psConfigInfo->realm, REALM);
		strcpy(psConfigInfo->mac_addr,gWiFiMacAddress);
		strcpy(psConfigInfo->serial_num, gSerialNo);
		PJ_LOG(2,(THIS_FILE, "1 ICE: PLG_SIGNATURE_EXP MAC ADDR=%s\n", psConfigInfo->serial_num));
		PJ_LOG(2,(THIS_FILE, "2 ICE: PLG_SIGNATURE_EXP\n"));
		psConfigInfo->signaure_expires = PLG_SIGNATURE_EXP;
		PJ_LOG(2,(THIS_FILE, "3 ICE: PLG_SIGNATURE_EXP\n"));
		psConfigInfo->method_id = NATTRAV_AUTH_METHOD_MAC_BASED;
		PJ_LOG(2,(THIS_FILE, "4 ICE: PLG_SIGNATURE_EXP\n"));

		/* copy peer info*/
		strcpy(psConfigInfo->peerIp, PLG_PEERIP);
		PJ_LOG(2,(THIS_FILE, "5 ICE: PLG_SIGNATURE_EXP\n"));
		psConfigInfo->peerPort = PLG_PEER_PORT;
		PJ_LOG(2,(THIS_FILE, "6 ICE: PLG_SIGNATURE_EXP\n"));

		PJ_LOG(2,(THIS_FILE, "ICE:77777  PLG_SIGNATURE_EXP\n"));
		snprintf(psConfigInfo->dlr_url,sizeof(psConfigInfo->dlr_url), "https://%s:8443/apis/http/dlr/", BL_DOMAIN_NM); // for postpeer inc case of ICE
		PJ_LOG(2,(THIS_FILE, "ICE:8888  PLG_SIGNATURE_EXP\n"));
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "TSIP=%s port=%d\n", gturn_serverip, TURN_SERVER_PORT );
		/* turn server related info*/
		strcpy(psConfigInfo->turn_server_ip, gturn_serverip);
		PJ_LOG(2,(THIS_FILE, "ICE:: PLG_SIGNATURE_EXP\n"));
		psConfigInfo->turn_srvr_port = TURN_SERVER_PORT;
		PJ_LOG(2,(THIS_FILE, "ICE:: PLG_SIGNATURE_EXP\n"));

		psConfigInfo->transport = NAT_TRAV_USE_TRANSPORT_UDP;
		PJ_LOG(2,(THIS_FILE, "ICE:: PLG_SIGNATURE_EXP\n"));

		/* ice related info*/
		psConfigInfo->ice_role = NAT_TRAV_ICE_ROLE_CONTROLLING;
		PJ_LOG(2,(THIS_FILE, "ICE:: PLG_SIGNATURE_EXP\n"));

		psConfigInfo->ice_capabilty = NAT_TRAV_ICE_TYPE_FULL;
		PJ_LOG(2,(THIS_FILE, "ICE:: PLG_SIGNATURE_EXP\n"));
		psConfigInfo->method_id = NATTRAV_AUTH_METHOD_MAC_BASED;
		PJ_LOG(2,(THIS_FILE, "ICE:: PLG_SIGNATURE_EXP\n"));
		APP_LOG("REMOTEACCESS", LOG_DEBUG, "configuration read returning success\n");
		return SUCCESS;

}

/*Function to get ICE Peer  IP using GetNode */
char *checkICEPeerPublicIP(char* srvName, char *mac, char* key, char *serial) {
		char *icePublicIP = NULL;
		char  *srvLBIP = NULL;
		int counter=0;
		srvLBIP = checkDomainNameResolved(ICE_PEER_LB_NAME);
		if (srvLBIP) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"SERVERS LB Name resolved %s\n", srvLBIP);
				memset(gServersLBIP, 0,sizeof(gServersLBIP));
				snprintf(gServersLBIP,sizeof(gServersLBIP), "%s", srvLBIP);
				free(srvLBIP);
				srvLBIP = NULL;
		}

		while (counter < ICE_GETNODE_MAX_COUNT) {
				APP_LOG("REMOTEACCESS", LOG_HIDE, "trying to get ICE Public IP:\nmac:%s\nkey:%s\nserial:%s", mac,key,serial);
				icePublicIP = getICEpeerPublicIP(gServersLBIP, mac, key, serial);
				if (icePublicIP) break;
				counter ++;
				sleep(5);

		}
		return icePublicIP;
}

char* getICEpeerPublicIP(char* srvName, char *mac, char* key, char *serial) {
		UserAppSessionData *pUsrAppSsnData = NULL;
		UserAppData *pUsrAppData = NULL;
		authSign *assign = NULL;
		int status = 1;
		int retVal = 0;
		char tmp_url[MAX_BUF_LEN];
		char *ptr = NULL;
		char *icePublicIP = NULL;
		//Resolve LB domain name to get SIP Server and other server IPs
		assign = createAuthSignature(mac, serial, key);
		if (!assign) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Signature Structure returned NULL\n");
				goto exit_below;
		}

		pUsrAppData = (UserAppData *)malloc(sizeof(UserAppData));
		if (!pUsrAppData) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Malloc Structure returned NULL\n");
				goto exit_below;
		}
		memset( pUsrAppData, 0x0, sizeof(UserAppData));

		pUsrAppSsnData = webAppCreateSession(0);
		if (!pUsrAppSsnData) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Session Structure returned NULL\n");
				goto exit_below;
		}
		/* prepare REST header*/
		{
#ifdef __IPRESOLVE__
				snprintf(tmp_url,sizeof(tmp_url), "https://%s:8444/ICEService/ice/getNode", gServersLBIP);
#else
				snprintf(tmp_url,sizeof(tmp_url), "https://%s:8444/ICEService/ice/getNode", ICE_PEER_LB_NAME);
#endif
				strcpy( pUsrAppData->url, tmp_url);
				strcpy( pUsrAppData->keyVal[0].key, "Content-Type");
				strcpy( pUsrAppData->keyVal[0].value, "application/xml");
				strcpy( pUsrAppData->keyVal[1].key, "Authorization");
				strcpy( pUsrAppData->keyVal[1].value,  assign->signature);


				strcpy( pUsrAppData->keyVal[0].value, "application/xml");
				strcpy( pUsrAppData->keyVal[1].key, "Authorization");
				strcpy( pUsrAppData->keyVal[1].value,  assign->signature);
				strcpy( pUsrAppData->keyVal[2].key, "Accept");
				strcpy( pUsrAppData->keyVal[2].value, "application/xml");
				strncpy( pUsrAppData->keyVal[3].key, "X-Belkin-Client-Type-Id", sizeof(pUsrAppData->keyVal[3].key)-1);   
				strncpy( pUsrAppData->keyVal[3].value, gClientType, sizeof(pUsrAppData->keyVal[3].value)-1);   
				pUsrAppData->keyValLen = 4;

				/* enable SSL if auth URL is on HTTPS*/
				ptr = strstr(pUsrAppData->url,"https" );
				if(ptr != NULL)
						pUsrAppData->httpsFlag =  1;
				else
						pUsrAppData->httpsFlag =  0;
		}
		pUsrAppData->disableFlag = 1;
		pUsrAppData->inDataLength = 0;
		retVal = webAppSendData( pUsrAppSsnData, pUsrAppData, 0);
		if (retVal)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Some error encountered while sending to %s, errorCode %d \n", tmp_url, retVal);
				goto exit_below;
		}
		/* check response header to see if user is authorized or not*/
		{
				ptr = strstr(pUsrAppData->outHeader,"200 OK" );
				if (ptr != NULL)
				{
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"Response 200 OK received from %s\n", tmp_url);
						icePublicIP = getSIPSrvPublicIPRespParser(pUsrAppData->outData);
						if (icePublicIP) {
								APP_LOG("REMOTEACCESS", LOG_DEBUG,"Response received from %s has SIP Public IP %s\n", tmp_url, icePublicIP);
								status = 0;
						}
				} else {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"Response other than 200 OK received from %s\n", tmp_url);
						if (strstr(pUsrAppData->outHeader, "403")) {
								if (strstr(pUsrAppData->outData, "ERR_002")) {
									APP_LOG("REMOTEACCESS", LOG_DEBUG,"error encountered: ERR_002");
								}
						}
						goto exit_below;
				}
		}
exit_below:
		if (pUsrAppData) {
				if (pUsrAppData->outData) {free(pUsrAppData->outData); pUsrAppData->outData = NULL;}
				free(pUsrAppData); pUsrAppData = NULL;
		}
		if (pUsrAppSsnData) {webAppDestroySession ( pUsrAppSsnData ); pUsrAppSsnData = NULL;}
		if (assign) { free(assign); assign = NULL; }
		if (!status) return icePublicIP;
		return NULL;
}

char *getSIPSrvPublicIPRespParser(char* sipIPResp) {
		char *respXml = NULL;
		mxml_node_t *tree = NULL;
		mxml_node_t *first_node = NULL;
		char *publicIP = NULL;
		char *publicPort = NULL;
		char *internalIP = NULL;
		char respData[MAX_BUF_LEN];

		respXml = (char*)sipIPResp;
		if (!respXml) {
				goto exit_below;
		}

		publicIP = (char*)malloc(SIZE_32B);
		if (!publicIP) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"malloc failed\n");
				goto exit_below;
		}
		memset(publicIP, 0x0, SIZE_32B);

		publicPort= (char*)malloc(SIZE_32B);
		if (!publicPort ) {
				goto exit_below;
		}
		memset(publicPort, 0x0, SIZE_32B);

		internalIP= (char*)malloc(SIZE_32B);
		if (!internalIP ) {
				goto exit_below;
		}
		memset(internalIP, 0x0, SIZE_32B);
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"Response XML received %s\n", respXml);
		memset(respData, 0x0, MAX_BUF_LEN);
		snprintf(respData,sizeof(respData), "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>%s\n", respXml);
		tree = mxmlLoadString(NULL, respData, MXML_OPAQUE_CALLBACK);
		if (tree){
				first_node = mxmlFindElement(tree, tree, "nodeIp", NULL, NULL, MXML_DESCEND);
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"--------SIP XML Tree Node\n");
				if ((first_node) && (first_node->child) && (first_node->child->value.opaque)){
						strcpy(publicIP, (first_node->child->value.opaque));
						memset(gIcePeerIp,0,sizeof(gIcePeerIp));
						strcpy(gIcePeerIp,publicIP );
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"ICE Node Public IP retrieved from XML Node is %s gIcePeerIp %s\n", publicIP,gIcePeerIp );
						mxmlDelete(first_node);
				}else {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"ICE Node Public IP XML Node is NULL\n");
						if (first_node) mxmlDelete(first_node);
						mxmlDelete(tree);
						goto exit_below;
				}
				first_node = mxmlFindElement(tree, tree, "nodeInternalIp", NULL, NULL, MXML_DESCEND);
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"--------SIP XML Tree Node\n");
				if ((first_node) && (first_node->child) && (first_node->child->value.opaque)){
						strcpy(internalIP, (first_node->child->value.opaque));
						memset(gIcePeerInternalIp,0,sizeof(gIcePeerInternalIp));
						strcpy(gIcePeerInternalIp,internalIP );
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"ICE Node Public IP retrieved from XML Node is %s\n", internalIP);
						mxmlDelete(first_node);
				}else {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"ICE Node Public IP XML Node is NULL\n");
						if (first_node) mxmlDelete(first_node);
						mxmlDelete(tree);
						goto exit_below;
				}
				first_node = mxmlFindElement(tree, tree, "nodePort", NULL, NULL, MXML_DESCEND);
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"--------SIP XML Tree Node\n");
				if ((first_node) && (first_node->child) && (first_node->child->value.opaque)){
						strcpy(publicPort, (first_node->child->value.opaque));
						memset(gIcePeerPort,0,sizeof(gIcePeerPort));
						strcpy(gIcePeerPort,publicPort );
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"ICE Node Public Port retrieved from XML Node is %s\n", publicPort);
						mxmlDelete(first_node);
				}else {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"ICE Node Public Port XML Node is NULL\n");
						if (first_node) mxmlDelete(first_node);
						mxmlDelete(tree);
						goto exit_below;
				}
				mxmlDelete(tree);
				if (publicIP) { free(publicIP); publicIP = NULL; }
				if (publicPort) { free(publicPort); publicPort = NULL; }
				return gIcePeerIp;
		}
exit_below:
		if (publicIP) { free(publicIP); publicIP = NULL; }
		if (publicPort) { free(publicPort); publicPort = NULL; }
		if (internalIP) { free(internalIP); internalIP = NULL; }
		return NULL;
}

struct tsxid_node{
		char transaction_id[31];
		struct tsxid_node * next;
};
struct tsxid_node *head_tsxid_list=NULL;
int search_running_tsx_id_list( char* tsx_id)
{
		struct tsxid_node *temp=NULL;
		APP_LOG("REMOTEACCESS",LOG_DEBUG,"Transaction id is %s \n", tsx_id);
		pthread_mutex_lock(&g_tsx_id_list_lock);
		temp = head_tsxid_list;
		while(temp!=NULL)
		{
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"Transaction id is %s \n",temp->transaction_id);
				if(! strncmp(temp->transaction_id,tsx_id,(sizeof(char)*30)))
				{
						APP_LOG("REMOTEACCESS",LOG_DEBUG,"Transaction id is %s \n",temp->transaction_id);
						pthread_mutex_unlock(&g_tsx_id_list_lock);
						return PLUGIN_SUCCESS;
				}		
				temp=temp->next;
		}
		APP_LOG("REMOTEACCESS",LOG_DEBUG,"Transaction id is %s \n",tsx_id);
		pthread_mutex_unlock(&g_tsx_id_list_lock);
		return PLUGIN_FAILURE;
}
void  delete_from_running_tsx_id_list(char *transaction_id)
{
		APP_LOG("REMOTEACCESS",LOG_DEBUG,"delete_from_running_tsx_id_list freeing Head  tsx_id %s\n",transaction_id);
		pthread_mutex_lock(&g_tsx_id_list_lock);
		if(head_tsxid_list==NULL)
		{
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"delete_from_running_tsx_id_list head is NULL\n");
				pthread_mutex_unlock(&g_tsx_id_list_lock);

				return;
		}
		else if(!(head_tsxid_list->next))
		{
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"delete_from_running_tsx_id_list freeing Head\n");
				if(!strncmp(head_tsxid_list->transaction_id,transaction_id,sizeof(char)*30))
				{
						APP_LOG("REMOTEACCESS",LOG_DEBUG,"delete_from_running_tsx_id_list freeing Head \n");
						free(head_tsxid_list);
						head_tsxid_list=NULL;
				}
		}
		else
		{
				struct tsxid_node *temp=NULL;
				temp=head_tsxid_list;
				while(temp->next!=NULL)
				{
						if(!strncmp(temp->next->transaction_id,transaction_id,sizeof(char)*30))
						{
								APP_LOG("REMOTEACCESS",LOG_DEBUG,"delete_from_running_tsx_id_list %s \n",transaction_id);
								struct tsxid_node *p=NULL;
								p=temp->next;
								temp->next=temp->next->next;
								free(p);
								p=NULL;
						}
						temp=temp->next;
				}
		}
		APP_LOG("REMOTEACCESS",LOG_DEBUG,"delete_from_running_tsx_id_list freeing Head\n");
		pthread_mutex_unlock(&g_tsx_id_list_lock);
}
struct tsxid_node * create_node(char *tsx_id)
{
		APP_LOG("REMOTEACCESS",LOG_DEBUG,"create_node with tsx_id ...........%s \n",tsx_id);
		struct tsxid_node *new_node=NULL;
		new_node = (struct tsxid_node*)malloc(sizeof(struct tsxid_node));
		memset(new_node,0,sizeof(struct tsxid_node));
		memcpy(new_node->transaction_id,tsx_id,sizeof(char)*30);
		new_node->next=NULL;

		return new_node;
}



void add_running_tsx_id_list(struct tsxid_node *node)
{
		APP_LOG("REMOTEACCESS",LOG_DEBUG,"add_running_tsx_id_list ...........\n");

		pthread_mutex_lock(&g_tsx_id_list_lock);
		if(head_tsxid_list==NULL)
		{

				head_tsxid_list=node;
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"add_running_tsx_id_list ...........\n");
		}
		else
		{
				APP_LOG("REMOTEACCESS",LOG_DEBUG,"add_running_tsx_id_list ...........\n");

				struct tsxid_node *temp=NULL;
				temp=head_tsxid_list;
				while(temp->next!=NULL)
				{
						temp=temp->next;
				}
				temp->next=node;
		}
		APP_LOG("REMOTEACCESS",LOG_DEBUG,"add_running_tsx_id_list ...........\n");

		pthread_mutex_unlock(&g_tsx_id_list_lock);
}

void dev_appl_on_rx_data_handler(void *hndl,  void *pkt, unsigned int pkt_len, char* sender_addr)
{
		unsigned int msg_type;
		unsigned int len;
		struct arg_struct *arguments=NULL;
    int retVal=PLUGIN_SUCCESS;
		arguments=(struct arg_struct *)malloc(sizeof(struct arg_struct));

		APP_LOG("REMOTEACCESS", LOG_DEBUG,"ice peer application: data received by peer:%s   data :%u \n",
						g_nattrav_local_peer_id,pkt_len);
		fflush(NULL);

		msg_type = ntohl((*(unsigned int *)pkt));
		len  =ntohl(*((unsigned int *)pkt+1));
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"length of mesage is %u \n",len);
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"length of msg_type is %u \n",msg_type);

		pkt_len=len;
		arguments->transaction_id = (char *)malloc(sizeof(char)*31);
		if(!(arguments->transaction_id))
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG," Transaction id malloc failed \n");
				return;
		}

		memset(arguments->transaction_id,0,(sizeof(char)*31));
		memcpy(arguments->transaction_id,((unsigned int *)pkt+2),30*sizeof(char));
		arguments->transaction_id[30]='\0';
		APP_LOG("REMOTEACCESS", LOG_DEBUG," Transaction id is %s \n",arguments->transaction_id);
		pkt=(char*)pkt+8;
		arguments->hndl = hndl;
		arguments->pkt=pkt;
		APP_LOG("REMOTEACCESS", LOG_ERR, "Finally exiting from ICE in before delete \n");
		APP_LOG("REMOTEACCESS", LOG_ERR, "Finally exiting from ICE in callback \n");
		arguments->pkt_len=pkt_len;
		pthread_t iceReqHandler_thread;
		retVal=pthread_create(&iceReqHandler_thread, NULL, UDSiceReqHandlThread,(void*) (arguments));
		pthread_join(iceReqHandler_thread,NULL);
		if(retVal)
		{
			if(arguments){free(arguments->transaction_id);free(arguments);arguments=NULL;}
		}	
		APP_LOG("REMOTEACCESS", LOG_ERR, "Finally exiting from ICE in before delete \n");
		APP_LOG("REMOTEACCESS", LOG_ERR, "Finally exiting from ICE in callback \n");
		return;
}

#define MAX_IP_NUM  10
char* resolveDomainName(char* name) {
		char ipArr[MAX_IP_NUM][SIZE_64B];
		int num = 0, i = 0;
		char *resolvedIP = NULL;

		memset(ipArr, 0x0, (sizeof(char)*((MAX_IP_NUM)*(SIZE_64B))));
		resolvedIP = (char*)malloc(SIZE_64B);
		if (!resolvedIP) {
				return NULL;
		}
		memset(resolvedIP, 0x0, SIZE_64B);
		remoteParseDomainLkup(name, ipArr, &num);
		for (i=0; i < num; i++) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"NAME %s IP-%d-%d-%s-\n", name, num, i, *(ipArr+i));
		}
		if (num > 0) {
				snprintf(resolvedIP,MAX_IP_LEN, "%s", *(ipArr));
				return resolvedIP;
		}
		if (resolvedIP) { free(resolvedIP); resolvedIP = NULL; }
		return NULL;
}

char* checkDomainNameResolved(char* name) {
		char *resolvedIP = NULL;
		while (1) {
				sleep(DELAY_5SEC);
				if(gWiFiClientDeviceCurrState != DEVICE_STATE_CONNECTED)
				{
						APP_LOG("REMOTEACCESS", LOG_DEBUG, "Internet  state is not connected");
						continue;
				}
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"trying to resolve dns name lookup");
				resolvedIP = resolveDomainName(name);
				if (resolvedIP) break;
		}
		return resolvedIP;
}
