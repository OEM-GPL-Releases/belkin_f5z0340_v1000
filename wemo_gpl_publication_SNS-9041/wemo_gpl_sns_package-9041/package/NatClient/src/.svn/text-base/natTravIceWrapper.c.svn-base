/***************************************************************************
*
*
* natTravIceWrapper.c
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
#include <syslog.h>
#include <ithread.h>
#include "types.h"
#include "sigGen.h"
#include "logger.h"
#include "natTravIceWrapper.h"
#include "natTravIceIntDef.h"
#include "httpsWrapper.h"
#include "natClient.h"


#define THIS_FILE   "natTravIceWrapper.c"

#define MAX_MSG_CNT   4

#define TS_AUTH_FAIL -1 
#define AUTH_SIGN_FAIL -2 
#define POST_RELAY_FAIL -3 
#define POST_STUN_FAIL -4 
#define NAT_FAILURE 0 
#define  PERM_FAIL -1 
#define  NO_REALY_ERR -2 
#define CONN_FAILURE -3 
#define  NAT_SUCCESS 0 
#define  MAX_ICE_INIT_COUNT   40

#define CHECK(expr)	status=expr; \
													 if (status!=PJ_SUCCESS) { \
															 err_exit(#expr, status); \
													 }

/* global variables*/
/* global configuration and other info */
sGlbInfo gNatTravGlbInfo;
/* Ice instance for every ICE session*/
sIceInstanceInfo  gNatTravList[NAT_TRAV_MAX_NAT_TRAV_LIB_INSTNC];
/* to protect the global flag*/
pthread_mutex_t g_NatTrav_lock = PTHREAD_MUTEX_INITIALIZER;
extern  char gIcePeerPort[SIZE_32B];
extern char gIcePeerIp[SIZE_32B];
extern char gIcePeerInternalIp[SIZE_32B];
extern ice_config_info gConfigInfo;
int postIceDataDlr(pj_ice_sess_check *valid_pair);
extern int gIceRunning;
int g_send_resp_to_peer;
char g_auth_key[SIZE_50B];
char g_nattrav_local_peer_id[SIZE_20B];
char g_nattrav_remote_peer_id[SIZE_20B];
ice_config_info gConfigInfo;
nattrav_cand_offer_Info gLocalnfo;
nattrav_cand_offer_Info gRemoteInfo;
int g_count;

extern char gWiFiMacAddress[SIZE_64B];
extern char gClientType[SIZE_128B];
extern int gWiFiClientDeviceCurrState;

void * nattrav_init(ice_config_info *psConfigInfo)
{
		int retVal = 0;
		sIceInstanceInfo *psInstInfo = &gNatTravList[gNatTravGlbInfo.IceInstCnt];
		{
				authSign *assign = NULL;
				assign = createAuthSignatureNoExp(psConfigInfo->mac_addr,psConfigInfo->serial_num, psConfigInfo->auth_key);
				if (!assign)
				{
						PJ_LOG(3,(THIS_FILE, "Some problem in authentication signature generation"));
						return NULL;
				}
				strcpy(psConfigInfo->signaure, assign->ssign);
				psConfigInfo->signaure_expires = assign->expiry;

				if (assign) {
					free(assign);
					assign = NULL;
				}
		}
		if(!gNatTravGlbInfo.libinitflag)
		{
				memset(&gNatTravGlbInfo,0,sizeof(gNatTravGlbInfo));

				if(gNatTravGlbInfo.IceInstCnt >=  NAT_TRAV_MAX_NAT_TRAV_LIB_INSTNC)
						return NULL;

				nattrav_lib_init(psConfigInfo); /*init gNatTravGlbInfo with logging, cach pool, stun and turn server credentials */
				gNatTravGlbInfo.libinitflag = 1;
		}
		memcpy(&psInstInfo->sNatLibConfInfo,psConfigInfo,sizeof(ice_config_info));    


		/*talk to turn server over properietary channel*/

		/* create ice instance wic in turn will send stun and turn requests*/    
		retVal = nattrav_create_ice_instance(psInstInfo); // pj_ice_strans_create && init callback cb_on_rx_data, cb_on_ice_complete. 
		if (retVal != PLUGIN_SUCCESS) {
				APP_LOG("REMOTEACCESS", LOG_CRIT,"ICE Instance Creation Failed\n");         
				return NULL;
		}

		/*wait for response from stun/turn server, so tat local candidates are gathered*/
		while(1)
		{
				pthread_mutex_lock(&g_NatTrav_lock);
				if(psInstInfo->NatTravFlag == NAT_TRAV_CANDIDATES_GATERED)
				{
						psInstInfo->NatTravFlag = 0;
						pthread_mutex_unlock(&g_NatTrav_lock);
						break;
				}
				else
				{
						pthread_mutex_unlock(&g_NatTrav_lock);
						sleep(1);
				}
		}

		retVal = nattrav_init_ice_session(psInstInfo->sNatLibConfInfo.ice_role,psInstInfo);
		if (retVal != PLUGIN_SUCCESS) {
				APP_LOG("REMOTEACCESS", LOG_CRIT,"ICE Instance Creation Failed\n");         
				return NULL;
		}
		gNatTravGlbInfo.IceInstCnt++;

		return ((void *)gNatTravGlbInfo.IceInstCnt);
}

int  nattrav_update_local_address_info(void *hndl, nattrav_cand_offer_Info *psLocalnfo)
{
		sIceInstanceInfo *psInstInfo = &gNatTravList[((int)hndl)-1];
#ifdef DLR_SAMPLE
		int sockfd = -1;
		struct sockaddr_in server_addr;
#endif
		int status = SUCCESS;
		nattrav_rest_field_Info sLocalRest;

		memset(&sLocalRest, 0, sizeof(nattrav_rest_field_Info));
		sLocalRest.ice_type = NAT_TRAV_ICE_TYPE_FULL;
		strcpy(sLocalRest.peer_id,psLocalnfo->peer_id);

		nattrav_get_local_ice_cand_info(psInstInfo,&sLocalRest);
		status = postPeerLocation(&sLocalRest,&psInstInfo->sNatLibConfInfo);
		if(status != SUCCESS)
		{
				PJ_LOG(2,(THIS_FILE, "Some problem in communication with DLR service"));
		}



#ifdef DLR_SAMPLE
		sockfd  = socket(AF_INET, SOCK_STREAM, 0);
		if(sockfd < 0)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"socket() fail\n");         
				return -1;
		}
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(DLR_PORT);        
		server_addr.sin_addr.s_addr = inet_addr(DLR_IP);
		bzero(&(server_addr.sin_zero),8);

		if (connect(sockfd, (struct sockaddr *)&server_addr,
								sizeof(struct sockaddr)) == -1)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Connect() to DLR fail");
				close(sockfd);
				return -1;
		}

		{
				sOpSetInfo sOpInfo;
				sOpInfo.op = DLR_OP_SET;    	
				strcpy(sOpInfo.peer_id,psLocalnfo->peer_id);
				memcpy(&sOpInfo.sCandInfo,&sLocalRest,sizeof(nattrav_rest_field_Info));
				send(sockfd,&sOpInfo,sizeof(sOpInfo), 0);
				close(sockfd);
		}
#endif


		return status;
}


int  nattrav_get_remote_address_info(void *hndl, nattrav_cand_offer_Info *psRemotePeerInfo)
{
		sIceInstanceInfo *psInstInfo = &gNatTravList[((int)hndl)-1];
		int status;
		nattrav_rest_field_Info peerInfo;
		strcpy(peerInfo.peer_id,psRemotePeerInfo->peer_id);

		status = getPeerLocation(&peerInfo, &psInstInfo->sNatLibConfInfo);
		if(status == SUCCESS)
		{
				int i;

				strcpy(psRemotePeerInfo->peer_id, peerInfo.peer_id);
				psRemotePeerInfo->ice_type = peerInfo.ice_type;
				memcpy(&psRemotePeerInfo->def_addr_info, &peerInfo.def_addr_info, sizeof(nattrav_def_addr_info));

				strcpy(psRemotePeerInfo->ice_info.ufrag, peerInfo.ice_info.username);
				strcpy(psRemotePeerInfo->ice_info.pwd, peerInfo.ice_info.passwd);

				psRemotePeerInfo->ice_info.cand_cnt = peerInfo.ice_info.cand_cnt;

				for(i =0; i< peerInfo.ice_info.cand_cnt;i++)
				{
						pj_ice_sess_cand *cand;

						cand = &psRemotePeerInfo->ice_info.cand[i];                      
						cand->comp_id = peerInfo.ice_info.candidate[i].compid;
						cand->type = peerInfo.ice_info.candidate[i].cand_type;
						cand->prio = peerInfo.ice_info.candidate[i].priority;

						if(cand->type == PJ_ICE_CAND_TYPE_HOST)
						{

						}

						nattrav_convert_addr_to_pjformat(peerInfo.ice_info.candidate[i].ipaddr,
										(pj_uint16_t)peerInfo.ice_info.candidate[i].port,&cand->addr);

						PJ_LOG(3,(THIS_FILE, "remote cand %d: %s: %d\n",
												i+1,peerInfo.ice_info.candidate[i].ipaddr,peerInfo.ice_info.candidate[i].port));

						pj_strdup2(gNatTravGlbInfo.pool, &cand->foundation, peerInfo.ice_info.candidate[i].foundation);
				}	         	

		}


#if DLR_SAMPLE
		int sockfd = -1;
		struct sockaddr_in server_addr;



		sockfd  = socket(AF_INET, SOCK_STREAM, 0);
		if(sockfd < 0)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"socket() fail\n");         
				return -1;
		}
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(DLR_PORT);        
		server_addr.sin_addr.s_addr = inet_addr(DLR_IP);
		bzero(&(server_addr.sin_zero),8);

		if (connect(sockfd, (struct sockaddr *)&server_addr,
								sizeof(struct sockaddr)) == -1)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Connect() to DLR fail");
				close(sockfd);
				return -1;
		}

		{
				sOpGetInfo sOpInfo;
				sOpInfo.op = DLR_OP_GET;
				strcpy(sOpInfo.peer_id, psRemotePeerInfo->peer_id);
				send(sockfd,&sOpInfo,sizeof(sOpInfo), 0);
		}

		{           
				int bytes_recieved = -1;
				char client_buf[1024];
				bytes_recieved=recv(sockfd,client_buf,1024,0);
				if(bytes_recieved > 0)

				{
						nattrav_rest_field_Info *psPeerInfo;
						int i;

						psPeerInfo =   (nattrav_rest_field_Info*)client_buf;

						strcpy(psRemotePeerInfo->peer_id, psPeerInfo->peer_id);
						psRemotePeerInfo->ice_type = psPeerInfo->ice_type;
						memcpy(&psRemotePeerInfo->def_addr_info, &psPeerInfo->def_addr_info, sizeof(nattrav_def_addr_info));

						strcpy(psRemotePeerInfo->ice_info.ufrag, psPeerInfo->ice_info.username);
						strcpy(psRemotePeerInfo->ice_info.pwd, psPeerInfo->ice_info.passwd);
						psRemotePeerInfo->ice_info.cand_cnt = psPeerInfo->ice_info.cand_cnt;

						for(i =0; i< psPeerInfo->ice_info.cand_cnt;i++)
						{
								pj_ice_sess_cand *cand;

								cand = &psRemotePeerInfo->ice_info.cand[i];                      
								cand->comp_id = psPeerInfo->ice_info.candidate[i].compid;
								cand->type = psPeerInfo->ice_info.candidate[i].cand_type;
								cand->prio = psPeerInfo->ice_info.candidate[i].priority;

								nattrav_convert_addr_to_pjformat(psPeerInfo->ice_info.candidate[i].ipaddr,
												(pj_uint16_t)psPeerInfo->ice_info.candidate[i].port,&cand->addr);

								PJ_LOG(3,(THIS_FILE, "remote cand %d: %s: %d\n",
														i+1,psPeerInfo->ice_info.candidate[i].ipaddr,psPeerInfo->ice_info.candidate[i].port));

								pj_strdup2(gNatTravGlbInfo.pool, &cand->foundation, psPeerInfo->ice_info.candidate[i].foundation);
						}	         	

				}

				close(sockfd);
		}
#endif

		return status;	 
}
void ice_shutdown(const char *title, pj_status_t status)
{
		if (status != PJ_SUCCESS) {
				nattrav_perror(title, status);
		}
		PJ_LOG(3,(THIS_FILE, "Shutting down.."));
		pj_thread_sleep(500);
		gNatTravGlbInfo.thread_quit_flag = PJ_TRUE;
		if (gNatTravGlbInfo.thread) {
				pj_thread_join(gNatTravGlbInfo.thread);
				pj_thread_destroy(gNatTravGlbInfo.thread);
				gNatTravGlbInfo.thread = NULL;
		}
		if (gNatTravGlbInfo.ice_cfg.stun_cfg.ioqueue)
		{
				pj_ioqueue_destroy(gNatTravGlbInfo.ice_cfg.stun_cfg.ioqueue);
				gNatTravGlbInfo.ice_cfg.stun_cfg.ioqueue = NULL;
		}
		if (gNatTravGlbInfo.ice_cfg.stun_cfg.timer_heap)
		{
				pj_timer_heap_destroy(gNatTravGlbInfo.ice_cfg.stun_cfg.timer_heap);
				gNatTravGlbInfo.ice_cfg.stun_cfg.timer_heap = NULL;
		}

		if (gNatTravGlbInfo.pool) {
				pj_pool_release(gNatTravGlbInfo.pool);
				gNatTravGlbInfo.pool = NULL;

				pj_caching_pool_destroy(&gNatTravGlbInfo.cp);

				pj_shutdown();
		}
		gNatTravGlbInfo.libinitflag = 0;
		gNatTravGlbInfo.IceInstCnt = 0;
#if 0
		pj_caching_pool_destroy(&gNatTravGlbInfo.cp);

		pj_shutdown();
#endif
		if (gNatTravGlbInfo.log_fhnd) {
				fclose(gNatTravGlbInfo.log_fhnd);
				gNatTravGlbInfo.log_fhnd = NULL;
		}

}

int nattrav_ice_destroy(void *hndl, nattrav_cand_offer_Info *psLocalInfo)
{
		nattrav_rest_field_Info candInfo;
		sIceInstanceInfo *psInstInfo = NULL;
		if (hndl) {
				psInstInfo = &gNatTravList[((int)hndl)-1];
		}else {
				hndl = (void*)gNatTravGlbInfo.IceInstCnt;
				psInstInfo = &gNatTravList[((int)hndl)-1];
		}

		if(!pj_thread_is_registered())
		{
				pj_thread_desc nat_ice_destroy_thread_desc;
				pj_thread_t         *nat_ice_destroy_thread;

				APP_LOG("REMOTEACCESS", LOG_DEBUG," Registering nattrav_ice_destroy thread to PJ\n");
				pj_thread_register("nattrav_ice_destroy", nat_ice_destroy_thread_desc, &nat_ice_destroy_thread);
		}
		APP_LOG("REMOTEACCESS", LOG_DEBUG," nattrav_ice_destroy thread already registerd to  PJ\n");
		if(psLocalInfo)
		{
				strcpy(candInfo.peer_id, psLocalInfo->peer_id);
				delPeerLocation(&candInfo,&psInstInfo->sNatLibConfInfo);
		}
		gIceRunning=0;
		nattrav_destroy_instance(hndl);
		ice_shutdown("normal re-init of ICE", -1);

		return SUCCESS;
}

int nattrav_start_session(void *hndl,nattrav_cand_offer_Info *psPeerInfo)
{
		sIceInstanceInfo *psInstInfo = &gNatTravList[((int)hndl)-1];
		pj_status_t status;

		if(psPeerInfo->ice_type == NAT_TRAV_ICE_TYPE_FULL)
		{
				pj_str_t rufrag, rpwd;

				/* start ice negotiation*/
				status = pj_ice_strans_start_ice(psInstInfo->icest, 
								pj_cstr(&rufrag,psPeerInfo->ice_info.ufrag),
								pj_cstr(&rpwd,psPeerInfo->ice_info.pwd),
								psPeerInfo->ice_info.cand_cnt,
								psPeerInfo->ice_info.cand);

				if (status != PJ_SUCCESS)
				{
						nattrav_perror("Error starting ICE", status);
						return PLUGIN_FAILURE;
				}
				else
						PJ_LOG(3,(THIS_FILE, "candidates info exchanged started"));
                                
                                int retry_count_nat=0;
				while(1)
				{
					APP_LOG("REMOTEACESS", LOG_DEBUG,"\n>>Inside ice init>>>\n");
					if(DEVICE_STATE_CONNECTED != gWiFiClientDeviceCurrState)
					{
						if(MAX_ICE_INIT_COUNT <= retry_count_nat)
						{
							APP_LOG("REMOTEACCESS", LOG_ERR,"\n>>ICE init fail count=%d>>>\n",retry_count_nat);
							return PLUGIN_FAILURE;
						}
						retry_count_nat++;
					}
					else
					{
						pthread_mutex_lock(&g_NatTrav_lock);
						if(psInstInfo->NatTravFlag == NAT_TRAV_NEGOTIATION_COMPLETE)
						{
							pthread_mutex_unlock(&g_NatTrav_lock);
							psInstInfo->NatTravFlag = 0;
							break;
						}
						pthread_mutex_unlock(&g_NatTrav_lock);
					}
					sleep(1);
				}
				{
						const pj_ice_sess_check *valid_pair;
						char ipaddr[PJ_INET6_ADDRSTRLEN];

						//check if there is any need to return the selected pair
						valid_pair = pj_ice_strans_get_valid_pair(psInstInfo->icest,NAT_TRAV_MAX_COMP_CNT);
						if(valid_pair)
						{            
								PJ_LOG(3,(THIS_FILE, "candidates info exchanged successfully\n"));
								PJ_LOG(3,(THIS_FILE, "nominated local cand%s: %d\n", 
														pj_sockaddr_print(&valid_pair->lcand->addr, ipaddr,sizeof(ipaddr), 0),
														(unsigned)pj_sockaddr_get_port(&valid_pair->lcand->addr)));

								PJ_LOG(3,(THIS_FILE, "nominated remote cand%s: %d\n", 
														pj_sockaddr_print(&valid_pair->rcand->addr, ipaddr,sizeof(ipaddr), 0),
														(unsigned)pj_sockaddr_get_port(&valid_pair->rcand->addr)));
								int retry_count=0;
								while(retry_count < MAX_DLR_RETRY_COUNT)
								{
										retry_count++;
										if((postIceDataDlr((pj_ice_sess_check*)valid_pair)) < 0 )
										{
												if(retry_count < MAX_DLR_RETRY_COUNT)
														continue;
												else
												{
														APP_LOG("REMOTEACCESS", LOG_CRIT,"DLR Failure ICE %d\n",retry_count);
														return PLUGIN_FAILURE;
												}
										}	
										else
										{
												break;
										}
								}		 
						} 
						else
						{
								APP_LOG("REMOTEACCESS", LOG_CRIT,"some error in candidates info exchange, no valid pair\n");
								return PLUGIN_FAILURE;
						}

				}
		}
		else if(psPeerInfo->ice_type == NAT_TRAV_ICE_TYPE_NONE)
		{
				pj_sockaddr  peer_def_addr;

				nattrav_convert_addr_to_pjformat(psPeerInfo->def_addr_info.def_ipaddr,(pj_uint16_t)psPeerInfo->def_addr_info.def_port,&peer_def_addr);

				pj_ice_strans_create_turn_perm_for_non_ice_peer(psInstInfo->icest,peer_def_addr);
				pj_ice_strans_set_data_port_for_turn_tcp(psInstInfo->icest,psInstInfo->sNatLibConfInfo.turn_tcp_data_port);
				/* to wait for turn response*/
				usleep(10000);
				PJ_LOG(3,(THIS_FILE, "after creating turn permission for non-ice peer\n"));

		}
		return PLUGIN_SUCCESS;
}

int nattrav_send_data(void *hndl, const char *data, int data_len,nattrav_cand_offer_Info *psRemotePeerInfo)
{
		sIceInstanceInfo *psInstInfo = &gNatTravList[((int)hndl)-1];
		pj_status_t status;
		pj_sockaddr  peer_def_addr;

		if (psInstInfo->icest == NULL) 
		{
				PJ_LOG(1,(THIS_FILE, "Error: No ICE instance, create it first"));
				return FAIL;
		}

		if((psRemotePeerInfo->ice_type != NAT_TRAV_ICE_TYPE_NONE) && (!pj_ice_strans_has_sess(psInstInfo->icest))) 
		{
				PJ_LOG(1,(THIS_FILE, "Error: No ICE session, initialize first"));
				return FAIL;
		}

		nattrav_convert_addr_to_pjformat(psRemotePeerInfo->def_addr_info.def_ipaddr,
						(pj_uint16_t)psRemotePeerInfo->def_addr_info.def_port,
						&peer_def_addr);



		status = pj_ice_strans_sendto(psInstInfo->icest, NAT_TRAV_MAX_COMP_CNT, data, data_len,
						&peer_def_addr,
						pj_sockaddr_get_len(&peer_def_addr));

		if (status != PJ_SUCCESS)
		{
				nattrav_perror("Error sending data", status);
				return FAIL;
		}
		else
		{
				PJ_LOG(3,(THIS_FILE, "Data sent"));
				return SUCCESS;
		}
}


/*
 * This is the callback that is registered to the ICE stream transport to
 * receive notification about incoming data. By "data" it means application
 * data such as RTP/RTCP, and not packets that belong to ICE signaling (such
 * as STUN connectivity checks or TURN signaling).
 */
void cb_on_rx_data(pj_ice_strans *ice_st,
				unsigned comp_id, 
				void *pkt, pj_size_t size,
				const pj_sockaddr_t *src_addr,
				unsigned src_addr_len)
{
		char ipstr[PJ_INET6_ADDRSTRLEN+10];

		PJ_UNUSED_ARG(ice_st);
		PJ_UNUSED_ARG(src_addr_len);
		PJ_UNUSED_ARG(pkt);

		if(!src_addr)
		{
				PJ_LOG(3,(THIS_FILE, "Component %d: received %d bytes data from", comp_id, size));
		}
		else
				PJ_LOG(3,(THIS_FILE, "Component %d: received %d bytes data from %s: ",
										comp_id, size,
										pj_sockaddr_print(src_addr, ipstr, sizeof(ipstr), 3)));
		{
				int i;
				sIceInstanceInfo *psInstInfo = NULL;
				for(i = 0; i < NAT_TRAV_MAX_NAT_TRAV_LIB_INSTNC; i++)
				{
						if(gNatTravList[i].icest == ice_st)
						{
								psInstInfo = &gNatTravList[i];		          
								break;
						}
				}

				if(psInstInfo == NULL)
				{
						PJ_LOG(1,(THIS_FILE, "no matching ice instance found in callback on ice complete"));
						return;
				}
				else {	
						(psInstInfo->sNatLibConfInfo.appl_cb_on_rx_data)(((void *)i+1),pkt,size,ipstr);
						PJ_LOG(1,(THIS_FILE, "Reached after callback\n"));
				}
		}

}

/*
 * This is the callback that is registered to the ICE stream transport to
 * receive notification about ICE state progression.
 */
void cb_on_ice_complete(pj_ice_strans *ice_st, pj_ice_strans_op op,pj_status_t status)
{

		const char *opname = 
				(op==PJ_ICE_STRANS_OP_INIT? "initialization" :
				 (op==PJ_ICE_STRANS_OP_NEGOTIATION ? "negotiation" : "unknown_op"));

		if (status == PJ_SUCCESS) {
				PJ_LOG(3,(THIS_FILE, "ICE %s successful", opname));
		} else {
				char errmsg[PJ_ERR_MSG_SIZE];

				pj_strerror(status, errmsg, sizeof(errmsg));
				PJ_LOG(1,(THIS_FILE, "ICE %s failed: %s", opname, errmsg));
		}

		{
				int i;
				sIceInstanceInfo *psInstInfo = NULL;

				for(i = 0; i < NAT_TRAV_MAX_NAT_TRAV_LIB_INSTNC; i++)
				{
						if(gNatTravList[i].icest == ice_st)
						{
								psInstInfo = &gNatTravList[i] ;
								break;
						}
				}

				if(psInstInfo == NULL)
				{
						PJ_LOG(1,(THIS_FILE, "no matching ice instance found in callback on ice complete"));
						return;
				}

				if(op==PJ_ICE_STRANS_OP_INIT)
				{
						pthread_mutex_lock(&g_NatTrav_lock);
						psInstInfo->NatTravFlag = NAT_TRAV_CANDIDATES_GATERED;
						pthread_mutex_unlock(&g_NatTrav_lock);
				}
				else if(op==PJ_ICE_STRANS_OP_NEGOTIATION)
				{
						pthread_mutex_lock(&g_NatTrav_lock);
						psInstInfo->NatTravFlag = NAT_TRAV_NEGOTIATION_COMPLETE;
						pthread_mutex_unlock(&g_NatTrav_lock);
				}
		}
}




/*
 * Create ICE stream transport instance
 */
int nattrav_create_ice_instance(sIceInstanceInfo *psInstInfo)
{
		pj_ice_strans_cb icecb;
		pj_status_t status;

		if (psInstInfo->icest != NULL) {
				puts("ICE instance already created, destroy it first");
				return PLUGIN_FAILURE;
		}

		/* init the callback */
		pj_bzero(&icecb, sizeof(icecb));
		icecb.on_rx_data = cb_on_rx_data;
		icecb.on_ice_complete = cb_on_ice_complete;

		/* create the instance */
		status = pj_ice_strans_create("gNatTravGlbInfo",		    /* object name  */
						&gNatTravGlbInfo.ice_cfg,	    /* settings	    */
						NAT_TRAV_MAX_COMP_CNT,	    /* comp_cnt	    */
						NULL,			    /* user data    */
						&icecb,			    /* callback	    */
						&psInstInfo->icest)		    /* instance ptr */
				;
		if (status != PJ_SUCCESS) {
				nattrav_perror("error creating ice", status);
				return PLUGIN_FAILURE;
		} else
				PJ_LOG(3,(THIS_FILE, "ICE instance successfully created"));

		return PLUGIN_SUCCESS;
}



/*
 * Create ICE session, 
 */
int nattrav_init_ice_session(unsigned short ice_role,sIceInstanceInfo *psInstInfo )
{
		pj_ice_sess_role role = ice_role  == NAT_TRAV_ICE_ROLE_CONTROLLING? 
				PJ_ICE_SESS_ROLE_CONTROLLING : 
				PJ_ICE_SESS_ROLE_CONTROLLED;
		pj_status_t status;

		if (psInstInfo->icest == NULL) {
				PJ_LOG(1,(THIS_FILE, "Error: No ICE instance, create it first"));
				return PLUGIN_FAILURE;
		}

		if (pj_ice_strans_has_sess(psInstInfo->icest)) {
				PJ_LOG(1,(THIS_FILE, "Error: Session already created"));
				return PLUGIN_FAILURE;
		}

		status = pj_ice_strans_init_ice(psInstInfo->icest, role, NULL, NULL);
		if (status != PJ_SUCCESS) {
				nattrav_perror("error creating session", status);
				return PLUGIN_FAILURE;
		} else
				PJ_LOG(3,(THIS_FILE, "ICE session created"));

		return PLUGIN_SUCCESS;
}

/* Destroy ICE stream transport instance
 */
void nattrav_destroy_instance(void *hndl)
{
		sIceInstanceInfo *psInstInfo = &gNatTravList[((int)hndl)-1];
		pj_status_t status;

		if (psInstInfo->icest == NULL) 
		{
				PJ_LOG(1,(THIS_FILE, "Error: No ICE instance, create it first"));
				return;
		}

		if (!pj_ice_strans_has_sess(psInstInfo->icest)) 
		{
				PJ_LOG(1,(THIS_FILE, "Error: No ICE session, initialize first"));
				return;
		}

		status = pj_ice_strans_stop_ice(psInstInfo->icest);
		if (status != PJ_SUCCESS)
		{
				nattrav_perror("error stopping session", status);
				return ;
		}
		else
				PJ_LOG(3,(THIS_FILE, "ICE session stopped"));

		pj_ice_strans_destroy(psInstInfo->icest);
		memset(psInstInfo,0,sizeof(sIceInstanceInfo));
		gNatTravGlbInfo.IceInstCnt--;

		PJ_LOG(3,(THIS_FILE, "ICE instance destroyed"));
}

#define PRINT(fmt, arg0, arg1, arg2, arg3, arg4, arg5)	    \
		printed = pj_ansi_snprintf(p, maxlen - (p-buffer),  \
						fmt, arg0, arg1, arg2, arg3, arg4, arg5); \
if (printed <= 0) return -PJ_ETOOSMALL; \
p += printed

void nattrav_get_local_ice_cand_info(sIceInstanceInfo *psInstInfo, nattrav_rest_field_Info *psLocalCandInfo)
{
		pj_ice_sess_cand def_cand;  
		pj_str_t lufrag, lpwd;
		char ipaddr[PJ_INET6_ADDRSTRLEN]; 
		unsigned int cand_cnt = -1,j;
		pj_ice_sess_cand cand[PJ_ICE_ST_MAX_CAND];
		pj_status_t status;

		/* Get ufrag and pwd from current session */
		pj_ice_strans_get_ufrag_pwd(psInstInfo->icest, &lufrag,&lpwd, NULL, NULL);


		/* copy in lib format*/
		pj_ansi_snprintf(psLocalCandInfo->ice_info.username,10,"%.*s",(int)lufrag.slen,lufrag.ptr);
		pj_ansi_snprintf(psLocalCandInfo->ice_info.passwd,10,"%.*s",(int)lpwd.slen,lpwd.ptr);
		status = pj_ice_strans_get_def_cand(psInstInfo->icest,1,&def_cand);
		if (status != PJ_SUCCESS)
		{
				PJ_LOG(1,(THIS_FILE, "get_def_cand failed"));
				return ;	   
		}
		if((def_cand.type == PJ_ICE_CAND_TYPE_RELAYED) && (psInstInfo->sNatLibConfInfo.transport == NAT_TRAV_USE_TRANSPORT_TCP))
				psLocalCandInfo->def_addr_info.def_transport = 1;
		else
				psLocalCandInfo->def_addr_info.def_transport = 0;

		/* copy in lib format*/
		pj_sockaddr_print(&def_cand.addr, ipaddr, sizeof(ipaddr), 0);
		strcpy(psLocalCandInfo->def_addr_info.def_ipaddr,ipaddr);
		psLocalCandInfo->def_addr_info.def_port = (unsigned)pj_sockaddr_get_port(&def_cand.addr);
		psLocalCandInfo->def_addr_info.ip_family = NAT_TRAV_FAMILY_IPV4;

		/* Enumerate all candidates for this component */
		status = pj_ice_strans_enum_cands(psInstInfo->icest, 1, &cand_cnt, cand);

		if (status != PJ_SUCCESS)
		{
				PJ_LOG(1,(THIS_FILE, "enum_cand failed"));
				return ;	   
		}
		psLocalCandInfo->ice_info.cand_cnt = cand_cnt;
		PJ_LOG(3,(THIS_FILE, "local cand count%d:\n",cand_cnt));

		for (j=0; j<cand_cnt; ++j) 
		{
				memcpy(psLocalCandInfo->ice_info.candidate[j].foundation, cand[j].foundation.ptr,cand[j].foundation.slen);
				psLocalCandInfo->ice_info.candidate[j].priority = cand[j].prio;
				psLocalCandInfo->ice_info.candidate[j].compid = 1;
				psLocalCandInfo->ice_info.candidate[j].cand_type = cand[j].type;

				if((cand[j].type == PJ_ICE_CAND_TYPE_RELAYED) && (psInstInfo->sNatLibConfInfo.transport == NAT_TRAV_USE_TRANSPORT_TCP))
						psLocalCandInfo->ice_info.candidate[j].transport = 1;
				else
						psLocalCandInfo->ice_info.candidate[j].transport = 0;

				psLocalCandInfo->ice_info.candidate[j].port  = (unsigned)pj_sockaddr_get_port(&cand[j].addr);
				pj_sockaddr_print(&cand[j].addr, ipaddr, sizeof(ipaddr), 0);
				strcpy(psLocalCandInfo->ice_info.candidate[j].ipaddr,ipaddr);
				PJ_LOG(3,(THIS_FILE, "local cand %d ip :%s, port:%d\n",j,psLocalCandInfo->ice_info.candidate[j].ipaddr,
										psLocalCandInfo->ice_info.candidate[j].port));
		}

}


int  nattrav_convert_addr_to_pjformat(char *ipaddr,unsigned int port,pj_sockaddr *pj_addr)
{
		int addr_family;     
		pj_str_t tmp_addr;
		pj_status_t status;
		char ip6_str[64];

		addr_family = pj_AF_INET();

		pj_sockaddr_init(addr_family, pj_addr, NULL, 0);
		strcpy(ip6_str,ipaddr);
		tmp_addr = pj_str(ip6_str);
		status = pj_sockaddr_set_str_addr(addr_family, pj_addr,&tmp_addr);
		if (status != PJ_SUCCESS) 
		{
				PJ_LOG(1,(THIS_FILE, "Invalid IP address"));
				return FAIL;	   
		}

		pj_sockaddr_set_port(pj_addr,(pj_uint16_t) port);

		return SUCCESS;

}

int nattrav_send_authinfo_turnserver(ice_config_info *psConfigInfo)
{
		sAuthInfo auth_info;        
		struct sockaddr_in server_addr;
		int resp_code;
		int sock;              


		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
		{
				PJ_LOG(1,(THIS_FILE, "Socket() fail"));
				return FAIL;
		}

		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(psConfigInfo->turn_auth_port);
		server_addr.sin_addr.s_addr = inet_addr(psConfigInfo->turn_server_ip);
		APP_LOG("REMOTEACCESS", LOG_DEBUG," auth port=%d, tS ip=%s\n", server_addr.sin_port, psConfigInfo->turn_server_ip);
		bzero(&(server_addr.sin_zero),8);

		if (connect(sock, (struct sockaddr *)&server_addr,
								sizeof(struct sockaddr)) == -1)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"connect fial\n");
				PJ_LOG(1,(THIS_FILE, "connect() fail"));
				return FAIL;
		}

		strcpy(auth_info.mac_addr, psConfigInfo->mac_addr);
		strcpy(auth_info.serial_no, psConfigInfo->serial_num);
		strcpy(auth_info.passwd, psConfigInfo->signaure);
		if(psConfigInfo->method_id == NATTRAV_AUTH_METHOD_MAC_BASED)
				strcpy(auth_info.login, psConfigInfo->mac_addr);
		else
				strcpy(auth_info.login, psConfigInfo->user_name);
		strcpy(auth_info.realm, psConfigInfo->realm);
		auth_info.expires = psConfigInfo->signaure_expires;
		auth_info.method_id = psConfigInfo->method_id;


		send(sock,(char *)&auth_info,sizeof(sAuthInfo), 0);
		PJ_LOG(3,(THIS_FILE, "Sent autentication info to Turn server"));

		{
				sAuthResult  *pauth_rslt;
				int bytes_recieved;
				char recv_buff[512];        	

				bytes_recieved=recv(sock,recv_buff,sizeof(recv_buff),0);        
				pauth_rslt  = (sAuthResult *)recv_buff;
				resp_code   = pauth_rslt->response_code;
				PJ_LOG(3,(THIS_FILE, "Response received from Turn server:%x",resp_code));
		}

		close(sock);

		if(resp_code != NAT_TRAV_AUTH_SUCCESS)
				return FAIL;
		else
				return SUCCESS;

}

/*
 * This is the main application initialization function. It is called
 * once (and only once) during application initialization sequence by 
 * main().
 */
int nattrav_lib_init(ice_config_info *psConfigInfo)
{
		pj_status_t status;

		gNatTravGlbInfo.log_fhnd =NULL;/* fopen(NAT_TRAV_LIB_LOG_FILE, "a");*/
		pj_log_set_log_func(&log_func);



		/* Initialize the libraries before anything else */
		CHECK( pj_init() );
		CHECK( pjlib_util_init() );
		CHECK( pjnath_init() );

		/* Must create pool factory, where memory allocations come from */
		pj_caching_pool_init(&gNatTravGlbInfo.cp, NULL, 0);

		/* Init our ICE settings with null values */
		pj_ice_strans_cfg_default(&gNatTravGlbInfo.ice_cfg);

		gNatTravGlbInfo.ice_cfg.stun_cfg.pf = &gNatTravGlbInfo.cp.factory;

		/* Create application memory pool */
		gNatTravGlbInfo.pool = pj_pool_create(&gNatTravGlbInfo.cp.factory, "NatTravLib", 
						512, 512, NULL);

		/* Create timer heap for timer stuff */
		CHECK( pj_timer_heap_create(gNatTravGlbInfo.pool, 100, 
								&gNatTravGlbInfo.ice_cfg.stun_cfg.timer_heap) );

		/* and create ioqueue for network I/O stuff */
		CHECK( pj_ioqueue_create(gNatTravGlbInfo.pool, 16, 
								&gNatTravGlbInfo.ice_cfg.stun_cfg.ioqueue) );

		/* something must poll the timer heap and ioqueue, 
		 * unless we're on Symbian where the timer heap and ioqueue run
		 * on themselves.
		 */
		CHECK( pj_thread_create(gNatTravGlbInfo.pool, "NatTravLib", &nattrav_worker_thread,
								NULL, 0, 0, &gNatTravGlbInfo.thread) );

		gNatTravGlbInfo.ice_cfg.af = pj_AF_INET();


		/* -= Start initializing ICE stream transport config =- */

		/* Maximum number of host candidates */

		gNatTravGlbInfo.ice_cfg.stun.max_host_cands = NAT_TRAV_MAX_HOST_CAND;

		/* Nomination strategy */
		gNatTravGlbInfo.ice_cfg.opt.aggressive = PJ_FALSE;

		/* Configure STUN/srflx candidate resolution */

		gNatTravGlbInfo.ice_cfg.stun.server = pj_str(psConfigInfo->turn_server_ip);
		gNatTravGlbInfo.ice_cfg.stun.port = (pj_uint16_t)(psConfigInfo->turn_srvr_port);
		gNatTravGlbInfo.ice_cfg.stun.cfg.ka_interval = KA_INTERVAL;
#if !defined(PRODUCT_WeMo_Baby)
		gNatTravGlbInfo.ice_cfg.stun.cfg.timerstat = 1;   // timerstat = 1 => dont start stun binding request keepalive 5 mins - nish
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"cfg.timerstat=%d\n", gNatTravGlbInfo.ice_cfg.stun.cfg.timerstat);	
#endif
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"TSIP=%s port=%d\n", (psConfigInfo->turn_server_ip), (pj_uint16_t)(psConfigInfo->turn_srvr_port));	
		/* Configure TURN candidate */ /* remove turn candidate so as to avoid allocation */

		/* TURN credential */
		gNatTravGlbInfo.ice_cfg.turn.auth_cred.type = PJ_STUN_AUTH_CRED_STATIC;
		gNatTravGlbInfo.ice_cfg.turn.auth_cred.data.static_cred.username = pj_str(psConfigInfo->mac_addr);
		gNatTravGlbInfo.ice_cfg.turn.auth_cred.data.static_cred.data_type = PJ_STUN_PASSWD_PLAIN;
		gNatTravGlbInfo.ice_cfg.turn.auth_cred.data.static_cred.data = pj_str(psConfigInfo->signaure);

		/* Connection type to TURN server */
		if (psConfigInfo->transport == NAT_TRAV_USE_TRANSPORT_TCP)
				gNatTravGlbInfo.ice_cfg.turn.conn_type = PJ_TURN_TP_TCP;
		else
				gNatTravGlbInfo.ice_cfg.turn.conn_type = PJ_TURN_TP_UDP;

		gNatTravGlbInfo.ice_cfg.turn.alloc_param.ka_interval = KA_INTERVAL;

		/* -= That's it for now, initialization is complete =- */
		return SUCCESS;
}




/* Utility to display error messages */
void nattrav_perror(const char *title, pj_status_t status)
{
		char errmsg[PJ_ERR_MSG_SIZE];

		pj_strerror(status, errmsg, sizeof(errmsg));
		PJ_LOG(1,(THIS_FILE, "%s: %s", title, errmsg));
}


/* log callback to write to file */
void log_func(int level, const char *data, int len)
{
		pj_log_write(level, data, len);
		if (gNatTravGlbInfo.log_fhnd) {
				if (fwrite(data, len, 1, gNatTravGlbInfo.log_fhnd) != 1)
						return;
		}
}

/* Utility: display error message and exit application (usually
 * because of fatal error.
 */
void err_exit(const char *title, pj_status_t status)
{
		if (status != PJ_SUCCESS) {
				nattrav_perror(title, status);
		}
		PJ_LOG(3,(THIS_FILE, "Shutting down.."));
		{
				sIceInstanceInfo *psInstInfo = &gNatTravList[gNatTravGlbInfo.IceInstCnt];
				if (psInstInfo->icest)
						pj_ice_strans_destroy(psInstInfo->icest);
		}

		pj_thread_sleep(500);

		gNatTravGlbInfo.thread_quit_flag = PJ_TRUE;
		if (gNatTravGlbInfo.thread) {
				pj_thread_join(gNatTravGlbInfo.thread);
				pj_thread_destroy(gNatTravGlbInfo.thread);
		}

		if (gNatTravGlbInfo.ice_cfg.stun_cfg.ioqueue)
				pj_ioqueue_destroy(gNatTravGlbInfo.ice_cfg.stun_cfg.ioqueue);

		if (gNatTravGlbInfo.ice_cfg.stun_cfg.timer_heap)
				pj_timer_heap_destroy(gNatTravGlbInfo.ice_cfg.stun_cfg.timer_heap);

		pj_caching_pool_destroy(&gNatTravGlbInfo.cp);

		pj_shutdown();

		if (gNatTravGlbInfo.log_fhnd) {
				fclose(gNatTravGlbInfo.log_fhnd);
				gNatTravGlbInfo.log_fhnd = NULL;
		}

		exit(status != PJ_SUCCESS);
}


/*
 * This function checks for events from both timer and ioqueue (for
 * network events). It is invoked by the worker thread.
 */
static pj_status_t handle_events(unsigned max_msec, unsigned *p_count)
{
		enum { MAX_NET_EVENTS = 1 };
		pj_time_val max_timeout = {0, 0};
		pj_time_val timeout = { 0, 0};
		unsigned count = 0, net_event_count = 0;
		int c;

		max_timeout.msec = max_msec;

		/* Poll the timer to run it and also to retrieve the earliest entry. */
		timeout.sec = timeout.msec = 0;
		c = pj_timer_heap_poll( gNatTravGlbInfo.ice_cfg.stun_cfg.timer_heap, &timeout );
		if (c > 0)
				count += c;

		/* timer_heap_poll should never ever returns negative value, or otherwise
		 * ioqueue_poll() will block forever!
		 */
		pj_assert(timeout.sec >= 0 && timeout.msec >= 0);
		if (timeout.msec >= 1000) timeout.msec = 999;

		/* compare the value with the timeout to wait from timer, and use the 
		 * minimum value. 
		 */
		if (PJ_TIME_VAL_GT(timeout, max_timeout))
				timeout = max_timeout;

		/* Poll ioqueue. 
		 * Repeat polling the ioqueue while we have immediate events, because
		 * timer heap may process more than one events, so if we only process
		 * one network events at a time (such as when IOCP backend is used),
		 * the ioqueue may have trouble keeping up with the request rate.
		 *
		 * For example, for each send() request, one network event will be
		 *   reported by ioqueue for the send() completion. If we don't poll
		 *   the ioqueue often enough, the send() completion will not be
		 *   reported in timely manner.
		 */
		do {
				c = pj_ioqueue_poll( gNatTravGlbInfo.ice_cfg.stun_cfg.ioqueue, &timeout);
				if (c < 0) {
						pj_status_t err = pj_get_netos_error();
						pj_thread_sleep(PJ_TIME_VAL_MSEC(timeout));
						if (p_count)
								*p_count = count;
						return err;
				} else if (c == 0) {
						break;
				} else {
						net_event_count += c;
						timeout.sec = timeout.msec = 0;
				}
		} while (c > 0 && net_event_count < MAX_NET_EVENTS);

		count += net_event_count;
		if (p_count)
				*p_count = count;

		return PJ_SUCCESS;

}

/*
 * This is the worker thread that polls event in the background.
 */
int nattrav_worker_thread(void *unused)
{
		PJ_UNUSED_ARG(unused);

		while (!gNatTravGlbInfo.thread_quit_flag) {
				handle_events(500, NULL);
		}

		return 0;
}

/* this is temporary till the time there is some EMS api to get device key*/
int get_device_auth_key(char *mac_addr, char *auth_key)
{
		strcpy(auth_key,g_auth_key);
		return 0;
}

int  nattrav_get_icelite_remote_address_info(void *hndl, nattrav_cand_offer_Info *psRemotePeerInfo)
{
		int status;
		nattrav_rest_field_Info peerInfo;
		authSign *assign = NULL;
		int retVal = NAT_SUCCESS;
		memset(&peerInfo, 0x0, sizeof(nattrav_rest_field_Info));
		strcpy(peerInfo.peer_id,psRemotePeerInfo->peer_id);

		status = SUCCESS;
		if(status == SUCCESS)
		{
				int i;

				APP_LOG("REMOTEACCESS", LOG_HIDE,"\n mac: %s, ser: %s, key%s\n", gConfigInfo.mac_addr, gConfigInfo.serial_num, gConfigInfo.auth_key);
				assign = createAuthSignatureNoExp(gConfigInfo.mac_addr,gConfigInfo.serial_num,gConfigInfo.auth_key);
				if (!assign) {
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Signature Structure returned NULL\n");
						retVal = AUTH_SIGN_FAIL;		
						return PLUGIN_FAILURE;
				}

				strcpy(psRemotePeerInfo->peer_id, peerInfo.peer_id);
				psRemotePeerInfo->ice_type = 2;
				psRemotePeerInfo->def_addr_info.def_port =atoi(gIcePeerPort);
				strcpy(psRemotePeerInfo->ice_info.ufrag, gWiFiMacAddress);
				APP_LOG("REMOTEACCESS", LOG_HIDE,"\n NC: username : %s\n", psRemotePeerInfo->ice_info.ufrag);
				if(assign)
				{
						strcpy(psRemotePeerInfo->ice_info.pwd, assign->ssign);
						APP_LOG("REMOTEACCESS", LOG_HIDE,"\n cc assign->ssign: %s\n", assign->ssign);
				}
				else
				{
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n assign NULL cannot proceed\n");
						return PLUGIN_FAILURE;
				}
				psRemotePeerInfo->ice_info.cand_cnt = 1;
				peerInfo.ice_info.cand_cnt = 2;

				for(i =0; i< peerInfo.ice_info.cand_cnt;i++)
				{
						pj_ice_sess_cand *cand;

						cand = &psRemotePeerInfo->ice_info.cand[i];
						cand->comp_id = 1;
						cand->type = 0;
						cand->prio = 2130706431;
						strcpy(peerInfo.ice_info.candidate[i].ipaddr, gIcePeerIp);
						PJ_LOG(3,(THIS_FILE, "ICE: remote cand %s:\n",peerInfo.ice_info.candidate[i].ipaddr ));
						peerInfo.ice_info.candidate[i].port = atoi(gIcePeerPort);
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"candidate port is %d\n",peerInfo.ice_info.candidate[i].port);
						strcpy(peerInfo.ice_info.candidate[i].foundation, "Hac351e1");

						nattrav_convert_addr_to_pjformat(peerInfo.ice_info.candidate[i].ipaddr,
										(pj_uint16_t)peerInfo.ice_info.candidate[i].port,&cand->addr);

						PJ_LOG(3,(THIS_FILE, "remote cand %d: %s: %d\n",
												i+1,peerInfo.ice_info.candidate[i].ipaddr,peerInfo.ice_info.candidate[i].port));

						pj_strdup2(gNatTravGlbInfo.pool, &cand->foundation, peerInfo.ice_info.candidate[i].foundation);
				}

		}

		if (assign) {
			free(assign);
			assign = NULL;
		}

#if DLR_SAMPLE
		int sockfd = -1;
		struct sockaddr_in server_addr;



		sockfd  = socket(AF_INET, SOCK_STREAM, 0);
		if(sockfd < 0)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"socket() fail\n");
				return -1;
		}
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(DLR_PORT);
		server_addr.sin_addr.s_addr = inet_addr(DLR_IP);
		bzero(&(server_addr.sin_zero),8);

		if (connect(sockfd, (struct sockaddr *)&server_addr,
								sizeof(struct sockaddr)) == -1)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Connect() to DLR fail");
				close(sockfd);
				return -1;
		}

		{
				sOpGetInfo sOpInfo;
				sOpInfo.op = DLR_OP_GET;
				strcpy(sOpInfo.peer_id, psRemotePeerInfo->peer_id);
				send(sockfd,&sOpInfo,sizeof(sOpInfo), 0);
		}

		{
				int bytes_recieved = -1;
				char client_buf[1024];
				bytes_recieved=recv(sockfd,client_buf,1024,0);
				if(bytes_recieved > 0)

				{
						nattrav_rest_field_Info *psPeerInfo;
						int i;

						psPeerInfo =   (nattrav_rest_field_Info*)client_buf;

						strcpy(psRemotePeerInfo->peer_id, psPeerInfo->peer_id);
						psRemotePeerInfo->ice_type = psPeerInfo->ice_type;
						memcpy(&psRemotePeerInfo->def_addr_info, &psPeerInfo->def_addr_info, sizeof(nattrav_def_addr_info));

						strcpy(psRemotePeerInfo->ice_info.ufrag, psPeerInfo->ice_info.username);
						strcpy(psRemotePeerInfo->ice_info.pwd, psPeerInfo->ice_info.passwd);
						psRemotePeerInfo->ice_info.cand_cnt = psPeerInfo->ice_info.cand_cnt;

						for(i =0; i< psPeerInfo->ice_info.cand_cnt;i++)
						{
								pj_ice_sess_cand *cand;

								cand = &psRemotePeerInfo->ice_info.cand[i];
								cand->comp_id = psPeerInfo->ice_info.candidate[i].compid;
								cand->type = psPeerInfo->ice_info.candidate[i].cand_type;
								cand->prio = psPeerInfo->ice_info.candidate[i].priority;

								nattrav_convert_addr_to_pjformat(psPeerInfo->ice_info.candidate[i].ipaddr,
												(pj_uint16_t)psPeerInfo->ice_info.candidate[i].port,&cand->addr);

								PJ_LOG(3,(THIS_FILE, "remote cand %d: %s: %d\n",
														i+1,psPeerInfo->ice_info.candidate[i].ipaddr,psPeerInfo->ice_info.candidate[i].port));

								pj_strdup2(gNatTravGlbInfo.pool, &cand->foundation, psPeerInfo->ice_info.candidate[i].foundation);
						}

				}

				close(sockfd);
		}
#endif

		return status;
}

int postIceDataDlr(pj_ice_sess_check *valid_pair)
{
		UserAppSessionData *pUsrAppSsnData = NULL;
		UserAppData *pUsrAppData = NULL;
		char httpRelayData[1024];
		int retVal = NAT_SUCCESS;
		int ip_family = 4;
		authSign *assign = NULL;
		char ipaddr[PJ_INET6_ADDRSTRLEN];

		APP_LOG("REMOTEACCESS", LOG_HIDE,"Auth key en=%s mac_addr:%s serial=%s\n",gConfigInfo.auth_key, gConfigInfo.mac_addr, gConfigInfo.serial_num);

		snprintf(httpRelayData,sizeof(httpRelayData), "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?><peerAddresses><iceType>1</iceType><candType>1</candType>"
						"<defaultAddress>%s</defaultAddress><defaultPort>%d</defaultPort><defaultTransport>%d</defaultTransport>"
						"<ipFamily>%d</ipFamily><publicIcePeerAddress>%s</publicIcePeerAddress><publicIcePeerPort>%s</publicIcePeerPort></peerAddresses>",
						pj_sockaddr_print(&valid_pair->lcand->addr, ipaddr,sizeof(ipaddr), 0),(unsigned)pj_sockaddr_get_port(&valid_pair->lcand->addr),0,ip_family,gIcePeerInternalIp,gIcePeerPort);



		assign = createAuthSignature(gConfigInfo.mac_addr,gConfigInfo.serial_num,gConfigInfo.auth_key);
		if (!assign) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Signature Structure returned NULL\n");
				retVal = AUTH_SIGN_FAIL;		
				goto on_return;
		}



		APP_LOG("REMOTEACCESS", LOG_HIDE,"httpRelayData=%s len=%d\n",httpRelayData, strlen(httpRelayData));

		pUsrAppData = (UserAppData *)malloc(sizeof(UserAppData));
		if (!pUsrAppData) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Malloc Failed\n");
				retVal = PLUGIN_FAILURE;
				goto on_return;
		}
		memset( pUsrAppData, 0x0, sizeof(UserAppData));

		pUsrAppSsnData = webAppCreateSession(0);
		if (!pUsrAppSsnData) {
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"Session Failed\n");
				retVal = PLUGIN_FAILURE;
				goto on_return;
		}

		strcpy(pUsrAppData->url, gConfigInfo.dlr_url);
		strcat(pUsrAppData->url,"peerAddresses");
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n%s",pUsrAppData->url);
		strcpy( pUsrAppData->keyVal[0].key, "Content-Type");   
		strcpy( pUsrAppData->keyVal[0].value, "application/xml");   
		strcpy( pUsrAppData->keyVal[1].key, "Accept");   
		strcpy( pUsrAppData->keyVal[1].value, "application/xml");   

		strcpy( pUsrAppData->keyVal[2].key, "Authorization");  
		strcpy( pUsrAppData->keyVal[2].value, assign->signature);
  
		strncpy( pUsrAppData->keyVal[3].key, "X-Belkin-Client-Type-Id", sizeof(pUsrAppData->keyVal[3].key)-1);   
		strncpy( pUsrAppData->keyVal[3].value, gClientType, sizeof(pUsrAppData->keyVal[3].value)-1);   
		pUsrAppData->keyValLen = 4;

		APP_LOG("REMOTEACCESS", LOG_HIDE,"\n%s",pUsrAppData->keyVal[2].value);

		strcpy( pUsrAppData->inData, httpRelayData);
		pUsrAppData->inDataLength = strlen(httpRelayData);
		pUsrAppData->inData[pUsrAppData->inDataLength]  = '\0';

		char *check = strstr(gConfigInfo.dlr_url, "https://");
		if (check) {
				pUsrAppData->httpsFlag = 1;
		}
		retVal = webAppSendData( pUsrAppSsnData, pUsrAppData, 1);  
		if (retVal)
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Some error encountered in adding info at dlr  , errorCode %d \n", retVal);
				APP_LOG("REMOTEACCESS", LOG_CRIT, "\n Some error encountered in adding info at dlr  , errorCode %d respCode %d\n", retVal, pUsrAppData->outResp);
				retVal = POST_RELAY_FAIL;
				goto on_return;
		} else {
				APP_LOG("REMOTEACCESS", LOG_DEBUG, "\n Response rcvd in adding info at dlr  , retVal %d respCode %d\n", retVal, pUsrAppData->outResp);
		}

		if(!strstr(pUsrAppData->outHeader, "200 OK"))
		{
				APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Some error encountered in adding info at dlr  , not 200 OK \n");
				if(strstr(pUsrAppData->outHeader, "403")) {
						if (strstr(pUsrAppData->outData, "ERR_002")) {
							APP_LOG("REMOTEACCESS", LOG_DEBUG,"error encountered: ERR_002");
						}
						retVal = POST_STUN_FAIL;
						goto on_return;
				}
				else if(strstr(pUsrAppData->outHeader, "400"))
				{
						retVal = POST_STUN_FAIL;
						goto on_return;
				}
				else
				{
						APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Some error encountered in adding info at dlr, other than 403 and 400\n");
						retVal = POST_RELAY_FAIL;
						goto on_return;
				}
		}

on_return:
		if (assign) { free(assign); assign = NULL;}
		if (pUsrAppData) {
				if (pUsrAppData->outData) {free(pUsrAppData->outData); pUsrAppData->outData = NULL;}
				free(pUsrAppData); pUsrAppData = NULL;
		}
		if (pUsrAppSsnData) {webAppDestroySession ( pUsrAppSsnData ); pUsrAppSsnData = NULL;}
		return retVal;

}

/* Function to be called for sending self and peer ICE info to DLR*/
