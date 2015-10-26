/***************************************************************************
*
*
* iceCandPost.c
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
#include <string.h>
#include <stdlib.h>
#include <syslog.h>
#include <ithread.h>
#include "types.h"
#include "natTravIceWrapper.h"
#include "curl/curl.h"
#include "httpsWrapper.h"
#include "sigGen.h"
#include "global.h"

#include "natTravIceIntDef.h"
#include "pjnath/ice_session.h"
#include "mxml.h"
#include "logger.h"
#include "defines.h"

extern char gClientType[SIZE_128B];

int postPeerLocation(nattrav_rest_field_Info *candInfo,ice_config_info *psConfInfo)
{
    UserAppSessionData *pUsrAppSsnData = NULL;
    UserAppData *pUsrAppData = NULL;
    char httpRelayData[SIZE_2048B], temp[SIZE_512B];
    int i, retVal = 0, port;
    char restUrl[SIZE_128B];
    char srcbuf[SIZE_32B];

    if((!candInfo) && (!psConfInfo))
	return FAILURE;

    delPeerLocation(candInfo,psConfInfo);

    /*
       "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?><StrorePeerLocation> <peerid>peer1</ peerid><icetype>full</icetype><defaultaddress>10.1.1.1</ defaultaddress><defaultport>4000</defaultport><transport>udp</transport><ipfamilty>IPV4</ipfamily><username>user1</ username><passwd>pass123$56</ passwd>

       <candidate><compid>1</compid><candtype>host</candtype><foundation>1</foundation>

       <priority>2130706178</priority><transport>udp</transport><address>1.1.1.1

       </address><port>1000</port></candidate>
       candInfo->peer_id
       candInfo->ice_type
       candInfo->def_ipaddr	//char arr
       candInfo->def_port
       candInfo->transport
       candInfo->cand_cnt
       candInfo->cand[0].comp_id
       candInfo->cand[0].type
       candInfo->cand[0].foundation
       candInfo->cand[0].prio
       candInfo->cand[0].addr // pj_sockaddr
     */ 


    snprintf(httpRelayData,sizeof(httpRelayData), "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?><peerAddresses><iceType>%d</iceType><defaultAddress>%s</defaultAddress><defaultPort>%d</defaultPort><defaultTransport>%d</defaultTransport><ipFamily>%d</ipFamily><userName>%s</userName><password>%s</password><candidateCount>%d</candidateCount>",candInfo->ice_type, candInfo->def_addr_info.def_ipaddr, candInfo->def_addr_info.def_port,candInfo->def_addr_info.def_transport, candInfo->def_addr_info.ip_family, candInfo->ice_info.username, candInfo->ice_info.passwd, candInfo->ice_info.cand_cnt);	//TODO include ip family in candInfo (nattrav_cand_offer_info)
    strcat(httpRelayData,"<candidateList>");

    for(i=0; i<candInfo->ice_info.cand_cnt; i++)
    {
	memset(srcbuf,0,sizeof(srcbuf));
	strcpy(srcbuf, candInfo->ice_info.candidate[i].ipaddr);
	port = candInfo->ice_info.candidate[i].port;
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"cand_cnt=%d \ncomponentId=%d \n<addressType>%d \n<foundation>%s \n<priority>%u \n<transport>%d \n<address>%s \n<port>%d\n",candInfo->ice_info.cand_cnt,  candInfo->ice_info.candidate[i].compid, candInfo->ice_info.candidate[i].cand_type, candInfo->ice_info.candidate[i].foundation, candInfo->ice_info.candidate[i].priority, candInfo->ice_info.candidate[i].transport, srcbuf, port);

	memset(temp, 0,sizeof(temp));

	snprintf(temp,sizeof(temp), " <candidate><componentId>%d</componentId><addressType>%d</addressType><foundation>%s</foundation><priority>%u</priority><transport>%d</transport><address>%s</address><port>%d</port></candidate>", candInfo->ice_info.candidate[i].compid, candInfo->ice_info.candidate[i].cand_type, candInfo->ice_info.candidate[i].foundation, candInfo->ice_info.candidate[i].priority, candInfo->ice_info.candidate[i].transport, srcbuf, port);	

	strcat(httpRelayData,temp);
	memset(temp, 0,strlen(temp));

    }
    strcat(httpRelayData,"</candidateList></peerAddresses>");
    strcat(httpRelayData,"\0");

    authSign *assign = NULL;
    assign = createAuthSignature(psConfInfo->mac_addr, psConfInfo->serial_num,psConfInfo->auth_key );
    if (!assign) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Signature Structure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }

    strcpy(restUrl,psConfInfo->dlr_url);
    strcat(restUrl,"peerAddresses");
    APP_LOG("REMOTEACCESS", LOG_HIDE,"httpRelayData=%s len=%d\n",httpRelayData, strlen(httpRelayData));

    pUsrAppData = (UserAppData *)malloc(sizeof(UserAppData));
    if (!pUsrAppData) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Malloca Failure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }
    memset( pUsrAppData, 0x0, sizeof(UserAppData));

    pUsrAppSsnData = webAppCreateSession(0);
    if (!pUsrAppSsnData) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Session Failure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }

    strcpy(pUsrAppData->url, restUrl);
    strcpy( pUsrAppData->keyVal[0].key, "Content-Type");
    strcpy( pUsrAppData->keyVal[0].value, "application/xml");
    strcpy( pUsrAppData->keyVal[1].key, "Accept");
    strcpy( pUsrAppData->keyVal[1].value, "application/xml");

    strcpy( pUsrAppData->keyVal[2].key, "Authorization");
    strcpy( pUsrAppData->keyVal[2].value, assign->signature);

    strncpy( pUsrAppData->keyVal[3].key, "X-Belkin-Client-Type-Id", sizeof(pUsrAppData->keyVal[3].key)-1);   
    strncpy( pUsrAppData->keyVal[3].value, gClientType, sizeof(pUsrAppData->keyVal[3].value)-1);   
    APP_LOG("REMOTEACCESS", LOG_HIDE,"signature\n ");
    APP_LOG("REMOTEACCESS", LOG_HIDE,"signature=%s\n ",assign->signature);
    pUsrAppData->keyValLen = 4;

    strncpy( pUsrAppData->inData, httpRelayData, sizeof(pUsrAppData->inData)-1);
    pUsrAppData->inDataLength = strlen(httpRelayData);

    pUsrAppData->inData[pUsrAppData->inDataLength]  = '\0';
    APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n in postPeerLocation url=%s\n", pUsrAppData->url);

    char *check = strstr(restUrl, "https://");
    if (check) {
	pUsrAppData->httpsFlag = 1;
    }
    pUsrAppData->disableFlag = 1;

    /* dont want HTTP wrapper library to do error handling*/
    retVal = webAppSendData( pUsrAppSsnData, pUsrAppData, 1);
    if (retVal)
    {
	retVal = PLUGIN_FAILURE;
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Some error encountered with SendData, errorCode %d \n", retVal);
	goto on_return;
    }

    if(!strstr(pUsrAppData->outHeader, "200"))
    {
	retVal = PLUGIN_FAILURE;
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Some error encountered with SendData, errorCode %d \n", retVal);
	goto on_return;
    }
on_return:
    if (pUsrAppData) {
	if (pUsrAppData->outData) {free(pUsrAppData->outData); pUsrAppData->outData = NULL;}
	free(pUsrAppData); pUsrAppData = NULL;
    }
    if (pUsrAppSsnData) {webAppDestroySession ( pUsrAppSsnData ); pUsrAppSsnData = NULL;}
    if (assign) {free(assign); assign = NULL;}
    return retVal;
}

int getPeerLocation(nattrav_rest_field_Info *candInfo, ice_config_info *psConfInfo)	//TODO-uc
{
    UserAppSessionData *pUsrAppSsnData = NULL;
    UserAppData *pUsrAppData = NULL;
    int i, retVal = 0;
    char restUrl[SIZE_128B], temp[SIZE_32B];

    mxml_node_t *tree = NULL;
    mxml_node_t *find_node = NULL, *first_node = NULL;
    mxml_index_t *node_index = NULL;

    /*
       "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?><StrorePeerLocation> <peerid>peer1</ peerid><icetype>full</icetype><defaultaddress>10.1.1.1</ defaultaddress><defaultport>4000</defaultport><transport>udp</transport><ipfamilty>IPV4</ipfamily><username>user1</ username><passwd>pass123$56</ passwd>

       <candidate><compid>1</compid><candtype>host</candtype><foundation>1</foundation>

       <priority>2130706178</priority><transport>udp</transport><address>1.1.1.1

       </address><port>1000</port></candidate>
       candInfo->peer_id
       candInfo->ice_type
       candInfo->def_ipaddr	//char arr
       candInfo->def_port
       candInfo->transport
       candInfo->cand_cnt
       candInfo->cand[0].comp_id
       candInfo->cand[0].type
       candInfo->cand[0].foundation
       candInfo->cand[0].prio
       candInfo->cand[0].addr // pj_sockaddr
     */ 


    APP_LOG("REMOTEACCESS", LOG_DEBUG,"in get Peer location=%s \n",candInfo->peer_id);
    authSign *assign = NULL;
    assign = createAuthSignature(psConfInfo->mac_addr, psConfInfo->serial_num, psConfInfo->auth_key);
    if (!assign) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Signature Structure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }

    strcpy(restUrl,psConfInfo->dlr_url);
    strcat(restUrl,"peerAddresses");
    memset(temp,0,sizeof(temp));
    snprintf(temp,sizeof(temp),"/%s", candInfo->peer_id);		//TODO
    APP_LOG("REMOTEACCESS", LOG_DEBUG,"temp=%s\n", temp);
    strcat(restUrl, temp);
    APP_LOG("REMOTEACCESS", LOG_DEBUG,"anj:in getlocation, url:%s\n",restUrl);
    pUsrAppData = (UserAppData *)malloc(sizeof(UserAppData));
    if (!pUsrAppData) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Malloca Failure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }
    memset( pUsrAppData, 0x0, sizeof(UserAppData));
    pUsrAppSsnData = webAppCreateSession(0);
    if (!pUsrAppSsnData) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Session Failure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }
    strcpy(pUsrAppData->url, restUrl);
    strcpy( pUsrAppData->keyVal[0].key, "Authorization");
    strcpy( pUsrAppData->keyVal[0].value, assign->signature);
    strcpy( pUsrAppData->keyVal[1].key, "Content-Type");
    strcpy( pUsrAppData->keyVal[1].value, "application/xml");
    strcpy( pUsrAppData->keyVal[2].key, "Accept");
    strcpy( pUsrAppData->keyVal[2].value, "application/xml");
    strncpy( pUsrAppData->keyVal[3].key, "X-Belkin-Client-Type-Id", sizeof(pUsrAppData->keyVal[3].key)-1);   
    strncpy( pUsrAppData->keyVal[3].value, gClientType, sizeof(pUsrAppData->keyVal[3].value)-1);   

    pUsrAppData->keyValLen = 4;
    pUsrAppData->inDataLength = 0;

    APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n in postPeerAddr restUrl=%s\n", pUsrAppData->url);

    char *check = strstr(restUrl, "https://");
    if (check) {
	pUsrAppData->httpsFlag = 1;
    }
    /* dont want HTTP wrapper library to do error handling*/
    pUsrAppData->disableFlag = 1;
#if 1
    retVal = webAppSendData( pUsrAppSsnData, pUsrAppData, 0);	//TODO - uc
#else
    strcpy(pUsrAppData->outData,tData); pUsrAppData->outHeaderLength = strlen(tData); retVal=0; //TODO -com
#endif
    if (retVal)
    {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Some error encountered with SendData, errorCode %d \n", retVal);
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }

    if(!strstr(pUsrAppData->outHeader, "200"))
    {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Some error encountered with SendData, errorCode %d \n", retVal);
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }
    tree = mxmlLoadString(NULL, pUsrAppData->outData, MXML_NO_CALLBACK);
    if (tree) 
    {
	APP_LOG("REMOTEACCESS", LOG_HIDE,"XML String Loaded is %s\n", pUsrAppData->outData);
	find_node = mxmlFindElement(tree, tree, "peerAddresses", NULL, NULL, MXML_DESCEND);
	if (find_node) 
	{
	    APP_LOG("REMOTEACCESS", LOG_DEBUG,"The value set in the node is %s\n", find_node->child->value.text.string);
	    find_node = mxmlFindElement(tree, tree, "iceType", NULL, NULL, MXML_DESCEND);
	    if (find_node) {
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"The iceType set in the node is %s\n", find_node->child->value.text.string);
		candInfo->ice_type = atoi(find_node->child->value.text.string);
	    }
	    find_node = mxmlFindElement(tree, tree, "defaultAddress", NULL, NULL, MXML_DESCEND);
	    if (find_node) {
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"The defaultAddress set in the node is %s\n", find_node->child->value.text.string);
		strcpy(candInfo->def_addr_info.def_ipaddr, "125.23.218.144");
	    }
	    find_node = mxmlFindElement(tree, tree, "defaultPort", NULL, NULL, MXML_DESCEND);
	    if (find_node) {
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"The defaultPort set in the node is %s\n", find_node->child->value.text.string);
		candInfo->def_addr_info.def_port = atoi(find_node->child->value.text.string);
	    }
	    find_node = mxmlFindElement(tree, tree, "defaultTransport", NULL, NULL, MXML_DESCEND);
	    if (find_node) {
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"The defaultTransport set in the node is %s\n", find_node->child->value.text.string);
		candInfo->def_addr_info.def_transport = atoi(find_node->child->value.text.string);
	    }
	    find_node = mxmlFindElement(tree, tree, "ipFamily", NULL, NULL, MXML_DESCEND);
	    if (find_node) {
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"The ipFamily set in the node is %s\n", find_node->child->value.text.string);
		candInfo->def_addr_info.ip_family = atoi(find_node->child->value.text.string);
	    }
	    find_node = mxmlFindElement(tree, tree, "userName", NULL, NULL, MXML_DESCEND);
	    if (find_node) {
		APP_LOG("REMOTEACCESS", LOG_HIDE,"The userName set in the node is %s\n", find_node->child->value.text.string);
		strcpy(candInfo->ice_info.username, find_node->child->value.text.string);	 //TODO chk ufrag
	    }
	    find_node = mxmlFindElement(tree, tree, "password", NULL, NULL, MXML_DESCEND);
	    if (find_node) {
		APP_LOG("REMOTEACCESS", LOG_HIDE,"The password set in the node is %s\n", find_node->child->value.text.string);
		strcpy(candInfo->ice_info.passwd, find_node->child->value.text.string);
	    }
	    find_node = mxmlFindElement(tree, tree, "candidateCount", NULL, NULL, MXML_DESCEND);
	    if (find_node) {
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"The candidateCount set in the node is %d\n", atoi(find_node->child->value.text.string));
		candInfo->ice_info.cand_cnt = atoi(find_node->child->value.text.string);
	    }
	    find_node = mxmlFindElement(tree, tree, "candidateList", NULL, NULL, MXML_DESCEND);
	    if (find_node) 
	    {	
		node_index = mxmlIndexNew(tree,"candidate", NULL);
		if (!node_index) {
		    retVal = PLUGIN_FAILURE;
		}
		first_node = mxmlIndexReset(node_index);
		if (!first_node) {
		    retVal = PLUGIN_FAILURE;
		}
		for(i=0; i<(candInfo->ice_info.cand_cnt); i++)
		{	
		    first_node = mxmlIndexFind(node_index,"candidate", NULL);
		    if (first_node && (first_node->child)) 
		    {
			find_node = mxmlFindElement(first_node, first_node, "componentId", NULL, NULL, MXML_DESCEND);
			if (find_node) 
			{
			    APP_LOG("REMOTEACCESS", LOG_DEBUG,"The componentId set in the node is %s\n", find_node->child->value.text.string);
			    candInfo->ice_info.candidate[i].compid = atoi(find_node->child->value.text.string);
			}

			find_node = mxmlFindElement(first_node, first_node, "addressType", NULL, NULL, MXML_DESCEND);
			if (find_node) 
			{
			    APP_LOG("REMOTEACCESS", LOG_DEBUG,"The addressType set in the node is %s\n", find_node->child->value.text.string);
			    candInfo->ice_info.candidate[i].cand_type = atoi(find_node->child->value.text.string);
			}

			find_node = mxmlFindElement(first_node, first_node, "foundation", NULL, NULL, MXML_DESCEND);
			if (find_node) 
			{
			    APP_LOG("REMOTEACCESS", LOG_DEBUG,"The foundation set in the node is %s\n", find_node->child->value.text.string);
			    strcpy(candInfo->ice_info.candidate[i].foundation, (find_node->child->value.text.string));
			}

			find_node = mxmlFindElement(first_node, first_node, "priority", NULL, NULL, MXML_DESCEND);
			if (find_node) 
			{
			    APP_LOG("REMOTEACCESS", LOG_DEBUG,"The priority set in the node is %s\n", find_node->child->value.text.string);
			    candInfo->ice_info.candidate[i].priority = atoi(find_node->child->value.text.string);
			}

			find_node = mxmlFindElement(first_node, first_node, "transport", NULL, NULL, MXML_DESCEND);	// nothing to do for transport
			if (find_node) 
			{
			    APP_LOG("REMOTEACCESS", LOG_DEBUG,"The transport set in the node is %s\n", find_node->child->value.text.string);
			    candInfo->ice_info.candidate[i].transport=  atoi(find_node->child->value.text.string);
			}

			find_node = mxmlFindElement(first_node, first_node, "address", NULL, NULL, MXML_DESCEND);	 //TODO port ??
			if (find_node) 
			{
			    APP_LOG("REMOTEACCESS", LOG_DEBUG,"The address set in the node is %s\n", find_node->child->value.text.string);
			    strcpy(candInfo->ice_info.candidate[i].ipaddr, find_node->child->value.text.string);
			}
			find_node = mxmlFindElement(first_node, first_node, "port", NULL, NULL, MXML_DESCEND);	 //TODO port ??
			if (find_node) 
			{
			    APP_LOG("REMOTEACCESS", LOG_DEBUG,"The port set in the node is %s\n", find_node->child->value.text.string);
			    candInfo->ice_info.candidate[i].port = atoi(find_node->child->value.text.string);
			}
		    }
		}
		mxmlDelete(tree);
		tree=NULL;
	    }
	}
	else
	{
	    if(tree){mxmlDelete(tree);tree=NULL;}
	}	
    }
on_return:
    if (pUsrAppSsnData) {webAppDestroySession( pUsrAppSsnData ); pUsrAppSsnData = NULL;}
    if (pUsrAppData) {
	if (pUsrAppData->outData) {free(pUsrAppData->outData); pUsrAppData->outData = NULL;}
	free(pUsrAppData); pUsrAppData = NULL;
    }
    if (assign) {free(assign); assign = NULL;}
    return retVal;
}

int delPeerLocation(nattrav_rest_field_Info *candInfo, ice_config_info *psConfInfo)	//TODO-uc
{
    UserAppSessionData *pUsrAppSsnData = NULL;
    UserAppData *pUsrAppData = NULL;
    int retVal = 0;
    char restUrl[SIZE_128B];

    authSign *assign = NULL;
    assign = createAuthSignature(psConfInfo->mac_addr, psConfInfo->serial_num, psConfInfo->auth_key);
    if (!assign) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Signature Structure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }

    strcpy(restUrl,psConfInfo->dlr_url); 
    strcat(restUrl,"deletePeerAddr");
    APP_LOG("REMOTEACCESS", LOG_DEBUG,"anj :del url%s\n",restUrl);
    pUsrAppData = (UserAppData *)malloc(sizeof(UserAppData));
    if (!pUsrAppData) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Malloc Structure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }
    memset( pUsrAppData, 0x0, sizeof(UserAppData));
    pUsrAppSsnData = webAppCreateSession(0);
    if (!pUsrAppSsnData) {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Signature Structure returned NULL\n");
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }

    strcpy(pUsrAppData->url, restUrl);
    strcpy( pUsrAppData->keyVal[0].key, "Content-Type");
    strcpy( pUsrAppData->keyVal[0].value, "application/xml");
    strcpy( pUsrAppData->keyVal[1].key, "Accept");
    strcpy( pUsrAppData->keyVal[1].value, "application/xml");
    strcpy( pUsrAppData->keyVal[2].key, "Authorization");
    strcpy( pUsrAppData->keyVal[2].value, assign->signature);
    strncpy( pUsrAppData->keyVal[3].key, "X-Belkin-Client-Type-Id", sizeof(pUsrAppData->keyVal[3].key)-1);   
    strncpy( pUsrAppData->keyVal[3].value, gClientType, sizeof(pUsrAppData->keyVal[3].value)-1);   

    pUsrAppData->keyValLen = 4;

    pUsrAppData->inDataLength = 2; 
    char *check = strstr(restUrl, "https://");
    if (check) {
	pUsrAppData->httpsFlag = 1;
    }
    pUsrAppData->disableFlag = 1;

    /* dont want HTTP wrapper library to do error handling*/
    retVal = webAppSendData( pUsrAppSsnData, pUsrAppData, 1);	//TODO - uc
    if (retVal)
    {
	APP_LOG("REMOTEACCESS", LOG_DEBUG,"\n Some error encountered with SendData, errorCode %d \n", retVal);	//TODO
	retVal = PLUGIN_FAILURE;
	goto on_return;
    }

on_return:
    if (assign) {free(assign); assign = NULL;}
    if (pUsrAppSsnData) {webAppDestroySession ( pUsrAppSsnData ); pUsrAppSsnData = NULL;}
    if (pUsrAppData) {
	if (pUsrAppData->outData) {free(pUsrAppData->outData); pUsrAppData->outData = NULL;}
	free(pUsrAppData); pUsrAppData = NULL;
    }
    return retVal;
}

