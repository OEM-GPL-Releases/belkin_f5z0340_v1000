/***************************************************************************
*
*
* UDSServerHandler.c
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
#include <sys/syscall.h>

#include "sigGen.h"
#include "defines.h"
#include "curl/curl.h"
#include "logger.h"
#include "ulog.h"
#include "natClient.h"
#include "global.h"
#include "ipcUDS.h"
#include "ithread.h"
#include "itc.h"
#include "natTravIceWrapper.h"
#include "natTravIceIntDef.h"

/* The uClibc in the toolchain used for netcam (buildroot-gcc342)
   doesn't support program_invocation_short_name, so define it here */
#ifdef PRODUCT_WeMo_NetCam
char *program_invocation_short_name;
#endif

extern char g_turnServerEnvIPaddr[SIZE_32B];
extern unsigned short gpluginRemAccessEnable;
extern int gpluginNatInitialized;
int gWiFiClientDeviceCurrState;
extern int gUDSServerDataFD;
char gPluginPrivatekey[MAX_PKEY_LEN];
char gClientType[SIZE_128B];
char gWiFiMacAddress[SIZE_64B];
char gSerialNo[SIZE_64B];
char gRestoreState[MAX_RES_LEN];
int gNTPTimeSet = 0;
void* remoteAccessInitThd(void *args);
extern int gIceRunning;
extern nattrav_cand_offer_Info gRemoteInfo;
pthread_mutex_t* arrPthreadMutex[100] = {NULL,};

extern void* invokeNatReInitThd(void *arg);



static void handleSigSegv(int signum)
{


	    uint32_t *up = (uint32_t *)&signum;
    unsigned int sp = 0;

#if defined (__mips__)

    APP_LOG("WiFiApp",LOG_ALERT,"[%d:%s] SIGNAL SIGSEGV [%d] RECEIVED, Best guess fault address: %08x, ra: %08x",
	(int)syscall(SYS_gettid), tu_get_my_thread_name(), signum, up[8], up[72]);

	pmortem_connect_and_send((uint32_t *) &signum, 4 * 1024);

#else
    APP_LOG("WiFiApp",LOG_ALERT,"[%d:%s] SIGNAL SIGSEGV [%d] RECEIVED, aborting...",
	(int)syscall(SYS_gettid), tu_get_my_thread_name(), signum);
#endif
    pthread_exit(NULL);

}


static void handleSigAbrt(int signum)
{
    APP_LOG("WiFiApp",LOG_ERR,"SIGNAL SIGABRT [%d] RECEIVED ..",signum);
    exit(0);
}
/* signal handler for wemoApp */
static void handleRtMinSignal(int signum)
{
	if(signum == SIGRTMIN)
	{
		APP_LOG("WiFiApp",LOG_ALERT,"SIGNAL [%d] RECEIVED ..",signum);
		pthread_exit(NULL);
	}
	else
	{
		APP_LOG("WiFiApp",LOG_ALERT,"UNEXPECTED SIGNAL RECEIVED [%d] ..",signum);
	}
}
static void handleSigPipe(int signum)
{
    APP_LOG("WiFiApp",LOG_ALERT,"SIGNAL SIGPIPE [%d] RECEIVED ..",signum);
    return;
}
void setSignalHandlers(void)
{
   struct sigaction act, oldact;
   act.sa_flags = (SA_NOCLDSTOP | SA_NOCLDWAIT | SA_RESTART);
   act.sa_handler = handleRtMinSignal;

   if(sigaction(SIGRTMIN, &act, &oldact))
   {
	APP_LOG("WiFiApp",LOG_ERR,
		"sigaction failed... errno: %d", errno);
   }
   else
   {
	if(oldact.sa_handler == SIG_IGN)
		APP_LOG("WiFiApp",LOG_DEBUG,"oldact RTMIN: SIGIGN");

        if(oldact.sa_handler == SIG_DFL)
		APP_LOG("WiFiApp",LOG_DEBUG,"oldact RTMIN: SIGDFL");
   }

   act.sa_flags = (SA_NOCLDSTOP | SA_NOCLDWAIT | SA_RESTART);
   act.sa_handler = handleSigSegv;
   if(sigaction(SIGSEGV, &act, &oldact))
   {
	APP_LOG("WiFiApp",LOG_ERR,
		"sigaction failed... errno: %d", errno);
   }
   else
   {
	if(oldact.sa_handler == SIG_IGN)
		APP_LOG("WiFiApp",LOG_DEBUG,"oldact RTMIN: SIGIGN");

        if(oldact.sa_handler == SIG_DFL)
		APP_LOG("WiFiApp",LOG_DEBUG,"oldact RTMIN: SIGDFL");
   }
   act.sa_flags = (SA_NOCLDSTOP | SA_NOCLDWAIT | SA_RESTART);
   act.sa_handler = handleSigAbrt;
   if(sigaction(SIGABRT, &act, &oldact))
   {
	APP_LOG("WiFiApp",LOG_ERR,
		"sigaction failed... errno: %d", errno);
   }
   else
   {
	if(oldact.sa_handler == SIG_IGN)
		APP_LOG("WiFiApp",LOG_DEBUG,"oldact RTMIN: SIGIGN");

        if(oldact.sa_handler == SIG_DFL)
		APP_LOG("WiFiApp",LOG_DEBUG,"oldact RTMIN: SIGDFL");
   }
		act.sa_flags = (SA_NOCLDSTOP | SA_NOCLDWAIT | SA_RESTART);
   act.sa_handler = handleSigPipe;
   if(sigaction(SIGPIPE, &act, &oldact))
   {
	APP_LOG("WiFiApp",LOG_ERR,
		"sigaction failed... errno: %d", errno);
   }
   else
   {
	if(oldact.sa_handler == SIG_IGN)
		APP_LOG("WiFiApp",LOG_DEBUG,"oldact RTMIN: SIGIGN");

        if(oldact.sa_handler == SIG_DFL)
		APP_LOG("WiFiApp",LOG_DEBUG,"oldact RTMIN: SIGDFL");
   }
}

int SendNatResponsePkt(void *hndl,char* statusResp,int statusRespLen,void* remoteInfo,void*data_sock)
{
	int retVal=PLUGIN_SUCCESS;
	int dsock=PLUGIN_SUCCESS;
	unsigned int byteSend = 0;

	if(gIceRunning)
	{
		APP_LOG("UDSServer", LOG_DEBUG, "send response data on ICE");
		retVal = nattrav_send_data(hndl, statusResp, statusRespLen, remoteInfo);
	}
	else
	{
		APP_LOG("UDSServer", LOG_DEBUG, "send response data on RELAY");
		dsock = *(int*)data_sock;
		while(statusRespLen)
		{
			if(statusRespLen <= MAX_SEND_BUF_LEN)
				byteSend = statusRespLen;
			else
				byteSend = MAX_SEND_BUF_LEN;

			if(send(dsock, statusResp, byteSend, 0) < 0)
			{
				APP_LOG("UDSServer:",LOG_ERR, "Socket Send error");
				retVal=PLUGIN_FAILURE;
				break;
			}
			else
			{
				statusResp += byteSend;
				statusRespLen -= byteSend;
				APP_LOG("UDSServer:",LOG_ERR, "remote Socket Sent success:%d, remaining:%d", byteSend,statusRespLen);
			}

		}
	}
	return retVal;
}

pIPCCommand UDSSendReceiveRemoteDataToApplication(char *pkt, unsigned int pkt_len, IPCCOMMAND cmd)
{
	pIPCCommand pcmd = NULL;
	pIPCCommand pgetResp = NULL;
	char *payload_buffer = NULL;
	unsigned int dataLengthLeft = 0;
	unsigned int byteReceive = 0;
	int nbytes;

	pcmd = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + pkt_len);
	if(pcmd == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "pcmd calloc failed");
		resetSystem();
	}

	/*fill response header*/
	pcmd->cmd = cmd;
	pcmd->arg_length = pkt_len;

	/*fill response payload*/
	memcpy((char *)pcmd + IPC_CMD_HDR_LEN, pkt, pkt_len);

	/*send it on Application socket*/
	APP_LOG("UDSServer:", LOG_DEBUG, "Sending remote data on UDS remote data path socket:%d", gUDSServerDataFD);
	nbytes = IpcUdsSend((pIPCCommand)pcmd, gUDSServerDataFD);
	if(nbytes < 0)
	{
		APP_LOG("UDSServer:", LOG_ERR, "UDS data send failed!");
		goto CLEAN_UP;
	}

	pgetResp = (pIPCCommand)calloc(1, sizeof(IPCCommand));
	if(pgetResp == NULL)
	{
		APP_LOG("UDSServer:", LOG_ERR, "calloc failed\n");
		resetSystem();
	}

	/*receive header on Application socket*/
	nbytes = recv(gUDSServerDataFD, (char*)pgetResp, IPC_CMD_HDR_LEN, 0x0);
	if(nbytes <= 0)
	{
		APP_LOG("UDSServer:", LOG_ERR, "header read error, nbytes:%d", nbytes);
		goto CLEAN_UP;
	}

	/*receive payload on Application socket*/
	if(pgetResp->arg_length > 0x0)
	{
		nbytes = 0;
		payload_buffer = (char*)calloc(1, pgetResp->arg_length);
		if(payload_buffer == NULL )
		{
			APP_LOG("UDSServer:", LOG_ERR, "payload buffer calloc failed");
			resetSystem();
		}

		/*receive data*/
		dataLengthLeft = pgetResp->arg_length;
		pgetResp->arg = (void *)payload_buffer;

		APP_LOG("UDSServer:", LOG_DEBUG, "Receive Payload, pgetResp->arg_length:%d", pgetResp->arg_length);

		while(dataLengthLeft)
		{
			if(dataLengthLeft <= MAX_SEND_BUF_LEN)
				byteReceive = dataLengthLeft;
			else
				byteReceive = MAX_SEND_BUF_LEN;

			nbytes = recv(gUDSServerDataFD, payload_buffer, byteReceive, 0x0);
			if(nbytes <= 0)
			{
				APP_LOG("UDSServer:", LOG_ERR, "payload read error, nbytes:%d", nbytes);
				free(payload_buffer);
				payload_buffer = NULL;
				goto CLEAN_UP;
			}
			else
			{
				payload_buffer += nbytes;
				dataLengthLeft -= nbytes;
				APP_LOG("UDSServer:",LOG_DEBUG, "IPC server Socket Receive success:%d, remaining:%d", nbytes, dataLengthLeft);
			}
		}
	}

	/*return handler response*/
	return pgetResp;

CLEAN_UP:
	if(pgetResp)
	{
		free(pgetResp->arg);pgetResp->arg = NULL;
		free(pgetResp);pgetResp = NULL;
	}
	return pgetResp;
}

int setUDSIceRunningstatus(int iceRunningStatus)
{
	pIPCCommand pgetResp = NULL;
	char iceRunningStatusStr[5] = {'\0',};
	int retVal = -1;

	snprintf(iceRunningStatusStr,sizeof(iceRunningStatusStr), "%d", iceRunningStatus);

	APP_LOG("UDSServer:", LOG_DEBUG, "ICE Running status is :%d", iceRunningStatus);
	/*send/receive RELAY handler data to/from aaplication socket*/
	pgetResp = UDSSendReceiveRemoteDataToApplication(iceRunningStatusStr, sizeof(iceRunningStatusStr), IPC_CMD_NAT_SET_ICE_RUNNING_STATUS);
	if(pgetResp)
	{
		if(pgetResp->arg_length)
			retVal = atoi(pgetResp->arg);
		free(pgetResp);pgetResp = NULL;

		APP_LOG("UDSServer:", LOG_DEBUG, "ICE Running status send is :%d", retVal);
	}
	return retVal;
}

int UDSrelayReqestHandler(void *relay,void *pkt,unsigned pkt_len,const void* peer_addr,unsigned addr_len,void *data_sock)
{
	APP_LOG("UDSServer:", LOG_DEBUG, "Received data on RELAY with len :%d", pkt_len);
	pIPCCommand pgetResp = NULL;
	int retVal = -1;

	/*send/receive RELAY handler data to/from aaplication socket*/
	pgetResp = UDSSendReceiveRemoteDataToApplication(pkt, pkt_len, IPC_CMD_NAT_REMOTE_DATA);
	if(pgetResp)
	{
		/*send reseponse to RELAY remote socket*/
		retVal = SendNatResponsePkt(NULL,pgetResp->arg,pgetResp->arg_length,NULL,data_sock);
		if(retVal < 0)
			{APP_LOG("UDSServer:", LOG_ERR, "Failed to send remote reply on RELAY, len:%d", pgetResp->arg_length);}
		else
			APP_LOG("UDSServer:", LOG_ERR, "Send remote reply on RELAY success, len :%d", pgetResp->arg_length);

		if(pgetResp->arg)free(pgetResp->arg);pgetResp->arg = NULL;
		free(pgetResp);pgetResp = NULL;
	}
	return retVal;
}

void* UDSiceReqHandlThread(void *arg)
{
	struct arg_struct *remoteData = NULL;
	pIPCCommand pgetResp = NULL;
	int retVal = -1;

	remoteData = (struct arg_struct*)arg;
	if(NULL == remoteData)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "Invalid data received on ICE");
		return NULL;
	}
	APP_LOG("UDSServer:", LOG_DEBUG, "Received data on ICE with len :%d", remoteData->pkt_len);

	pj_thread_desc iceReq_thread_desc;
	pj_thread_t *iceReq_thread;
	pj_thread_register("iceAccessServiceHandler", iceReq_thread_desc, &iceReq_thread);

	/*send/receive ICE handler data to/from aaplication socket*/
	pgetResp = UDSSendReceiveRemoteDataToApplication(remoteData->pkt, remoteData->pkt_len, IPC_CMD_NAT_REMOTE_DATA);
	if(pgetResp)
	{
		/*send reseponse to ICE remote socket*/
		retVal = SendNatResponsePkt(remoteData->hndl,pgetResp->arg,pgetResp->arg_length,&gRemoteInfo,NULL);
		if(retVal < 0)
			{APP_LOG("UDSServer:", LOG_ERR, "Failed to send remote reply on ICE, len:%d", pgetResp->arg_length);}
		else
			APP_LOG("UDSServer:", LOG_ERR, "Send remote reply on ICE success, len :%d", pgetResp->arg_length);

		if(pgetResp->arg)free(pgetResp->arg);pgetResp->arg = NULL;
		free(pgetResp);pgetResp = NULL;
	}

	if(remoteData){free(remoteData->transaction_id);free(remoteData);remoteData = NULL;}
	return NULL;
}

int setUDSCurrentClientState(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	pIPCCommandIntData cmdData = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	cmdData = (pIPCCommandIntData)(sa_cmd->arg);
	if (cmdData)
	{
		gWiFiClientDeviceCurrState = cmdData->value;
		APP_LOG("UDSServer:", LOG_DEBUG, "WiFi Client Device Current State:%d", gWiFiClientDeviceCurrState);
	}

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = gWiFiClientDeviceCurrState;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

void createRemoteAccessReInitThd(int natReInitStatus)
{
	int retVal = -1;
	ithread_t remoteReInit_thread;
	int *arg = NULL;

	arg = (int*)calloc(1, sizeof(int));
	if(NULL == arg)
		resetSystem();

	*arg = natReInitStatus;
	retVal = ithread_create(&remoteReInit_thread, NULL, invokeNatReInitThd, arg);
	if(retVal != 0)
	{
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"failed to create remoteAccessInit thread ");
		resetSystem();
	}
}

int sendUDSNATReInitStatus(IPCCommand *sa_cmd, int aClientFD)
{
	int NATReinitStatus = NATCL_HEALTH_NOTGOOD;
	pIPCCommand sa_resp = NULL;
	pIPCCommandIntData cmdData = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	cmdData = (pIPCCommandIntData)(sa_cmd->arg);
	if (cmdData)
	{
		NATReinitStatus = cmdData->value;
	}

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	createRemoteAccessReInitThd(NATReinitStatus);

	/*fill response payload*/
	replyData.value = 0;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

int UDSGetRemoteDataBytes(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = pjGetRemoteDataBytes();
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

int UDSSetServerdstData(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	pjSetRemoteDataBytes();

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = 0;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

int sendUDSNatInitializedState(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = gpluginNatInitialized;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

void createRemoteAccessInitThd()
{
	int retVal = -1;
	ithread_t remoteinit_thread;
	retVal = ithread_create(&remoteinit_thread, NULL, remoteAccessInitThd, NULL);
	if(retVal != 0)
	{
		APP_LOG("REMOTEACCESS", LOG_DEBUG,"failed to create remoteAccessInit thread ");
		resetSystem();
	}
}

int UDSTriggerNat(IPCCommand *sa_cmd, int aClientFD)
{
        pIPCCommand sa_resp = NULL;
        IPCCommandIntData replyData;

        sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
        if(sa_resp == NULL)
        {
                APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
        }

        /*fill response header*/
        sa_resp->cmd = sa_cmd->cmd;
        sa_resp->arg_length = sizeof(IPCCommandIntData);

	trigger_nat();
        /*fill response payload*/
        replyData.value = 0;
        memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

        /*send it on socket*/
        IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

        return SUCCESS;
}

int UDSInvokeNatDestroy(IPCCommand *sa_cmd, int aClientFD)
{
        pIPCCommand sa_resp = NULL;
        IPCCommandIntData replyData;

        sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
        if(sa_resp == NULL)
        {
                APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
        }

        /*fill response header*/
        sa_resp->cmd = sa_cmd->cmd;
        sa_resp->arg_length = sizeof(IPCCommandIntData);

	invokeNatDestroy();
        /*fill response payload*/
        replyData.value = 0;
        memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

        /*send it on socket*/
        IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

        return SUCCESS;
}

int UDSRemoteAccessInit(IPCCommand *sa_cmd, int aClientFD)
{
        pIPCCommand sa_resp = NULL;
        IPCCommandIntData replyData;

        sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
        if(sa_resp == NULL)
        {
                APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
        }

        /*fill response header*/
        sa_resp->cmd = sa_cmd->cmd;
        sa_resp->arg_length = sizeof(IPCCommandIntData);

	createRemoteAccessInitThd();
        /*fill response payload*/
        replyData.value = 0;
        memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

        /*send it on socket*/
        IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

        return SUCCESS;
}

int sendUDSMonitorNatClientStatus(IPCCommand *sa_cmd, int aClientFD)
{
        pIPCCommand sa_resp = NULL;
        IPCCommandIntData replyData;

        sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
        if(sa_resp == NULL)
        {
                APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
        }

        /*fill response header*/
        sa_resp->cmd = sa_cmd->cmd;
        sa_resp->arg_length = sizeof(IPCCommandIntData);

        /*fill response payload*/
        replyData.value = monitorNATCLStatus(NULL);
        memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

        /*send it on socket*/
        IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

        return SUCCESS;
}

#ifndef _OPENWRT_
int UDSSetServerdstData(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	pIPCSetDstData cmdData = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	cmdData = (pIPCSetDstData)(sa_cmd->arg);
	if (cmdData)
	{
		pj_dst_data_os(cmdData->idx, cmdData->timezone, cmdData->dstEnable);
	}

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = 0;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}
#endif

int UDSSetServerNATData(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	pIPCNATInitData cmdData = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	cmdData = (pIPCNATInitData)(sa_cmd->arg);
	if (cmdData)
	{
		snprintf(gSerialNo,sizeof(gSerialNo), "%s", cmdData->serialNo);
		snprintf(gWiFiMacAddress,sizeof(gWiFiMacAddress), "%s", cmdData->WiFiMacAddress);
		snprintf(gPluginPrivatekey,sizeof(gPluginPrivatekey), "%s", cmdData->pluginPrivatekey);
		snprintf(gRestoreState,sizeof(gRestoreState), "%s", cmdData->restoreState);
		snprintf(gClientType,sizeof(gClientType), "%s", cmdData->clientType);
                APP_LOG("UDSServer:", LOG_DEBUG, "NAT Data Received\nSerialNo:%s\nWiFiMacAddress:%s\nPluginPrivatekey:%s\nRestoreState:%s\nClientType:%s", gSerialNo,gWiFiMacAddress,gPluginPrivatekey,gRestoreState,gClientType);
	}

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = 0;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

int UDSSetTurnServerEnvIPaddr(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	strncpy(g_turnServerEnvIPaddr, sa_cmd->arg, sizeof(g_turnServerEnvIPaddr)-1);
	APP_LOG("UDSServer:", LOG_DEBUG, "g_turnServerEnvIPaddr set is:%s", g_turnServerEnvIPaddr);

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = 0;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

int UDSStartServerDataPath(IPCCommand *sa_cmd, int aClientFD)
{
        pIPCCommand sa_resp = NULL;
        IPCCommandIntData replyData;

        sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
        if(sa_resp == NULL)
        {
                APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
        }

        /*fill response header*/
        sa_resp->cmd = sa_cmd->cmd;
        sa_resp->arg_length = sizeof(IPCCommandIntData);

        /*fill response payload*/
        replyData.value = 0;
        memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

        /*send it on socket*/
        IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

        return SUCCESS;
}

int UDSSetServerNTPTimeSyncStatus(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	pIPCCommandIntData cmdData = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	cmdData = (pIPCCommandIntData)(sa_cmd->arg);
	if (cmdData)
	{
		gNTPTimeSet = cmdData->value;
		APP_LOG("UDSServer:", LOG_DEBUG, "NTP time sync set is:%d", gNTPTimeSet);
	}

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = gNTPTimeSet;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

int UDSSetServerRemoteEnableStatus(IPCCommand *sa_cmd, int aClientFD)
{
	pIPCCommand sa_resp = NULL;
	pIPCCommandIntData cmdData = NULL;
	IPCCommandIntData replyData;

	sa_resp = (pIPCCommand)calloc(1, IPC_CMD_HDR_LEN + sizeof(IPCCommandIntData));
	if(sa_resp == NULL)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "sa_resp calloc failed");
		resetSystem();
	}

	cmdData = (pIPCCommandIntData)(sa_cmd->arg);
	if (cmdData)
	{
		gpluginRemAccessEnable = cmdData->value;
		APP_LOG("UDSServer:", LOG_DEBUG, "Remote Enable status set is:%d", gpluginRemAccessEnable);
	}

	/*fill response header*/
	sa_resp->cmd = sa_cmd->cmd;
	sa_resp->arg_length = sizeof(IPCCommandIntData);

	/*fill response payload*/
	replyData.value = gpluginRemAccessEnable;
	memcpy((char *)sa_resp + IPC_CMD_HDR_LEN, &replyData, sizeof(IPCCommandIntData));

	/*send it on socket*/
	IpcUdsSend((pIPCCommand)sa_resp, aClientFD);

	return SUCCESS;
}

int ProcessNatUDSServerIpcCmd(IPCCommand* s_cmd, int aClientFD)
{
        if (NULL == s_cmd)
                return FAILURE;

        switch(s_cmd->cmd)
        {
                case IPC_CMD_NAT_INITIALIZED_STATE:
                        sendUDSNatInitializedState(s_cmd, aClientFD);
                        break;

                case IPC_CMD_MONITOR_NAT_CLIENT_STATUS:
                        sendUDSMonitorNatClientStatus(s_cmd, aClientFD);
                        break;

                case IPC_CMD_NAT_REINIT:
                        sendUDSNATReInitStatus(s_cmd, aClientFD);
                        break;

		case IPC_CMD_NAT_SET_CLIENT_STATE:
			setUDSCurrentClientState(s_cmd, aClientFD);
			break;

		case IPC_CMD_NAT_START_REMOTE_DATA_PATH:
			UDSStartServerDataPath(s_cmd, aClientFD);
			break;

		case IPC_CMD_NAT_TRIGGER:
			UDSTriggerNat(s_cmd, aClientFD);
			break;

		case IPC_CMD_NAT_INVOKE_DESTROY:
			UDSInvokeNatDestroy(s_cmd, aClientFD);
			break;

		case IPC_CMD_NAT_REMOTE_ACCESS_INIT:
			UDSRemoteAccessInit(s_cmd, aClientFD);
			break;

#ifndef _OPENWRT_
		case IPC_CMD_NAT_DST_DATA:
			UDSSetServerdstData(s_cmd, aClientFD);
			break;
#endif

		case IPC_CMD_NAT_SET_NAT_DATA:
			UDSSetServerNATData(s_cmd, aClientFD);
			break;

		case IPC_CMD_NAT_SET_TURN_SERVER_IP_ADDRESS:
			UDSSetTurnServerEnvIPaddr(s_cmd, aClientFD);
			break;

		case IPC_CMD_NAT_SET_REMOTE_ENABLE_STATE:
			UDSSetServerRemoteEnableStatus(s_cmd, aClientFD);
			break;

		case IPC_CMD_NAT_SET_NTP_TIME_SYNC:
			UDSSetServerNTPTimeSyncStatus(s_cmd, aClientFD);
			break;

                case IPC_CMD_NAT_GET_REMOTE_DATA_BYTES:
                        UDSGetRemoteDataBytes(s_cmd, aClientFD);
                        break;

                case IPC_CMD_NAT_SET_REMOTE_DATA_BYTES:
                        UDSSetServerdstData(s_cmd, aClientFD);
                        break;

                default:
                        APP_LOG("ProcessServerIpcCmd:", LOG_DEBUG, "Unknown command: %d", s_cmd->cmd);
                        break;
        }

        return SUCCESS;
}

int main(int argc, char **argv )
{
#ifdef PRODUCT_WeMo_NetCam
        if ((program_invocation_short_name = strrchr(argv[0], '/')) != NULL) {
            program_invocation_short_name += 1;
        }
        else {
            program_invocation_short_name = argv[0];
        }
#endif
#ifndef PRODUCT_WeMo_NetCam
        libNvramInit();
#endif

	/*start logger*/
#ifndef PRODUCT_WeMo_LEDLight
	initLogger();
#else
	ulog_init();
#endif

	/*set Signal Handlers*/
	setSignalHandlers();
	/*set Web Session parameters*/
	webAppInit(0);
	/*set log level*/
	nat_set_log_level();
	/*init NAT core*/
	initNATCore();

	SserverTaskData *serverTaskData = NULL;

	serverTaskData = (SserverTaskData*)calloc(1,sizeof(SserverTaskData));
	if(NULL == serverTaskData)
	{
		APP_LOG("UDSServer:", LOG_DEBUG, "serverTaskData calloc failed");
		resetSystem();
	}

	serverTaskData->funptr  = ProcessNatUDSServerIpcCmd;
	snprintf(serverTaskData->UDSSockPath, sizeof(serverTaskData->UDSSockPath), "%s", SOCK_NAT_CLIENT_PATH);

	APP_LOG("UDSServer:", LOG_DEBUG, "Going to start UDS NAT server");
	StartIpcUdsServer(serverTaskData);

	APP_LOG("UDSServer", LOG_DEBUG, "Exiting UDS NAT server");
	return 0;
}
