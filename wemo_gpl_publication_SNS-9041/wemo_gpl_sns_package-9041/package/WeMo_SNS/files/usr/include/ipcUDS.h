/***************************************************************************
*
*
* ipcUDS.h
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

#ifndef __IPC_UDS__
#define __IPC_UDS__

#include "types.h"
#include "defines.h"

#define ERROR 				-1

#ifdef PRODUCT_WeMo_Baby
#define SOCK_BABY_PATH			"/tmp/ipcudssocket.sock"
#endif /* #ifdef PRODUCT_WeMo_Baby */
#define SOCK_NAT_CLIENT_PATH 		"/tmp/ipcNatAppUdsSocket.sock"
#if defined(PRODUCT_WeMo_LEDLight) || defined(PRODUCT_WeMo_SNS)
#define MAX_SEND_BUF_LEN	1537
#else
#define MAX_SEND_BUF_LEN	1025
#endif
/** Enumerated type of end node command type*/
typedef enum{
	IPC_CMD_INVALID = -1,
#ifdef PRODUCT_WeMo_Baby
	IPC_CMD_GET_NETWORK_STATE,	
	IPC_CMD_GET_SERVER_ENVIRONMENT,	
	IPC_CMD_GET_SIGNAL_STRENGTH,
	IPC_CMD_PUNCH_APP_WATCHDOG_STATE,
#endif /* #ifdef PRODUCT_WeMo_Baby */
	IPC_CMD_NAT_START_REMOTE_DATA_PATH,
	IPC_CMD_NAT_REMOTE_DATA,
	IPC_CMD_NAT_REMOTE_DATA_REPLY,
	IPC_CMD_NAT_GET_REMOTE_DATA_BYTES,
	IPC_CMD_NAT_SET_REMOTE_DATA_BYTES,
	IPC_CMD_NAT_INITIALIZED_STATE,
	IPC_CMD_NAT_INVOKE_DESTROY,
	IPC_CMD_NAT_TRIGGER,
	IPC_CMD_MONITOR_NAT_CLIENT_STATUS,
	IPC_CMD_NAT_REINIT,
	IPC_CMD_NAT_SET_REMOTE_ENABLE_STATE,
	IPC_CMD_NAT_SET_NAT_DATA,
	IPC_CMD_NAT_SET_CLIENT_STATE,
	IPC_CMD_NAT_DST_DATA,
	IPC_CMD_NAT_SET_TURN_SERVER_IP_ADDRESS,
	IPC_CMD_NAT_REMOTE_ACCESS_INIT,
	IPC_CMD_NAT_SET_ICE_RUNNING_STATUS,
	IPC_CMD_NAT_SET_NTP_TIME_SYNC
}IPCCOMMAND;

// GENERIC DATA STRUCTURE
#define IPC_CMD_HDR_LEN     (sizeof(IPCCOMMAND) + sizeof(UINT32))

/** Structure representing IPC action command request */
typedef struct IPC_command{
        IPCCOMMAND cmd;                                 /* command name*/
        UINT32 arg_length;                              /* length of payload*/
	PVOID arg;					/* Pointer to the actual data of command */
}IPCCommand, *pIPCCommand;

typedef int (*entryFunPtr)(IPCCommand* cmd, int clientFD);

typedef struct serverTaskData
{
	entryFunPtr funptr;
	char UDSSockPath[SIZE_64B];
}SserverTaskData;

typedef struct childFDTaskData
{
	entryFunPtr funptr;
	int fd;
}SChildFDTaskData;

/** Structure representing IPC structure for command data type string */
typedef struct CommandStringData{
	INT8 *strPtr;
}IPCCommandStringData, *pIPCCommandStringData;

/** Structure representing IPC response for command data type integer */
typedef struct CommandIntData{
	INT32 value;
}IPCCommandIntData, *pIPCCommandIntData;

/** Structure representing IPC structure for NAT INIT data */
typedef struct NATInitData{
	char serialNo[SIZE_64B];
	char WiFiMacAddress[SIZE_64B];
	char pluginPrivatekey[MAX_PKEY_LEN];
	char restoreState[MAX_RES_LEN];
	char clientType[SIZE_128B];
}IPCNATInitData, *pIPCNATInitData;

/** Structure representing IPC structure for DST data */
typedef struct SetDstData{
	INT32 idx;
	INT8  timezone[SIZE_16B];
	INT32 dstEnable;
}IPCSetDstData, *pIPCSetDstData;

#ifdef PRODUCT_WeMo_Baby
/** Structure representing IPC response for Network state */
typedef struct Network_State_response{
	INT32 value;
}IPCNetworkStateResp, *pIPCNetworkStateResp;

/** Structure representing IPC response for Server Environment */
typedef struct Server_EVN_response{
	INT32 value;
}IPCServerEnvResp, *pIPCServerEnvResp;

/** Structure representing IPC response for Signal strength */
typedef struct Signal_Strenght_response{
        INT32 value;
}IPCSignalStrengthResp, *pIPCSignalStrengthResp;

/** Structure representing IPC response for WachDog Status */
typedef struct WachDog_Status_response{
        INT32 value;
}IPCWachDogStatusResp, *pIPCWachDogStatusResp;

int ProcessServerIpcCmd(IPCCommand* s_cmd, int aChildFD);
int SendNetworkState(IPCCommand *sa_cmd, int aChildFD);
int SendServerEnvironmnet(IPCCommand *sa_cmd, int aChildFD);
int SendSignalStrength(IPCCommand *sa_cmd, int aChildFD);
int SendWachDogStatus(IPCCommand *sa_cmd, int aChildFD);

/*API For Evos*/
int GetNetworkState();
int GetServerEnv();
int GetSignalStrength();
int PunchAppWatchDogState();
#endif /* #ifdef PRODUCT_WeMo_Baby */

/*IPC UDS Server API's*/
void* ChildFDTask(void *pClientFD);
int StartIpcUdsServer(SserverTaskData *serverTaskData);
int IpcUdsSend(pIPCCommand sa_resp, int aChildFD);

/*IPC UDS Client API's*/
pIPCCommand ProcessClientIPCCommand(pIPCCommand psCmd, char *UDSSocketPath);
int SendIPCCommand(pIPCCommand psCmd, int aClientFD);
int GetIPCResponse(pIPCCommand respBuffer, int aClientFD);
int StartIPCClient(char *UDSSocketPath);

#endif /*__IPC_UDS__*/



