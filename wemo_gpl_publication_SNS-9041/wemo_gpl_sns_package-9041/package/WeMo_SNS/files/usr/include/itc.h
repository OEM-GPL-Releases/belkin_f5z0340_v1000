/***************************************************************************
*
*
* itc.h
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

#ifndef __MESSAGE_H__
#define __MESSAGE_H__


#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>

typedef unsigned int MsgID;

/*
 * 	Message struct
 * 
 * 
 * 
 * 
 * 
 *******************/
struct __message
{
  MsgID ID;
  void* message;
};

typedef struct __message Message, *pMessage;

/***
 * Queue struct
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 ****************************************************/
struct __node
{
  pMessage message;
  struct __node* next;
};

typedef struct __node Node, *pNode;

struct __queue
{
    pNode header;
    pNode tailer;
};

typedef struct __queue Queue, *pQueue;

struct __IPC_IF
{
  pthread_mutex_t 	mutex;
  sem_t   		semap;
  pQueue		recvQueue;
};

typedef struct __IPC_IF	ipcIF;


#define		UPNP_MESSAGE_BASE 100
#define		UPNP_FIRMWARE_UPDATE_IND UPNP_MESSAGE_BASE + 1


#define		BTN_MESSAGE_BASE 200
#define		BTN_MESSAGE_SHORT_PRESS_IND 		BTN_MESSAGE_BASE + 1
#define		BTN_MESSAGE_LONG_PRESS_IND  		BTN_MESSAGE_BASE + 2
#define		BTN_MESSAGE_ON_IND					BTN_MESSAGE_BASE + 3
#define		BTN_MESSAGE_OFF_IND					BTN_MESSAGE_BASE + 4
#define		UPNP_MESSAGE_ON_IND					BTN_MESSAGE_BASE + 5
#define		UPNP_MESSAGE_OFF_IND				BTN_MESSAGE_BASE + 6
#define		UPNP_ACTION_MESSAGE_OFF_IND			BTN_MESSAGE_BASE + 7
#define		UPNP_ACTION_MESSAGE_ON_IND			BTN_MESSAGE_BASE + 8
#define		RULE_MESSAGE_OFF_IND				BTN_MESSAGE_BASE + 9
#define		RULE_MESSAGE_ON_IND					BTN_MESSAGE_BASE + 10
#define		RULE_MESSAGE_RESTART_REQ			BTN_MESSAGE_BASE + 11
#define		LOCAL_MESSAGE_ON_IND				BTN_MESSAGE_BASE + 12
#define		LOCAL_MESSAGE_OFF_IND				BTN_MESSAGE_BASE + 13
#define		UPNP_MESSAGE_SBY_IND				BTN_MESSAGE_BASE + 14
#define		UPNP_MESSAGE_PWR_IND				BTN_MESSAGE_BASE + 15
#define		UPNP_MESSAGE_PWRTHRES_IND			BTN_MESSAGE_BASE + 16
#define		UPNP_MESSAGE_ENERGY_COST_CHANGE_IND		BTN_MESSAGE_BASE + 17
#define		BTN_MESSAGE_SBY_IND				BTN_MESSAGE_BASE + 18
#define		UPNP_MESSAGE_DATA_EXPORT			BTN_MESSAGE_BASE + 19
#ifdef	PRODUCT_WeMo_Light
#define		NIGHTLIGHT_DIMMING_MESSAGE_REBOOT		BTN_MESSAGE_BASE + 20
#endif





#define		NETWORK_MESSAGE_BASE 300
#define		NETWORK_INTERNET_CONNECTED			NETWORK_MESSAGE_BASE + 1
#define		NETWORK_AP_OPEN_UPNP				NETWORK_MESSAGE_BASE + 2


#define		META_MESSAGE_BASE 				400
#define		META_SAVE_DATA 					META_MESSAGE_BASE + 1
#define		META_SOFT_RESET 				META_MESSAGE_BASE + 2
#define		META_FULL_RESET 				META_MESSAGE_BASE + 3
#define		META_FIRMWARE_UPDATE				META_MESSAGE_BASE + 4
#define		META_REMOTE_RESET				META_MESSAGE_BASE + 5
#ifdef PRODUCT_WeMo_Insight
#define		META_CLEAR_USAGE				META_MESSAGE_BASE + 6
#endif
#define		META_WIFI_SETTING_RESET				META_MESSAGE_BASE + 7
#define		META_CONTROLLEE_DEVICE_STOP			META_MESSAGE_BASE + 8




//-----------------------------------------------
typedef enum{
  PLUGIN_E_MAIN_THREAD = 0x00,
  PLUGIN_E_RELAY_THREAD,
  PLUGIN_E_BUTTON_THREAD,
  PLUGIN_E_CLOUD_THREAD, 
  MAX_THREAD_NUMBER /** Alway the last one**/
  
} PLUGIN_E_THREAD_INDEX;


extern ipcIF ipcIFArray[MAX_THREAD_NUMBER];


void initIPC();
void SendMessage2App(pMessage msg);
void SendMessage(PLUGIN_E_THREAD_INDEX threadIndex, pMessage msg);

pNode readMessage(PLUGIN_E_THREAD_INDEX threadIndex);

pMessage createMessage(int ID, void* payload, int size);
int openApUpnp(void);

#endif	//-__MESSAGE_H__
