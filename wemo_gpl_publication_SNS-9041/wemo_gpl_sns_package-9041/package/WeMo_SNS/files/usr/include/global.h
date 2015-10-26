/***************************************************************************
*
*
* global.h
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

#ifndef _GLOBAL_H
#define _GLOBAL_H

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <bits/sockaddr.h>
#include <linux/types.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <errno.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <netinet/ether.h>
#include <asm/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>

#define SUCCESS 0


#define FAILURE -1
#define INVALID_PARAMS    (FAILURE-1)

#define TURN_ON 1
#define TURN_OFF 0

#define INTERFACE_AP		"ra0"
#define INTERFACE_CLIENT	"apcli0"
#define INTERFACE_INSTA		"ra1"
#define INTERFACE_BRIDGE	"ra2"
#define INTERFACE_BR		"br0"

#define WAN_IP_ADR    "10.22.22.1"

#define WIFI_BRIDGE_LIST	"BridgeList"
#define WIFI_CLIENT_SSID	"ClientSSID"
#define WIFI_CLIENT_PASS	"ClientPass"
#define WIFI_CLIENT_AUTH	"ClientAuth"
#define WIFI_CLIENT_ENCRYP	"ClientEncryp"
#define WIFI_AP_CHAN		"APChannel"
#define WIFI_ROUTER_MAC		"RouterMac"
#define WIFI_ROUTER_SSID	"RouterSsid"
#define SYNCTIME_LASTTIMEZONE	"LastTimeZone"
#define SYNCTIME_DSTSUPPORT     "DstSupportFlag"
#define LASTDSTENABLE		"LastDstEnable"
#define NOTIFICATION_VALUE      "NotificationFlag"
#define LAST_PUSH_TIME		"LastPushTime"
#define NAT_TYPE        "nat_type"
#define WD_DEFAULT_PUSH_TIME	21600 //6hours

#define CHAN_HIDDEN_NW 255
#define SETTIME_SEC "settime_sec"
#define DEVICE_NETWORK_STATE "dev_network_state"
#define APP_WD_STATUS "app_wd_status"
#define MAX_APSSID_LEN		30
#endif //_GLOBAL_H
