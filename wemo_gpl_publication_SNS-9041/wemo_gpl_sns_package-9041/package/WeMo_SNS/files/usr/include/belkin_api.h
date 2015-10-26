/***************************************************************************
*
*
* belkin_api.h
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

#ifndef _BELKIN_API_H_
#define _BELKIN_API_H_

#define BELKIN_SUCCESS 0
#define BELKIN_FAILURE -1

#define Gemtek_Success 0
#define Gemtek_Error  -1

extern int nvramget(int argc, char *argv[], char *buffer);
extern int nvramset(int argc, char *argv[]);

char* GetSerialNumber(void);
int SetBelkinParameter(char* ParameterName, char* ParameterValue);
int Firmware_Update(char* File_Path);
int New_Firmware_Update(char* File_Path, int is_signed);
char* GetLanIPAddress(void);
char* GetMACAddress(void);
unsigned long int GetNTPUpdatedTime(void);
int GetRebootStatus(int *Status, unsigned long int *UTC_seconds);
unsigned long int GetUTCTime(void);
int EnableWatchDog(int isEnable, int seconds);
char* GetBelkinParameter(char* ParameterName);
int GetEnableAP(void);
char* UpdateWanDefaultGateway(void);
char* GetWanDefaultGateway(void);
char* GetWanIPAddress(void);
int ResetToFactoryDefault(int runScript);
// Return zero if we set time and left ntpclient as a daemon
int RunNTP();
int SetNTP(char *  ServerIP,int Index, int EnableDaylightSaving);
int SetWiFiLED(int state);
int SyncWatchDogTimer(void);
int UnSetBelkinParameter(char* ParameterName);
int SetSerialNumber(char *serial_number);
int SetMACAddress(char *mac_addr);
int SetTime(unsigned int Seconds, int Index, int EnableDaylightSaving);
int SaveSetting(void);
int EnableMotionSensorDetect(int isEnable);
int SetMotionSensorDelay(int Delay_Seconds, int Sensitivity_Percent);
int SetActivityLED(int state);

/* 
   Test if a carrier is present on the wired Ethernet port (eth2).
   We can't do this in a generic way because the Ralink drivers don't
   implement carrier status.  We use Ralink IOCTL to read the status
   directly from the PHY.
 
   Since no WeMo products to date use any PHYs other than 0 we will
   power them down unconditionally when this routine is called.  The
   Ralink SOC runs HOT, powering down the unused PHYS saves a significant
   amount of power which results in a cooler and more green product.
 
   If bPowerDown is true we will also power down PHY0 if carrier is not
   present when this routine is called, otherwise it is lft powered up.
 
   returns:
      0 - Link Down
      -1 = Link up
      > 0 - error
*/
int WiredEthernetUp(int bPowerDown);
int SetTimeAndTZ(time_t utc,char *Offset,int bIsDst,int bDstSupported);
#endif


