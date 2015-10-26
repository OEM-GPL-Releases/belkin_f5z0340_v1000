/***************************************************************************
*
*
* utils.h
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

#ifndef __PLUGIN_UTILS_H
#define __PLUGIN_UTILS_H

#ifndef _mxml_h_
#include "mxml.h"
#endif

typedef struct {
	const char *Tag;
	void *Value;
	char type;
} XmlElementList;
#ifdef __STR_UTILS__
//convert ":" separated hex string to byte array of size 8
unsigned char* utilsStrToByte8(const char *pstrId, unsigned char* pbyId);
//convert to integer, similar to atoi
int utilsConvertToInt(char* rValue);
//convert to double, similar to atof
double utilsConvertToDouble(char* rValue);
//converting byte array to a double taking care of denominations
double utilsBytesToDouble(unsigned char* pbyId, unsigned char size);
//coverts 8 byte hex array to string
unsigned char *utilsByteToStr(unsigned char *pbyId, char* pstrId);
//coverts hex array to string
unsigned char* utilsByteToStrG(unsigned char *bytebuf, int len, char *s);
//coverts 8 byte hex array of any length to string
unsigned char* utilsByteToStrGS(unsigned char *bytebuf, int len, char *s);
unsigned char* utilsByteToStrAS(unsigned char *bytebuf, int len, char *s);
//convert ":" separated hex string to byte array of size 16
unsigned char* utilsStrToByte16(const char *pstrId, unsigned char* pbyId);
//BCD to decimal format
int BCDtoDecimal(int valBCD);
unsigned char* HexStrToBytesStr(char *str, int len, unsigned char *buffer);
#endif // - end of __STR_UTILS__

int utilsReplaceFileString(FILE* updfp, FILE* origfp, const char *str, char *value);
char *utilsRemDelimitStr(char *src, char *key);
void remoteParseDomainLkup(char *dname, char ipArr[][64], int *num);
int pluginUsleep (unsigned int delay);
void* saveSettingThread(void *arg);
char *convertSSID(char *ssid);
char *convertMonth(int month);
int utilsReplaceString(char* fName, const char *str,char *value);
char convert(unsigned char ch);
int convertInRawBytes(char *ssidStr, char *str);
int isStrPrintAble(char *pInPutStr, int strLen);

#ifndef _OPENWRT_
// system time is correctly set to GMT/UTC on OpenWRT so this unneeded
int computeDstToggleTime(int updateDstime);
#endif

// Wrapper for system command that closes all handles before calling system()
// to prevent the child process from inheriting open file handles and sockets.
int System(const char *command);
#endif
