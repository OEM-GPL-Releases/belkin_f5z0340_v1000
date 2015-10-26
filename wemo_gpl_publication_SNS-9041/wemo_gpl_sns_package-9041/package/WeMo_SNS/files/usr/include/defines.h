/***************************************************************************
*
*
* defines.h
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

#ifndef __DEFINES_H
#define __DEFINES_H

/* */
#define SIZE_1B		    1
#define SIZE_2B		    2
#define SIZE_4B		    4
#define SIZE_8B		    8
#define SIZE_10B	    10
#define SIZE_16B	    16
#define SIZE_20B	    20
#define SIZE_32B	    32
#define SIZE_50B	    50
#define SIZE_64B	    64
#define SIZE_100B	    100
#define SIZE_128B	    128
#define SIZE_256B	    256
#define SIZE_512B	    512
#define SIZE_768B	    768
#define SIZE_1024B	    1024
#define SIZE_2048B	    2048
#define SIZE_4096B          4096
#define SIZE_8192B	    8192

/* Macros for delay in seconds */
#define DELAY_1SEC	    1
#define DELAY_3SEC	    3
#define DELAY_5SEC	    5
#define DELAY_10SEC	    10
#define DELAY_20SEC	    20
#define DELAY_60SEC	    60

//
#define P_MIN_THREADS			2
#define P_MAX_THREADS 			6 
#define P_THREAD_STACK_SIZE 		0
#define P_JOBS_PER_THREAD 		10
#define P_THREAD_IDLE_TIME 		5000
#define P_MAX_JOBS_TOTAL 		20

#define P_THREAD_LOW_PRIORITY		0
#define P_THREAD_MED_PRIORITY		1
#define	P_THREAD_HIGH_PRIORITY		2

//
#define MAX_RVAL_LEN			SIZE_16B
#define MAX_DVAL_LEN 			SIZE_32B
#define MAX_LVALUE_LEN 			SIZE_64B
#define MAX_RVALUE_LEN 			SIZE_64B
#define MAX_DBVAL_LEN			SIZE_128B
#define MAX_FILE_LINE			SIZE_256B
#define MAX_KEY_LEN			SIZE_256B
#define MAX_BUF_LEN   			SIZE_1024B
#define MAX_DATA_LEN			SIZE_1024B
#define MAX_MMSG_LEN                    SIZE_1024B
#define MAX_PKEY_LEN			SIZE_50B
#define MAX_RES_LEN			SIZE_2B
#define MAX_MAC_LEN			SIZE_20B
#define MAX_ESSID_LEN			SIZE_64B
#define MAX_ESSID_SPLCHAR_LEN	SIZE_128B
#define MAX_IP_COUNT			SIZE_10B
#define MAX_DESC_LEN			180
#define	MAX_SKEY_LEN			100
#define MAX_MIN_LEN			1
#define MAX_OCT_LEN			SIZE_8B
#define MAX_RESP_LEN			SIZE_512B
#define MAX_FW_URL_SIGN_LEN		SIZE_50B
#define LOGS_BUFF_LEN			SIZE_256B
#define PASSWORD_MAX_LEN		SIZE_128B
#define MAX_FW_URL_LEN			SIZE_256B	

#define THREAD_WAIT_TIME		10

#define PLUGIN_SUCCESS 			0
#define PLUGIN_FAILURE 			(-1)
#define ERROR_BASE 			(-10)
#define ERROR_NO_HANDLER 		(ERROR_BASE-2)

#define	MAX_DEV_UDID_LEN		SIZE_64B

/* Product Serial Number Schema
 * Serial Number    - Val-Index- Description
 *
 * 221238K01FFFFF   - 22 - 0,1 - Supplier Id
 *                  - 12 - 2,3 - Year of Mfg
 *                  - 38 - 4,5 - Week of Mfg
 *                  - K  - 6   - Product Code
 *                  - 01 - 7,8 - Product sub code
 *                  - FFFFF - 9,10,11,12,13 - Unique Sequence Identifier
 */

#define SUPPLIER_ID_INDEX                           0   /*length=2 digits*/
#define YEAR_OF_MFG_INDEX                           2   /*length=2 digits*/
#define WEEK_OF_MFG_INDEX                           4   /*length=2 digits*/
#define PRODCT_TYPE_INDEX                           6   /*length=1 digits*/
#define PRODCT_SUB_TYPE_INDEX                       7   /*length=2 digits*/
#define UNIQUE_SEQ_NO_INDEX                         9   /*length=5 digits*/

/* 
   Safe versions of usual string functions that guard against buffer overruns.
 
   A common design pattern that occurs frequently in WeMo sources code is to
   copy strings into local and global buffers for various reasons.  This pattern
   looks similar to this:
 
			char Buffer[10];
   		strncpy(Buffer,"test",sizeof(Buffer));
 
   The function strncpy is used instead of strcpy to prevent an buffer overflow
   when an unexpected input is encountered.  
 
	The hope is that these macros will make coding less error prone than using
	strncpy(), strncat(), etc directly while also making the code more readable.
 
	WARNING:  Since these macros are implemented using the sizeof operator
   the destination should not be a bare character pointer to avoid unexpected
	results.
 
   For example:
		// Expected usage:
			char Buffer[10];
			SAFE_STRCPY(Buffer,"test");

		// problematic usage:
			char *Buffer = malloc(10);
   		SAFE_STRCPY(Buffer,"test");
 
   A bonus of using these macros is that truncation is automatically detected
	and reported in debug builds.
*/
#if !defined(BOARD_PVT) || defined(DEBUG)
char *safe_strcpy(char *Dst,const char *Src,int DstLen,const char *Func,int Line);
char *safe_strcat(char *Dst,const char *Src,int DstLen,const char *Func,int Line);

#define SAFE_STRCPY(d,s)	safe_strcpy(d,s,sizeof(d),__FILE__,__LINE__)
#define SAFE_STRCAT(d,s)	safe_strcat(d,s,sizeof(d),__FILE__,__LINE__)
#else
#define SAFE_STRCPY(d,s)	do {strncpy(d,s,sizeof(d)-1);	d[sizeof(d)-1] = 0;} while(0)
#define SAFE_STRCAT(d,s)	strncat(d,s,sizeof(d)-strlen(d)-1)
#endif

#endif // __DEFINES_H
