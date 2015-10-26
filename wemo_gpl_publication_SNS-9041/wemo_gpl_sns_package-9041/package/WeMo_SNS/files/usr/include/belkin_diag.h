/***************************************************************************
*
*
* belkin_diag.h
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
/**
 * @file Belkin diagnostics
 * Central point for configuring diagnostic strumentation for WeMo.
 * Currently used to configure dmalloc library (if enabled).
 */

#ifndef __BELKIN_DIAG
#define __BELKIN_DIAG

#  ifdef DMALLOC
#    include <dmalloc.h>

     /* Output of "dmalloc -n -v -d 0x4e4ed03 -i 10 -l /var/log/dmalloc.log" */
#    define DEFAULT_DM_CONFIG "debug=0x4e4ed03,inter=10,log=/var/log/dmalloc"

#    define init_diagnostics() do {                                  \
       char *dm_config_from_nvram = GetBelkinParameter( "dm_config" );\
       char *dm_config = NULL;                                         \
       if( dm_config_from_nvram && strlen( dm_config_from_nvram )) {    \
         dm_config = dm_config_from_nvram;                               \
         printf( ">>> Note: NVRAM dmalloc config \"%s\".\n", dm_config ); \
       } else {                                                            \
         dm_config = DEFAULT_DM_CONFIG;                                     \
         printf( ">>> Note: Default dmalloc config \"%s\".\n", dm_config );  \
       }                                                                      \
       dmalloc_debug_setup(dm_config);                                         \
     } while(0)
#  else
#    define init_diagnostics() do {} while(0)
#  endif

void *CheckAlloc(void *ptr, size_t size, const char *File, int Line);
void *CheckAllocAndZero(void *ptr, size_t size, const char *File, int Line);
char *CheckedStrdup(const char *String,const char *File, int Line);

#define MALLOC(s)		CheckAlloc(malloc(s),s,__FILE__,__LINE__)
#define ZALLOC(s)		CheckAllocAndZero(malloc(s),s,__FILE__,__LINE__)
#define REALLOC(p,s)	CheckAlloc(realloc(p,s),s,__FILE__,__LINE__)
#define CALLOC(n,s)	CheckAlloc(calloc(n,s),n*s,__FILE__,__LINE__)
#define STRDUP(s)		CheckedStrdup(s,__FILE__,__LINE__)

#endif /* __BELKIN_DIAG */
