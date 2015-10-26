/***************************************************************************
 *
*
* ulog.h
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

/*
===================================================================
    This library provide logging API and routines to filter logs
    based on defined component.subcomponent
===================================================================
*/

#ifndef _ULOG_H_
#define _ULOG_H_

#define ULOG_UNKNOWN -1

typedef enum {
    ULOG_SYSTEM,
    ULOG_CORE,
    ULOG_APP,
    ULOG_LAN,
    ULOG_WLAN,
    ULOG_CONFIG,
    ULOG_SERVICE,
    ULOG_UPNP,
    ULOG_REMOTE,
#ifndef PRODUCT_WeMo_LEDLight
    ULOG_DB,
#else
	ULOG_RULE,
#endif
    ULOG_FIRMWARE_UPDATE
} UCOMP;

#define UL_UNKNOWN -1

typedef enum {
    /* GENERAL */
    UL_INFO,
    UL_DEBUG,
    UL_ERROR,
    UL_STATUS,
    UL_SYSEVENT,
} USUBCOMP;


#ifdef __cplusplus
extern "C" {
#endif
#ifdef PRODUCT_WeMo_LEDLight
/*
 * Procedure     : ulog_init
 * Purpose       : Per process initialization of logging infrastruture
 * Parameters    : None
 * Return Values : None
 * Notes         :
 *    Opens connect to system logget and sets up a prefix string
 *    Current prefix string is "WEMO: "
 */
void ulog_init();
#endif
/*
 * Procedure     : ulog
 * Purpose       : Log a general message to system logger
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     mesg     - message string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.NOTICE facility
 */
void ulog (UCOMP comp, USUBCOMP sub, const char *mesg);

/*
 * Procedure     : ulogf
 * Purpose       : Log a message to system logger with variable arg
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     fmt     - format of message string
 *     ...     - variable args format for message string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.NOTICE facility
 */
void ulogf (UCOMP comp, USUBCOMP sub, const char *fmt, ...);

/*
 * Procedure     : ulog_debug
 * Purpose       : Log a debug message to system logger
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     mesg     - message string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.DEBUG facility
 */
void ulog_debug (UCOMP comp, USUBCOMP sub, const char *mesg);

/*
 * Procedure     : ulog_debug_packet
 * Purpose       : DEBUG_DUMP debug message to /var/log/message with variable arg
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     mesg     - packet message string
 *     length   - packet length
 */
void ulog_debug_packet (UCOMP comp, USUBCOMP sub, const char *mesg, int length);

#ifdef PRODUCT_WeMo_LEDLight
/*
 * Procedure     : ulog_debugf
 * Purpose       : Log debug message to system logger with variable arg
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     fmt     - format of message string
 *     ...     - variable args format for message string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.DEBUG facility
 */
 void ulog_debugf (UCOMP comp, USUBCOMP sub, const char *fmt, ...);

/*
 * Procedure     : ulog_debugf_console
 * Purpose       : Log debug message to console with variable arg
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     fmt     - format of message string
 *     ...     - variable args format for message string
 * Return Values : None
 * Notes         :
 *
 */
void ulog_debugf_console (UCOMP comp, USUBCOMP sub, const char *fmt, ...);

/*
 * Procedure     : ulog_debug_brief
 * Purpose       : Log debug message to system logger with variable arg
 * Parameters    :
 *      tag - brief tag
 *      fmt - format of message string
 *      ... - variable args format for message string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.DEBUG facility
 */
void ulog_debug_brief (const char* tag, const char *fmt, ...);

/*
 * Procedure     : ulog_debug_console
 * Purpose       : Log debug message to console with variable arg
 * Parameters    :
 *      tag - brief tag
 *      fmt - format of message string
 *      ... - variable args format for message string
 */
void ulog_debug_console(const char* tag, const char *fmt, ...);
#endif

/*
 * Procedure     : ulog_error
 * Purpose       : Log an error message to system logger
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     mesg     - message string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.ERROR facility
 */
void ulog_error (UCOMP comp, USUBCOMP sub, const char *mesg);

/*
 * Procedure     : ulog_errorf
 * Purpose       : Log error message to system logger with variable arg
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     mesg     - message string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.ERR facility
 */
void ulog_errorf (UCOMP comp, USUBCOMP sub, const char *fmt, ...);

/*
 * Procedure     : ulog_get_mesgs
 * Purpose       : Retrieve mesgs for given component.subcomponent
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     mesgbuf  - message strings output buffer
 *     size     - size of above buffer
 * Return Values : None
 * Notes         :
 *     mesgbuf will be truncated before mesgs are stored,
 *     and upto allowed size
 */
void ulog_get_mesgs (UCOMP comp, USUBCOMP sub, char *mesgbuf, unsigned int size);

/*
 * Procedure     : ulog_runcmd
 * Purpose       : Log and run command string
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     cmd     - command string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.NOTICE facility
 */
void ulog_runcmd (UCOMP comp, USUBCOMP sub, const char *cmd);

/*
 * Procedure     : ulog_runcmdf
 * Purpose       : Log and run command string with variable arg
 * Parameters    :
 *     UCOMP - component id
 *     USUBCOMP - subcomponent id
 *     mesg     - message string
 * Return Values : None
 * Notes         :
 *     uses syslog LOCAL7.NOTICE facility
 */
int ulog_runcmdf (UCOMP comp, USUBCOMP sub, const char *fmt, ...);

/*
 * Async-signal-safe logging APIs
 *
 * Parameters
 *  fd - output file descriptor (if not sure, use 1. mesg will go to standard output)
 *
 *  variable argument flavor is not available as it is not clear if variadic functions
 *  like vsnprintf() are async signal safe in GNU libc
 *
 *  formatting equivalent to ,
 *     snprintf(sfmt, sizeof(sfmt), "%s.%s %s", getcomp(comp), getsubcomp(sub), mesg);
 */
void ulog_safe (int fd, UCOMP comp, USUBCOMP sub, const char *mesg);

#ifdef __cplusplus
}
#endif

#ifdef PRODUCT_WeMo_LEDLight
//Declare function pointer for using both console and syslog
void (*ulog_debug_brief_output) (const char* tag, const char *fmt, ...);
void (*ulog_debug_output)(UCOMP comp, USUBCOMP sub, const char *fmt, ...);
#endif

#define DEBUG_LOG_HDR   "<%s:%s:%d> [0x%x] "
#define DEBUG_LOG_FTR   __FILE__, __FUNCTION__, __LINE__, pthread_self()
#define DEBUG_DUMP(source, level, mesg, length)     ulog_debug_packet(source, level, mesg, length);

#endif /* _ULOG_H_ */
