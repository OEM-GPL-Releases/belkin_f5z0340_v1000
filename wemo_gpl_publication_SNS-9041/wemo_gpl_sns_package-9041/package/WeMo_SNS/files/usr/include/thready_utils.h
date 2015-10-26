/***************************************************************************
*
*
* thread_utils.h
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

#ifndef __THREAD_UTILS_H
#define __THREAD_UTILS_H

/** The pthread standard states that thread names are never more than
 * 16 characters long.  Oddly, there is no pre-defined constant with
 * this value so we make our own here.
 */
#define PTHREAD_NAME_SIZE (16)

/** @brief Set calling thread's name.
 * @param name Null-terminated string holding name.  Only
 * PTHREAD_NAME_SIZE bytes will be stored.  Longer strings will be
 * truncated.  Even when truncated there will always be a terminating
 * NULL.
 */
extern void tu_set_my_thread_name( const char* name );

/** @brief Get name of calling thread.
 * @return NULL terminated string containing thread name.  Storage is
 * managed automaticall.  DO NOT FREE.
 */
extern const char *tu_get_my_thread_name( void );

#endif

/* -------------------- End of file -------------------- */
