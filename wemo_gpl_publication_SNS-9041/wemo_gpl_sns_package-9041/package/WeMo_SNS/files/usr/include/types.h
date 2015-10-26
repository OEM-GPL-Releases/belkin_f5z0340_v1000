/***************************************************************************
*
*
* types.h
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

#ifndef __TYPES_H__
#define __TYPES_H__

typedef unsigned char           UINT8;
typedef          char           INT8;
typedef unsigned short          UINT16;
typedef          short          INT16;
typedef unsigned int            UINT32;
typedef          int            INT32;

typedef UINT32                  PTR_INT;

//typedef unsigned char           BOOL;

typedef void                    VOID;
typedef void*                   PVOID;

typedef long int                LONG;
typedef unsigned short int      WORD;

#ifndef INT
typedef int                     INT;
#endif
//typedef unsigned char           BYTE;
typedef unsigned char           *PTR ;

#ifndef UCHAR
typedef unsigned char           UCHAR;
#endif


#ifndef TRUE
#define		TRUE	0x01
#endif

#ifndef FALSE
#define		FALSE	0x00
#endif

#endif
