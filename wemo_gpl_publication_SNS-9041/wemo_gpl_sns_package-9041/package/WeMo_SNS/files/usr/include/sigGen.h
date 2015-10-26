/***************************************************************************
*
*
* sigGen.h
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

#ifndef __SIGGEN__
#define __SIGGEN__

#include "defines.h"

typedef struct auth_sign {
	char mac[SIZE_32B];
	char serial[SIZE_32B];
	int expiry;
	char signature[SIZE_256B];
	char ssign[SIZE_128B];
}authSign;

typedef struct sdu_auth_sign {
	char udid[SIZE_64B];
	int expiry;
	char signature[SIZE_256B];
	char ssign[SIZE_128B];
}sduauthSign;

authSign* createAuthSignature(char* mac, char* serial, char* key);
authSign* createAuthSignatureNoExp(char* mac, char* serial, char* key);
char* encryptStringHmacSha(char* str, char* key);
#endif
