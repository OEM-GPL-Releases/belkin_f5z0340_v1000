/***************************************************************************
*
*
* osUtils.h
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

#ifndef __PLUGIN_OSUTILS_H
#define __PLUGIN_OSUTILS_H


/*
 * NOTE:::: The below interfaces currently supports operations on UDP 
 * sockets only
*/

//Global or local sockets
#define GLOBAL 0
#define OTHER 1

//Connection Modes
#define BROADCAST 0
#define UNICAST 1
#define MULTICAST 2 /* We won't need this rightnow. just a place holder*/

//Connection Types, will use UDP here, TCP is a place holder for future
#define TCP_CONN 0
#define UDP_CONN 1

/* 
 *creates socket and return socket fd if successfull or some error code
 * if failed
*/
int osUtilsCreateSocket(int connMode, int connType, int fdType);

//Socket binds with the provided ipAddr and port
int osUtilsBindSocket(int sockFd, int connMode, char* ipAddr, int ipPort);

//Read from the created and bind socket
int	osUtilsReadSocket(int sockFd, char* dataBuffer, int timeout);

//write to the remote socket
int osUtilsWriteSocket(int sockFd, int connMode, char* remoteAddr, int remotePort, char* writeBuffer, int len);

//Macro to check mutex operation results for debugging
#define checkLockResults(string, val) {         \
 if (val) {                                     \
   printf("Failed with %d at %s", val, string); \
   exit(1);                                     \
 }                                              \
}
//Create lock
int osUtilsCreateLock(pthread_mutex_t *lock);
//Destroy lock
int osUtilsDestroyLock(pthread_mutex_t *lock);	
//Get lock
int osUtilsGetLock(pthread_mutex_t *lock);
//Try lock
int osUtilsTryLock(pthread_mutex_t *theLock);
//Release lock
int osUtilsReleaseLock (pthread_mutex_t *lock);

#endif
