/*********************************************************************
 * All Right Reserved: Copyright (c) 2021-2022  AIOT STUDIO Company. All rights reserved.
 * System Name: 
 * File Name: ota-bootload-ncp-uart.h
 * Summary: describe the content include module,functions
 * Version: 
 * Author: sven yang
 * Designe Date: 2021-07-03 11:23
 * Modification Record: 
 * Date          Versoin          Modify by           Modify summary
**********************************************************************/




#ifndef __OTA_BOOTLOAD_UART_H__
#define __OTA_BOOTLOAD_UART_H__
/********************************** Other Conditional Compilation Option ***********************************/


/********************************** Standard Library Head File ***********************************/
#include <stdlib.h>

#include <stdio.h>

#include <string.h>

#include <errno.h>

#include <unistd.h>

#include <sys/types.h>

#include <sys/stat.h>

#include <fcntl.h>

#include <termios.h>

#include <unistd.h>

#include <stdbool.h>

#include "stdint.h"

/********************************** Non-Standard Library Head File ***********************************/
//#include "xxx.h"


/********************************** Constant Definition ***********************************/


/********************************** Global Macro ***********************************/


/********************************** Definition For Custom Data Type ***********************************/


// Define CRTSCTS for both ingoing and outgoing hardware flow control
// Try to resolve the numerous aliases for the bit flags
#if defined(CCTS_OFLOW) && defined(CRTS_IFLOW) && !defined(__NetBSD__)
  #undef CRTSCTS
  #define CRTSCTS (CCTS_OFLOW | CRTS_IFLOW)
#endif
#if defined(CTSFLOW) && defined(RTSFLOW)
  #undef CRTSCTS
  #define CRTSCTS (CTSFLOW | RTSFLOW)
#endif
#ifdef CNEW_RTSCTS
  #undef CRSTCTS
  #define CRTSCTS CNEW_RTSCTS
#endif
#ifndef CRTSCTS
  #define CRTSCTS 0
#endif



// Define the termios bit fields modified by ezspSerialInit
// (CREAD is omitted as it is often not supported by modern hardware)
#define CFLAG_MASK (CLOCAL | CSIZE | PARENB | HUPCL | CRTSCTS)
#define IFLAG_MASK (IXON | IXOFF | IXANY | BRKINT | INLCR | IGNCR | ICRNL \
                    | INPCK | ISTRIP | IMAXBEL)
#define LFLAG_MASK (ICANON | ECHO | IEXTEN | ISIG)
#define OFLAG_MASK (OPOST)






/********************************** Function Declaration ***********************************/
bool emAfBootloadWaitChar(uint8_t *data, bool expect, uint8_t expected);

bool emAfBootloadSendByte(uint8_t byte);

/********************************** Class Definition ***********************************/


/********************************** Template ***********************************/


#endif /* __OTA_BOOTLOAD_UART_H__ */


