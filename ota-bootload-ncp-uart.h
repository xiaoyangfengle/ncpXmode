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


/********************************** Non-Standard Library Head File ***********************************/
//#include "xxx.h"


/********************************** Constant Definition ***********************************/


/********************************** Global Macro ***********************************/


/********************************** Definition For Custom Data Type ***********************************/


/********************************** Function Declaration ***********************************/
bool emAfBootloadWaitChar(uint8_t *data, bool expect, uint8_t expected);

bool emAfBootloadSendByte(uint8_t byte);

/********************************** Class Definition ***********************************/


/********************************** Template ***********************************/


#endif /* __OTA_BOOTLOAD_UART_H__ */


