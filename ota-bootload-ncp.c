/***************************************************************************//**
 * @file
 * @brief Routines for bootloading an NCP UART.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "ota-bootload-ncp.h"
//#include "ota-bootload-xmodem.h"
#include "xmodem.h"

//------------------------------------------------------------------------------
// Globals

// Xmodem requires all blocks be 128 bytes in size
#define TRANSFER_BLOCK_SIZE 128


// We arbitrarily chose 5% as the minimum update amount when we are
// transfering the file to the NCP.  This provides a good amount of feedback
// during the process but not too much.
#define BOOTLOAD_PERCENTAGE_UPDATE  5


#define START_IMMEDIATELY false

static uint8_t gbl_file[] = "xncp-led_BRD4180A.gbl";

//------------------------------------------------------------------------------
// Forward Declarations

static bool transferFile();

//------------------------------------------------------------------------------

// This hands control of the entire application to this code to perform
// the bootloading.  It will not return until the bootload has completed
// or it has failed.

bool emberAfOtaBootloadCallback()
{
    bool success = true;

    printf("Starting bootloader communications.\r\n");

    if (!emAfStartNcpBootloaderCommunications())
    {
        printf("Failed to start bootloading communications\n");
        return -1;
    }
    else
    {
        // send all images with matching tag Id
        success = transferFile(); //????????????

        // Regardless of success or failure we reboot the NCP in hopes
        // of returning the system back to its previous state.

        // Use &= here to preserve the possible failed status returned
        // by transferFile()
        success &= emAfRebootNcpAfterBootload();
    }
    return success;
}
static int read_data(char **result,char *fileName,int *length)
{
    struct stat fileInfo;
    FILE *filePointer;
    char *fileDate;

    if((fileName==NULL)||(result==NULL))
    {
        printf("read_data fail\n");
        return 0;
    }

    if(!(filePointer=fopen(fileName,"rb ")))
    {
        printf("read_data open file fail\n");
        return 0;
    }

    stat(fileName,&fileInfo);
    //printf("read_data  file size %d\n",fileInfo.st_size);
    *length = fileInfo.st_size;
    fileDate = malloc(sizeof(char)*(fileInfo.st_size+1));
    fread(fileDate,sizeof(char),fileInfo.st_size,filePointer);
    fclose(filePointer);
    fileDate[fileInfo.st_size]=0;

    // printf("\n%s\n",fileDate);
    *result=fileDate;
    return 1;
}
static bool transferFile()
{
    printf("Transferring GBL file to NCP..\r\n");
	
    char  * buf = NULL;
    int length = 0;
	
    if(read_data(&buf,gbl_file,&length) == 0)
    {
        printf("read_data fail \n");
        return false;
    }
    else
    {
        printf("read_data success length:%d\n",length);
    }


	if(xmodem_transmit_package(buf, length))
	{
        printf("xmodem trans err,exit");
		return false;
	}
    printf("Transfer completed successfully.");


//    emAfInitXmodemState(START_IMMEDIATELY);

//    if (!emAfSendXmodemData(buf,length,true))
//    {
//        // finish?
//        printf("Failed to send data to NCP.");
//        //emberAfCoreFlush();
//        return false;
//    }
    return true;
}
