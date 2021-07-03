/***************************************************************************//**
 * @file
 * @brief Routines for sending data via xmodem
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
#include "ota-bootload-xmodem.h"
#include "crc16.h"
//------------------------------------------------------------------------------

#define SOH   (0x01)
#define EOT   (0x04)
#define ACK   (0x06)
#define NAK   (0x15)
#define CAN   (0x18)

#define DATA_SIZE        (128)
#define HEADER_SIZE      (3)
#define FOOTER_SIZE      (2)
#define FULL_BLOCK_SIZE  (HEADER_SIZE + DATA_SIZE + FOOTER_SIZE)

#define CONTROL_OFFSET      (0)
#define BLOCK_NUM_OFFSET    (1)
#define BLOCK_COMP_OFFSET   (2)
#define DATA_OFFSET         (3)
#define CRC_H_OFFSET        (131)
#define CRC_L_OFFSET        (132)

#define UNINITIALIZED       (0)
#define START_TRANSMISSION  (1)
#define SENDING             (2)


static uint8_t state = UNINITIALIZED;
static uint8_t buffFinger;
static uint8_t dataBuff[DATA_SIZE];
static int blockNum;

/**
 * @brief Returns the low byte of the 16-bit value \c n as an \c uint8_t.
 */
#define LOW_BYTE(n)                     ((uint8_t)((n) & 0xFF))

/**
 * @brief Returns the high byte of the 16-bit value \c n as an \c uint8_t.
 */
#define HIGH_BYTE(n)                    ((uint8_t)(LOW_BYTE((n) >> 8)))


/* The maximum number of times XMODEM tries to send a packet / control byte */
#define MAX_RETRANSMIT (8)



//------------------------------------------------------------------------------

uint16_t halCommonCrc16(uint8_t byte, uint16_t crc)
{
    crc = (crc >> 8) | (crc << 8);
    crc ^= byte;
    crc ^= (crc & 0xff) >> 4;
    crc ^= (crc << 8) << 4;

    crc ^= ((uint8_t) ((uint8_t) ((uint8_t) (crc & 0xff)) << 5))
           | ((uint16_t) ((uint8_t) ((uint8_t) (crc & 0xff)) >> 3) << 8);

    return crc;
}


static bool sendBlock(uint8_t blockNum, const uint8_t *data)
{
    int i;
    int retry = 5;
    uint16_t crc = 0;
    uint8_t status = NAK;
    uint8_t fullBlock[FULL_BLOCK_SIZE];

    fullBlock[CONTROL_OFFSET] = SOH;
    fullBlock[BLOCK_NUM_OFFSET] = blockNum;
    fullBlock[BLOCK_COMP_OFFSET] = ~blockNum;

    for (i = 0; i < DATA_SIZE; i++)
    {
        crc = halCommonCrc16(data[i], crc); //crc
        fullBlock[DATA_OFFSET + i] = data[i];
    }

    fullBlock[CRC_H_OFFSET] = HIGH_BYTE(crc);
    fullBlock[CRC_L_OFFSET] = LOW_BYTE(crc);

    while ( (status != ACK) && (retry > 0) )
    {
        //print "block %d (" % num,
        //for i in range(0,len(data)):
        //  print "%02x" % ord(data[i]),
        if (!emAfBootloadSendData(fullBlock, FULL_BLOCK_SIZE))
        {
            printf("sendBlock() fail 1\n");
            return false;
        }
        retry--;
        if (!emAfBootloadWaitChar(&status, false, 0))
        {
            printf("sendBlock() fail 2\n");
            return false;
        }
        while ( status == 'C' )
        {
            // may have leftover C characters from start of transmission
            if (!emAfBootloadWaitChar(&status, false, 0))
            {
                printf("sendBlock() fail 3\n");
                return false;
            }
        }
    }

    return (status == ACK);
}


/**
 * Try to send an XMODEM packet for MAX_RETRANSMIT times.
 *
 * This function sends an XMODEM packet and checks if an ACK is received. If no
 * ACK is received, the packet is retransmitted. This is done until
 * 'MAX_RETRANSMIT' is exceeded.
 *
 * @param[in] data     The payload of the packet. Must not be null.
 * @param[in] data_len The length of the payload. Must be at most 128 bytes.
 *		       If less, (random) padding is automatically added.
 * @param[in] pkt_no   The packet sequence number.
 *
 * @return Exit status.
 * @retval 0  Success, the packet has been transmitted and an ACK received.
 * @retval -1 Error, retransmit count exceeded.
 */
static int xmodem_send_pkt_with_retry(const uint8_t *data, size_t data_len,
				      uint8_t pkt_no)
{
	uint8_t retransmit = MAX_RETRANSMIT;
	uint8_t rsp;

	printf("xmodem_send_pkt_with_retry(): pkt_no: %d\n", pkt_no);
	while (retransmit--) {
		xmodem_send_pkt(data, data_len, pkt_no);
		rsp = ERR;
		xmodem_io_getc(&rsp);
		if (rsp == ACK) {
			printf("xmodem_send_pkt_with_retry(): done\n");
			return 0;
		}
		printf("xmodem_send_pkt_with_retry(): failure (%d)\n",
		       retransmit);
	}

	return -1;
}




void emAfInitXmodemState(bool startImmediately)
{
    if (startImmediately)
    {
        // skip checking for 'C' characters
        state = SENDING;
    }
    else
    {
        state = START_TRANSMISSION;
    }

    buffFinger = 0;
    blockNum = 1;
}

bool emAfSendXmodemData(const uint8_t *data, int length, bool finished)
{
    printf("emAfSendXmodemData length:%d \n",length);
    uint8_t rxData;
    int i;

    if (state == START_TRANSMISSION)
    {
        sleep(1);
        if (emAfBootloadWaitChar(&rxData, true, 'C'))
        {
            printf("sending\n");
            state = SENDING;
        }
        else
        {
            printf("NoC\n");
            return false;
        }
    }

    if (state == SENDING)
    {
        for (i = 0; i < length; i++)
        {
            dataBuff[buffFinger++] = data[i];
            if (buffFinger >= DATA_SIZE)
            {
                usleep(100);
                printf("%d %d\n", blockNum,i);        //fsync();
                if (!sendBlock(blockNum, dataBuff))
                {
                    printf("sendblock err\n");
                    // emberAfCoreFlush();
                    return false;
                }
                buffFinger = 0;
                blockNum++;
            }
        }
        printf("buffFinger :%d\n",buffFinger);
        printf("finished :%d\n",finished);
        if ( finished )
        {
            if ( buffFinger != 0)
            {
                // pad and send final block
                bool result;
                while (buffFinger < DATA_SIZE)
                {
                    dataBuff[buffFinger++] = 0xFF;
                }
                printf("final block %d\n", blockNum);
                result = sendBlock(blockNum, dataBuff);
                if (!result)
                {
                    return false;
                }
            }
            //printf("EOT\n", blockNum);
            emAfBootloadSendByte(EOT);
            if (!emAfBootloadWaitChar(&rxData, true, ACK))
            {
                printf("NoEOTAck\n");
                return false;
            }
        }
    }
    else
    {
        printf("badstate\n");
        return false;
    }
    return true;
}