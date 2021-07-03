
#include <sys/select.h>
#include "ota-bootload-ncp.h"
#include "ota-bootload-ncp-uart.h"

//------------------------------------------------------------------------------
// Globals

#define NULL_FILE_DESCRIPTOR  (-1)
static const char returnChar = 0x0A; // line feed
static const char runChar =  '2';
static const char beginDownload = '1';
static const char xmodemStartChar = 'C';

#define BOOTLOADER_DELAY  4     // seconds
#define MAX_ERROR_LENGTH  255

// This is setup to be greater than the time between 'C' characters
// spit out by the bootloader to indicate that it is ready for an Xmodem
// transfer.
#define READ_TIMEOUT_SEC  3

#define MAX_BYTES_WITHOUT_PROMPT 200

// We store the last few bytes so that we can check whether we received
// the expected bootloader prompt "BL >"
#define MAX_RECEIVED_BYTES  4
static uint8_t receivedBytesCache[MAX_RECEIVED_BYTES];
static const char menuPrompt[] = "BL >";

// This is sized to be big enough to read "\r\nbegin upload\r\n"
// with some additional extra fuzz.
#define MAX_BYTES_WITHOUT_XMODEM_START  2000

//------------------------------------------------------------------------------
// Forward Declarations

static void delay(void);
static int checkFdForData(void);
static void storeReceivedByte(uint8_t byte);
static bool checkForBootloaderMenu(void);
static bool checkForXmodemStart(void);

#define errorPrint(...) fprintf(stderr, __VA_ARGS__)
#define messagePrint(...) printf(__VA_ARGS__)

static int serialFd = NULL_FILE_DESCRIPTOR;

static uint8_t ttydev[] = "/dev/ttyS17";

//------------------------------------------------------------------------------

// Config 0 (default) : EM2xx/EM3xx @ 115200 bps with RTS/CTS flow control
#define ASH_HOST_CONFIG_DEFAULT                                                   \
  {                                                                               \
    "/dev/ttyS17",       /* serial port name                                  */   \
    38400,             /* baud rate (bits/second)                           */   \
    1,                  /* stop bits                                         */   \
    false,               /* true enables RTS/CTS flow control, false XON/XOFF */   \
    256,                /* max bytes to buffer before writing to serial port */   \
    256,                /* max bytes to read ahead from serial port          */   \
    0,                  /* trace output control bit flags                    */   \
    3,                  /* max frames sent without being ACKed (1-7)         */   \
    true,               /* enables randomizing DATA frame payloads           */   \
    800,                /* adaptive rec'd ACK timeout initial value          */   \
    400,                /*  "     "     "     "     "  minimum value         */   \
    2400,               /*  "     "     "     "     "  maximum value         */   \
    2500,               /* time allowed to receive RSTACK after ncp is reset */   \
    0,        /* if free buffers < limit, host receiver isn't ready */  \
    12,        /* if free buffers > limit, host receiver is ready   */   \
    480,                /* time until a set nFlag must be resent (max 2032)  */   \
    0, /* method used to reset ncp                          */ \
    0 /* type of ncp processor                         */  \
  }

#define ASH_PORT_LEN              40  /*!< length of serial port name string */


/** @brief Configuration parameters: values must be defined before calling ashResetNcp()
 * or ashStart(). Note that all times are in milliseconds.
 */
typedef struct
{
    char serialPort[ASH_PORT_LEN];  /*!< serial port name */
    uint32_t baudRate;      /*!< baud rate (bits/second) */
    uint8_t  stopBits;      /*!< stop bits */
    uint8_t  rtsCts;        /*!< true enables RTS/CTS flow control, false XON/XOFF */
    uint16_t outBlockLen;   /*!< max bytes to buffer before writing to serial port */
    uint16_t inBlockLen;    /*!< max bytes to read ahead from serial port */
    uint8_t  traceFlags;    /*!< trace output control bit flags */
    uint8_t  txK;           /*!< max frames sent without being ACKed (1-7) */
    uint8_t  randomize;     /*!< enables randomizing DATA frame payloads */
    uint16_t ackTimeInit;   /*!< adaptive rec'd ACK timeout initial value */
    uint16_t ackTimeMin;    /*!< adaptive rec'd ACK timeout minimum value */
    uint16_t ackTimeMax;    /*!< adaptive rec'd ACK timeout maximum value */
    uint16_t timeRst;       /*!< time allowed to receive RSTACK after ncp is reset */
    uint8_t  nrLowLimit;    /*!< if free buffers < limit, host receiver isn't ready */
    uint8_t  nrHighLimit;   /*!< if free buffers > limit, host receiver is ready */
    uint16_t nrTime;        /*!< time until a set nFlag must be resent (max 2032) */
    uint8_t  resetMethod;   /*!< method used to reset ncp */
    uint8_t  ncpType;       /*!< type of ncp processor */
} AshHostConfig;

// Host configuration structure
AshHostConfig ashHostConfig = ASH_HOST_CONFIG_DEFAULT;


bool emAfStartNcpBootloaderCommunications(void)
{
    serialFd = NULL_FILE_DESCRIPTOR;
    char errorString[MAX_ERROR_LENGTH];
    int tryTimes = 1;
    bool ret = false;

    printf("emAfStartNcpBootloaderCommunications\r\n");

    if (!uart_init())
    {
        printf("Error: Could not setup serial port\r\n");
		return false;
    }


    if(!emAfBootloadSendByte(beginDownload))   //发送‘1’；
    {
        errorPrint("Failed to set bootloader in download mode.\n");
        return false;
    }
	
    return checkForXmodemStart();

}

static bool checkForXmodemStart(void)
{
    printf("checkForXmodemStart:wait CCCCCCCC\n");
    uint8_t bytesSoFar = 0;
    int startTimes=0;
    while (bytesSoFar < MAX_BYTES_WITHOUT_XMODEM_START)
    {
        int status = checkFdForData();
        if (status <= 0)
        {
            // Timeout or error
            return false;
        }

        uint8_t data;
        ssize_t bytes = read(serialFd, &data, 1);
        if (bytes < 0)
        {
            errorPrint("Read failed: %s\n", strerror(errno));
            return false;
        }

        // debug
        printf("checkForXmodemStart Got<%c,%d>\n", (char)data,(char)data);

        if (data == xmodemStartChar)
        {
            startTimes++;
        }
        else
        {
            startTimes=0;
        }
        if(startTimes>=1)
        {
            return true;
        }
        bytesSoFar++;
    }

    errorPrint("Failed to get Xmodem start message from bootloader.\n");
    return false;
}

bool emAfBootloadSendData(const uint8_t *data, uint16_t length)
{
    //printf("emAfBootloadSendData lenght:%d\n",length);
    if (length != write(serialFd,
                        data,
                        length))  // count
    {
        errorPrint("Error: Failed to write to serial port: %s\r\n",
                   strerror(errno));
        return false;
    }
    fsync(serialFd);
    return true;
}

bool emAfBootloadSendByte(uint8_t byte)
{
    return emAfBootloadSendData(&byte, 1);
}

static bool checkForBootloaderMenu(void)
{
    // verbose debug
    printf("checkForBootloaderMenu()\n");

    memset(receivedBytesCache, 0, MAX_RECEIVED_BYTES);

    // Send a CR to the bootloader menu to trigger the prompt to echo back.
    if (!emAfBootloadSendByte(returnChar))
    {
        return false;
    }
    bool done = false;
    int totalBytes = 0;
    while (!done)
    {
        int status = checkFdForData();

        if (status <= 0)
        {
            return false;
        }
        uint8_t data;
        ssize_t bytes = read(serialFd, &data, 1);
        if (bytes < 0)
        {
            errorPrint("Error: read() failed: %s\n", strerror(errno));
            return false;
        }
        else if (bytes == 0)
        {
            continue;
        }
        totalBytes++;
        storeReceivedByte(data);

        // verbose debug
        //printf("%c", data);
        int i = 0;
        for(; i< MAX_RECEIVED_BYTES; i++)
        {
            if(receivedBytesCache[i] != menuPrompt[i])
            {
                break;
            }
        }
        if(i == MAX_RECEIVED_BYTES)
        {
            done = true;
            continue;
        }

        if (totalBytes > MAX_BYTES_WITHOUT_PROMPT)
        {
            errorPrint("Got %d bytes without seeing the bootloader menu prompt.\n",
                       MAX_BYTES_WITHOUT_PROMPT);
            return false;
        }
    }
    tcflush(serialFd, TCIFLUSH);
    // while(checkFdForData()>0)
    // {
    //     uint8_t data;
    //     ssize_t bytes = read(serialFd, &data, 1);
    //     printf("Got checkFdForData<%c,%x>\n", (char)data, (char)data);
    // }
    return true;
}

bool emAfRebootNcpAfterBootload(void)
{
    delay();
    messagePrint("Rebooting NCP\n");

    if (!checkForBootloaderMenu())
    {
        errorPrint("Failed to get bootloader menu prompt.\n");
        return false;
    }

    if (!emAfBootloadSendByte(runChar))
    {
        return false;
    }

    delay();        // arbitrary delay to give NCP time to reboot.
    close(serialFd);
    serialFd = NULL_FILE_DESCRIPTOR;
    return true;
}

static void storeReceivedByte(uint8_t newByte)
{
    // We right shift all the bytes in the array.  The first byte on the array
    // will be dumped, and the new byte will become the last byte.
    uint8_t i;
    for (i = 0; i < MAX_RECEIVED_BYTES - 1; i++)
    {
        receivedBytesCache[i] = receivedBytesCache[i + 1];
    }
    receivedBytesCache[i] = newByte;
}

int init_tty(int fd)
{
    struct termios tios,checkTios;
    uint8_t i ;
    bool flowControl = 0;
    bool rtsCts = ashHostConfig.rtsCts;
    uint8_t stopBits = ashHostConfig.stopBits;
    tcgetattr(fd, &tios);		 // get current serial port options
    uint32_t baud = ashHostConfig.baudRate;      /*!< baud rate (bits/second) */
    uint32_t bps = baud;

    cfsetispeed(&tios, baud);
    cfsetospeed(&tios, baud);

    tios.c_cflag |= CLOCAL;				// ignore modem inputs
    tios.c_cflag |= CREAD;				// receive enable (a legacy flag)
    tios.c_cflag &= ~CSIZE;				// 8 data bits
    tios.c_cflag |= CS8;
    tios.c_cflag &= ~PARENB;				// no parity
    if (stopBits == 1)
    {
        tios.c_cflag &= ~CSTOPB;
    }
    else
    {
        tios.c_cflag |= CSTOPB;
    }
    if (flowControl && rtsCts)
    {
        tios.c_cflag |= CRTSCTS;
    }
    else
    {
        tios.c_cflag &= ~CRTSCTS;
    }

    tios.c_iflag &= ~(BRKINT | INLCR | IGNCR | ICRNL | INPCK
                      | ISTRIP | IMAXBEL | IXON | IXOFF | IXANY);

    if (flowControl && !rtsCts)
    {
        tios.c_iflag |= (IXON | IXOFF); 		 // SW flow control
    }
    else
    {
        tios.c_iflag &= ~(IXON | IXOFF);
    }

    tios.c_lflag &= ~(ICANON | ECHO | IEXTEN | ISIG);  // raw input

    tios.c_oflag &= ~OPOST;				// raw output

    memset(tios.c_cc, _POSIX_VDISABLE, NCCS);  // disable all control chars
    tios.c_cc[VSTART] = CSTART;			// define XON and XOFF
    tios.c_cc[VSTOP] = CSTOP;

    ////////////////////////////////////////////////////////////////////////////
    // The POSIX standard states that when VMIN > 0 and VTIME == 0, read()
    // is supposed to block if it cannot return VMIN bytes. In fact,
    // if O_NONBLOCK is set, under Cygwin/WinXP read() does not block, but
    // instead sets errno to EAGAIN and returns -1.
    //
    // It is possible that under certain Linux or Embedded Linux variants
    // read() will block, and this code will have to be modified since
    // EZSP cannot function reliably with blocking serial I/O.
    //
    // Some alternative approaches in that case would include:
    //  o using Linux-specific ioctl()s
    //  o using select() on the serial port
    //  o spawning child processes to do serial I/O
    ////////////////////////////////////////////////////////////////////////////

    tios.c_cc[VMIN] = 1;
    tios.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSAFLUSH, &tios);  // set EZSP serial port options
    tcgetattr(fd, &checkTios);	   // and read back the result

    // Verify that the fields written have the right values
    i = (tios.c_cflag ^ checkTios.c_cflag) & CFLAG_MASK;
    if (i)
    {
        // Try again since macOS mojave seems to not have hardware flow control enabled
        tios.c_cflag &= ~CRTSCTS;
        tcsetattr(fd, TCSAFLUSH, &tios);  // set EZSP serial port options
        tcgetattr(fd, &checkTios); 	 // and read back the result
        i = (tios.c_cflag ^ checkTios.c_cflag) & CFLAG_MASK;
        if (i)
        {
            printf("Termios cflag(s) in error: 0x%04X\r\n", i);
        }
    }
    i = (tios.c_iflag ^ checkTios.c_iflag) & IFLAG_MASK;
    if (i)
    {
        printf("Termios iflag(s) in error: 0x%04X\r\n", i);
    }
    i = (tios.c_lflag ^ checkTios.c_lflag) & LFLAG_MASK;
    if (i)
    {
        printf("Termios lflag(s) in error: 0x%04X\r\n", i);
    }
    i = (tios.c_oflag ^ checkTios.c_oflag) & OFLAG_MASK;
    if (i)
    {
        printf("Termios oflag(s) in error: 0x%04X\r\n", i);
    }
    for (i = 0; i < NCCS; i++)
    {
        if (tios.c_cc[i] != checkTios.c_cc[i])
        {
            //break;
        }
    }
    if (i != NCCS)
    {
        printf("Termios error at c_cc[%d]\r\n", i);
    }
    if (	(cfgetispeed(&checkTios) != baud)
            || (cfgetospeed(&checkTios) != baud))
    {
        printf("Could not set baud rate to %d bps\r\n", bps);
    }


// Make sure the string is NULL terminated in case it got truncated.
    //errorStringLocation[maxErrorLength - 1] = '\0';

//    if (fd != NULL_FILE_DESCRIPTOR)
//    {
//        close(fd);
//        fd = NULL_FILE_DESCRIPTOR;
//    }

    return 0;

}


int init_tty1(int fd)
{
    struct termios termios_rfid;

    bzero(&termios_rfid, sizeof(termios_rfid));//清空结构体

    cfmakeraw(&termios_rfid);//设置终端属性，激活选项

    cfsetispeed(&termios_rfid, B38400);//输入波特率
    cfsetospeed(&termios_rfid, B38400);//输出波特率

    termios_rfid.c_cflag |= CLOCAL | CREAD;//本地连接和接收使能

    termios_rfid.c_cflag &= ~CSIZE;//清空数据位
    termios_rfid.c_cflag |= CS8;//数据位为8位

    termios_rfid.c_cflag &= ~PARENB;//无奇偶校验

    termios_rfid.c_cflag &= ~CSTOPB;//一位停止位

    tcflush(fd,TCIFLUSH);

    termios_rfid.c_cc[VTIME] = 10;//设置等待时间
    termios_rfid.c_cc[VMIN] = 1;

    tcflush(fd, TCIFLUSH);//清空输入缓冲区

    if(tcsetattr(fd, TCSANOW, &termios_rfid))//激活串口设置
        return 0;

    return 1;
}


int uart_init()
{
    int rv = -1;
    struct termios options;

    serialFd = open(ttydev,O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(serialFd < 0)
    {
        printf("Serial port %s open failed: %s\n", ttydev,strerror(errno));
        return -1;
    }

    //tcflush(serialFd, TCIOFLUSH);       // flush all input and output data
    //fcntl(serialFd, F_SETFL, FNDELAY);

    if(init_tty(serialFd) == -1)
    {
        printf("init_tty in failed!\n");
    }
    return true;
}

static void delay(void)
{
    // Empirically I have found that we have to delay to give the bootloader
    // time to launch before we see the LEDs flash and the bootloader menu
    // is available.  Not sure if this could be improved.
    messagePrint("Delaying %d seconds\n", BOOTLOADER_DELAY);
    sleep(BOOTLOADER_DELAY);
}

// CYGWIN NOTE
//   Cygwin does NOT properly handle select() with regard to serial ports.
//   I am not sure exactly what will happen in that case but I believe
//   it will not timeout.  If all goes well we will never get a timeout,
//   but if things go south we won't handle them gracefully.
static int checkFdForData(void)
{
    fd_set readSet;
    struct timeval timeout =
    {
        READ_TIMEOUT_SEC,
        0,                  // ms. timeout value
    };

    FD_ZERO(&readSet);
    FD_SET(serialFd, &readSet);

    int fdsWithData = select(serialFd + 1,         // per the select() manpage
                             //   highest FD + 1
                             &readSet,             // read FDs
                             NULL,                 // write FDs
                             NULL,                 // exception FDs
                             (READ_TIMEOUT_SEC > 0 // passing NULL means wait
                              ? &timeout           //   forever
                              : NULL));

    // Ideally we should check for EINTR and retry the operation.
    if (fdsWithData < 0)
    {
        errorPrint("FATAL: select() returned error: %s\n",
                   strerror(errno));
        return -1;
    }

    if (fdsWithData == 0)
    {
        errorPrint("Error: Timeout occurred waiting for read data.\n");
    }
    return (fdsWithData);  // If timeout has occurred, this will be 0,
    // otherwise it will be the number of FDs
    // with data.
}

// TODO: would be better to make this work based on any qty of characters
//  being allowed to be received until the timeout is reached.  Current
//  behavior only looks at one character and waits up to the timeout for it to
//  arrive
bool emAfBootloadWaitChar(uint8_t *data, bool expect, uint8_t expected)
{
    /*
      int tryTimes = 1;
      do
      {
         int status = checkFdForData();
        if (status <= 0) {
          // Timeout or error
          printf("emAfBootloadWaitChar timeout \n");
          continue;
        }

        ssize_t bytes = read(serialFd, data, 1);
        if (bytes < 0) {
          printf("Read failed: %s\n", strerror(errno));
          continue;
        }

        // debug
       printf("Got <%c,%x>\n", (char)*data, (char)*data);

        if (expect)
         {
          if(((*data) == expected))
          {
            return true;
          }
          else{
            continue;
          }
        } else {
          return true;
        }
      } while (tryTimes--);
      return false;
    */
    int status = checkFdForData();
    if (status <= 0)
    {
        // Timeout or error
        return false;
    }

    ssize_t bytes = read(serialFd, data, 1);
    if (bytes < 0)
    {
        errorPrint("Read failed: %s\n", strerror(errno));
        return false;
    }

    // debug
    printf("Got <%c,%x>\n", (char)*data, (char)*data);

    if (expect)
    {
        return ((*data) == expected);
    }
    else
    {
        return true;
    }
}

