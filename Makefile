all:
	gcc crc16.* ota-bootload-* xmodem.c -o xmodem -static