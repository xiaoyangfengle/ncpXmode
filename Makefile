all:
	gcc crc16.* ota-bootload-* xmodem.c ncpUpgrade.c -o xmodem -static