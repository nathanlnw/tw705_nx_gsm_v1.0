#ifndef _H_CRC
#define _H_CRC

//#define  CRC_CODE_16  0x1021			// CRC-CCITT = X16 + X12 + X5 + X0
#define  CRC_CODE_16  0x8005		// CRC-16_IBM = X16 + X15 + X2 + X0 
#define  CRC_CODE_32  0x04C10DB7		// CRC-32 = X32 + X26 + X23 + X22 + X16 + X11 + X10 + X8 + X7 + X5 + X4 + X2 + X1 + X0

extern unsigned short int CRC16_1(unsigned char *ptr, unsigned short int len,unsigned short int crc_value); 

#endif
