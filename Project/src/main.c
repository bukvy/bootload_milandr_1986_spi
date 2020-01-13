/**
  ******************************************************************************
  * @file    Main.c
  * @author  Phyton Application Team
  * @version V3.0.0
  * @date    10.09.2011
  * @brief   Main program body
  ******************************************************************************
  * <br><br>
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 Phyton</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "types.h"
#include "time.h"
#include <MDR32F9Qx_uart.h>
#include "Serial.h"
#include <MDR32F9Qx_it.h>
#include "main.h";







#define xyzModem_xmodem 1
#define xyzModem_ymodem 2

#define xyzModem_access   -1
#define xyzModem_noZmodem -2
#define xyzModem_timeout  -3
#define xyzModem_eof      -4
#define xyzModem_cancel   -5
#define xyzModem_frame    -6
#define xyzModem_cksum    -7
#define xyzModem_sequence -8

#define xyzModem_close 1
#define xyzModem_abort 2



typedef struct {
    char *filename;
    int   mode;
    int   chan;
} connection_info_t;

//typedef unsigned int bool;

#define false 0
#define true 1


/* Values magic to the protocol */
#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define BSP 0x08
#define NAK 0x15
#define CAN 0x18
#define EOFmodem 0x1A		/* ^Z for DOS officionados */

//unsigned int testaddr;


/* Data & state local to the protocol */
static struct
{

	unsigned char pkt[1024], *bufp;
	unsigned char blk, cblk, crc1, crc2;
	unsigned char next_blk;	/* Expected block */
	int len, mode, total_retries;
	int total_SOH, total_STX, total_CAN;
	bool crc_mode, at_eof, tx_ack;

} xyz;


#define xyzModem_CHAR_TIMEOUT            2000	/* 2 seconds */
#define xyzModem_MAX_RETRIES             20
#define xyzModem_MAX_RETRIES_WITH_CRC    10
#define xyzModem_CAN_COUNT                3	/* Wait for 3 CAN before quitting */
#define XMODEM_BUF_SIZE 4096

char xmodemBuf[XMODEM_BUF_SIZE];

//uint32_t  xeeadr[200];  Only for testing Koval
//uint32_t  xeedata[200]; Only for testing Koval



//##################  that is for programm of FLASH  memory =============

#define EEPROM_CMD	(*(volatile unsigned long *) 0x40018000)
#define EEPROM_ADR	(*(volatile unsigned long *) 0x40018004)
#define EEPROM_DI	(*(volatile unsigned long *) 0x40018008)
#define EEPROM_DO	(*(volatile unsigned long *) 0x4001800C)
#define EEPROM_KEY	(*(volatile unsigned long *) 0x40018010)
#define NewStackAddr        (*(volatile unsigned long *) 0x08004000)

#define NVSTR	0x2000
#define PROG	0x1000
#define MAS1    0x800
#define ERASE	0x400
#define IFREN	0x200
#define SE		0x100
#define YE		0x80
#define XE   	0x40
#define CON	 	0x1
#define WR  0x2
/*
void bootuart(void);
void sleep(int id);
*/
__ramfunc  int Erase(void);
__ramfunc  int Program(void);
__ramfunc void sleep(int id);


//========== That is address where to we put main programm Koval ======================
uint32_t programadr=0x08004000;
void testfunc(void); // for putting of start of next programm










/* Table of CRC constants - implements x^16+x^12+x^5+1 */
static const uint16_t crc16_tab[] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
		0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
		0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
		0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
		0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
		0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
		0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
		0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
		0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
		0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
		0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
		0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
		0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
		0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
		0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
		0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
		0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
		0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
		0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
		0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

uint16_t cyg_crc16(unsigned char *buf, int len)
{
	int i;
	uint16_t cksum;

	cksum = 0;
	for (i = 0;  i < len;  i++) {
		cksum = crc16_tab[((cksum>>8) ^ *buf++) & 0xFF] ^ (cksum << 8);
	}
	return cksum;
}



typedef int cyg_int32;

/* функция чтения 1 байта с таймаутом */
int CYGACC_COMM_IF_GETC_TIMEOUT(char *c)
{
	unsigned long counter = TimerCounter + xyzModem_CHAR_TIMEOUT;  // about 2 seconds
	while ((UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF)!= SET) && (counter > TimerCounter)) {
        asm (" NOP");
	}
	if (UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF)== SET) {
		*c = UART_ReceiveData (MDR_UART2);;
		return 1;
	}
	return 0;
}

void CYGACC_COMM_IF_PUTC(char y)   // here x I don't know what for from original project Koval
{
	      UART_SendData (MDR_UART2,y);
}




  void main(void)
{
  System_Init();
unsigned char ReciveByte;
int size=loadx_bin(0x08001000);

  while (1){

//    /* Check RXFF flag */
//    while (UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF)!= SET)
//    {
//    }

    /* Recive data */
 //   ReciveByte = UART_ReceiveData (MDR_UART2);
 //   UART_SendData (MDR_UART2,ReciveByte);

  asm(" NOP");

  };


}


//#########################################################

int xyzModem_stream_open (connection_info_t * info)
{
	int stat = 0;
	int retries = xyzModem_MAX_RETRIES ;
	int crc_retries = xyzModem_MAX_RETRIES_WITH_CRC;
        int counter;
        unsigned int newstack;
	/* TODO: CHECK ! */

	xyz.len = 0;
	xyz.crc_mode = true;
	xyz.at_eof = false;
	xyz.tx_ack = false;
	xyz.mode = info->mode;
	xyz.total_retries = 0;
	xyz.total_SOH = 0;
	xyz.total_STX = 0;
	xyz.total_CAN = 0;
//====== Отсылка сообщения о том , что будет расширенный CRC Koval  Зацикливанеи , если
//  по адресу 0x08004000 нет в старшем сегменте адреса установки стека  (0х20000) если есть
// то после 10 попыток переходит на выполнение программы с 0х08004000 ( учтите что по этому адресу
// находится адрес стека а программа начинается с 0х08004004
        while (retries-- > 0){
          newstack=NewStackAddr;
          if ((newstack & 0x2FFF0000 ) != 0x20000000 ){retries++;}  // if there is not stack in 0x080004000 then forever

            counter=TimerCounter + xyzModem_CHAR_TIMEOUT;
                    	CYGACC_COMM_IF_PUTC ((xyz.crc_mode ? 'C' : NAK));
                while ((UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF)!= SET) && (counter > TimerCounter)) {
                  asm(" NOP");
                }
                if ((UART_GetFlagStatus (MDR_UART2, UART_FLAG_RXFF)== SET)) {
       		xyz.next_blk = 1;
                  return 0 ;
                }
        }
		return 1;
}

//#########################################################


/* Wait for the line to go idle */
static void xyzModem_flush(void)
{
	int res;
	char c;
	while (true) {
		res = CYGACC_COMM_IF_GETC_TIMEOUT (&c);
		if (!res)
			return;
	}
}

static int xyzModem_get_hdr(void)   // Receive not only header , but all message 128 bytes with checking
{
	char c;
	int res;
	bool hdr_found = false;
	int i, can_total, hdr_chars;
	unsigned short cksum;

	/* Find the start of a header */
	can_total = 0;
	hdr_chars = 0;

	if (xyz.tx_ack)	{
		CYGACC_COMM_IF_PUTC(ACK);
		xyz.tx_ack = false;
	}
	while (!hdr_found) {
		res = CYGACC_COMM_IF_GETC_TIMEOUT (&c);
		if (res) {
			hdr_chars++;
			switch (c) {
			case SOH:
				xyz.total_SOH++;
				hdr_found = true;
				break;
			case CAN:
				xyz.total_CAN++;
				if (++can_total == xyzModem_CAN_COUNT) {
					return xyzModem_cancel;
				} else {
					/* Wait for multiple CAN to avoid early quits */
					break;
				}
			case EOT:
				/* EOT only supported if no noise */
				if (hdr_chars == 1) {
					CYGACC_COMM_IF_PUTC (ACK);
					return xyzModem_eof;
				}
			default:
				/* Ignore, waiting for start of header */
				;
			}
		} else {  // If this byte is not header ----------------
			/* Data stream timed out */
			xyzModem_flush();	/* take all bytes from comport Koval*/
			return xyzModem_timeout;
		}
	}

	/* Header found, now read the data */
	res = CYGACC_COMM_IF_GETC_TIMEOUT ((char *) &xyz.blk);  // Read number of block

	if (!res) {
		return xyzModem_timeout;
	}
	res = CYGACC_COMM_IF_GETC_TIMEOUT ((char *) &xyz.cblk);  // Read number of block complimentary part Koval
	if (!res) {
		return xyzModem_timeout;
	}
	xyz.len = 128;
	xyz.bufp = xyz.pkt;
	for (i = 0; i < xyz.len; i++) {
		res = CYGACC_COMM_IF_GETC_TIMEOUT (&c);
		if (res) {
			xyz.pkt[i] = c;
		} else {
			return xyzModem_timeout;
		}
	}
	res = CYGACC_COMM_IF_GETC_TIMEOUT ((char *) &xyz.crc1);
	if (!res) {
		return xyzModem_timeout;
	}
	if (xyz.crc_mode) {
		res = CYGACC_COMM_IF_GETC_TIMEOUT ((char *) &xyz.crc2);
		if (!res) {
			return xyzModem_timeout;
		}
	}
	/* Validate the message */
	if ((xyz.blk ^ xyz.cblk) != (unsigned char) 0xFF) {  // summ of block number and complimentary block number should= 0XFF
		xyzModem_flush();
		return xyzModem_frame;
	}
	/* Verify checksum/CRC */
	if (xyz.crc_mode) {
		cksum = cyg_crc16 (xyz.pkt, xyz.len);
		if (cksum != ((xyz.crc1 << 8) | xyz.crc2)) {
			return xyzModem_cksum;
		}
	} else {
		cksum = 0;
		for (i = 0; i < xyz.len; i++) {
			cksum += xyz.pkt[i];
		}
		if (xyz.crc1 != (cksum & 0xFF)) {
			return xyzModem_cksum;
		}
	}
	/* If we get here, the message passes [structural] muster */
	return 0;
}


//#########################################################


int xyzModem_stream_read (char *buf, int size)
{
	int stat, total, len;
	int retries;

	total = 0;
	stat = xyzModem_cancel;
	/* Try and get 'size' bytes into the buffer */
	while (!xyz.at_eof && (size > 0)) {
		if (xyz.len == 0) {
			retries = xyzModem_MAX_RETRIES;
			while (retries-- > 0) {
				stat = xyzModem_get_hdr(); // read whole block of data 128 bytes to buffer   xyz.pkt return 0 if normal Koval
				if (stat == 0) {
					if (xyz.blk == xyz.next_blk) {
						xyz.tx_ack = true;
						xyz.next_blk = (xyz.next_blk + 1) & 0xFF;

								/* Data blocks can be padded with ^Z (EOF) characters */
								/* This code tries to detect and remove them */
								if ((xyz.bufp[xyz.len - 1] == EOFmodem) &&
										(xyz.bufp[xyz.len - 2] == EOFmodem) &&
										(xyz.bufp[xyz.len - 3] == EOFmodem))	{
									while (xyz.len
											&& (xyz.bufp[xyz.len - 1] == EOFmodem)) {
										xyz.len--;
									}
								}
						break;
					} else if (xyz.blk == ((xyz.next_blk - 1) & 0xFF)) {
						/* Just re-ACK this so sender will get on with it */
						CYGACC_COMM_IF_PUTC (ACK);
						continue;	/* Need new header */
					} else {
						stat = xyzModem_sequence;
					}
				}
				if (stat == xyzModem_cancel) {
					break;
				}
				if (stat == xyzModem_eof) {
					CYGACC_COMM_IF_PUTC (ACK);
					xyz.at_eof = true;
					break;
				}
				CYGACC_COMM_IF_PUTC ((xyz.crc_mode ? 'C' : NAK));
				xyz.total_retries++;
			}
			if (stat < 0) {
				xyz.len = -1;
				return total;
			}
		}
		/* Don't "read" data from the EOF protocol package */
		if (!xyz.at_eof) {
			len = xyz.len;
			if (size < len)
				len = size;
			memcpy (buf, xyz.bufp, len);
			size -= len;
			buf += len;
			total += len;
			xyz.len -= len;
			xyz.bufp += len;
		}
	}
	return total;
}
//====================#########################################################======================
void xyzModem_stream_close (void)
{

}
// #########################################################================
/* Need to be able to clean out the input buffer, so have to take the */
/* getc */
void xyzModem_stream_terminate (bool abort)
{
	int c;

	if (abort) {

			/* The X/YMODEM Spec seems to suggest that multiple CAN followed by an equal */
			/* number of Backspaces is a friendly way to get the other end to abort. */
			CYGACC_COMM_IF_PUTC (CAN);
			CYGACC_COMM_IF_PUTC (CAN);
			CYGACC_COMM_IF_PUTC (CAN);
			CYGACC_COMM_IF_PUTC (CAN);
			CYGACC_COMM_IF_PUTC (BSP);
			CYGACC_COMM_IF_PUTC (BSP);
			CYGACC_COMM_IF_PUTC (BSP);
			CYGACC_COMM_IF_PUTC (BSP);
			/* Now consume the rest of what's waiting on the line. */
			xyzModem_flush();
			xyz.at_eof = true;

	} else {
		/*
		 * Consume any trailing crap left in the inbuffer from
		 * previous recieved blocks. Since very few files are an exact multiple
		 * of the transfer block size, there will almost always be some gunk here.
		 * If we don't eat it now, RedBoot will think the user typed it.
		 */
                    xyzModem_flush();
		/*
		 * Make a small delay to give terminal programs like minicom
		 * time to get control again after their file transfer program
		 * exits.
		 */
		CYGACC_CALL_IF_DELAY_MS ((cyg_int32) 250);
	}
}




int loadx_bin(uint32_t flash_addr_beg)
{
	connection_info_t info;
	int res, addr = 0, err,param;
        uint32_t flash_addr=flash_addr_beg;
	info.mode = xyzModem_xmodem;




	res = xyzModem_stream_open(&info);

	if (!res) {


		while ((res = xyzModem_stream_read(&xmodemBuf[0], XMODEM_BUF_SIZE)) > 0) {

// ==============   If we receive block of data in xmodemBuf then we erase next page (4096 kb) and programm data in
// this area of memory (flash memory)
                        __disable_interrupt();
                 res=Erase();
                  __enable_interrupt();

                  if (res!=0){
                    asm (" NOP");
                  }
                  asm (" NOP");
                  sleep(1000);

                  __disable_interrupt();
                  res=Program();
                  __enable_interrupt();

                  if (res!=0){
                    asm (" NOP");
                  }


                }

	} else {

 	}
//==== after success programming or if nobody wants to programm and there is programm already in memory goto bravely,
//       ( but dont forget to  tie camel before )   Koval ( and Sindbad Seaman) ============
//              asm ("VTOR	EQU		0xe000ed08\n"
 //       asm ( "LDR.N     R0,    = 8000000\n");
uint32_t  newstack;
      newstack=NewStackAddr;
        __set_MSP(newstack);

        asm (  "movw      R0, #0x4000\n"
                "movt       R0, #0x0800\n"
                "LDR	R0,[R0,#4]\n");

        asm (" NOP");

                  asm ( "BX      R0");


        testfunc();   // here we setup address when add additional segment to linker

/*
                      "LDR    R1, 0xe000ed08	\n"
			"STR		R0,[R1]\n"
                                  "LDR     R1,[R0]\n"
				"MSR		MSP,R1\n"
				"LDR		R0,[R0,#4]\n");
//				"BX      R0\n"
*/
}


//###############   this function should be placed in RAM

__ramfunc void sleep(int id)
{
	do
	{
	 id++;
	 id -= 2;
	}
 	while (id!=0);
}


__ramfunc int Erase(void)  // Here we do not erase all memory  ( only one page ) so this code differs from examples Koval
{
unsigned int i,i1,i2,eraddr;
        char *prograddrcheck;
	EEPROM_CMD |= CON;
        prograddrcheck=(char * )programadr;


        EEPROM_CMD |= CON;
	EEPROM_KEY = 0x8AAA5551;

	EEPROM_CMD &= ~(XE|YE|SE|MAS1|ERASE|NVSTR|IFREN);


//	for (i1=0x08003000;i1<0x08010000;i1+=4096)   // this is a pagenumber bits 16:12 in all 32 pages we start from 4th page (0x8004000)
//	{
         for(i2=0; i2 < 16; i2+=4){  // that is sector A,B,C,D

                eraddr=programadr + i2;
		EEPROM_ADR = eraddr;
		EEPROM_DI = 0;
		EEPROM_CMD |= WR;  // NEW     EEPROM_CMD = 0x0003;
		EEPROM_CMD &= ~WR;  // NEW

		EEPROM_CMD |= XE|ERASE;  //EEPROM_CMD = 0x0441;
		sleep(50);// 6us
		EEPROM_CMD |= NVSTR;  //EEPROM_CMD = 0x2441;
		sleep(400000);// 50ms
		EEPROM_CMD &= ~ERASE;   //EEPROM_CMD = 0x2041;
		sleep(400);// 5us
		EEPROM_CMD &= ~(XE|NVSTR);
		sleep(10);// 2us

	sleep(20);// 2us

         }

/* this is checking no need for check when programm
         for (i1=0x08004000;i1<0x08020000;i1+=4)
	{
 		EEPROM_ADR = i1;
		EEPROM_CMD |= XE|YE|SE;
		sleep(1);
		i2 = EEPROM_DO;
		EEPROM_CMD &= ~(XE|YE|SE);
		sleep(1);
		if(i2 != 0xffffffff)
			break;
	}
*/
	EEPROM_CMD &= ~(XE|YE|SE|MAS1|ERASE|NVSTR|IFREN);
	EEPROM_CMD &= ~CON;

//-------------  Check the erase ---------------
         sleep(100);
        for (i1=0;i1<4096;i1++){
          if (*prograddrcheck == 0xFF){
            asm (" NOP");
          } else {
          return (uint32_t)prograddrcheck;
          }
          prograddrcheck++;
        }

	return 0;
}



__ramfunc  int Program(void)
{
unsigned int i1,i3=0,i2=0,a1,a2;

       uint32_t data; // For testing ,xxeeadr,xxeedata,xxeekey;
       char *prograddrcheck;
	EEPROM_CMD |= CON;
	MDR_EEPROM->KEY = 0x8AAA5551;
// For testint        xxeekey=MDR_EEPROM->KEY;
        prograddrcheck=(char * )programadr;

	EEPROM_CMD &= ~(XE|YE|SE|MAS1|ERASE|NVSTR|PROG|IFREN);
	for(i1=0;i1<16;i1+=4)
	{
          for (i2=0;i2<4096;i2+=16){
		MDR_EEPROM->ADR = programadr+i1 +i2 ;
                data=(xmodemBuf[i1+i2] ) | (xmodemBuf[i1+i2 +1 ]<< 8) | (xmodemBuf[i1+i2 +2 ] << 16) | (xmodemBuf[i1+i2 +3 ] <<24) ;
                EEPROM_DI=data;

// For testing               xeedata[i3]=MDR_EEPROM->DI;;
// For testing                 xeeadr[i3]=MDR_EEPROM->ADR;; //MDR_EEPROM->DI;
                if (i2==0){
		EEPROM_CMD |= XE|PROG;
		sleep(100);// 5us
		EEPROM_CMD |= NVSTR;
		sleep(110);// 11us
                }else {
		sleep(20);// 20ns

                }

			EEPROM_CMD |= YE;
			sleep(1000);// 50us
			EEPROM_CMD &= ~YE;
			sleep(10);// 20ns
                        i3++;
		}
		EEPROM_CMD &= ~PROG;
	   	sleep(70);// 6us
		EEPROM_CMD &= ~(XE|NVSTR);
	   	sleep(20);// 1us
        }
	EEPROM_CMD &= ~CON;
 //--------------  now check what we flashed---------
         sleep(100);
        for (i1=0;i1<4096;i1++){
          if (xmodemBuf[i1]==*prograddrcheck){
            asm (" NOP");
          } else {

          return (uint32_t)prograddrcheck;
          }
          prograddrcheck++;
        }
	programadr += 4096;
        return 0;
}






















/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the source file ID, the source line number
*                : and (if USE_ASSERT_INFO == 2) expression text where
*                : the assert_param error has occurred.
* Input          : file_id - pointer to the source file name
*                : line - assert_param error line source number
*                : expr - expression text
* Output         : None
* Return         : None
*******************************************************************************/

#if (USE_ASSERT_INFO == 1)
void assert_failed(uint32_t file_id, uint32_t line)
{
  /* User can add his own implementation to report the source file ID and line number.
     Ex: printf("Wrong parameters value: file Id %d on line %d\r\n", file_id, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#elif (USE_ASSERT_INFO == 2)
void assert_failed(uint32_t file_id, uint32_t line, const uint8_t* expr)
{
  /* User can add his own implementation to report the source file ID, line number and
     expression text.
     Ex: printf("Wrong parameters value (%s): file Id %d on line %d\r\n", expr, file_id, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_ASSERT_INFO */

/** @} */ /* End of group Main_Functions */

/** @} */ /* End of group Main */

/** @} */ /* End of group __MDR32F9Qx_Eval_Demo */

/******************* (C) COPYRIGHT 2011 Phyton *********************************
*
* END OF FILE Main.c */


