#ifndef __serial__h__
#define __serial__h__

#define MAX_BUF 240

enum uart_mode_t{
	UART_RECEIVE_MODE=0, UART_TRANSMIT_MODE=1, UART_BUSY_MODE=2, UART_TURN_MODE=3
};

typedef void (*SerialEvent)(void *buf,unsigned char cnt);

typedef void (*SerialEventHandlerPtr)(void);

typedef unsigned char (*SerialEventHandlerPtr1)(void);

typedef struct
{
  unsigned char cnt;
  unsigned char ptr;
  unsigned char mode;
  unsigned char *buf;
  unsigned long rx_last_time;
  SerialEvent Start_TX;
  SerialEventHandlerPtr Stop_TX;
  SerialEventHandlerPtr1 Check_end_pack;
} CComBuf;

extern CComBuf U2;

unsigned short CRC16(unsigned char *buf, unsigned char lng);

void Init_UART2(void);

#endif
