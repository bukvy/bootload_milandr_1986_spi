#include <MDR32Fx.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_uart.h>
#include "MDR32F9Qx_dma.h"
#include "config.h"
#include "Serial.h"


CComBuf U2;

// for flash only
#define mask 0x1021
unsigned short updcrc(unsigned short crc, unsigned short c)
{
    unsigned char i;
    c <<= 8;
    for (i = 0; i < 8; i++)
    {
        if((crc ^ c) & 0x8000) crc = (crc << 1) ^ mask;
        else crc <<= 1;
        c <<= 1;
    }
    return crc;
}
//вычисление кс по буферу
unsigned short CRC16(unsigned char *buf, unsigned char lng)
{
    unsigned char i, j;
    unsigned short c, crc = 0xffff;
    for (j = 0; j < lng; j++)
    {
        c = buf[j] << 8;
        for (i = 0; i < 8; i++)
        {
            if((crc ^ c) & 0x8000) crc = (crc << 1) ^ mask;
            else crc <<= 1;
            c <<= 1;
        }
    }
    return crc;
}
// for flash only

unsigned char bbrrbb;
///**************************************************
SerialEventHandlerPtr Rx0EventHandlerPtr;
SerialEventHandlerPtr Tx0EventHandlerPtr;


void Start_TX2(void *buf, unsigned char cnt)
{
    U2.buf = (unsigned char *)buf;
    U2.ptr = 1;
    U2.mode = UART_TRANSMIT_MODE;
    U2.cnt = cnt;

}
unsigned char  check2 (void)
{

}


void Stop_TX2(void)
{
    U2.mode = UART_RECEIVE_MODE;
}

void Init_UART2(void)
{
    U2.Start_TX = Start_TX2;
    U2.Stop_TX = Stop_TX2;
    U2.mode = UART_TURN_MODE;
//    U2.buf = (unsigned char *)g_ModbusBuf;
    U2.Check_end_pack = check2;
    U2.cnt = 0;
    U2.ptr = 0;
}