#include <MDR32Fx.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_rst_clk.h>
#include <MDR32F9Qx_i2c.h>
#include "MDR32F9Qx_dma.h"
#include "config.h"
#include "modbus.h"
#include "i2c.h"
#include "module.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADRESS		0x00
#define TRANS_COUNT 	5
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef I2C_InitStruct;
PORT_InitTypeDef PortInit;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*
#define I2C_EVENT_BUS_FREE                          (I2C_FLAG_BUS_FREE | I2C_FLAG_nSTOP)
#define I2C_EVENT_BUS_HELD                          (I2C_FLAG_BUS_BUSY | I2C_FLAG_ARB_OK)
#define I2C_EVENT_TRANSFER_IN_PROGRESS              (I2C_EVENT_BUS_HELD | I2C_FLAG_TRANS)
#define I2C_EVENT_TRANSFER_ENABLED                  (I2C_EVENT_BUS_HELD | I2C_FLAG_nTRANS | I2C_FLAG_CMD_nWR | I2C_FLAG_CMD_nRD)
#define I2C_EVENT_ACK_FOUND                         (I2C_EVENT_TRANSFER_ENABLED | I2C_FLAG_SLAVE_ACK)
#define I2C_EVENT_NACK_FOUND                        (I2C_EVENT_TRANSFER_ENABLED | I2C_FLAG_SLAVE_NACK)
#define I2C_EVENT_LOST_ARB_FOUND                    (I2C_FLAG_LOST_ARB
#define I2C_EVENT_STOP_CONDITION_TRANSFER           (I2C_FLAG_STOP)
*/

Ci2cBuf i2c;
unsigned long last_temperature=0;



uint8_t I2C_WriteData(uint8_t *data,uint8_t cnt)
{
  uint16_t i=0,err=0;
    // Просто вызываем готоваую функцию из SPL и ждем, пока данные улетят
    
  while(i<cnt){
    
    if (I2C_GetFlagStatus(I2C_FLAG_SLAVE_ACK) == SET){
      I2C_SendByte(data[i]);
      while (I2C_GetFlagStatus(I2C_FLAG_nTRANS) != SET){}
      i++;
      err=0;
    }else err++;
    if(err>65000)
      return 0;
  }
  return cnt;
}
 
 
 
/*******************************************************************/
uint8_t I2C_ReadData(uint8_t *data,uint8_t cnt)
{
 uint16_t err=0,i=0;
    
 while(i<cnt){
    I2C_StartReceiveData(I2C_Send_to_Slave_ACK);
    err=0;
    // Тут картина похожа, как только данные пришли быстренько считываем их и возвращаем
    while ((I2C_GetFlagStatus(I2C_FLAG_nTRANS) != SET)&&(err<65000)){err++;}
    data[i++] = I2C_GetReceivedData();
  }
  return cnt;
}



//Сама запись
uint8_t Write(uint32_t adr, uint8_t *buf, uint32_t lng){
  unsigned char tbuff[3]={(unsigned char)(adr >> 8),(unsigned char)adr};
  i2c.buf=tbuff;
  while (I2C_GetFlagStatus(I2C_FLAG_BUS_FREE) != SET){I2C_SendSTOP();}
  I2C_StartTransmission(I2C_Direction_Transmitter,0xA8,2,0);
  I2C_WriteData(buf,lng);
  I2C_SendSTOP();
	return 0;
}



// чтение произвольного буфера по произвольному адресу
uint8_t ReadFlashBuf(uint32_t adr, uint8_t *buf, uint32_t lng,uint8_t v) 
{
    uint8_t result = 1;
    unsigned char tbuff[3]={(unsigned char)(adr >> 8),(unsigned char)adr};
    i2c.buf=tbuff;
    while (I2C_GetFlagStatus(I2C_FLAG_BUS_FREE) != SET){I2C_SendSTOP();}
    I2C_StartTransmission(I2C_Direction_Transmitter,0xA8,2,0);
    i2c.buf=buf;
    I2C_StartTransmission(I2C_Direction_Receiver,0xA8,lng,1);
    I2C_SendSTOP();
    return result;
}

// запись произвольного буфера по произвольному адресу
uint8_t WriteFlashBuf(uint32_t adr, uint8_t *buf, uint32_t lng) {
    Write(adr, buf, lng);
    return 1;
  //Delay(1000);
  //return ReadFlashBuf(adr, buf, lng,1);                     //Читае
}

 
void GetI2cTemperatue(void)
{
    unsigned char tbuff[10]={0,0,0,0,0,0,0,0,0,0};
    if (last_temperature>get_ticks())
        return;
    
    Init_ClkDiv(150);
    
    i2c.buf=tbuff;
    I2C_StartTransmission(I2C_Direction_Receiver,0x90,2,1);
    board_set.values.temperature = (short)(((tbuff[0]<<3)|((tbuff[1]>>5)&0x07))*0.125);
    last_temperature = get_ticks()+10000;
}

/*******************************************************************/
void I2C_StartTransmission(uint8_t transmissionDirection,  uint8_t slaveAddress,uint8_t cnt,  uint8_t stop)
{
    i2c.cnt=cnt;i2c.ptr=0;
    // На всякий слуыай ждем, пока шина осовободится
    
    //while (I2C_GetFlagStatus(I2C_FLAG_BUS_FREE) != SET){}

    // Посылаем адрес подчиненному
    I2C_Send7bitAddress(slaveAddress,transmissionDirection);
    
   // На всякий слуыай ждем, пока шина осовободится
    while (I2C_GetFlagStatus(I2C_FLAG_nTRANS) != SET) 
    {
    }
 
    // Если пришло подтверждение то работаем дальше
    if (I2C_GetFlagStatus(I2C_FLAG_SLAVE_ACK) == SET)
    {
      // А теперь у нас два варианта развития событий - в зависимости от выбранного направления обмена данными
      if(transmissionDirection== I2C_Direction_Transmitter)
      {
        I2C_WriteData(i2c.buf,i2c.cnt);
        //while(i2c.cnt>i2c.ptr)I2C_WriteData(i2c.buf[i2c.ptr++]);
      }
   
      if(transmissionDirection== I2C_Direction_Receiver)
      {
        I2C_ReadData(i2c.buf,i2c.cnt);
        //while(i2c.cnt>i2c.ptr)i2c.buf[i2c.ptr++]=I2C_ReadData();
      }
    }
    
    /* По оканчании не забываем послать СТОП */
    if(stop)I2C_SendSTOP();
}


void Init_ClkDiv(int clk_div){
  
    if (I2C_InitStruct.I2C_ClkDiv!= clk_div) {
        I2C_InitStruct.I2C_ClkDiv = clk_div;
        I2C_Init(&I2C_InitStruct);
    }
}

void Init_i2c(void)
{
    /* Enables the HSI clock on PORTC */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC,ENABLE);

  /* Fill PortInit structure */
  PortInit.PORT_PULL_UP = PORT_PULL_UP_ON;
  PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
  PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
  PortInit.PORT_PD = PORT_PD_DRIVER;
  PortInit.PORT_GFEN = PORT_GFEN_OFF;
  PortInit.PORT_FUNC = PORT_FUNC_ALTER;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PortInit.PORT_MODE = PORT_MODE_DIGITAL;

  /* Configure PORTC pins 0,1 (I2C_SCL,I2C_SDA) */
  PortInit.PORT_Pin = PORT_Pin_0 | PORT_Pin_1;
  PORT_Init(MDR_PORTC, &PortInit);

  /* Enables the HSI clock on I2C */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_I2C,ENABLE);

  /* Enables I2C peripheral */
  I2C_Cmd(ENABLE);

  /* Initialize I2C_InitStruct */
  I2C_InitStruct.I2C_Speed = I2C_SPEED_UP_TO_400KHz;

  I2C_InitStruct.I2C_ClkDiv = 150;

  /* Configure I2C parameters */
  I2C_Init(&I2C_InitStruct);

}