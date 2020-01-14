#include "config.h"
#include <MDR32Fx.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_adc.h>
#include <MDR32F9Qx_rst_clk.h>
#include <MDR32F9Qx_uart.h>
#include "MDR32F9Qx_board.h"
#include "MDR32F9Qx_ssp.h"
#include "MDR32F9Qx_dma.h"
#include "MDR32F9Qx_it.h"
#include "MDR32F9Qx_timer.h"
#include "leds.h"
#include "fp.h"
#include "Serial.h"
#include "i2c.h"
#include "modbus.h"
#include "ferret_proto.h"
#include "spi_ferret_proto.h"
#include "adc.h"
#include "MDR32F9Qx_usb_handlers.h"

#define BSL_UART_CONFIG


#define BufferSize         Led_group
#define ALL_PORTS_CLK   (RST_CLK_PCLK_PORTA | RST_CLK_PCLK_PORTB | \
                         RST_CLK_PCLK_PORTC | RST_CLK_PCLK_PORTD | \
                         RST_CLK_PCLK_PORTE | RST_CLK_PCLK_PORTF)

/*
static struct cfg_params default_params = {
		0, // crc
		
		"0000000000000000", // serial
		
		1, // is_leds_on
		0, // user_led_type
		10, // leds_timeout
		0, // rtc_alarm
		0, // is_rtc_alarm_periodic
		
		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    }, // port_state
		{ 0, 0, 0, 0 }, // port_mode
		//{ 0, 0, 0, 0 }, // alarm_out_state
		//{ 0, 0, 0, 0 } // ps_out_state
};*/ 

unsigned char SSP_Target=0;
#pragma data_alignment = 2
cfg_board_settings board_set;

DMA_ChannelInitTypeDef  DMA_InitSSP1_RX,  DMA_InitSSP2_RX,  DMA_InitSSP1_TX,  DMA_InitSSP2_TX,DMA_InitADC1,   DMA_InitUART;
DMA_CtrlDataInitTypeDef DMA_PriSSP1_RX,   DMA_PriSSP2_RX,   DMA_PriSSP1_TX,   DMA_PriSSP2_TX, DMA_PriADC1,    DMA_PriUART;
DMA_CtrlDataInitTypeDef DMA_AltADC1;

SSP_InitTypeDef sSSP;
PORT_InitTypeDef PortInitStructure;
static UART_InitTypeDef UART_InitStructure;

TIMER_CntInitTypeDef sTIM_CntInit;
TIMER_ChnInitTypeDef sTIM_ChnInit;
TIMER_ChnOutInitTypeDef sTIM_ChnOutInit;



uint16_t SrcBuf1[30];
uint8_t SrcBuf2[BufferSize];
uint16_t DstBuf1[100];
uint8_t DstBuf2[BufferSize];
unsigned char ssp2_cnt=0,ssp1_cnt=0;

ADC_InitTypeDef sADC;
ADCx_InitTypeDef sADCx;
uint16_t ADCConvertedValue[10];


void ClockConfigure(void)
{
  
  /*RST_CLK_HSEconfig(RST_CLK_HSE_ON);
  RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);
  RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSEdiv1,RST_CLK_CPU_PLLmul10);
  */
  RST_CLK_DeInit();
  RST_CLK_HSEconfig(RST_CLK_HSE_ON);
  while(RST_CLK_HSEstatus() != SUCCESS);
  if (RST_CLK_HSEstatus() == SUCCESS)                     /* Good HSE clock */
  {
      /* Select HSE clock as CPU_PLL input clock source */
      /* Set PLL multiplier to 7                        */
      RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSEdiv1, RST_CLK_CPU_PLLmul10);
      /* Enable CPU_PLL */
      RST_CLK_CPU_PLLcmd(ENABLE);
      if (RST_CLK_HSEstatus() == SUCCESS)                     /* Good CPU PLL */
      {
        /* Set CPU_C3_prescaler to 2 */
        RST_CLK_CPUclkPrescaler(RST_CLK_CPUclkDIV1);
        /* Set CPU_C2_SEL to CPU_PLL output instead of CPU_C1 clock */
        RST_CLK_CPU_PLLuse(ENABLE);
        /* Select CPU_C3 clock on the CPU clock MUX */
        RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);
        /* LED1 blinking with 7*HSE/2 clock as input clock source */
      }
  }
  

  /* Configure CPU_PLL clock */
  //RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv1,0);

  /* Enables the RTCHSE clock on all ports */
  RST_CLK_PCLKcmd(ALL_PORTS_CLK, ENABLE);
  
  RST_CLK_PCLKcmd((RST_CLK_PCLK_RST_CLK | RST_CLK_PCLK_SSP1 | RST_CLK_PCLK_SSP2 | RST_CLK_PCLK_DMA),ENABLE);
  RST_CLK_PCLKcmd((RST_CLK_PCLK_UART1 | RST_CLK_PCLK_UART2 | RST_CLK_PCLK_ADC| RST_CLK_PCLK_TIMER1),ENABLE);
}

void PortConfigure(void)
{
  /************************ PortA *************************/
  
  PortInitStructure.PORT_Pin = PORT_Pin_4 | PORT_Pin_6 |PORT_Pin_7;  // That is A0-A2 for address of DAC
  PortInitStructure.PORT_FUNC = PORT_FUNC_PORT;
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;

  PORT_Init(MDR_PORTA, &PortInitStructure);
  PORT_ResetBits(MDR_PORTA, PORT_Pin_4);
  
  
  
  PortInitStructure.PORT_Pin = PORT_Pin_1;
  PortInitStructure.PORT_FUNC = PORT_FUNC_ALTER;
  PortInitStructure.PORT_OE = PORT_OE_IN;
  PortInitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;

  PORT_Init(MDR_PORTA, &PortInitStructure);


  /************************ PortA END *************************/
  
  
  /************************ PortB ************************/

  PortInitStructure.PORT_Pin = PORT_Pin_5 | PORT_Pin_8;
  PortInitStructure.PORT_FUNC = PORT_FUNC_PORT;
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;

  PORT_Init(MDR_PORTB, &PortInitStructure);
  PORT_ResetBits(MDR_PORTB, PORT_Pin_5);

  //AZ: инициализация ноги EXT_SPI_CHK
  PortInitStructure.PORT_Pin = PORT_Pin_6;
  PortInitStructure.PORT_FUNC = PORT_FUNC_ALTER;
  PortInitStructure.PORT_OE = PORT_OE_IN;
  PortInitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;
  PORT_Init(MDR_PORTB, &PortInitStructure);
  
  
  /************************ PortB END *************************/
  
  /************************ PortD ************************/
 
  PortInitStructure.PORT_Pin = LED_PORTD | PORT_Pin_4;
  PortInitStructure.PORT_FUNC = PORT_FUNC_PORT;
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_SPEED = PORT_SPEED_SLOW;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;
  
  PORT_Init(MDR_PORTD, &PortInitStructure);
  
  PORT_ResetBits(MDR_PORTD, LED_PORTD);
//Room420  24.09.2019  No need   PORT_SetBits(MDR_PORTD, LED_PWR_H);
  /************************ PortD END *************************/
  
  
  /************************ PortF ************************/
 
  PortInitStructure.PORT_Pin = LED_PORTF;
  PortInitStructure.PORT_FUNC = PORT_FUNC_PORT;
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;

  PORT_Init(MDR_PORTF, &PortInitStructure);
  PORT_SetBits(MDR_PORTF, LED_OE_H);
  PORT_SetBits(MDR_PORTF, LED_RSK_L);
  PORT_ResetBits(MDR_PORTD, LED_SCLR_H);
  /************************ PortF END *************************/
  
  
  PortInitStructure.PORT_Pin = PORT_Pin_3+PORT_Pin_2 + PORT_Pin_0 + PORT_Pin_1;
  PortInitStructure.PORT_FUNC = PORT_FUNC_PORT;
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;

  PORT_Init(MDR_PORTE, &PortInitStructure);
  
  PORT_SetBits(MDR_PORTE, PORT_Pin_3);  // AO_RSK_L  trig in store registr  should give pusle 
  PORT_SetBits(MDR_PORTE, PORT_Pin_2);  // AO_OE_H  enable shift registr  if set then enable
  PORT_ResetBits(MDR_PORTE, PORT_Pin_1);   // Reset CLR REGISTR   If set then normal operation

  
 /* 
  PortInitStructure.PORT_Pin = PORT_Pin_6+PORT_Pin_7+PORT_Pin_8+PORT_Pin_9+PORT_Pin_10;
  PortInitStructure.PORT_FUNC = PORT_FUNC_PORT;
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_SPEED = PORT_SPEED_FAST;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;

  PORT_Init(MDR_PORTB, &PortInitStructure);
  PORT_SetBits(MDR_PORTB, PORT_Pin_6+PORT_Pin_7+PORT_Pin_8+PORT_Pin_9+PORT_Pin_10);*/
}

void InitDMA_SSP1_tx(unsigned char *buf,unsigned char cnt){
  
  /* Reset all DMA settings */
  DMA_StructInit(&DMA_InitSSP1_TX);

  /* DMA_Channel_SSP1_RX configuration ---------------------------------*/
  /* Set Primary Control Data */
  DMA_PriSSP1_TX.DMA_SourceBaseAddr = (uint32_t)buf;
  DMA_PriSSP1_TX.DMA_DestBaseAddr = (uint32_t)(&(MDR_SSP1->DR));
  DMA_PriSSP1_TX.DMA_SourceIncSize = DMA_SourceIncHalfword;
  DMA_PriSSP1_TX.DMA_DestIncSize = DMA_DestIncNo;
  DMA_PriSSP1_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_PriSSP1_TX.DMA_Mode = DMA_Mode_Basic;
  DMA_PriSSP1_TX.DMA_CycleSize = cnt;
  DMA_PriSSP1_TX.DMA_NumContinuous = DMA_Transfers_1;
  DMA_PriSSP1_TX.DMA_SourceProtCtrl = DMA_SourcePrivileged;
  DMA_PriSSP1_TX.DMA_DestProtCtrl = DMA_DestPrivileged;
  /* Set Channel Structure */
  DMA_InitSSP1_TX.DMA_PriCtrlData = &DMA_PriSSP1_TX;
  DMA_InitSSP1_TX.DMA_Priority = DMA_Priority_Default;
  DMA_InitSSP1_TX.DMA_UseBurst = DMA_BurstClear;
  DMA_InitSSP1_TX.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;
  /* Init DMA channel */
  DMA_Init(DMA_Channel_SSP1_TX, &DMA_InitSSP1_TX);
  
  SSP_ITConfig(MDR_SSP1,SSP_IT_RX,DISABLE);
  NVIC_ClearPendingIRQ(SSP1_IRQn);
  NVIC_DisableIRQ(SSP1_IRQn);
  
  DMA_Cmd(DMA_Channel_SSP1_TX, ENABLE);
  SSP_DMACmd(MDR_SSP1, (SSP_DMA_TXE), ENABLE); //включаем запросы от SSP
}

void InitDMA_SSP1(void)
{
  /* Reset all DMA settings */
  DMA_StructInit(&DMA_InitSSP1_TX);

  /* DMA_Channel_SSP1_RX configuration ---------------------------------*/
  /* Set Primary Control Data */
  DMA_PriSSP1_TX.DMA_SourceBaseAddr = (uint32_t)tx_buf;
  DMA_PriSSP1_TX.DMA_DestBaseAddr = (uint32_t)(&(MDR_SSP1->DR));
  DMA_PriSSP1_TX.DMA_SourceIncSize = DMA_SourceIncHalfword;
  DMA_PriSSP1_TX.DMA_DestIncSize = DMA_DestIncNo;
  DMA_PriSSP1_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_PriSSP1_TX.DMA_Mode = DMA_Mode_Basic;
  DMA_PriSSP1_TX.DMA_CycleSize = Led_group;
  DMA_PriSSP1_TX.DMA_NumContinuous = DMA_Transfers_4;
  DMA_PriSSP1_TX.DMA_SourceProtCtrl = DMA_SourcePrivileged;
  DMA_PriSSP1_TX.DMA_DestProtCtrl = DMA_DestPrivileged;
  /* Set Channel Structure */
  DMA_InitSSP1_TX.DMA_PriCtrlData = &DMA_PriSSP1_TX;
  DMA_InitSSP1_TX.DMA_Priority = DMA_Priority_Default;
  DMA_InitSSP1_TX.DMA_UseBurst = DMA_BurstClear;
  DMA_InitSSP1_TX.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;
  /* Init DMA channel */
  DMA_Init(DMA_Channel_SSP1_TX, &DMA_InitSSP1_TX);
  
  //DMA_Cmd(DMA_Channel_SSP1_RX, ENABLE); //включаем каналы DMA
  DMA_Cmd(DMA_Channel_SSP1_TX, ENABLE);
  SSP_DMACmd(MDR_SSP1, (SSP_DMA_TXE), ENABLE); //включаем запросы от SSP
}

void stopdma_ssp2(void)
{
  DMA_PriSSP2_RX.DMA_Mode = DMA_Mode_Stop;
  DMA_Init(DMA_Channel_SSP2_RX, &DMA_InitSSP2_RX);
}

void InitDMA_SSP2(unsigned char *buf,unsigned char cnt)
{
//stopdma_ssp2();
  /* Reset all DMA settings */
  PORT_SetBits(MDR_PORTA, PORT_Pin_4);
  PORT_ResetBits(MDR_PORTA, PORT_Pin_4);
  /* DMA_Channel_SSP1_RX configuration ---------------------------------*/
  /* Set Primary Control Data */
  DMA_PriSSP2_RX.DMA_SourceBaseAddr = (uint32_t)(&(MDR_SSP2->DR));
  DMA_PriSSP2_RX.DMA_DestBaseAddr = (uint32_t)DstBuf2;
  DMA_PriSSP2_RX.DMA_SourceIncSize = DMA_SourceIncNo;
  DMA_PriSSP2_RX.DMA_DestIncSize = DMA_DestIncByte;
  DMA_PriSSP2_RX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_PriSSP2_RX.DMA_Mode = DMA_Mode_Basic;
  DMA_PriSSP2_RX.DMA_CycleSize = cnt;
  DMA_PriSSP2_RX.DMA_NumContinuous = DMA_Transfers_4;
  DMA_PriSSP2_RX.DMA_SourceProtCtrl = DMA_SourcePrivileged;
  DMA_PriSSP2_RX.DMA_DestProtCtrl = DMA_DestPrivileged;
  /* Set Channel Structure */
  DMA_InitSSP2_RX.DMA_PriCtrlData = &DMA_PriSSP2_RX;
  DMA_InitSSP2_RX.DMA_Priority = DMA_Priority_High;
  DMA_InitSSP2_RX.DMA_UseBurst = DMA_BurstClear;
  DMA_InitSSP2_RX.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;
  /* Init DMA channel */
  //DMA_Init(DMA_Channel_SSP2_RX, &DMA_InitSSP2_RX);
  
  
  /* Reset all DMA settings */
 

  /* DMA_Channel_SSP1_RX configuration ---------------------------------*/
  /* Set Primary Control Data */
  DMA_PriSSP2_TX.DMA_SourceBaseAddr = (uint32_t)buf;
  DMA_PriSSP2_TX.DMA_DestBaseAddr = (uint32_t)(&(MDR_SSP2->DR));
  DMA_PriSSP2_TX.DMA_SourceIncSize = DMA_SourceIncByte;
  DMA_PriSSP2_TX.DMA_DestIncSize = DMA_DestIncNo;
  DMA_PriSSP2_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_PriSSP2_TX.DMA_Mode = DMA_Mode_Basic;
  DMA_PriSSP2_TX.DMA_CycleSize = cnt;
  DMA_PriSSP2_TX.DMA_NumContinuous = DMA_Transfers_4;
  DMA_PriSSP2_TX.DMA_SourceProtCtrl = DMA_SourcePrivileged;
  DMA_PriSSP2_TX.DMA_DestProtCtrl = DMA_DestPrivileged;
  /* Set Channel Structure */
  DMA_InitSSP2_TX.DMA_PriCtrlData = &DMA_PriSSP2_TX;
  DMA_InitSSP2_TX.DMA_Priority = DMA_Priority_Default;
  DMA_InitSSP2_TX.DMA_UseBurst = DMA_BurstClear;
  DMA_InitSSP2_TX.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;
  /* Init DMA channel */
  DMA_Init(DMA_Channel_SSP2_TX, &DMA_InitSSP2_TX);
  
  DMA_Cmd(DMA_Channel_SSP2_RX, ENABLE); //включаем каналы DMA
  DMA_Cmd(DMA_Channel_SSP2_TX, ENABLE);
  SSP_DMACmd(MDR_SSP2, (SSP_DMA_TXE | SSP_DMA_RXE), ENABLE); //включаем запросы от SSP
}


void SPIConfigure(void)
{

  PortInitStructure.PORT_Pin   = (PORT_Pin_2 );
  PortInitStructure.PORT_OE    = PORT_OE_IN;
  PortInitStructure.PORT_FUNC  = PORT_FUNC_ALTER;
  PortInitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInitStructure.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTD, &PortInitStructure);
  PortInitStructure.PORT_OE    = PORT_OE_OUT;
  PortInitStructure.PORT_Pin   = (PORT_Pin_6 | PORT_Pin_5);
  PORT_Init(MDR_PORTD, &PortInitStructure);

  PortInitStructure.PORT_Pin   = (PORT_Pin_3| PORT_Pin_1 | PORT_Pin_2);
  PortInitStructure.PORT_OE    = PORT_OE_IN;
  PortInitStructure.PORT_FUNC  = PORT_FUNC_ALTER;
  PortInitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
  PortInitStructure.PORT_SPEED = PORT_SPEED_MAXFAST;
  PORT_Init(MDR_PORTF, &PortInitStructure);
  PortInitStructure.PORT_Pin   = (PORT_Pin_0 );
  PortInitStructure.PORT_OE    = PORT_OE_OUT;
  PORT_Init(MDR_PORTF, &PortInitStructure);

  /* Reset all SSP settings */
  SSP_DeInit(MDR_SSP1);


  SSP_BRGInit(MDR_SSP1,SSP_HCLKdiv2);


  /* SSP2 MASTER configuration ------------------------------------------------*/
  SSP_StructInit (&sSSP);

  /* SSP1 SLAVE configuration ------------------------------------------------*/
  sSSP.SSP_SPH = SSP_SPH_2Edge;
  sSSP.SSP_SPO = SSP_SPO_High;
  sSSP.SSP_WordLength = SSP_WordLength16b;
  sSSP.SSP_SCR  = 0;
  sSSP.SSP_CPSDVSR = 12;
  sSSP.SSP_Mode = SSP_ModeSlave;
  SSP_Init (MDR_SSP1,&sSSP);
 
  /* Enable SSP1 */
  SSP_Cmd(MDR_SSP1, ENABLE);  
  
//  Это из проекта где используется SSP2 для светодиодов и EEPROM

#ifndef BSL_UART_CONFIG 
 
  /* SSP2 Interrupt ------------------------------------------------*/ 
//Room420  23.10.2019  SSP_ITConfig(MDR_SSP2,SSP_IT_RX,ENABLE);  // DISABLE as it is not need for SSP2

  DMA_StructInit(&DMA_InitSSP2_RX);
  DMA_StructInit(&DMA_InitSSP2_TX);

  
  SSP_DeInit(MDR_SSP2);
  SSP_ITConfig(MDR_SSP2,SSP_IT_RX,DISABLE);
  sSSP.SSP_SCR  = 1;
  sSSP.SSP_CPSDVSR = 10;
  sSSP.SSP_Mode = SSP_ModeMaster;
  sSSP.SSP_WordLength = SSP_WordLength8b;
  sSSP.SSP_SPH = SSP_SPH_1Edge;  //Room420  23.10.2019       SSP_SPH_1Edge;
  sSSP.SSP_SPO = SSP_SPO_High;   //SSP_SPO_High;// SSP_SPO_Low;//Room420  23.10.2019 SSP_SPO_High;
  sSSP.SSP_FRF =  SSP_FRF_SPI_Motorola; //Room420 was  working with LED   //SSP_FRF_SSI_TI;
  sSSP.SSP_HardwareFlowControl = SSP_HardwareFlowControl_SSE;//Room420  23.10.2019 SSP_HardwareFlowControl_SSE;
  SSP_Init (MDR_SSP2,&sSSP);
  
  
  //Room420  23.10.2019 NVIC_ClearPendingIRQ(SSP2_IRQn);//Room420  23.10.2019
//Room420  23.10.2019 NVIC_EnableIRQ(SSP2_IRQn);  //Room420  23.10.2019 
  
  SSP_ITConfig(MDR_SSP1,SSP_IT_RX,ENABLE);
  NVIC_ClearPendingIRQ(SSP1_IRQn);
  NVIC_EnableIRQ(SSP1_IRQn);
  
  /* Enable SSP1 DMA Rx and Tx request */
  SSP_DMACmd(MDR_SSP1,(SSP_DMA_TXE), ENABLE);


  /* Enable SSP2 DMA Rx and Tx request */
  SSP_BRGInit(MDR_SSP2,SSP_HCLKdiv2);  
//Room420  23.10.2019  SSP_DMACmd(MDR_SSP2,(SSP_DMA_RXE | SSP_DMA_TXE), ENABLE);
  
  
  SSP_DMACmd(MDR_SSP2,(SSP_DMA_TXE), ENABLE);
  
  InitDMA_SSP2(Led_Buf,Led_group);
  /* Enable SSP2 */
  SSP_Cmd(MDR_SSP2, ENABLE);
#endif
}

void init_SPI_Mode(unsigned short mode)
{
  SSP_DeInit(MDR_SSP1);
  
  SSP_BRGInit(MDR_SSP1,SSP_HCLKdiv2);
  
  SSP_StructInit (&sSSP);
  sSSP.SSP_SPH = mode;
  sSSP.SSP_SPO = SSP_SPO_Low;
  sSSP.SSP_WordLength = SSP_WordLength16b;
  sSSP.SSP_SCR  = 0;
  sSSP.SSP_CPSDVSR = 12;
  sSSP.SSP_Mode = SSP_ModeSlave;
  sSSP.SSP_FRF = SSP_FRF_SPI_Motorola;
  sSSP.SSP_HardwareFlowControl = SSP_HardwareFlowControl_SSE;
  
  SSP_Init (MDR_SSP1,&sSSP);
  SSP_ITConfig(MDR_SSP1,SSP_IT_RX,ENABLE);
  
  //NVIC_ClearPendingIRQ(SSP1_IRQn);
  NVIC_EnableIRQ(SSP1_IRQn);
  
  if(&DMA_InitSSP1_TX!=0x00){
    DMA_PriSSP1_TX.DMA_Mode = DMA_Mode_Stop;
    DMA_Init(DMA_Channel_SSP1_TX, &DMA_InitSSP1_TX);
  }
  /* Enable SSP1 DMA Rx and Tx request */
  SSP_DMACmd(MDR_SSP1,(SSP_DMA_TXE|SSP_DMA_RXE), DISABLE);
  DMA_Cmd(DMA_Channel_SSP1_TX, DISABLE);
  DMA_Cmd(DMA_Channel_SSP1_RX, DISABLE);
  
  SSP_Cmd(MDR_SSP1, ENABLE);
}
void UARTConfigure(void)
{
  
  

#ifdef BSL_UART_CONFIG 
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_FUNC = PORT_FUNC_ALTER;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;
  PortInitStructure.PORT_SPEED = PORT_SPEED_MAXFAST;
  PortInitStructure.PORT_Pin = PORT_Pin_1;
  PORT_Init(MDR_PORTD, &PortInitStructure);

  PortInitStructure.PORT_OE = PORT_OE_IN;
  PortInitStructure.PORT_Pin = PORT_Pin_0;
  PORT_Init(MDR_PORTD, &PortInitStructure);

  UART_DeInit(MDR_UART2);
  UART_BRGInit(MDR_UART2, UART_HCLKdiv1);
  NVIC_EnableIRQ(UART2_IRQn);

  UART_StructInit (&UART_InitStructure);

  UART_InitStructure.UART_BaudRate                = 115200;  //57600; //
  UART_InitStructure.UART_WordLength              = UART_WordLength8b;
  UART_InitStructure.UART_StopBits                = UART_StopBits1;
  UART_InitStructure.UART_Parity                  = UART_Parity_No;
  UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
  UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
  /* Configure UART1 parameters */
  UART_Init (MDR_UART2,&UART_InitStructure);

  //UART_ITConfig (MDR_UART2, UART_IT_RX, ENABLE);

  UART_Cmd(MDR_UART2,ENABLE);
#elif  
 
  
  
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_FUNC = PORT_FUNC_ALTER;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;
  PortInitStructure.PORT_SPEED = PORT_SPEED_MAXFAST;
  PortInitStructure.PORT_Pin = PORT_Pin_1;
  PORT_Init(MDR_PORTD, &PortInitStructure);

  PortInitStructure.PORT_OE = PORT_OE_IN;
  PortInitStructure.PORT_Pin = PORT_Pin_0;
  PORT_Init(MDR_PORTD, &PortInitStructure);
  
  UART_DeInit(MDR_UART2);
  UART_BRGInit(MDR_UART2, UART_HCLKdiv1);
  NVIC_EnableIRQ(UART2_IRQn);

  UART_StructInit (&UART_InitStructure);
  
  UART_InitStructure.UART_BaudRate                = 115000;
  UART_InitStructure.UART_WordLength              = UART_WordLength8b;
  UART_InitStructure.UART_StopBits                = UART_StopBits1;
  UART_InitStructure.UART_Parity                  = UART_Parity_No;
  UART_InitStructure.UART_FIFOMode                = UART_FIFO_OFF;
  UART_InitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
  /* Configure UART1 parameters */
  UART_Init (MDR_UART2,&UART_InitStructure);
  UART_DMAConfig(MDR_UART2,UART_IT_FIFO_LVL_2words,UART_IT_FIFO_LVL_2words);
  UART_DMACmd(MDR_UART2,(UART_DMA_RXE | UART_DMA_TXE), ENABLE);

  DMA_StructInit(&DMA_InitUART);

  /* DMA_Channel_UART1_RX configuration ---------------------------------*/
  /* Set Primary Control Data */
  DMA_PriUART.DMA_SourceBaseAddr = (uint32_t)(&(MDR_UART2->DR));
  DMA_PriUART.DMA_DestBaseAddr = (uint32_t)DstBuf2;
  DMA_PriUART.DMA_SourceIncSize = DMA_SourceIncNo;
  DMA_PriUART.DMA_DestIncSize = DMA_DestIncByte;
  DMA_PriUART.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_PriUART.DMA_Mode = DMA_Mode_Basic;
  DMA_PriUART.DMA_CycleSize = BufferSize;
  DMA_PriUART.DMA_NumContinuous = DMA_Transfers_8;
  DMA_PriUART.DMA_SourceProtCtrl = DMA_SourcePrivileged;
  DMA_PriUART.DMA_DestProtCtrl = DMA_DestPrivileged;
  /* Set Channel Structure */
  DMA_InitUART.DMA_PriCtrlData = &DMA_PriUART;
  DMA_InitUART.DMA_Priority = DMA_Priority_High;
  DMA_InitUART.DMA_UseBurst = DMA_BurstClear;
  DMA_InitUART.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;
  /* Init DMA channel */
  //DMA_Init(DMA_Channel_UART2_RX, &DMA_InitUART);
  UART_ITConfig (MDR_UART2, UART_IT_RX, ENABLE);


  /* DMA_Channel_UART1_TX configuration ---------------------------------*/
  /* Set Primary Control Data */
  DMA_PriUART.DMA_SourceBaseAddr = (uint32_t)g_ModbusBuf;
  DMA_PriUART.DMA_DestBaseAddr = (uint32_t)(&(MDR_UART2->DR));
  DMA_PriUART.DMA_SourceIncSize = DMA_SourceIncByte;
  DMA_PriUART.DMA_DestIncSize = DMA_DestIncNo;
  DMA_InitUART.DMA_Priority = DMA_Priority_Default;
  /* Init DMA channel */
  DMA_Init(DMA_Channel_UART2_TX, &DMA_InitUART);

  UART_Cmd(MDR_UART2,ENABLE);
#endif
}

void SendUart2_DMA(unsigned char *ptr,unsigned char cnt){
  
  DMA_StructInit(&DMA_InitUART);
  /* DMA_Channel_UART1_RX configuration ---------------------------------*/
  /* Set Primary Control Data */
  DMA_PriUART.DMA_SourceBaseAddr = (uint32_t)ptr;
  DMA_PriUART.DMA_DestBaseAddr = (uint32_t)(&(MDR_UART2->DR));
  DMA_PriUART.DMA_SourceIncSize = DMA_SourceIncByte;
  DMA_PriUART.DMA_DestIncSize = DMA_DestIncNo;
  DMA_PriUART.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_PriUART.DMA_Mode = DMA_Mode_Basic;
  DMA_PriUART.DMA_CycleSize = cnt;
  DMA_PriUART.DMA_NumContinuous = DMA_Transfers_8;
  DMA_PriUART.DMA_SourceProtCtrl = DMA_SourcePrivileged;
  DMA_PriUART.DMA_DestProtCtrl = DMA_DestPrivileged;
  /* Set Channel Structure */
  DMA_InitUART.DMA_PriCtrlData = &DMA_PriUART;
  DMA_InitUART.DMA_Priority = DMA_Priority_Default;
  DMA_InitUART.DMA_UseBurst = DMA_BurstClear;
  DMA_InitUART.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;
  DMA_Init(DMA_Channel_UART2_TX, &DMA_InitUART);

  UART_Cmd(MDR_UART2,ENABLE);
  
}

void SysTickStart(uint32_t ticks)
{
  SysTick->LOAD = 0x14000;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | \
                  SysTick_CTRL_CLKSOURCE_Msk;
}

#define FP_BOARD_SERIAL_LEN (16) 
struct fp_ack_board_info {
	uint16_t id;
	
	uint16_t hw_vers;
	uint16_t pof_vers;
	uint16_t sw_vers;
	uint8_t serial[FP_BOARD_SERIAL_LEN];
	uint32_t uptime;
}; 

void ADCConfigure(void){
  
  DMA_StructInit(&DMA_InitADC1);
  /* Set Primary Control Data */
  DMA_PriADC1.DMA_SourceBaseAddr = (uint32_t)(&(MDR_ADC->ADC1_RESULT));
  DMA_PriADC1.DMA_DestBaseAddr = (uint32_t)ADCConvertedValue;
  DMA_PriADC1.DMA_SourceIncSize = DMA_SourceIncNo;
  DMA_PriADC1.DMA_DestIncSize = DMA_DestIncHalfword;
  DMA_PriADC1.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_PriADC1.DMA_Mode = DMA_Mode_Stop;
  DMA_PriADC1.DMA_CycleSize = 10;
  DMA_PriADC1.DMA_NumContinuous = DMA_Transfers_1;
  DMA_PriADC1.DMA_SourceProtCtrl = DMA_SourcePrivileged;
  DMA_PriADC1.DMA_DestProtCtrl = DMA_DestPrivileged;
  
  /* Set Channel Structure */
  DMA_InitADC1.DMA_PriCtrlData = &DMA_PriADC1;
  DMA_InitADC1.DMA_Priority = DMA_Priority_Default;
  DMA_InitADC1.DMA_UseBurst = DMA_BurstClear;
  DMA_InitADC1.DMA_SelectDataStructure = DMA_CTRL_DATA_PRIMARY;

  /* Init DMA channel ADC1 */
  DMA_Init(DMA_Channel_ADC1, &DMA_InitADC1);

  /* Enable dma_req or dma_sreq to generate DMA request 
  MDR_DMA->CHNL_REQ_MASK_CLR = (1<<DMA_Channel_ADC1);
  MDR_DMA->CHNL_USEBURST_CLR = (1<<DMA_Channel_ADC1);*/

  /* Enable DMA channel ADC1 
  DMA_Cmd(DMA_Channel_ADC1, ENABLE);*/

  /* ADC Configuration */
  /* Reset all ADC settings */
  ADC_DeInit();
  ADC_StructInit(&sADC);

  sADC.ADC_SynchronousMode      = ADC_SyncMode_Independent;
  sADC.ADC_StartDelay           = 0;
  sADC.ADC_TempSensor           = ADC_TEMP_SENSOR_Enable;
  sADC.ADC_TempSensorAmplifier  = ADC_TEMP_SENSOR_AMPLIFIER_Enable;
  sADC.ADC_TempSensorConversion = ADC_TEMP_SENSOR_CONVERSION_Enable;
  sADC.ADC_IntVRefConversion    = ADC_VREF_CONVERSION_Disable;
  sADC.ADC_IntVRefTrimming      = 1;
  ADC_Init (&sADC);

  /* ADC1 Configuration */
  ADCx_StructInit (&sADCx);
  sADCx.ADC_ClockSource      = ADC_CLOCK_SOURCE_CPU;
  sADCx.ADC_SamplingMode     = ADC_SAMPLING_MODE_CICLIC_CONV;
  sADCx.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
  sADCx.ADC_ChannelNumber    = ADC_CH_TEMP_SENSOR;
  sADCx.ADC_Channels         = 0;
  sADCx.ADC_LevelControl     = ADC_LEVEL_CONTROL_Disable;
  sADCx.ADC_LowLevel         = 0;
  sADCx.ADC_HighLevel        = 0;
  sADCx.ADC_VRefSource       = ADC_VREF_SOURCE_INTERNAL;
  sADCx.ADC_IntVRefSource    = ADC_INT_VREF_SOURCE_INEXACT;
  sADCx.ADC_Prescaler        = ADC_CLK_div_512;
  sADCx.ADC_DelayGo          = 7;
  ADC1_Init (&sADCx);

  /* Enable ADC1 EOCIF and AWOIFEN interupts */
  ADC1_ITConfig((ADCx_IT_END_OF_CONVERSION  | ADCx_IT_OUT_OF_RANGE), DISABLE);

  /* ADC1 enable */
  ADC1_Cmd (ENABLE);

  /* Enable DMA IRQ */
  //NVIC_EnableIRQ(DMA_IRQn);
}


void ExtIntConfigure(void){
  
  TIMER_DeInit(MDR_TIMER1);

  TIMER_BRGInit(MDR_TIMER1,TIMER_HCLKdiv1);
  sTIM_CntInit.TIMER_Prescaler                = 0x10;
  sTIM_CntInit.TIMER_Period                   = 0x200;
  sTIM_CntInit.TIMER_CounterMode              = TIMER_CntMode_ClkFixedDir;
  sTIM_CntInit.TIMER_CounterDirection         = TIMER_CntDir_Up;
  sTIM_CntInit.TIMER_EventSource              = TIMER_EvSrc_CH1;
  sTIM_CntInit.TIMER_FilterSampling           = TIMER_FDTS_TIMER_CLK_div_1;
  sTIM_CntInit.TIMER_ARR_UpdateMode           = TIMER_ARR_Update_Immediately;
  sTIM_CntInit.TIMER_ETR_FilterConf           = TIMER_Filter_1FF_at_TIMER_CLK;
  sTIM_CntInit.TIMER_ETR_Prescaler            = TIMER_ETR_Prescaler_None;
  sTIM_CntInit.TIMER_ETR_Polarity             = TIMER_ETRPolarity_NonInverted;
  sTIM_CntInit.TIMER_BRK_Polarity             = TIMER_BRKPolarity_NonInverted;
  TIMER_CntInit (MDR_TIMER1,&sTIM_CntInit);

   /* Initializes the TIMER1 Channel2 -------------------------------------*/
  TIMER_ChnStructInit(&sTIM_ChnInit);

  sTIM_ChnInit.TIMER_CH_Number              = TIMER_CHANNEL1;
  sTIM_ChnInit.TIMER_CH_Mode                = TIMER_CH_MODE_CAPTURE;

  TIMER_ChnInit(MDR_TIMER1, &sTIM_ChnInit);

  /* Initializes the TIMER1 Channel2 Output -------------------------------*/

  TIMER_ChnOutStructInit(&sTIM_ChnOutInit);

  sTIM_ChnOutInit.TIMER_CH_Number                   = TIMER_CHANNEL1;
  sTIM_ChnOutInit.TIMER_CH_DirOut_Polarity          = TIMER_CHOPolarity_NonInverted;
  sTIM_ChnOutInit.TIMER_CH_DirOut_Source            = TIMER_CH_OutSrc_Only_0;
  sTIM_ChnOutInit.TIMER_CH_DirOut_Mode              = TIMER_CH_OutMode_Input;

  TIMER_ChnOutInit(MDR_TIMER1, &sTIM_ChnOutInit); 
  TIMER_ITConfig(MDR_TIMER1,TIMER_STATUS_CCR_CAP_CH1,ENABLE);
  NVIC_EnableIRQ(Timer1_IRQn);
  TIMER_Cmd(MDR_TIMER1,ENABLE);
}

void System_Init(void)
{
    struct fp_ack_board_info *ack;
  SCB->AIRCR = 0x05FA0000 | ((uint32_t)0x500);
  SCB->VTOR = 0x08000000;
//    SCB->VTOR = 0x08004000;  // for bootload 
  /* Disable all interrupt */
  NVIC->ICPR[0] = 0xFFFFFFFF;
  NVIC->ICER[0] = 0xFFFFFFFF;


  ClockConfigure();

  PortConfigure();
  SysTickStart(0);
  DMA_DeInit();
  
  
 
  SPIConfigure();
  init_SPI_Mode(SSP_SPH_2Edge);
  
  UARTConfigure();
 
  Init_UART2();
  
  fp_init();
  mod_fp_init();
  
}
