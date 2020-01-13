/**
  ******************************************************************************
  * @file    MDR32F9Qx_it.c
  * @author  Phyton Application Team
  * @version V2.0.0
  * @date    22.07.2011
  * @brief   Main Interrupt Service Routines.
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
#include <MDR32F9Qx_uart.h>
#include <MDR32F9Qx_adc.h>
#include <MDR32F9Qx_dma.h>
#include <MDR32F9Qx_ssp.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_timer.h>
#include <MDR32F9Qx_it.h>
#include "time.h"
#include "serial.h"
#include "config.h"

/** @addtogroup __MDR32F9Qx_Eval_Demo MDR32F9Qx Demonstration Example
  * @{
  */

/** @addtogroup Interrupt_Service_Routines Interrupt Service Routines
  * @{
  */

/** @addtogroup Interrupt_Service_Private_Variables Interrupt Service Private Variables
  * @{
  */

/* Timer counter */
vuint32_t TimerCounter = 0;

vuint32_t g_uptime = 0;   //аптайм в секундах

/** @} */ /* End of group Interrupt_Service_Private_Variables */

/** @defgroup Interrupt_Service_Private_Functions Interrupt Service Private Functions
  * @{
  */



/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
  static int counter_for_seconds = 0;
  TimerCounter+=1;
  if (++counter_for_seconds == 1000) { counter_for_seconds = 0; g_uptime++;
  if (g_uptime % 2){

// Koval for testing      UART_SendData (MDR_UART2,0x31);
  } else {
// Koval for testing      UART_SendData (MDR_UART2,0x32);
  }
  } //TODO: убедиться на осциллоскопере в корректности интервала
}

/*******************************************************************************
* Function Name  : UARTx_HandlerWork, UART1_IRQHandler, UART2_IRQHandler
* Description    : These functions handle UARTx global interrupt requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void UART1_IRQHandler(void)
{
//  UARTx_HandlerWork(MDR_UART1);
}

void UART2_IRQHandler(void)
{
  //UARTx_HandlerWork(MDR_UART2);
  if (UART_GetITStatusMasked(MDR_UART2, UART_IT_RX) == SET)
  {
    UART_ClearITPendingBit(MDR_UART2, UART_IT_RX);
    unsigned char b = UART_ReceiveData(MDR_UART2);
    if (U2.mode != UART_RECEIVE_MODE) return;	//прием только в соответствующем состоянии
    if (U2.cnt >= MAX_BUF) U2.cnt = 0;  	//переполнение
    U2.buf[U2.cnt++] = b;                       // нормальное добавление в буфер
    U2.rx_last_time = TimerCounter;
  }

}


void SSP1_IRQHandler(void)
{
  unsigned short b = SSP_ReceiveData(MDR_SSP1);
  NVIC_ClearPendingIRQ(SSP1_IRQn);

  //rx_buf[ssp1_cnt/2]|=((b^0xFFFF)<<16*((ssp1_cnt%2)));
  // rx_buf[ssp1_cnt]=(b^0xFFFF);
  DstBuf1[ssp1_cnt++]=(b^0xFFFF);

  if(ssp1_cnt>90){
    ssp1_cnt=0;
    //SSP_Target=tNONE;
  }
}

void SSP2_IRQHandler(void)
{
  unsigned short b = SSP_ReceiveData(MDR_SSP2);
  NVIC_ClearPendingIRQ(SSP2_IRQn);
  DstBuf2[ssp2_cnt++]=(unsigned char)(b);
  if(ssp2_cnt>17){
    ssp2_cnt=0;
    //SSP_Target=tNONE;
  }
}





/*******************************************************************************
* Function Name  : BACKUP_IRQHandler
* Description    : This function handles BACKUP global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BACKUP_IRQHandler(void)
{
}


void Timer1_IRQHandler(void)
{
  NVIC_ClearPendingIRQ(Timer1_IRQn);
  TIMER_ClearFlag(MDR_TIMER1,0xFFFF);
  //PORT_XORBits(MDR_PORTB, PORT_Pin_5);
}


/** @} */ /* End of group Interrupt_Service_Private_Functions */

/** @} */ /* End of group Interrupt_Service_Routines */

/** @} */ /* End of group __MDR32F9Qx_Eval_Demo */

/******************* (C) COPYRIGHT 2011 Phyton *********************************
*
* END OF FILE MDR32F9Qx_it.c */


