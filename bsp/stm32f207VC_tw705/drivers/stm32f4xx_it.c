/**
  ******************************************************************************
  * @file    IO_Toggle/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <./include/drivers/serial.h>
#include "stm32f2xx.h"
#include <rtthread.h>
#include "board.h"
#include "App_moduleConfig.h"


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
//void HardFault_Handler(void)
//{
//    // definition in libcpu/arm/cortex-m4/context_*.S
//}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//    // definition in libcpu/arm/cortex-m4/context_*.S
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//    // definition in boarc.c
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
void WWDG_IRQHandler(void)
{
    WWDG_SetCounter(0x7F);
    /* Clear EWI flag */
    WWDG_ClearFlag();
    /* Update WWDG counter */
    // WWDG_SetCounter(0x7F);

}


void USART2_IRQHandler(void)
{
#ifdef RT_USING_UART2
    extern struct rt_device uart2_device;
    extern void rt_hw_serial_isr(struct rt_device * device);
    u8  data = 0;

    /* enter interrupt */
    rt_interrupt_enter();

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        data = USART_ReceiveData(USART2) & 0xFF;
        _485_RxHandler(data);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
#endif
}

void USART3_IRQHandler(void)
{
#ifdef RT_USING_UART3
    extern struct rt_device uart3_device;
    extern void rt_hw_serial_isr(struct rt_device * device);
    u8  data = 0;
    /* enter interrupt */
    rt_interrupt_enter();

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        data = USART_ReceiveData(USART3) & 0xFF;
        CAN2_RxHandler(data);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }

    /* leave interrupt */
    rt_interrupt_leave();
#endif
}

void UART4_IRQHandler(void)
{
#ifdef RT_USING_UART4
    extern struct rt_device uart4_device;
    extern void rt_hw_serial_isr(struct rt_device * device);
    u8  data = 0;
    /* enter interrupt */
    rt_interrupt_enter();

    // rt_hw_serial_isr(&uart4_device);
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        data = USART_ReceiveData(UART4) & 0xFF;
        GSM_RxHandler(data);
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
#endif
}

void CAN1_RX0_IRQHandler(void)
{

    if(SET == CAN_GetITStatus(CAN1, CAN_IT_FF0))
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
    }
    else if(SET == CAN_GetITStatus(CAN1, CAN_IT_FOV0))
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FOV0);
    }
    else
    {
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessageData);
        CAN1_Rx_Process();
    }

}


/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
