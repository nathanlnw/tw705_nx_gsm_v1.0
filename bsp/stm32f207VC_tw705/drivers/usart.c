/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 */

#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <stdio.h>
#include <./include/drivers/serial.h>

/*
 * Use UART1 as console output and finsh input
 * interrupt Rx and poll Tx (stream mode)
 *
 * Use UART2 with interrupt Rx and poll Tx
 * Use UART3 with DMA Tx and interrupt Rx -- DMA channel 2
 *
 * USART DMA setting on STM32
 * USART1 Tx --> DMA Channel 4
 * USART1 Rx --> DMA Channel 5
 * USART2 Tx --> DMA Channel 7
 * USART2 Rx --> DMA Channel 6
 * USART3 Tx --> DMA Channel 2
 * USART3 Rx --> DMA Channel 3
 */

#ifdef RT_USING_UART1
struct stm32_serial_int_rx uart1_int_rx;
struct stm32_serial_device uart1 =
{
    USART1,
    &uart1_int_rx,
    RT_NULL
};
struct rt_device uart1_device;
#endif

#ifdef RT_USING_UART2
struct stm32_serial_int_rx uart2_int_rx;
struct stm32_serial_device uart2 =
{
    USART2,
    &uart2_int_rx,
    RT_NULL
};
struct rt_device uart2_device;
#endif

#ifdef RT_USING_UART3
struct stm32_serial_int_rx uart3_int_rx;
//struct stm32_serial_dma_tx uart3_dma_tx;  // mask by nathan
struct stm32_serial_device uart3 =
{
    USART3,
    &uart3_int_rx,
    RT_NULL                        //&uart3_dma_tx      modify  by  nathan
};
struct rt_device uart3_device;
#endif

#ifdef RT_USING_UART4
struct stm32_serial_int_rx uart4_int_rx;
struct stm32_serial_device uart4 =
{
    UART4,
    &uart4_int_rx,
    RT_NULL
};
struct rt_device uart4_device;
#endif

#ifdef RT_USING_UART5
struct stm32_serial_int_rx uart5_int_rx;
struct stm32_serial_device uart5 =
{
    UART5,
    &uart5_int_rx,
    RT_NULL
};
struct rt_device uart5_device;
#endif

//------ Related with  DMA -------
#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404
#define USART3_DR_Base  0x40004804


/* USART1_REMAP = 0 */
#define UART1_GPIO_TX		GPIO_Pin_9
#define UART1_GPIO_RX		GPIO_Pin_10
#define UART1_GPIO			GPIOA
#define RCC_APBPeriph_UART1	RCC_APB2Periph_USART1
#define UART1_TX_DMA		DMA_Channel_4
#define UART1_RX_DMA		DMA_Channel_5

/* USART2 */
#define UART2_GPIO_TX	    GPIO_Pin_2
#define UART2_GPIO_RX	    GPIO_Pin_3
#define UART2_GPIO	    	GPIOA
#define RCC_APBPeriph_UART2	RCC_APB1Periph_USART2

/* USART3_REMAP[1:0] = 00 */
#define UART3_GPIO_RX		GPIO_Pin_11
#define UART3_GPIO_TX		GPIO_Pin_10
#define UART3_GPIO			GPIOB
#define RCC_APBPeriph_UART3	RCC_APB1Periph_USART3
#define UART3_TX_DMA		DMA_Channel_2
#define UART3_RX_DMA		DMA_Channel_3

/*USART4 */
#define UART4_GPIO_TX	    GPIO_Pin_10
#define UART4_GPIO_RX	    GPIO_Pin_11
#define UART4_GPIO	    	    GPIOC
#define RCC_APBPeriph_UART4	RCC_APB1Periph_UART4

/*USART5 */
#define UART5_GPIO_TX	    GPIO_Pin_12      // PC12
#define UART5_GPIO_RX	    GPIO_Pin_2        // PD2
#define UART5_GPIO_TxC	    	    GPIOC
#define UART5_GPIO_RxD	    	    GPIOD
#define RCC_APBPeriph_UART5	RCC_APB1Periph_UART5


static void RCC_Configuration(void)
{
#ifdef RT_USING_UART1
    /* Enable USART1 and GPIOA clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

#endif

#ifdef RT_USING_UART2
    /* Enable USART2 and GPIOA clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Enable USART2 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif

#ifdef RT_USING_UART3
    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* Enable USART3 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* DMA clock enable */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
#endif


#ifdef RT_USING_UART4
    /* Enable USART4 and GPIOC clocks    PC10 -RX    PC11--Tx*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* Enable USART4 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#endif

#ifdef RT_USING_UART5
    /* Enable USART5 and GPIOC clocks    PC12 -RX    PD2--Tx*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable USART5 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
#endif
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

#ifdef RT_USING_UART1
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

#endif

#ifdef RT_USING_UART2
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX | UART2_GPIO_RX;
    GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
#endif

#ifdef RT_USING_UART3
    /* Configure USART3 Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

    /* Configure USART3 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
#endif


#ifdef RT_USING_UART4
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* Configure USART3 Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = UART4_GPIO_RX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);

    /* Configure USART3 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = UART4_GPIO_TX;
    GPIO_Init(UART4_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
#endif

#ifdef RT_USING_UART5

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* Configure USART1 Rx (PD.2) as input floating */
    GPIO_InitStructure.GPIO_Pin = UART5_GPIO_RX;
    GPIO_Init(UART5_GPIO_RxD, &GPIO_InitStructure);

    /* Configure USART1 Tx (PC.12) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = UART5_GPIO_TX;
    GPIO_Init(UART5_GPIO_TxC, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);


#endif

    //------------------------------------

}

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

#ifdef RT_USING_UART1
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef RT_USING_UART2
    /* Enable the USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef RT_USING_UART3
    /* Enable the USART3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#endif

#ifdef RT_USING_UART4
    /*  485  Bus   ½Ó¿Ú*/
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef RT_USING_UART5
    /*  GPS interface  */
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

static void DMA_Configuration(void)
{

}

//volatile USART_TypeDef * uart2_debug = USART2;
/*
 * Init all related hardware in here
 * rt_hw_serial_init() will register all supported USART device
 */
void rt_hw_usart_init()
{
    USART_InitTypeDef USART_InitStructure;

    RCC_Configuration();

    GPIO_Configuration();

    NVIC_Configuration();

    DMA_Configuration();

    /* uart init */
#ifdef RT_USING_UART1
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* register uart1 */
    rt_hw_serial_register(&uart1_device, "uart1",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          &uart1);

    /* enable interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif

#ifdef RT_USING_UART2
    USART_InitStructure.USART_BaudRate = 57600;  //485
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    /* register uart2 */
    rt_hw_serial_register(&uart2_device, "uart2",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          &uart2);

    /* Enable USART2 DMA Rx request */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#endif

#ifdef RT_USING_UART3
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);


    /* register uart3 */
    //rt_hw_serial_register(&uart3_device, "uart3",
    //RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX,
    //&uart3);

    rt_hw_serial_register(&uart3_device, "uart3",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          &uart3);

    /* Enable USART3 DMA Tx request */
    //USART_DMACmd(USART3, USART_DMAReq_Tx , ENABLE);  //mask by nathan

    /* enable interrupt */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
#endif

#ifdef RT_USING_UART4
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &USART_InitStructure);

    /* register uart4 */
    rt_hw_serial_register(&uart4_device, "uart4",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          &uart4);

    /* enable interrupt */
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
#endif

#ifdef RT_USING_UART5
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    /* register uart1 */
    rt_hw_serial_register(&uart5_device, "uart5",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          &uart5);

    /* enable interrupt */
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
#endif
}
