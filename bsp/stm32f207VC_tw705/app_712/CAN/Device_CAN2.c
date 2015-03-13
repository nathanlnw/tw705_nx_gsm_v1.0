/*
       Device  CAN    2
*/
#include <rtthread.h>
#include <rthw.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "Device_CAN.h"
#include <finsh.h>
#include "Device_CAN2.h"


u8   U3_Rx[100];
u8   U3_content[100];
u16   U3_content_len = 0;
u8   U3_flag = 0;
u16   U3_rxCounter = 0;


#ifdef RT_USING_DEVICE
struct rt_device  Device_CAN2;
#endif

void U3_RxProcess(void)
{
    u8  iRX;


    if(U3_content[1] == 0x33) // CAN 2
    {
        if(U3_content[2] == 0x03) // CAN 数据接收
        {
            //-------------------------------------------------
            //7e 33 03 08 00 00 00 00 00 20 00 04 00 08 30 31 32 33 34 35 36 07 00 7e
            memcpy(&RxMessageData, U3_content + 3, 20);


            DataTrans.Data_Tx[DataTrans.Tx_Wr++] = (BD_EXT.CAN_2_ID >> 24); //  BIT 7 can1  bit6 --1  CAN2 扩展
            DataTrans.Data_Tx[DataTrans.Tx_Wr++] = BD_EXT.CAN_2_ID >> 16;
            DataTrans.Data_Tx[DataTrans.Tx_Wr++] = BD_EXT.CAN_2_ID >> 8;
            DataTrans.Data_Tx[DataTrans.Tx_Wr++] = BD_EXT.CAN_2_ID;

            //  for(iRX=0;iRX<RxMessageData.DLC;iRX++)
            for(iRX = 0; iRX < 8; iRX++)
            {
                DataTrans.Data_Tx[DataTrans.Tx_Wr++] = RxMessageData.Data[iRX];
            }

            DataTrans.Data_TxLen = DataTrans.Tx_Wr;
            DataTrans.TYPE = 0x01; //  类型
            //--------------------------------------------------
        }

    }

}

u16  Protocol_808_Decode_Good(u8 *Instr , u8 *Outstr, u16  in_len) // 解析指定buffer :  UDP_HEX_Rx
{
    //-----------------------------------
    u16 i = 0, decode_len = 0;

    // 1.  clear  write_counter
    decode_len = 0; //clear DecodeLen

    // 2   decode process
    for(i = 0; i < in_len; i++)
    {
        if((Instr[i] == 0x7d) && (Instr[i + 1] == 0x02))
        {
            Outstr[decode_len] = 0x7e;
            i++;
        }
        else if((Instr[i] == 0x7d) && (Instr[i + 1] == 0x01))
        {
            Outstr[decode_len] = 0x7d;
            i++;
        }
        else
        {
            Outstr[decode_len] = Instr[i];
        }
        decode_len++;
    }
    //  3.  The  End
    return decode_len;
}

void CAN2_RxHandler(unsigned char rx_data)
{
#if 0
    if(U3_flag)
    {
        U3_Rx[U3_rxCounter++] = rx_data;
        if(rx_data == 0x7e)
        {
            U3_content_len = Protocol_808_Decode_Good(U3_Rx, U3_content, U3_rxCounter);

            U3_RxProcess();

            U3_flag = 0;
            U3_rxCounter = 0;
        }

    }
    else if((rx_data == 0x7e) && (U3_flag == 0))
    {
        U3_Rx[U3_rxCounter++] = rx_data;
        U3_flag = 1;
    }
    else
        U3_rxCounter = 0;
#endif

    if( rx_data != 0x0d )
    {
        U3_Rx[U3_rxCounter++] = rx_data;
    }
    else
    {
        rt_kprintf("%s", U3_Rx);
        U3_rxCounter = 0;
    }

}

void CAN2_putc(char c)
{
    USART_SendData( USART3, c );
    while( USART_GetFlagStatus( USART3, USART_FLAG_TC ) == RESET )
    {
    }
}

void u3_power(u8 i)
{
    if(i)
    {
        U3_OUT_PWR_ON;
    }
    else
    {
        U3_OUT_PWR_OFF;
    }
}
//FINSH_FUNCTION_EXPORT(u3_power, u3_power[1|0]);

static rt_err_t   Device_CAN2_init( rt_device_t dev )
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;


    //  1 . Clock
    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* Enable USART3 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    //   2.  GPIO
    /* Configure USART3 Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure USART3 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    //  3.  Interrupt
    /* Enable the USART3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    //   4.  uart  Initial
    USART_InitStructure.USART_BaudRate = 9600;    //CAN2
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(USART3, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);


    return RT_EOK;
}

static rt_err_t Device_CAN2_open( rt_device_t dev, rt_uint16_t oflag )
{
    return RT_EOK;
}
static rt_err_t Device_CAN2_close( rt_device_t dev )
{
    return RT_EOK;
}

static rt_size_t Device_CAN2_read( rt_device_t dev, rt_off_t pos, void *buff, rt_size_t count )
{

    return RT_EOK;
}

static rt_size_t Device_CAN2_write( rt_device_t dev, rt_off_t pos, const void *buff, rt_size_t count )
{
    unsigned int  Info_len485 = 0;
    const char		*p	= (const char *)buff;


    Info_len485 = (unsigned int)count;
    /* empty console output */
    //--------  add by  nathanlnw ---------
    while (Info_len485)
    {
        CAN2_putc (*p++);
        Info_len485--;
    }
    //--------  add by  nathanlnw  --------
    return RT_EOK;
}
static rt_err_t Device_CAN2_control( rt_device_t dev, rt_uint8_t cmd, void *arg )
{
    return RT_EOK;
}


void  Device_CAN2_regist(void )
{
    Device_CAN2.type	= RT_Device_Class_Char;
    Device_CAN2.init	=   Device_CAN2_init;
    Device_CAN2.open	=  Device_CAN2_open;
    Device_CAN2.close	=  Device_CAN2_close;
    Device_CAN2.read	=  Device_CAN2_read;
    Device_CAN2.write	=  Device_CAN2_write;
    Device_CAN2.control = Device_CAN2_control;

    rt_device_register( &Device_CAN2, "CAN2", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE );
    rt_device_init( &Device_CAN2 );
}
