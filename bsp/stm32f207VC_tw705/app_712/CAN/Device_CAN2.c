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

// 油耗相关
#define   ABNORMAL_MAXTIMES      10   //30
#define   Zero_Clear_MAXTIMES    10   // 60
#define   OIL_CONNECT_MAXTIME    120 //  300  5 分钟


YH   Oil;


#ifdef RT_USING_DEVICE
struct rt_device  Device_CAN2;
#endif



unsigned long AssicBufToUL(char * buf,unsigned int num)
{
 unsigned char tempChar;
 unsigned int i,j;
 unsigned long retLong=0;
 
 for(i=0;i<num;i++)
 	{
 	tempChar=(unsigned char)buf[i];
	if((tempChar>='0')&&(tempChar<='9'))
		{
	 	retLong*=10;
		retLong+=tempChar-'0';
		}
	else
		{
		return retLong;
		}
 	}
 return retLong;
}

uint8_t u3_process_YH( uint8_t * pinfo )
{
	//检查数据完整性,执行数据转换
	uint8_t		i;
	uint8_t		buf[32];
	uint8_t		commacount	= 0, count = 0;
	uint8_t		*psrc		= pinfo + 7; //指向开始位置
	uint8_t		crc = 0,crc2;
	u32  temp_u32data=0;

	for(i=0;i<52;i++) 
		{
		crc += pinfo[i];
		}
	//rt_kprintf("\n   CRC1    =0x%2X",crc); 
	while( *psrc++ )
	{
		if(( *psrc != ',' )&&(*psrc != '#'))
		{
			buf[count++]	= *psrc;
			buf[count]		= 0;
			if(count+1 >= sizeof(buf))
				return 1;
			continue;
		}
		commacount++;
		switch( commacount )
		{
			case 1: /*协议相关内容，4字节*/
				break;

			case 2: /*硬件相关版本等，3字节*/
				break;

			case 3: /*设备上线时长，千时，百时，十时，个时，十分，个分；6字节*/
				break;

			case 4: /*平滑处理后的液位值；5字节*/
				if(count<5)
					return 0;
				Oil.oil_average_value = AssicBufToUL(buf,5);
				//rt_kprintf("\n 液位平均值=%d",Oil.oil_average_value);
				break;
			case 5: /*数据滤波等级；1字节*/
				break;
			case 6: /*车辆运行和停止状态；2字节*/
				break;
			case 7: /*当前实时液位正负1cm范围内的液位个数；2字节*/
				break;
			case 8: /*接收信号灵明度，共85级；2字节*/
				break;
			case 9: /*信号强度，最大99；2字节*/
				break;
			case 10: /*实时液位，精度0.1mm；5字节*/
				if(count<5)
					return 0;
				Oil.oil_realtime_value = AssicBufToUL(buf,5);
				//rt_kprintf(" 实时值=%d",Oil.oil_realtime_value);
				break;
			case 11: /*满量程，默认为800；5字节*/
				if(count<5)
					return 0;
				temp_u32data = AssicBufToUL(buf,5);
				//rt_kprintf(" 满量程=%d",temp_u32data);
				break;
			case 12: /*从"*"开始前面52个字符的和；2字节*/
				  Ascii_To_Hex(buf,&crc2,1);  
				//rt_kprintf("\n   CRC2    =0x%2X",crc2);  
				if(crc2 == crc)
				 {	
				   //  Data come  ,  update  every packet
				    Oil.oil_YH_no_data_Counter=0;

                   //  check the   change  
					if(Oil.oil_realtime_value!= Oil.oil_average_value)  // 实时数值和 平均不等更新实时数值
						{
						  Oil.oil_value = Oil.oil_average_value;
						 if((GB19056.workstate == 0)&&(DispContent))
						     rt_kprintf("\r\n Finalvalue update =%d",Oil.oil_value);

						   //  update Oil workstate
						   if(Oil.oil_realtime_value)
				            {
				               Oil.oil_YH_workstate=OIL_NORMAL;
							   Oil.oil_YH_no_data_Counter=0;
							   Oil.oil_YH_0_value_cacheCounter=0;
							   Oil.oil_YH_Abnormal_counter=0;
							   Warn_Status[0] &= ~0x02; //   油量异常还原正常
						   	}  
						   else
						   	{   // check  with vehicle  speed 
						   	  if(Oil.oil_YH_workstate==OIL_NORMAL)
						   	  {
								   	    if(Speed_gps >100)  //  Speed_gps > 10 km/h    
		                                {
		                                   Oil.oil_YH_Abnormal_counter++;
										   if(Oil.oil_YH_Abnormal_counter>ABNORMAL_MAXTIMES)  //  30次是300s   5分钟
										   	{
										   	   Oil.oil_YH_workstate=OIL_ABNORMAL;
											   Oil.oil_YH_Abnormal_counter=0;
											    Warn_Status[0] |= 0x02; //   油量异常还原正常
		                                      // rt_kprintf("\r\n 油耗盒工作异常"); 
											   
										   	}
										    Oil.oil_YH_0_value_cacheCounter=0; 

								   	    } 
										else
										{
										   if(Oil.oil_YH_Abnormal_counter)
										   	{
										   	   Oil.oil_YH_0_value_cacheCounter++;
											   if(Oil.oil_YH_0_value_cacheCounter>Zero_Clear_MAXTIMES)
											   	{
		                                           Oil.oil_YH_0_value_cacheCounter=0;
									               Oil.oil_YH_Abnormal_counter=0;    // just  clear coutner  not  change the state
									              // rt_kprintf("\r\n 速度小于10km 清除中"); 
											   	}
										   	}							    

										}
								
						       }
						   	}
						}
				    // 	Debug related
						if((GB19056.workstate == 0)&&(DispContent))
					       rt_kprintf("\r\n Average =%d,realtime=%d,final=%d 剩余量=%d.%d升  speed=%d  \r\n",Oil.oil_average_value,Oil.oil_realtime_value,Oil.oil_value,Oil.oil_value/10,Oil.oil_value%10,Speed_gps);  
				 } 
				break;
		}
		count	= 0;
		buf[0]	= 0;
	}
	return 9;
}


//   检查油耗盒的连接状态      in     1s  
void Oil_Sensor_Connect_Checking(void)
{
  if(Oil.oil_YH_workstate)
   {
     Oil.oil_YH_no_data_Counter++;
	 if(Oil.oil_YH_no_data_Counter>OIL_CONNECT_MAXTIME)
	 {
	       Oil.oil_YH_no_data_Counter=0;
           Oil.oil_YH_workstate=OIL_NOCONNECT;
	 	   Oil.oil_YH_no_data_Counter=0;
	 	   Oil.oil_YH_0_value_cacheCounter=0;
	 	   Oil.oil_YH_Abnormal_counter=0;    // 油耗异常状态
	 	   Warn_Status[0] &= ~0x02; //   油量异常还原正常
	 	   	if((GB19056.workstate == 0)&&(DispContent))
	 	   		rt_kprintf("\r\n     油耗盒断开"); 
  	 }
	
   }
}


void U3_RxProcess(void)
{
   u8  iRX;
   
   #if  0  
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
	#endif

	if(U3_flag)
	{
		if((GB19056.workstate == 0)&&(DispContent)) 
		  rt_kprintf("%s", U3_Rx);
		u3_process_YH(U3_Rx); 
		U3_flag=0;
        U3_rxCounter = 0;
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

void u3_RxHandler(unsigned char rx_data)
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
        U3_flag = 1; 
		U3_RxProcess(); 
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
