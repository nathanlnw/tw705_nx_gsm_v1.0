/************************************************************
 * Copyright (C), 2008-2012,
 * FileName:		// 文件名
 * Author:			// 作者
 * Date:			// 日期
 * Description:		// 模块描述
 * Version:			// 版本信息
 * Function List:	// 主要函数及其功能
 *     1. -------
 * History:			// 历史修改记录
 *     <author>  <time>   <version >   <desc>
 *     David    96/10/12     1.0     build this moudle
 ***********************************************************/
#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <dfs_posix.h>

#include "stm32f2xx.h"
#include "gps.h"
#include "App_moduleConfig.h"

#include <finsh.h>

#define GPS_PWR_PORT	GPIOD
#define GPS_PWR_PIN		GPIO_Pin_10

#define GPS_GPIO_TX			GPIO_Pin_12 // PC12
#define GPS_GPIO_RX			GPIO_Pin_2  // PD2
#define GPS_GPIO_TxC		GPIOC
#define GPS_GPIO_RxD		GPIOD
#define RCC_APBPeriph_UART5 RCC_APB1Periph_UART5  

/*声明一个gps设备*/
static struct rt_device dev_gps;

/*串口接收缓存区定义*/
#define UART5_RX_SIZE 256

#define GPS_RAWINFO_SIZE    2048 
#define  NMEA_SEGlength     128


#define  GPS_V_LIMIT         500  // 500s       

typedef __packed struct
{
	uint16_t	wr;
	uint8_t		body[UART5_RX_SIZE];
}LENGTH_BUF;

static LENGTH_BUF uart5_rxbuf;
static LENGTH_BUF gps_rx;
//static uint8_t	uart5_rxbuf[UART5_RX_SIZE];	/*预留前两个字节，保存长度*/
//static uint16_t uart5_rxbuf_wr = 2;

/*gps原始信息数据区定义*/
static struct rt_messagequeue	mq_gps;

uint8_t							flag_bd_upgrade_uart = 0;

//---------------------
static   u8   gps_rawinfo[GPS_RAWINFO_SIZE];
//---------------------



//==========================

/*  RT_thread    应用相关*/
u8	 systemTick_TriggerGPS=1;  //	触发系统时间定时发送GPS标志    默认启动本地定时 
u16  systemTick_trigGPS_counter=0;//触发系统定时下的时间计数器


const char BD_MODE[]={"$CCSIR,1,0*49\r\n"};
const char GPS_MODE[]={"$CCSIR,2,0*4A\r\n"};
const char GPSBD_MODE[]={"$CCSIR,3,0*4B\r\n"};  

GPSSTATUS    GpsStatus;    
GPS_ABNORMAL   Gps_Exception;  // gps  模块异常处理
POS_ASC Posit_ASCII;  // 位置信息 ASCII
//==========================
static rt_err_t dev_gps_init( rt_device_t dev );


void GPS_Abnormal_init(void)
{
   Gps_Exception.current_datacou=0;
   Gps_Exception.no_updateTimer=0;
   Gps_Exception.last_datacou=0;
   Gps_Exception.GPS_Rst_counter=0;
   Gps_Exception.Reset_gps=0; 
   Gps_Exception.GPS_V_counter=0;
}

//    GPS  长期保持 V  处理
void  GPS_Keep_V_timer(void)
{
   if(((Warn_Status[3]&0x60)==0x00)&&(UDP_dataPacket_flag==0x03) ) // 在天线状态正常情况下 判断V 
   	{
   	     Gps_Exception.GPS_V_counter++;
		 if(Gps_Exception.GPS_V_counter==GPS_V_LIMIT)
		   {
		     gps_onoff(0);
			 StatusReg_GPS_V();	
            // rt_kprintf("\r\n gps long v  cut power\r\n");  
		   }	 
		 if(Gps_Exception.GPS_V_counter>=(GPS_V_LIMIT+3)) 
		 	{
		 	      gps_onoff(1);  // Gps module Power on	GPS 模块开电  		 	
                  Gps_Exception.GPS_V_counter=0;				  
				//  rt_kprintf("\r\n gps long v  recover power\r\n");  
		 	}
   	}
   else
   	if(UDP_dataPacket_flag==0x02)
   	{   	  
		  Gps_Exception.GPS_V_counter=0;
   	}

}

void  GPS_Abnormal_process(void)
{
    		  //----------GPS 异常情况处理-----------------------
		   Gps_Exception.no_updateTimer++;
		  if( Gps_Exception.no_updateTimer>=300) 
			{
			   Gps_Exception.no_updateTimer=0;
			  if(Gps_Exception.Reset_gps==1)
				{
				  Gps_Exception.Reset_gps=0;
				  gps_onoff(1); // GPS power on
				   Gps_Exception.GPS_Rst_counter++;
				   if(Gps_Exception.GPS_Rst_counter>5)
				   	{
                                        Gps_Exception.GPS_Rst_counter=0; 
				   	}
				}  
				else
				{
					if(Gps_Exception.current_datacou-Gps_Exception.last_datacou<300) 
						{
						        gps_onoff(0); //GPS  power off
							  Gps_Exception.Reset_gps=1;						  
							  //---------断电就置为不定位 -----------------						  
							  UDP_dataPacket_flag=0X03;
							  Car_Status[3]&=~0x02;   //Bit(1)
	                                            ModuleStatus &= ~Status_GPS;
							  GPS_getfirst=0;
						  //-------------------------------------------
						}
					else
						{ 
						   Gps_Exception.last_datacou=0;
						   Gps_Exception.current_datacou=0;          
						}
				}
	    }




}


//$GPRMC,063835.00,A,3909.11361,N,11712.50398,E,0.192,150.85,110305,,,A*64
//$GPRMC,063835.00,A,4000.81000,N,11556.40000,E,0.192,150.85,110305,,,A*64
//$GNRMC,125146.000,V,0000.0000,N,00000.0000,E,0.0,,080614,,E,N*0D


u8 Process_RMC(u8* packet)  
{
	u8  CommaCount=0,iCount=0,k=0; 
	u8  tmpinfo[15]; // 新版本的北斗模块经度更高了
	                          //$GNRMC,085928.00,A,3920.020977,N,11744.385579,E,0.7,,020113,,,A*67
	                          //$GNRMC,090954.00,A,3920.024800,N,11744.384457,E,0.3,,020113,,,A*65
	                          //           11744.385579    
	u8  hour=0,min=0,sec=0,fDateModify=0;

    //----------------------------------------------

    //----------------- Initial Speed and Direction -----------------    
    GPRMC_Funs.Speed(tmpinfo, INIT, k);	
    GPRMC_Funs.Direction(tmpinfo, INIT, k);  	
    //-------------------------------------------------------------------	
 	while (*packet!=0){ 
		if(*packet==','){
			CommaCount++;
			packet++; 
			if(iCount==0) continue; 
			switch(CommaCount){
				case 2: //时间 
				   //systemTickGPS_Set();    
				   if ( iCount < 6 ) 	  //  格式检查
				  {    
				      StatusReg_GPS_V();  
					  return false;
				  }

				    if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)&&(tmpinfo[5]>=0x30)&&(tmpinfo[5]<=0x39)) 
				          ;
				   else
					{  StatusReg_GPS_V();   	return false;}

				       
					hour=(tmpinfo[0]-0x30)*10+(tmpinfo[1]-0x30)+8;
					min=(tmpinfo[2]-0x30)*10+(tmpinfo[3]-0x30);
					sec=(tmpinfo[4]-0x30)*10+(tmpinfo[5]-0x30);
					if(hour>23)
					{
						fDateModify=1;
						hour-=24;
						tmpinfo[0]=(hour/10)+'0';
						tmpinfo[1]=(hour%10)+'0';
					}
					//systemTickGPS_Clear();   
					//----------------------------------------------------
					GPRMC_Funs.Time(tmpinfo, hour, min, sec); 
                    //-----------------------------------------------------  
					break;
				case 3://数据有效性
					GPRMC_Funs.Status(tmpinfo);
					break;
				case 4://纬度
					if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)&&(tmpinfo[5]>=0x30)&&(tmpinfo[5]<=0x39)&&(tmpinfo[6]>=0x30)&&(tmpinfo[6]<=0x39)&&(tmpinfo[7]>=0x30)&&(tmpinfo[7]<=0x39)&&(tmpinfo[8]>=0x30)&&(tmpinfo[8]<=0x39))
						  ;
					else
						 break; 
					GPRMC_Funs.Latitude(tmpinfo);
					break;
				case 5://纬度半球 
					GPRMC_Funs.Latitude_NS(tmpinfo);  
					break;
				case 6://经度 
				     if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)&&(tmpinfo[6]>=0x30)&&(tmpinfo[6]<=0x39)&&(tmpinfo[7]>=0x30)&&(tmpinfo[7]<=0x39)&&(tmpinfo[8]>=0x30)&&(tmpinfo[8]<=0x39)&&(tmpinfo[9]>=0x30)&&(tmpinfo[9]<=0x39))
								;
					else
						break; 
					GPRMC_Funs.Longitude(tmpinfo);
					break;
				case 7://经度半球 
					GPRMC_Funs.Longitude_WE(tmpinfo);
					break;
				case 8://速率 
						   for ( k = 0; k < iCount; k++ )
							{
									if ( tmpinfo[k] == '.' )
									{
											break;
									}
							} 						
							if(k>=iCount)
							{ 
								//break;
								k=0;
							}
                            GPRMC_Funs.Speed(tmpinfo, PROCESS,k);
							 
					break;
					
				case 9://方向 				    
						   
						   if ( iCount < 3 )			// 格式检查
							{
							    
								 break; 									
							}
						
							for ( k = 0; k < iCount; k++ )
							{
									if ( tmpinfo[k] == '.' )
									{
											break;
									}
							}
						 if(k>=iCount)
							  break;	
						 GPRMC_Funs.Direction(tmpinfo,PROCESS,k);
					break;		
				case 10://日期 
					 if((tmpinfo[0]>=0x30)&&(tmpinfo[0]<=0x39)&&(tmpinfo[1]>=0x30)&&(tmpinfo[1]<=0x39)&&(tmpinfo[2]>=0x30)&&(tmpinfo[2]<=0x39)&&(tmpinfo[3]>=0x30)&&(tmpinfo[3]<=0x39)&&(tmpinfo[4]>=0x30)&&(tmpinfo[4]<=0x39)&&(tmpinfo[5]>=0x30)&&(tmpinfo[5]<=0x39)) 
				          ;
					 else
					 	break;
					  GPRMC_Funs.Date(tmpinfo,fDateModify,hour,min,sec);
					break;
				default:
					break;
			}	
			iCount=0;
		}
		else
		{
			tmpinfo[iCount++]=*packet++;
			if(iCount<15)
			tmpinfo[iCount]=0;
			if(iCount>15)
			{  
				//return CommaCount;
				break;
			}	
		}
	} 
    //-----------------------------------------------------    
    GPS_Delta_DurPro();   //  用GPS 定时触发  
    //-----------------------------------------------------
	return CommaCount;
} 

//----------------
u8 Process_GSA(u8* packet) 
{

	   return true;
}
//---------------------
u8 Process_GGA(u8* packet)  
{	//检查数据完整性,执行数据转换
	u8 CommaCount=0,iCount=0;
	u8  tmpinfo[12];
	float dop;	
	float Hight1=0,Hight2=0; 
	while (*packet!=0){
		if(*packet==','){
			CommaCount++;
			packet++;
			if(iCount==0)	 
			  { 
			     if(CommaCount==8)
			   	     Satelite_num=0;   
			   	continue;
			   }
			switch(CommaCount){
				case 8:
						        //搜到星的个数$GPGGA,045333,3909.1849,N,11712.3104,E,1,03,4.3,20.9,M,-5.4,M,,*66
						        Satelite_num = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[1] - 0x30 );
							//	rt_kprintf("\r\n 卫星颗数:%d \r\n",Satelite_num);  
                                                                   //--------- 北斗 卫星颗数  --------------
					                 BD_EXT.FJ_SignalValue=BD_EXT.FJ_SignalValue&0xf0;  // 先清除 GSM 信号位
							   BD_EXT.FJ_SignalValue|=	Satelite_num;  // 低4 位

						        break;             

				case 9:	dop=atof((char *)tmpinfo); 
				        
						HDOP_value=dop;		 //  Hdop 数值	  
				break;
				case 10:// MSL altitude
					Hight1=atof((const char*)tmpinfo);
				break;
				case 12:// Geoid Separation
					Hight2=atof((const char*)tmpinfo);
					GPS_Hight=(u16)(Hight1+Hight2);  	 				
					//printf("\r\n 当前海拔高度为:%f,  %f ,%d m\r\n",Hight1,Hight2,GPS_Hight); 
					break;
				default:
					break;
			}
			iCount=0;
		}else{
			tmpinfo[iCount++]=*packet++;
			tmpinfo[iCount]=0;
			if(iCount>12)
				return CommaCount;
		}
	}
	return CommaCount;


}
//------------------------------------------------------------------
void  GPS_ANTENNA_status(void)     //  天线开短路状态检测 
{
    // 2013-4-20    更改PCB   用PD4 : GPS 天线开路      PB6 : GPS  天线短路
   	if(GPIO_ReadOutputDataBit(GPS_PWR_PORT, GPS_PWR_PIN )) // 在GPS 有电时有效   
		{
			if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4))//开路检测	1:天线开路
			{
				 if(OutGPS_Flag==0)
				 {
					   if((Warn_Status[3]&0x20)==0)
					         rt_kprintf("\r\n	ANT 开路"); 
					   Warn_Status[3]|=0x20;
					   Warn_Status[3]&=~0x40; 	 
					   GpsStatus.Antenna_Flag=1;
					   Gps_Exception.GPS_circuit_short_couter=0;
					   self_checking_Antenna=1;
					   self_checking_result=2;
			 	}  
			}
			else if(!GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6))//短路检测  0:天线短路 
			{
			         if(( Warn_Status[3]&0x40)==0)   
				     {
                        Gps_Exception.GPS_short_keepTimer++;
						if( Gps_Exception.GPS_short_keepTimer>200)     
					    {
					       Gps_Exception.GPS_short_keepTimer=0;  // clear 
					        if(GB19056.workstate==0)
					       rt_kprintf("\r\n	ANT 短路");     
						  // rt_kprintf("\r\n	发现短路，立即断开GPS电源");      
						   GPIO_ResetBits( GPS_PWR_PORT, GPS_PWR_PIN ); 
                           self_checking_Antenna=2;
						   self_checking_result=2;

						      //------------------------------------------ 
							  Gps_Exception.GPS_circuit_short_couter++;
							  if(Gps_Exception.GPS_circuit_short_couter>=4)  
							   {
									Gps_Exception.GPS_short_checkFlag=2;
									Gps_Exception.GPS_short_timer=0; // clear  
									rt_kprintf("\r\n   短路检测大于3次 ，一直断开GPS 电源\r\n");   
									
									//	断开 GPS 电源后，得启动 本地定时 ，否则人家说丢包.NND
									/*
							  
									   */ 
							   }	
							  else
							   {
									  Gps_Exception.GPS_short_checkFlag=1; 
							   } 
			                   //-----------------------------------------------------
                             
							   // set  flag 	
							   Warn_Status[3]&=~0x20;
							   Warn_Status[3]|=0x40;	 
							   //------------------------------------------
					     }
			         } 		
				   

			}
			else
			{
				  if(Warn_Status[3]&0x20)
				  	      rt_kprintf("\r\n	天线恢复正常");   
	              Warn_Status[3]&=~0x20;
				  Warn_Status[3]&=~0x40;   
				  GpsStatus.Antenna_Flag=0;
				  Gps_Exception.GPS_circuit_short_couter=0;
			} 
			
		}
}

void  GPS_short_judge_timer(void)
{
       if(Gps_Exception.GPS_short_checkFlag==1)
       {
        Gps_Exception.GPS_short_timer++; 
		if(Gps_Exception.GPS_short_timer>100)
		{   //   关电 后开启
		     Gps_Exception.GPS_short_timer=0;  
		     gps_onoff(1);
		     rt_kprintf("\r\n	 再次开启GPS模块\r\n"); 
			 //---------------期待模块正常-----------
               Warn_Status[3]&=~0x20;
			   Warn_Status[3]&=~0x40; 
			 //----------------
			 Gps_Exception.GPS_short_checkFlag=0; 
		} 
       }



}

//--------------------------------------------------------------------------------
void  GPS_Rx_Process(u8 * Gps_str ,u16  gps_strLen) 
{    
     u8  Gps_instr[160];  
     u8  GPRMC_Enable=0;	 
	 
	                     memset(Gps_instr,0,sizeof(Gps_instr));     
	                     memcpy(Gps_instr,Gps_str,gps_strLen); 

				if(GpsStatus.Raw_Output==1)		  
		                   rt_kprintf((const char*)Gps_instr);        // rt_kprintf((const char*)Gps_str);         

			   //----------------  Mode  Judge    ---------------------			 
				if(strncmp((char*)Gps_instr,"$GNRMC,",7)==0)
				{	
				     GpsStatus.Position_Moule_Status=3;
				     GPRMC_Enable=1;
				     Car_Status[1]&=~0x0C; // clear bit3 bit2 
				     Car_Status[1]|=0x0C; // BD+GPS  mode	1100	 	 
				}
				if(strncmp((char*)Gps_instr,"$BDRMC,",7)==0)
				{	
				       GpsStatus.Position_Moule_Status=1;
				       GPRMC_Enable=1;
					Car_Status[1]&=~0x0C; // clear bit3 bit2 
					Car_Status[1]|=0x08; // BD mode	1000	   
			        }
				if(strncmp((char*)Gps_instr,"$GPRMC,",7)==0)    
				{	
				       GpsStatus.Position_Moule_Status=2;	
				       GPRMC_Enable=1;
				       Car_Status[1]&=~0x0C; // clear bit3 bit2      1100 
					   Car_Status[1]|=0x04; // Gps mode   0100 
		        }  	 	  
				
                //-------------------------------------------------- 
                            //----------- Pick up useful  --------------------------
			      if(GPRMC_Enable==1) 
			      	{
					Process_RMC(Gps_instr); 	
					Gps_Exception.current_datacou+=gps_strLen;
                                  return;
			      	}	
			     if((strncmp((char*)Gps_instr,"$GPGGA,",7)==0)||(strncmp((char*)Gps_instr,"$GNGGA,",7)==0)||(strncmp((char*)Gps_instr,"$BDGGA,",7)==0))  
			     {   
					   Process_GGA(Gps_instr);  
					    return;	
		         }
                if((strncmp((char*)Gps_instr,"$GPGSA,",7)==0)||(strncmp((char*)Gps_instr,"$BDGSA,",7)==0)||(strncmp((char*)Gps_instr,"$GNGSA,",7)==0))
		          {   
					Process_GSA(Gps_instr);
				      return;	
			      }
}


//================================================



void GpsIo_Init(void)
{
GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );

	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPS_PWR_PIN;
	GPIO_Init( GPS_PWR_PORT, &GPIO_InitStructure );
	GPIO_ResetBits( GPS_PWR_PORT, GPS_PWR_PIN );

/*uart5 管脚设置*/

	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_12;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	GPIO_PinAFConfig( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
	GPIO_PinAFConfig( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );

/*NVIC 设置*/
	NVIC_InitStructure.NVIC_IRQChannel						= UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	gps_baud( 9600 );
	USART_Cmd( UART5, ENABLE );
	USART_ITConfig( UART5, USART_IT_RXNE, ENABLE );

	GPIO_SetBits( GPIOD, GPIO_Pin_10 );


        //------------------- PD9 -----------------------------
   GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_9;   // 功放
   GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT; 
   GPIO_Init(GPIOD, &GPIO_InitStructure);  

}
	




/*
   gps接收中断处理，收到\n认为收到一包
   收到一包调用处理函数
 */
	static uint8_t	last_ch = 0;
void UART5_IRQHandler( void )
{
	uint8_t			ch;
	rt_interrupt_enter( );
	if( USART_GetITStatus( UART5, USART_IT_RXNE ) != RESET )
	{
		ch = USART_ReceiveData( UART5 );
		if( ( ch == 0x0a ) && ( last_ch == 0x0d ) ) /*遇到0d 0a 表明结束*/
		{
			uart5_rxbuf.body[uart5_rxbuf.wr++] = ch;
			if( uart5_rxbuf.wr < 124 )
			{
			   //--------- fliter  useful------
			   if((strncmp((char*)uart5_rxbuf.body+3,"RMC,",4)==0)||(strncmp((char*)uart5_rxbuf.body+3,"TXT,",4)==0)||  \
			   	(strncmp((char*)uart5_rxbuf.body+3,"GGA,",4)==0))//||(strncmp((char*)uart5_rxbuf.body+3,"GSA,",4)==0))
			   	{
			      rt_mq_send( &mq_gps, (void*)&uart5_rxbuf, uart5_rxbuf.wr + 2 ); 
			   	}
			}
			uart5_rxbuf.wr = 0;
		}else
		{ 
		      // 1. get  head char
		       if(ch=='$')                                 
			   	      uart5_rxbuf.wr = 0;
		      // 2.  judge  head char	   
		       if(uart5_rxbuf.body[0]!='$')  // add later 
			   	    uart5_rxbuf.wr = 0;
		      // 3.  rx data  	   
			uart5_rxbuf.body[uart5_rxbuf.wr++] = ch;     
			if( uart5_rxbuf.wr == UART5_RX_SIZE )
			{
				uart5_rxbuf.wr = 0;
			}
			uart5_rxbuf.body[uart5_rxbuf.wr] = 0;
		}
		last_ch = ch;
		USART_ClearITPendingBit( UART5, USART_IT_RXNE ); 
	}
	rt_interrupt_leave( );   
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void gps_baud( int baud )
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate				= baud;
	USART_InitStructure.USART_WordLength			= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits				= USART_StopBits_1;
	USART_InitStructure.USART_Parity				= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode					= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init( UART5, &USART_InitStructure );
}

//FINSH_FUNCTION_EXPORT( gps_baud, config gsp_baud );


/*初始化*/

void  gps_io_init(void)
{
  GPIO_InitTypeDef	GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
#if 1
  //-------- 开短路检测  -------- 
	 //#ifdef HC_595_CONTROL	  
  // 2013-4-20		更改PCB   用PD4 : GPS 天线开路		PB6 : GPS  天线短路
  GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd	  = GPIO_PuPd_NOPULL;  
  GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_2MHz;	
  //	 IN
  //------------------- PD4 -----------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;    //GPS 天线开路 
  GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //------------------- PB6 -----------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;    //GPS 天线短路 
  GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IN; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
#endif

}


/*初始化*/
static rt_err_t dev_gps_init( rt_device_t dev )
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );

	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPS_PWR_PIN;
	GPIO_Init( GPS_PWR_PORT, &GPIO_InitStructure );
	GPIO_ResetBits( GPS_PWR_PORT, GPS_PWR_PIN );



/*uart5 管脚设置*/

	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_12;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	GPIO_PinAFConfig( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
	GPIO_PinAFConfig( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );

/*NVIC 设置*/
	NVIC_InitStructure.NVIC_IRQChannel						= UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	gps_baud( 9600 );
	USART_Cmd( UART5, ENABLE );
	USART_ITConfig( UART5, USART_IT_RXNE, ENABLE );

	GPIO_SetBits( GPIOD, GPIO_Pin_10 );



	return RT_EOK;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static rt_err_t dev_gps_open( rt_device_t dev, rt_uint16_t oflag )
{
	GPIO_SetBits( GPS_PWR_PORT, GPS_PWR_PIN );
	return RT_EOK;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static rt_err_t dev_gps_close( rt_device_t dev )
{
	GPIO_ResetBits( GPS_PWR_PORT, GPS_PWR_PIN );
	return RT_EOK;
}

/***********************************************************
* Function:gps_read
* Description:数据模式下读取数据
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static rt_size_t dev_gps_read( rt_device_t dev, rt_off_t pos, void* buff, rt_size_t count )
{
	return RT_EOK; 
}

/***********************************************************
* Function:		gps_write
* Description:	数据模式下发送数据，要对数据进行封装
* Input:		const void* buff	要发送的原始数据
       rt_size_t count	要发送数据的长度
       rt_off_t pos		使用的socket编号
* Output:
* Return:
* Others:
***********************************************************/

static rt_size_t dev_gps_write( rt_device_t dev, rt_off_t pos, const void* buff, rt_size_t count )
{
	rt_size_t	len = count;
	uint8_t		*p	= (uint8_t*)buff;

	while( len )
	{
		USART_SendData( UART5, *p++ );
		while( USART_GetFlagStatus( UART5, USART_FLAG_TC ) == RESET )
		{
		}
		len--;
	}
	return RT_EOK;
}

/***********************************************************
* Function:		gps_control
* Description:	控制模块
* Input:		rt_uint8_t cmd	命令类型
    void *arg       参数,依据cmd的不同，传递的数据格式不同
* Output:
* Return:
* Others:
***********************************************************/
static rt_err_t dev_gps_control( rt_device_t dev, rt_uint8_t cmd, void *arg )
{
	int i = *(int*)arg;
	switch( cmd )
	{
		case CTL_GPS_OUTMODE:
			break;
		case CTL_GPS_BAUD:
			gps_baud( i );
	}
	return RT_EOK;
}

ALIGN( RT_ALIGN_SIZE )
static char thread_gps_stack[4096]; 
struct rt_thread thread_gps;


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static void rt_thread_entry_gps( void* parameter )
{
	rt_err_t	res;

      // 1.  init
       GPS_Abnormal_init();

	//2.  main while
	while( 1 )
	{
	    // 1. rx data
		res = rt_mq_recv( &mq_gps, (void*)&gps_rx, 124, 2 ); //等待100ms,实际上就是变长的延时,最长100ms
		if(res== RT_EOK )                                                     //收到一包数据
		{
		     //---------------------------------
			if( flag_bd_upgrade_uart == 0 )
			{
				GPS_Rx_Process( gps_rx.body, gps_rx.wr );  
				GPS_Abnormal_process();//  GPS 模块异常检测 

			}else
			{
				if( gps_rx.body[0] == 0x40 )
				{
					rt_device_write( &dev_vuart, 0, gps_rx.body, gps_rx.wr);
				}
			}
		}
        
		//2. GPS_ANTENNA_status   
				GPS_ANTENNA_status();	  
				GPS_short_judge_timer(); 
				
		rt_thread_delay(30);     
		gps_thread_runCounter=0;
	}
}

/*gps设备初始化*/
void gps_init( void )
{
	//rt_sem_init( &sem_gps, "sem_gps", 0, 0 );
	
	rt_mq_init( &mq_gps, "mq_gps", &gps_rawinfo[0], 128 - sizeof( void* ), GPS_RAWINFO_SIZE, RT_IPC_FLAG_FIFO );
	rt_thread_init( &thread_gps,
	                "gps",
	                rt_thread_entry_gps,
	                RT_NULL,
	                &thread_gps_stack[0],
	                sizeof( thread_gps_stack ), Prio_GPS, 6 ); 
	rt_thread_startup( &thread_gps );

	dev_gps.type	= RT_Device_Class_Char;
	dev_gps.init	= dev_gps_init;
	dev_gps.open	= dev_gps_open;
	dev_gps.close	= dev_gps_close;
	dev_gps.read	= dev_gps_read;
	dev_gps.write	= dev_gps_write;
	dev_gps.control = dev_gps_control; 

	rt_device_register( &dev_gps, "gps", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE );
	rt_device_init( &dev_gps );
}

/*gps开关*/
rt_err_t gps_onoff( uint8_t openflag )
{
	if( openflag == 0 )
	{
		GPIO_ResetBits( GPIOD, GPIO_Pin_10 );
	} else
	{
		GPIO_SetBits( GPIOD, GPIO_Pin_10 );
	}
	return 0;
}

FINSH_FUNCTION_EXPORT( gps_onoff, gps_onoff([1 | 0] ) );

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
rt_size_t gps_write( uint8_t *p, uint8_t len )
{
	return dev_gps_write( &dev_gps, 0, p, len );
}

//FINSH_FUNCTION_EXPORT( gps_write, write to gps );



void  gps_mode(u8 *str) 
{
   if (strlen((const char*)str)==0)
	{
		     rt_kprintf("\r\n    定位模式:    ");  
		      switch(GpsStatus.Position_Moule_Status)
		      	{  
		      	         case 1:           rt_kprintf(" BD\r\n");  
						 	break;
				  case 2:           rt_kprintf(" GPS\r\n");  
				  	              break;
				  case 3:           rt_kprintf(" BD+GPS\r\n");  
				  	              break;
		      	}
	}
	else 
	{
		if(str[0]=='1')
			{
			  dev_gps_write( &dev_gps, 0, BD_MODE, strlen((const char*)BD_MODE));
			   GpsStatus.Position_Moule_Status=1;
			//GPS_PutStr(BD_MODE);
			 rt_kprintf ("\r\n    BD MODE\r\n"); 
			}
		else if(str[0]=='2')
			{
			 dev_gps_write( &dev_gps, 0, GPS_MODE, strlen((const char*)BD_MODE));
			  GpsStatus.Position_Moule_Status=2;
			//GPS_PutStr(GPS_MODE);
			   rt_kprintf("\r\n    GPS MODE\r\n");
			}
		else if(str[0]=='3')
			{
			   dev_gps_write( &dev_gps, 0, GPSBD_MODE, strlen((const char*)BD_MODE));    
			   GpsStatus.Position_Moule_Status=3;
			//GPS_PutStr(GPSBD_MODE);
			rt_kprintf("\r\n    GPS&BD MODE\r\n");
			}
	 }
 }
FINSH_FUNCTION_EXPORT(gps_mode, posit_mode setting);


void  gps_raw(u8* str)
{
      if(str[0]=='1')
      	{
      	    GpsStatus.Raw_Output=1;      
      	}
	else  
	if(str[0]=='2')
      	{
      	    GpsStatus.Raw_Output=2;       
      	}  
      else
     if(str[0]=='0') 
     	{
           GpsStatus.Raw_Output=0;         
     	}
     	 
}

FINSH_FUNCTION_EXPORT(gps_raw, gps raw output);  

/************************************** The End Of File **************************************/

