#ifndef   _CAN2_DEV
#define    _CAN2_DEV
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


#define  OIL_NOCONNECT 0
#define  OIL_NORMAL    1
#define  OIL_ABNORMAL  2



#define      U3_OUT_PWR_ON       GPIO_SetBits(GPIOE,GPIO_Pin_7)
#define      U3_OUT_PWR_OFF      GPIO_ResetBits(GPIOE,GPIO_Pin_7)


typedef  struct   _YH
{    
	uint32_t	oil_value;			///邮箱油量，单位为1/10升
	uint16_t	oil_realtime_value; 		///邮箱油量高度实时值，单位为1/10mm
	uint16_t	oil_average_value;	///邮箱油量高度平均实时值，单位为1/10mm
	
	u8			oil_YH_workstate; 	 //  0:  表示工作异常	1: 表示工作正常  2:能够检测到，但传感器故障
	
	u32 		oil_YH_Abnormal_counter;	 //   车辆行驶过程中， 但实时油耗数值为 0  ，
										//	 计算300 次，如果大于300 则判断油耗故障
	u32 		oil_YH_0_value_cacheCounter;	//	速度为0 ，缓冲 ，速度为0 ，且连续30 次 ，才清除避免误判 								   
	u32         oil_YH_no_data_Counter;  //  没有数据计数器
}YH;


extern  YH   Oil;

extern  struct rt_device  Device_CAN2;

extern void  u3_RxHandler(unsigned char rx_data);
extern void  Device_CAN2_regist(void );
extern void Oil_Sensor_Connect_Checking(void);



#endif
