/*
     IC  card .h
*/


#ifndef    IC_COMMON
#define    IC_COMMON


#include <rtthread.h>
#include <rthw.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include "App_moduleConfig.h"



#define b_CardEdge   0x0001

#define  MAX_DriverIC_num   3

typedef struct _DRIVE_STRUCT
{
    DRV_INFO     Driver_BASEinfo;  //  驾驶员基本信息
    u8           Working_state;  //  0:  尚未被启用   1:   被启用但是不是当前驾驶员  2 :  被启用且为当前驾驶员
    u8           Start_Datetime[6];  // 开始时间 BCD
    u8           End_Datetime[6];  // 开始时间 BCD
    u32          Running_counter; // 行驶时间
    u32          Stopping_counter; // 停止行驶结束时间
    u8           H_11_start;  // 1 开始完成 2 。结束完成  3 。存储完成 clear
    u8           H_11_lastSave_state; // 是否存储过
    u8           Lati[4];  //    纬度
    u8           Longi[4]; //	 经度
    u16          Hight;  //  高度

} DRIVE_STRUCT;

extern DRV_INFO	Read_ICinfo_Reg;   // 临时读取的IC 卡信息
extern DRIVE_STRUCT     Drivers_struct[MAX_DriverIC_num]; // 预留5 个驾驶员的插卡对比

extern unsigned char IC_CardInsert;//1:IC卡插入正确  2:IC卡插入错误
extern unsigned char IC_Check_Count;
extern unsigned char administrator_card;
extern 	u8		  powerOn_first;	 //    首次上电后不判断拔卡


extern void CheckICInsert(void);
extern void KeyBuzzer(unsigned char num);
extern void IC_info_default(void);

extern void  Different_DriverIC_InfoUpdate(void);
extern void  Different_DriverIC_Start_Process(void);
extern void  Different_DriverIC_End_Process(void);
extern void  Different_DriverIC_Checking(void);


#endif
