/*
     APP_808 .h
*/
#ifndef  _APP_808
#define   _APP_808

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include <App_moduleConfig.h>
#include "spi_sd.h"
#include "Usbh_conf.h"
#include <dfs_posix.h>


// 1  : connect  not find      0:  not  find      2: connect find  
#define  USB_NOTCONNECT                        0
#define  USB_CONNECT_NOTFIND              1
#define  USB_FIND                                     2 


extern   rt_device_t   Udisk_dev;
extern   u8  Udisk_filename[30];  
extern   int  udisk_fd;   
// Dataflash  Operate Semaphore  

//------- change  on  2013 -7-24  --------
extern 	rt_thread_t app_tid; // app 线程 pid
extern u32  gps_thread_runCounter;



extern void  SIMID_Convert_SIMCODE( void ); 
extern void  Protocol_app_init(void);
extern void  Distance_Accumulate (u16 Speed_Using); 

//          System  reset  related  
extern  void  system_reset(void);
extern  void  reset(void);  
extern  void  Pic_Data_Process(void);  
extern 	void   MainPower_cut_process(void);
extern  void  MainPower_Recover_process(void);

#endif
