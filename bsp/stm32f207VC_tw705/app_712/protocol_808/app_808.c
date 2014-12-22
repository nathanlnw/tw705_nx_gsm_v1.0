/*
     App_808.C
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
#include "spi_sd.h"
#include "Usbh_conf.h"
#include <dfs_posix.h>
//#include "usbh_usr.h"P


/* 定时器的控制块 */
 static rt_timer_t timer_app; 


//------- change  on  2013 -7-24  --------
rt_thread_t app_tid = RT_NULL; // app 线程 pid



//----- app_thread   rx     gsm_thread  data  related ----- 	
ALIGN(RT_ALIGN_SIZE)
//static  struct rt_semaphore app_rx_gsmdata_sem;  //  app 提供数据 给gsm发送信号量


// Dataflash  Operate   Mutex 







//----- app_thread   rx     gps_thread  data  related ----- 	
//ALIGN(RT_ALIGN_SIZE)
//static  MSG_Q_TYPE  app_rx_gps_infoStruct;  // app   接收从gsm  来的数据结构
//static  struct rt_semaphore app_rx_gps_sem;  //  app 提供数据 给gps发送信号量

//----- app_thread   rx    485 _thread  data  related ----- 	
//ALIGN(RT_ALIGN_SIZE)
//static  MSG_Q_TYPE  app_rx_485_infoStruct;  // app   接收从gsm  来的数据结构
//static  struct rt_semaphore app_rx_485_sem;  //  app 提供数据 给gps发送信号量 


u8  TF_test_workState=0;

rt_device_t   Udisk_dev= RT_NULL;
u8 Udisk_filename[30]; 
int  udisk_fd=0;

u32       WarnTimer=0; 

u8   OneSec_CounterApp=0;
u32  app_thread_runCounter=0;	
u32  gps_thread_runCounter=0;


 //  1. MsgQueue Rx
 void  App_thread_timer(void)
 {
	app_thread_runCounter++;
	if(app_thread_runCounter>300)	// 400* app_thread_delay(dur)
	 {
		reset();  
	 }	 
 }
 
 void  gps_thread_timer(void)
 {
	gps_thread_runCounter++;
	if((gps_thread_runCounter>300)&&(BD_ISP.ISP_running==0))   // 400* app_thread_delay(dur)   
	 {
		reset();  
	 }	 
 } 

void Device_RegisterTimer(void)
{
      if(0==JT808Conf_struct.Regsiter_Status)    //注册   
          {
             DEV_regist.Sd_counter++;
			 if(DEV_regist.Sd_counter>32)
			 	{
                   DEV_regist.Sd_counter=0;
				   DEV_regist.Enable_sd=1;  
			 	}
          }
}

void Device_LoginTimer(void)
{  
  if(1==DEV_Login.Operate_enable)
  {
     DEV_Login.Sd_counter++;
	 if(DEV_Login.Sd_counter>32)    
	 {
          DEV_Login.Sd_counter=0;
	      DEV_Login.Enable_sd=1;

		  DEV_Login.Sd_times++;
		  if(DEV_Login.Sd_times>10)
		  	{
		  	   DEV_Login.Sd_times=0;
			   if(GB19056.workstate==0)
			   rt_kprintf("\r\n  鉴权次数Max，重新注册!\r\n");  
               DEV_regist.Enable_sd=1;   
			   JT808Conf_struct.Regsiter_Status=0; //clear   
		  	}
	 }
  }
}

void  App808_tick_counter(void) 
{
    Systerm_Reset_counter++;
    if((Systerm_Reset_counter>Max_SystemCounter)&&(Spd_Using<=10))
     {   	Systerm_Reset_counter=0;	
	        //rt_kprintf("\r\n Sysem  Control   Reset \r\n"); 
               reset(); 
     }  
    //-------------------------------------------
    if((Systerm_Reset_counter&0x1FF)==0x1FF)  //0x1ff   
    {
        DistanceWT_Flag=1;  
    }  
}

void  Distance_Accumulate (u16 Speed_Using) // Speed_Using  单位 0.1km/h
{
	
	u32  Distance_1s_m=0;  // 一秒钟运行 多少米
	#if 1  
          //  里程
       if((Sps_larger_5_counter>10)&&(Speed_Using>50))  
       {
			  //------- GPS  里程计算  -------- 		  
			  Distance_1s_m=Speed_Using/36;    //m   
			  JT808Conf_struct.Distance_m_u32+=Distance_1s_m;	// 除以3600 是m/s 
			  if(JT808Conf_struct.Distance_m_u32>0xFFFFFF)
					   JT808Conf_struct.Distance_m_u32=0;	   //里程最长这么多米	   
			  Distance_m_u32=JT808Conf_struct.Distance_m_u32;// add later


	           //----- 定距回传处理---  
	                     if(1==JT808Conf_struct.SD_MODE.DIST_TOTALMODE)
	                     {
	                        DistanceAccumulate+=Distance_1s_m;
	                        if(DistanceAccumulate>=Current_SD_Distance)
	                        	{
	                               DistanceAccumulate=0;
					   PositionSD_Enable();        //发送
					   Current_UDP_sd=1;
	                        	}
	                     }
			   //------- 定距处理结束 -----

       	} 
		  if(DispContent==4)  //  disp	显示	
		  { 
			rt_kprintf("\r\n 当前校准状态: %d  传感器速度=%d  速度脉冲系数:%d   卫星速度spd=%d\r\n",JT808Conf_struct.DF_K_adjustState,Speed_cacu,JT808Conf_struct.Vech_Character_Value,Speed_gps);	
		  }    
	#endif

}

void  Emergence_Warn_Process(void)
{
  //----------- 紧急报警下拍照相关的处理  ------------------
  if(WARN_StatusGet()) 
  {
     //  拍照过程中和传输过程中不予处理                        
	 if((CameraState.status==other)&&(Photo_sdState.photo_sending==0)&&(0==MultiTake.Taking)&&(0==MultiTake.Transfering))
	  {
		  WarnTimer++;
		  if(WarnTimer>=4)
			  {
			      WarnTimer=0;    	
				       //----------  多路摄像头拍照 -------------
					 // MultiTake_Start();
					 // Camera_Start(1);  //先拍一路满足演示   
			  }   
	  }   
     //-------------------------------------------------------
     
	 fTimer3s_warncount++;
	 if(fTimer3s_warncount>=4)			 
	{
		 // fTimer3s_warncount=0;
		 if ( ( warn_flag == 0 ) && ( f_Exigent_warning == 0 )&&(fTimer3s_warncount==4) )
		 {
			    warn_flag = 1; 					// 报警标志位
			    Send_warn_times = 0;		  //  发送次数 
		        //-----------------------------------
				#if  0
					 rt_kprintf( "紧急报警 ");				 
					 StatusReg_WARN_Enable(); // 修改报警状态位
					 PositionSD_Enable();  
					 Current_UDP_sd=1;   	
			   #endif  
			   //---------------------
			   
			   if((Key_MaskWord[3]&0x01)==0x00)
				 {
					if((Warn_MaskWord[3]&0x01)==0x01)
					 {
						 ;// rt_kprintf( "紧急报警 Fail，MaskEnable !");		 
			   
					 }
					 else
					 {
						  //rt_kprintf( "紧急报警 "); 			  
						  StatusReg_WARN_Enable(); // 修改报警状态位
						  PositionSD_Enable();	
						  Current_UDP_sd=1;  
						 // warn_msg_sd(); // 短息报警
					  }
				  }
				  else
					 {
								  StatusReg_WARN_Enable(); // 修改报警状态位
								  PositionSD_Enable();	
								  Current_UDP_sd=1; 
								//  warn_msg_sd();// 短息报警
			   
			   
					 }
		 }
	}

	 //------------------------------------------------- 
  } 
   else
  { 	   
	   WarnTimer=0;
	   fTimer3s_warncount=0;  
	   //------------ 检查是否需要报警拍照 ------------
	 /*  if(CameraState.status==enable)
	   {		   
		 if((CameraState.camera_running==0)||(0==Photoing_statement))  // 在不传输报警图片的情况下执行
		 {
			CameraState.status=disable;
		 }
	   }
	   else*/
	   if(CameraState.camera_running==0)
			CameraState.status=other;
  }   


}

void SIMID_Convert_SIMCODE( void ) 
{
		SIM_code[0] = SimID_12D[0] - 0X30;
		SIM_code[0] <<= 4;
		SIM_code[0] |= SimID_12D[1] - 0X30;	  

		SIM_code[1] = SimID_12D[2] - 0X30;
		SIM_code[1] <<= 4;
		SIM_code[1] |= SimID_12D[3] - 0X30;	

		SIM_code[2] = SimID_12D[4] - 0X30;
		SIM_code[2] <<= 4;
		SIM_code[2] |= SimID_12D[5] - 0X30;	

		SIM_code[3] = SimID_12D[6] - 0X30;
		SIM_code[3] <<= 4;
		SIM_code[3] |= SimID_12D[7] - 0X30;	

		SIM_code[4] = SimID_12D[8] - 0X30;
		SIM_code[4] <<= 4;
		SIM_code[4] |= SimID_12D[9] - 0X30;	

		SIM_code[5] = SimID_12D[10] - 0X30;
		SIM_code[5] <<= 4;
		SIM_code[5] |= SimID_12D[11] - 0X30; 
}

void  Recorder_sd_timer(void)
{
    // 行车记录仪数据发送
    Recode_Obj.timer++;     
    if((Recode_Obj.CountStep==2)&&(Recode_Obj.timer>=2))   
       {    
    	 Recode_Obj.CountStep=1; 
 		 Recode_Obj.timer=0;
       } 
}

static void timeout_app(void *  parameter)
{     //  100ms  =Dur 
    u8  SensorFlag=0,i=0;


    GPRS_GSM_PowerOFF_Working();
	//---------  Step timer
	Dial_step_Single_10ms_timer();	   	
	//---------- AT Dial upspeed---------
	if((OneSec_CounterApp%2)==0)   	 
	{ 
	   if((CommAT.Total_initial==1)) 
		{	
		    if(CommAT.cmd_run_once==0)
		        CommAT.Execute_enable=1;	 //  enable send   periodic 		
		    if(( CommAT.Initial_step==17)&&(Login_Menu_Flag==0)) 
				CommAT.cmd_run_once=1;
			else
				CommAT.cmd_run_once=0;  
				
	   	}
	}	
     OneSec_CounterApp++;
     if(OneSec_CounterApp>=5)	 
	{        
	    OneSec_CounterApp=0;
		 
	     //---------------------------------- 	
	        if(DataLink_Status())
		    {
		          Device_RegisterTimer();
		          Device_LoginTimer();	 
			      SendMode_ConterProcess(); 	  
				  Meida_Trans_Exception();	   
		    }	 
		     Distance_Accumulate(Spd_Using);  	     
		     Emergence_Warn_Process();   

             App_thread_timer(); 
			 gps_thread_timer(); 
			 Photo_send_TimeOut();  
			 GPS_Keep_V_timer();
			 Recorder_sd_timer(); 
			 
			 //  Camera power	on 
			 Camera_powerOn_timer(); 
			 

			 //车辆信号线状态指示  //刹车线//左转//右转//远光//近光//雾灯//车门//null
		  SensorFlag=0x80;
			for(i=1;i<8;i++)  
			{
				  if(Vehicle_sensor&SensorFlag)     
					    XinhaoStatus[i+10]=0x31;
				   else
					    XinhaoStatus[i+10]=0x30; 
				     SensorFlag=SensorFlag>>1;   
			} 


			//   gps using   confirm    确保车辆确实是在运行
			if(Spd_Using>50)
				{
				  Sps_larger_5_counter++;
				}
			else
				 Sps_larger_5_counter=0;
                   //  system timer
                   App808_tick_counter(); 
           
            //-----------------------------------
           if( ReadCycle_status==RdCycle_SdOver)
		   {	   
			        ReadCycle_timer++;	   
				 if(ReadCycle_timer>5)  //5s No resulat
				 {
				      ReadCycle_timer=0; 
                      ReadCycle_status=RdCycle_RdytoSD; 
						// Api_cycle_Update();	   
				 }		
           }


		   //--------  多媒体时间信息上传 后处理(不判应答)-----
           //--------------  多媒体上传相关   天地通有时不给多媒体信息上传应答  --------------                                       
	       if(MediaObj.Media_transmittingFlag==1)  // clear		 							      
		     {
		         MediaObj.Media_transmittingFlag=2;   
				 if(Duomeiti_sdFlag==1)
				  { 
				      Duomeiti_sdFlag=0; 
				      Media_Clear_State();
					Photo_send_end();
					#ifdef REC_VOICE_ENABLE
				    	Sound_send_end();
					#endif
					//Video_send_end();
				  }	
				// rt_kprintf("\r\n  多媒体信息前的多媒体发送完毕 \r\n");  
		   	  }	
			WatchDog_Feed();       

		    if( MediaObj.SD_media_Flag==2)
			 {
			        Multimedia_0800H_ACK_process();      // timeout  replace  RxACK
				  MediaObj.SD_media_Flag=0; 
	         }		 
	         //-----------------------------------------------------------  
	         //   from 485
	          OpenDoor_TakePhoto();	 
	          Camra_Take_Exception();	   

             
		     //====疲劳驾驶===========================
			 if(Car_Status[3]&0x01)   // 疲劳驾驶是基于ACC开的情况下进行的 
					Tired_Check();	   
              //  --- 超速报警  -----------
		     SpeedWarnJudge();	// 速度报警判断    
			  //------ add later  -----------	 
	          CAN_send_timer();  	
       }	 
	 
 
	      //---------------0.2 s  一次    
	      Media_Timer_Service();  

}

 void   MainPower_cut_process(void)
 {
    Powercut_Status=0x02;
	//----------断电了  ------------
	//---------------------------------------------------
	//        D4  D3
	//        1   0  主电源掉电 
	StatusReg_POWER_CUT(); 
	//----------------------------------------------------
	Power_485CH1_OFF;  // 第一路485的电			 关电工作
	//-----------------------------------------------------			    
	//------- ACC 不休眠 ----------
	Vehicle_RunStatus|=0x01;
	//   ACC ON		 打火 
	StatusReg_ACC_ON();  // ACC  状态寄存器   
	Sleep_Mode_ConfigExit(); // 休眠相关
	//-------  不欠压--------------
	Warn_Status[3]&=~0x80; //取消欠压报警   
	SleepCounter=0; 

 }


 void  MainPower_Recover_process(void)
 { 
	//----------------------------------------------
	StatusReg_POWER_NORMAL();
	//----------------------------------------------
}


 ALIGN(RT_ALIGN_SIZE)
char app808_thread_stack[4096];       
struct rt_thread app808_thread;

static void App808_thread_entry(void* parameter) 
{
	
//    u32  a=0;	

	 // rt_kprintf("\r\n ---> app808 thread start !\r\n");  

	    pulse_init();
		TIM3_Config();
       //  step 3:    usb host init	   	    	//  step  4:   TF card Init    
          usbh_init();    
          Init_ADC(); 
		  gps_io_init();
		  
		  
		  CAN_App_Init();	// CAN初始化  		  
		  CAN_struct_init();   
		  AD_PowerInit(); 
		  
        /* watch dog init */
	    WatchDogInit();       

	     Camera_Init();  
         DoorCameraInit();

       //BUZZER
	   GPIO_Config_PWM();
	   TIM_Config_PWM();  
	   buzzer_onoff(0); 
        
	while (1)
	{

		//   1.   处理相关接收到的   808 数据
            if(Receive_DataFlag==1)
		   {
              memcpy( UDP_HEX_Rx,GSM_HEX,GSM_HEX_len);
			  UDP_hexRx_len=GSM_HEX_len;	 
              TCP_RX_Process(LinkNum);         
			  Receive_DataFlag=0; 	 	   
          }	
	 // 2.  485  Related  ---------
	   //--------------------- 拍照数据处理-----	
	   if(_485_CameraData_Enable)	   
	   {
	              delay_ms(5);
                  Pic_Data_Process();
                _485_CameraData_Enable=0;
	   }
	   rt_thread_delay(10);  
	   // 3.    检查顺序存储 gps  标准信息的状态 
		   Api_CHK_ReadCycle_status();//   循环存储状态检测			   
		   app_thread_runCounter=0; 
	   // 4.    808   Send data   		
        if(DataLink_Status()&&(CallState==CallState_Idle)&&(print_workingFlag==0))    
	   {   
	        Do_SendGPSReport_GPRS();    
	   } 
       // 5. ---------------  顺序存储 GPS  -------------------		    
		if(GPS_getfirst)	 //------必须搜索到经纬度
		{
		     if(Current_UDP_sd==0)  
		       Save_GPS();       
		} 		 	   
	   // 7.   系统延时
	   // 8.	 行车记录仪相关的数据存储 
	   if(BD_ISP.ISP_running==0)
		{  
		   delay_ms(15); 
		   JT808_Related_Save_Process();  
		   delay_ms(5);  	
	   	}
   //  10  . Redial_reset_save
         if(Redial_reset_save)
         	{
         	   delay_ms(10); 
			   delay_ms(8); 
               Redial_reset_save=0;  
         	}
	   //-------     485  TX ------------------------
        Send_const485(CameraState.TX_485const_Enable); 	   
	    rt_thread_delay(15);    	     
      //    485   related  over   	 	   
      //----------------------------------------      
	  app_thread_runCounter=0; 
	   //--------------------------------------------------------
	}
}

/* init app808  */
void Protocol_app_init(void)
{
        rt_err_t result;

        
       //---------  timer_app ----------
	         // 5.1. create  timer     100ms=Dur
	   timer_app=rt_timer_create("tim_app",timeout_app,RT_NULL,20,RT_TIMER_FLAG_PERIODIC); 
	        //  5.2. start timer
	   if(timer_app!=RT_NULL)
	           rt_timer_start(timer_app);      

      result=rt_thread_init(&app808_thread, 
		"app808", 
		App808_thread_entry, RT_NULL,
		&app808_thread_stack[0], sizeof(app808_thread_stack),   
		Prio_App808,10);          

    if (result == RT_EOK)
    {
         rt_thread_startup(&app808_thread); 
    }
}



