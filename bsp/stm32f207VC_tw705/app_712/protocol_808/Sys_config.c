#include <rtthread.h> 
#include <rthw.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "Device_808.h"
#include  "Vdr.h"

 
#define   SYSID            0x55AA      //55AA   
                                /*        
                                                        0x0000   -----   0x00FF  生产和研发用
                                                        0x0100   -----   0x0FFF  产品出货用
                                                        0x1000   -----   0xF000  远程升级用
                                                       */    



ALIGN(RT_ALIGN_SIZE)  
SYS_CONF          SysConf_struct;   //  系统配置   

ALIGN(RT_ALIGN_SIZE) 
JT808_CONF       JT808Conf_struct;    //  JT 808   相关配置   
JT808_CONF       JT808_struct_Bak;    //  JT808 相关模式设置备份
JT808_CONF       JT808_struct_Bak2;   //  JT808  bak 2

ALIGN(RT_ALIGN_SIZE) 
TIRED_CONF      TiredConf_struct;    //  疲劳驾驶相关配置





//----------  Basic  Config---------------------------
u8      DeviceNumberID[13];//="800130100001";    // 车辆DeviceID    ---- 河北天地通用
u8      SimID_12D[13]; // 入网ID  号码 12 位 首位 为 0

u8          RemoteIP_Dnsr[4]={255,255,255,255}; 
u8		RemoteIP_main[4]={111,113,14,154};  //{111,113,14,154}; 宁夏北星         
u16		RemotePort_main= 9131;//山西中航北斗   7000
u8		RemoteIP_aux[4]={60,28,50,210};    //{60,28,50,210}
u16		RemotePort_aux=4000; 
//           Link2  Related 
u8      Remote_Link2_IP[4]={111,113,14,154}; 
u16     Remote_Link2_Port=7000;     



u8           APN_String[30]="UNINET"; //"CMNET";   //  河北天地通  移动的卡
u8           DomainNameStr[50]="jt1.gghypt.net"; ;  // 域名  天地通up.gps960.com //jt1.gghypt.net 
u8           DomainNameStr_aux[50]="jt2.gghypt.net";     //"www.sina.com";//jt2.gghypt.net
u16         ACC_on_sd_Duration=30;    //  ACC 开启的时候 上报的时间间隔  
u16         ACC_off_sd_Duration=60;    //  ACC 关闭时候上报的时间间隔  
u8          TriggerSDsatus=0x80;   // 传感器触发上报状态位






u32	     Current_SD_Duration=20;  //GPS 信息报告的时间间隔   
u32      Current_SD_Distance=100; // GPS 信息定距上报距离
u32      DistanceAccumulate=0;    // 定距上报累加器
u8		 Current_State=0; //正式为0 ； 演示为1		 // 上报实时标志位信息	 预防DF 损坏 

u16      StopLongDuration=15300; //255minutes 15300s   //超长停车报警最长时间    
u16      Stop_counter=0;               //超长停车的临时计数器
   
u8      EmergentWarn=0;               // 紧急报警

//-------------- Vehicle Recorder Setting --------------------

//---------------------------   驾驶员信息  ------------------------------
//u8     DriverCard_ID[18]="000000000000000000";  // 驾驶员驾驶证号码 18位
//u8     DriveName[21]="张三";                    // 驾驶员 姓名

//-----------------  车辆信息 ------------------------------------------
//u8     Vech_VIN[17]="00000000000000000";        // 车辆VIN号
//u8     Vech_Num[12]="冀A00001";	                // 车牌号
//u8     Vech_Type[12]="000000000000";            // 车辆类型 


//-----------------  车辆信息 ------------------------------------------
//VechINFO  Vechicle_Info; 



u8     Vechicle_TYPE=1;                 //   车辆类型    1:大型货车  2: 小型货车  3:大型客车  4: 中型客车   5:小型客车
u8     OnFire_Status=0;                      //   1 : ACC 点火操作完成     0 :  ACC  关火操作完成 
u8     Login_Status=0x02;                    //   01H:登录，02H：退出，03H：更换驾驶员
u8     Powercut_Status=0x01;                 //01H:上电，02H：断电

u8     Settingchg_Status=0x00;                 /*
												82H:设置车辆信息，84H：设置状态量
												C2H:设置记录仪时钟 
												C3H:设置记录仪速度脉冲系数
											*/
               
//u16    DaySpdMax=0;                                //  当天最大速度
//u16    DayDistance=0;                              //  当天行驶距离

//------ 定距回传 ------

//--------------  定距上报  --------------------
u32  former_distance_meter=0;     //   上一次距离    定距回传时候有用
u32  current_distance_meter=0;    //   当前距离

//---------  SytemCounter ------------------
u32  Systerm_Reset_counter=0;
u8   DistanceWT_Flag=0;  //  写里程标志位
u8   SYSTEM_Reset_FLAG=0;        // 系统复位标志位 
u32  Device_type=0x00000001; //硬件类型   STM32103  新A1 
u32  Firmware_ver=0x0000001; // 软件版本
u8   ISP_resetFlag=0;        //远程升级复位标志位



//     终端属性相关     1  个版本说明
void ProductAttribute_init(void)
{
     ProductAttribute._1_DevType=0x07; //  客、货、危
     memcpy(ProductAttribute._2_ProducterID,"7_1_2",5);
     memset(ProductAttribute._3_Dev_TYPENUM,0,sizeof(ProductAttribute._3_Dev_TYPENUM));
     memcpy(ProductAttribute._3_Dev_TYPENUM,"TW703_BD",8);

   //终端ID 和SIM卡 ICCID 在初始化时读取

     
     ProductAttribute._6_HardwareVer_Len=14;
     memset(ProductAttribute._7_HardwareVer,0,sizeof(ProductAttribute._7_HardwareVer));
     memcpy(ProductAttribute._7_HardwareVer,"TW703_BD-HW1.0",14);

    ProductAttribute._8_SoftwareVer_len=14;
     memset(ProductAttribute._9_SoftwareVer,0,sizeof(ProductAttribute._9_SoftwareVer));
     memcpy(ProductAttribute._9_SoftwareVer,"TW703_BD-SW1.0",14);

    ProductAttribute._10_FirmWareVer_len=16;
     memset(ProductAttribute._11_FirmWare,0,sizeof(ProductAttribute._11_FirmWare));
     memcpy(ProductAttribute._11_FirmWare,"TW703_BD-Firm1.0",16);

    ProductAttribute._12_GNSSAttribute=0x03;  //   支持GPS 定位 北斗定位  

    ProductAttribute._13_ComModuleAttribute=0x01; // 支持GPRS 通信
		 
};



/*
          创建系统目录
*/
void    Create_Sys_Directory(void)
{


	 //  记录 
	  Api_MediaIndex_Init();    //  多媒体索引   图片+  语音
}


/*
       系统配置信息写入
*/
u8  SysConfig_init(void)
{
	  
       //  1. Stuff
	                 //   系统版本
	        SysConf_struct.Version_ID=SYSID; 
	                         //   APN  
		memset((u8*)SysConf_struct.APN_str,0 ,sizeof(SysConf_struct.APN_str));
		memcpy(SysConf_struct.APN_str,(u8*)APN_String,strlen((const char*)APN_String));  
	                        //   域名main 
		memset((u8*)SysConf_struct.DNSR,0 ,sizeof(SysConf_struct.DNSR));
		memcpy(SysConf_struct.DNSR,(u8*)DomainNameStr,strlen((const char*)DomainNameStr)); 
                            //   域名aux
		memset((u8*)SysConf_struct.DNSR_Aux,0 ,sizeof(SysConf_struct.DNSR_Aux));
		memcpy(SysConf_struct.DNSR_Aux,(u8*)DomainNameStr_aux,strlen((const char*)DomainNameStr_aux));   
		

			 
	        //   主 IP   +  端口
	        memcpy(SysConf_struct.IP_Main,(u8*)RemoteIP_main,4); 
	        SysConf_struct.Port_main=RemotePort_main;
	       //   备用 IP   +  端口
	        memcpy(SysConf_struct.IP_Aux,(u8*)RemoteIP_aux,4); 
	        SysConf_struct.Port_Aux=RemotePort_aux;	 				 

           //   LINK2 +      端口           
		   memcpy(SysConf_struct.Link2_IP,(u8*)Remote_Link2_IP,4); 
		   SysConf_struct.Link2_Port=Remote_Link2_Port;  					


           
				//	 IC 卡中心IP   TCP 端口 UDP  端口
				 memcpy(SysConf_struct.BD_IC_main_IP,(u8*)RemoteIP_aux,4);
		   SysConf_struct.BD_IC_TCP_port=RemotePort_aux;	  
		   SysConf_struct.BD_IC_UDP_port=29;


	       //  传感器触发上报状态
	       SysConf_struct.TriggerSDsatus=TriggerSDsatus;
		//   ACC  on   off  设置
		SysConf_struct.AccOn_Dur=ACC_on_sd_Duration;
		SysConf_struct.AccOff_Dur=ACC_off_sd_Duration; 
   //    2. Operate
            return(Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct)));       

}

void SysConfig_Read(void)
{
        if( Api_Config_read(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct))==false)   //读取系统配置信息
                      rt_kprintf("\r\nConfig_ Read Error\r\n");   

			
 		memset((u8*)APN_String,0 ,sizeof(APN_String)); 
		memcpy((u8*)APN_String,SysConf_struct.APN_str,strlen((const char*)SysConf_struct.APN_str));  
	                        //   域名
		memset((u8*)DomainNameStr,0 ,sizeof(DomainNameStr));
		memcpy((u8*)DomainNameStr,SysConf_struct.DNSR,strlen((const char*)SysConf_struct.DNSR)); 
	                        //   域名aux
		memset((u8*)DomainNameStr_aux,0 ,sizeof(DomainNameStr_aux));
		memcpy((u8*)DomainNameStr_aux,SysConf_struct.DNSR_Aux,strlen((const char*)SysConf_struct.DNSR_Aux)); 

		
			 
	        //   主 IP   +  端口
	        memcpy((u8*)RemoteIP_main,SysConf_struct.IP_Main,4); 
	        RemotePort_main=SysConf_struct.Port_main;
	       //   备用 IP   +  端口
	        memcpy((u8*)RemoteIP_aux,SysConf_struct.IP_Aux,4); 
	        RemotePort_aux=SysConf_struct.Port_Aux;	 		
		   //  Link2  
	        memcpy((u8*)Remote_Link2_IP,SysConf_struct.Link2_IP,4); 
	        Remote_Link2_Port=SysConf_struct.Link2_Port;	 				 

	       //  传感器触发上报状态
	        TriggerSDsatus=SysConf_struct.TriggerSDsatus;
		//   ACC  on   off  设置
		ACC_on_sd_Duration=SysConf_struct.AccOn_Dur;
		ACC_off_sd_Duration=SysConf_struct.AccOff_Dur;              


}


/*
       JT808    Related 
*/
void JT808_DURATION_Init(void)
{
    JT808Conf_struct.DURATION.Heart_Dur=60;       // 心跳包发送间隔 
	JT808Conf_struct.DURATION.TCP_ACK_Dur=20;     //  TCP 应答超时
	JT808Conf_struct.DURATION.TCP_ReSD_Num=3;     //  TCP 重发次数
	JT808Conf_struct.DURATION.TCP_ACK_Dur=20;     //  UDP 应答超时
	JT808Conf_struct.DURATION.UDP_ReSD_Num=5;     //  UDP 重发次数
 	JT808Conf_struct.DURATION.NoDrvLogin_Dur=40;  //  驾驶员没登陆时的发送间隔
 	JT808Conf_struct.DURATION.Sleep_Dur=30;       //  休眠时上报的时间间隔    
	JT808Conf_struct.DURATION.Emegence_Dur=20;    //  紧急报警时上报时间间隔
	JT808Conf_struct.DURATION.Default_Dur=30;     //  缺省情况下上报的时间间隔
	JT808Conf_struct.DURATION.SD_Delta_maxAngle=60; // 拐点补传的最大角度
	JT808Conf_struct.DURATION.IllgleMovo_disttance=300; // 非法移动阈值  
}

void JT808_SendDistances_Init(void)
{
   JT808Conf_struct.DISTANCE.Defalut_DistDelta=200;    // 默认定距回传距离
   JT808Conf_struct.DISTANCE.NoDrvLogin_Dist=300;      // 驾驶员未登录时回传距离
   JT808Conf_struct.DISTANCE.Sleep_Dist=500;           // 休眠情况下上报的定距回传
   JT808Conf_struct.DISTANCE.Emergen_Dist=100;         // 紧急报警情况下上报的定距回传 
}

//------------------------- 终端数据发送方式 -----------------------
void JT808_SendMode_Init(void)
{
  JT808Conf_struct.SD_MODE.DUR_TOTALMODE=1;  // 使能定时发送
  JT808Conf_struct.SD_MODE.Dur_DefaultMode=1; //  缺省方式上报
  JT808Conf_struct.SD_MODE.Dur_EmegencMode=0;
  JT808Conf_struct.SD_MODE.Dur_NologinMode=0;
  JT808Conf_struct.SD_MODE.Dur_SleepMode=0;

  JT808Conf_struct.SD_MODE.DIST_TOTALMODE=0;
  JT808Conf_struct.SD_MODE.Dist_DefaultMode=0;
  JT808Conf_struct.SD_MODE.Dist_EmgenceMode=0;
  JT808Conf_struct.SD_MODE.Dist_NoLoginMode=0;
  JT808Conf_struct.SD_MODE.Dist_SleepMode=0;

  
  JT808Conf_struct.SD_MODE.Send_strategy=0; 
}

//------------------------------------------------------------------
void  JT808_RealTimeLock_Init(void)
{                         // 预设值默认值     
   JT808Conf_struct.RT_LOCK.Lock_state=0;
   JT808Conf_struct.RT_LOCK.Lock_Dur=20;   
   JT808Conf_struct.RT_LOCK.Lock_KeepDur=300;
   JT808Conf_struct.RT_LOCK.Lock_KeepCnter=0;
}

void  Vehicleinfo_Init(void) 
{
	memset((u8*)&Vechicle_Info,0,sizeof(Vechicle_Info));
	//-----------------------------------------------------------------------
	memcpy(Vechicle_Info.Vech_VIN,"00000000000000000",17);
	memcpy(Vechicle_Info.Vech_Num,"未知车牌",8);        
	memcpy(Vechicle_Info.Vech_Type,"未知型",6);       
	memcpy(Vechicle_Info.ProType,"TW703",5);  
	Vechicle_Info.Dev_ProvinceID=64;  // 默认省ID   0      13  河北省  14 : 山西省  64: 宁夏
	Vechicle_Info.Dev_CityID=101;      // 默认市ID   0		 石家庄       101  :  银川  
	Vechicle_Info.Dev_Color=1;       // 默认颜色    // JT415    1  蓝 2 黄 3 黑 4 白 9其他     
	
	//Vechicle_Info.loginpassword_flag=0;
	Vechicle_Info.Link_Frist_Mode=0; //     0  : dnsr first     1: mainlink  first  

	DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));  
	
	WatchDog_Feed();
	DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
	
	WatchDog_Feed();
	DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
	
}

u8     JT808_Conf_init( void ) 
  {
         u8  FirstUseDate[6]={0x14,0x05,0x08,0x08,0x30,0x26};
            //  1.  clear
                    memset((u8*)&(JT808Conf_struct),0,sizeof(JT808Conf_struct)); 	  // 驾驶员信息						

		  
        //   2.  Stuff 
  		   JT808_DURATION_Init();
                         //  JT808Conf_struct.
                 JT808_SendDistances_Init();
		   JT808_SendMode_Init();
		   JT808Conf_struct.LOAD_STATE=0; //  负载状态		   
		   JT808Conf_struct.PositionSd_Stratage=0; // 根据ACC 状态
		   
		   memset((u8*)JT808Conf_struct.ConfirmCode,0,sizeof(JT808Conf_struct.ConfirmCode));
		   memcpy((u8*)JT808Conf_struct.ConfirmCode,"012345\x00",7); //  鉴权码

		   JT808Conf_struct.Regsiter_Status=0;   //  注册状态 

		   memset((u8*)JT808Conf_struct.LISTEN_Num,0,sizeof(JT808Conf_struct.LISTEN_Num));
		   memcpy((u8*)JT808Conf_struct.LISTEN_Num,"10086",5); //  监听号码

		   
		   	
		   memset((u8*)JT808Conf_struct.SMS_RXNum,0,sizeof(JT808Conf_struct.SMS_RXNum));
		   memcpy((u8*)JT808Conf_struct.SMS_RXNum,"106220801",9); //  监听号码

                 JT808Conf_struct.Vech_Character_Value=6240; // 特征系数  速度脉冲系数 



		   memset((u8*)JT808Conf_struct.FirstSetupDate,0,sizeof(JT808Conf_struct.FirstSetupDate));
		   memcpy((u8*)JT808Conf_struct.FirstSetupDate,FirstUseDate,6); // 首次安装时间
  

                 memset((u8*)JT808Conf_struct.DeviceOnlyID,0,sizeof(JT808Conf_struct.DeviceOnlyID));
		   memcpy((u8*)JT808Conf_struct.DeviceOnlyID,"00000010000000000000001",23);   //   行车记录仪的唯一ID

		   JT808Conf_struct.Msg_Float_ID=0;   // 消息流水号

  
		

                JT808Conf_struct.Distance_m_u32=0;            //  行驶记录仪行驶里程  单位: 米
                JT808Conf_struct.DayStartDistance_32=0;     //  每天的起始里程数目

                JT808Conf_struct.Speed_warn_MAX=200;           //  速度报警门限
                JT808Conf_struct.Spd_Exd_LimitSeconds=10;  //  超速报警持续时间门限 s
                JT808Conf_struct.Speed_GetType=0;             //  记录仪获取速度的方式  00  gps取速度  01 表示从传感器去速度 
                JT808Conf_struct.DF_K_adjustState=0; // 特征系数自动校准状态说明  1:自动校准过    0:尚未自动校准   

   	
                JT808Conf_struct.OutGPS_Flag=1;     //  0  默认  1  接外部有源天线 
                JT808Conf_struct.concuss_step=40;
				JT808Conf_struct.Auto_ATA_flag=0; // 不启用自动接听  
				
		   JT808_RealTimeLock_Init();   //  实时跟踪设置	

		    		 
                 memset((u8*)&(JT808Conf_struct.StdVersion),0,sizeof(JT808Conf_struct.StdVersion));  // 标准国家版本 
  		   memcpy((u8*)(JT808Conf_struct.StdVersion.stdverStr),"GB/T19056-2011",14); // 标准版本 
  	          JT808Conf_struct.StdVersion.MdfyID=0x02; //修改单号



		  memset((u8*)&(JT808Conf_struct.Driver_Info),0,sizeof(JT808Conf_struct.Driver_Info)); 	  // 驾驶员信息						
		//--------------------------------------------------------------------------
		
		  JT808Conf_struct.Driver_Info.Effective_Date[0]=0x20;
		  JT808Conf_struct.Driver_Info.Effective_Date[1]=0x07;
	      JT808Conf_struct.Driver_Info.Effective_Date[2]=0x01;
		  memcpy(JT808Conf_struct.Driver_Info.DriverCard_ID,"000000000000000000",18);  
		  memcpy(JT808Conf_struct.Driver_Info.DriveName,"未知",4);   
		  memcpy(JT808Conf_struct.Driver_Info.Drv_CareerID,"00000000000000000000",20); 
		  memcpy(JT808Conf_struct.Driver_Info.Comfirm_agentID,"000000000000000",16);
							
            //    3. Operate
            return(Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)));       

  }

void TIRED_DoorValue_Init(void)
{
    TiredConf_struct.TiredDoor.Door_DrvKeepingSec=14400;  // 国家标准是 3小时        
    TiredConf_struct.TiredDoor.Door_MinSleepSec=1200;     // 
    TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec=28800; //8小时
    TiredConf_struct.TiredDoor.Door_MaxParkingSec=7200;  // 2 小时  
    TiredConf_struct.TiredDoor.Parking_currentcnt=0;  // 停车状态计数器      
}

 void TIRED_DoorValue_Read(void)   //读时候要注意 有3个地方要清除 0
 {
 	  Api_Config_read(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct)); 
 }


u8   TIRED_CONF_Init(void)
{
    //  1.  clear
     memset((u8*)&(TiredConf_struct),0,sizeof(TiredConf_struct)); 	  // 驾驶员信息						

    // 2. stuff 
    TIRED_DoorValue_Init();
 
    //    2. Operate
     return(Api_Config_write(tired_config,0,(u8*)&TiredConf_struct,sizeof(TiredConf_struct)));       

}

/*
         事件 
*/
//-----------------------------------------------------------------
void Event_Write_Init(void)
{
u8 len_write=8;
    //事件写入
    len_write=8;
       EventObj.Event_ID=1;//事件ID
	EventObj.Event_Len=len_write;//长度 4*2
	EventObj.Event_Effective=1;//事件有效
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"申请出车",EventObj.Event_Len);  
       Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	len_write=16;
    EventObj.Event_ID=2;//事件ID
	EventObj.Event_Len=len_write;//长度 8*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"货己装齐准备启运",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	EventObj.Event_ID=3;//事件ID
	EventObj.Event_Len=len_write;//长度 8*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"平安到达一切顺利",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 
	
    EventObj.Event_ID=4;//事件ID
	EventObj.Event_Len=len_write;//长度 8*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"指定地点货未备齐",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	EventObj.Event_ID=5;//事件ID
	EventObj.Event_Len=len_write;//长度 8*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"指定地点无人接待",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	EventObj.Event_ID=6;//事件ID
	EventObj.Event_Len=len_write;//长度 8*2
	EventObj.Event_Effective=0;//事件有效
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"货到无法联系货主",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

       len_write=14;
	EventObj.Event_ID=7;//事件ID
	EventObj.Event_Len=len_write;//长度 7*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"货到因货损拒收",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 

	EventObj.Event_ID=8;//事件ID
	EventObj.Event_Len=len_write;//长度 7*2
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	memcpy(EventObj.Event_Str,"有急事请速回话",EventObj.Event_Len);  
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 
}

void Event_Read(void)
{
  u8 i=0;
  for(i=0;i<8;i++)
    {
	memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str)); 
       Api_RecordNum_Read(event_808, i+1, (u8*)&EventObj,sizeof(EventObj)); 
	
	EventObj_8[i].Event_ID=EventObj.Event_ID;
	EventObj_8[i].Event_Len=EventObj.Event_Len;
	EventObj_8[i].Event_Effective=EventObj.Event_Effective;
	memcpy(EventObj_8[i].Event_Str,EventObj.Event_Str,sizeof(EventObj.Event_Str));
//	memcpy(DisInfor_Affair[i],EventObj.Event_Str,20);
	//rt_kprintf("\r\n事件ID:%d  长度:%d  是否有效:%d(1显示0不显示) Info: %s",EventObj.Event_ID,EventObj.Event_Len,EventObj.Event_Effective,EventObj.Event_Str); 
	}
} 


void Event_Init(u8  Intype) 
{
  u8 i=0;

	  if(Intype==0)
	  {
	    EventObj.Event_Len=8; 
	    memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
	    memcpy(EventObj.Event_Str,"雨天路滑",8);
		EventObj.Event_Effective=1;
	  } 
	  else
	  if(Intype==1)
		{
		  EventObj.Event_Len=0;  
		  memset(EventObj.Event_Str,0,sizeof(EventObj.Event_Str));
		  EventObj.Event_Effective=0;
		}
	  
   for(i=0;i<8;i++)
   {
          EventObj.Event_ID=i+1;
          Api_RecordNum_Write(event_808, EventObj.Event_ID, (u8*)&EventObj,sizeof(EventObj)); 
   }
}

/*
          信息 
*/
//----------------------------------------------------------------
void MSG_BroadCast_Write_Init(void)
{
u8 len_write=8;
	MSG_BroadCast_Obj.INFO_TYPE=1;//类型
	MSG_BroadCast_Obj.INFO_LEN=len_write;//长度 4*2
	MSG_BroadCast_Obj.INFO_PlyCancel=1;//点播
	MSG_BroadCast_Obj.INFO_SDFlag=1;//发送标志位
	MSG_BroadCast_Obj.INFO_Effective=1;//显示有效
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"天气预报",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 

	MSG_BroadCast_Obj.INFO_TYPE=2;//类型
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"娱乐信息",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=3;//类型
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"交通信息",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=4;//类型
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"美食信息",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 

	MSG_BroadCast_Obj.INFO_Effective=0;//显示有效
	MSG_BroadCast_Obj.INFO_TYPE=5;//类型
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"记录信息",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=6;//类型
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"事件信息",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=7;//类型
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"时尚信息",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
	
	MSG_BroadCast_Obj.INFO_TYPE=8;//类型
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));
	memcpy(MSG_BroadCast_Obj.INFO_STR,"美容信息",MSG_BroadCast_Obj.INFO_LEN);  
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
}

void MSG_BroadCast_Read(void) 
{
  u8 i=0;
  
  for(i=0;i<8;i++)
    {
	memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR)); 
	Api_RecordNum_Read(msg_broadcast, i+1, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 

	MSG_Obj_8[i].INFO_TYPE=MSG_BroadCast_Obj.INFO_TYPE;
	MSG_Obj_8[i].INFO_LEN=MSG_BroadCast_Obj.INFO_LEN;
	MSG_Obj_8[i].INFO_PlyCancel=MSG_BroadCast_Obj.INFO_PlyCancel;
	MSG_Obj_8[i].INFO_SDFlag=MSG_BroadCast_Obj.INFO_SDFlag;
	MSG_Obj_8[i].INFO_Effective=MSG_BroadCast_Obj.INFO_Effective;
       memcpy(MSG_Obj_8[i].INFO_STR,MSG_BroadCast_Obj.INFO_STR,sizeof(MSG_BroadCast_Obj.INFO_STR));
	
	//memcpy(DisInfor_Menu[i],MSG_BroadCast_Obj.INFO_STR,20);    
	//rt_kprintf("\r\n 消息TYPE:%d  长度:%d  是否点播:%d 是否显示有效:%d(1显示0不显示) Info: %s",MSG_BroadCast_Obj.INFO_TYPE,MSG_BroadCast_Obj.INFO_LEN,MSG_BroadCast_Obj.INFO_PlyCancel,MSG_BroadCast_Obj.INFO_Effective,MSG_BroadCast_Obj.INFO_STR); 
    }
}

void MSG_BroadCast_Init(u8  Intype)
{
  u8 i=0;

	  if(Intype==0)
	  {
	    MSG_BroadCast_Obj.INFO_LEN=8; 
		MSG_BroadCast_Obj.INFO_Effective=1;
	    memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR)); 
	    memcpy(MSG_BroadCast_Obj.INFO_STR,"北京你好",8);
	  } 
	  else
	  if(Intype==1)
		{
		  MSG_BroadCast_Obj.INFO_LEN=0;  		  
		  MSG_BroadCast_Obj.INFO_Effective=0;   
		  memset(MSG_BroadCast_Obj.INFO_STR,0,sizeof(MSG_BroadCast_Obj.INFO_STR));    
		}
	  
   for(i=0;i<8;i++)
   {
       MSG_BroadCast_Obj.INFO_TYPE=i+1;
	Api_RecordNum_Write(msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj,sizeof(MSG_BroadCast_Obj)); 
   }
}

/*
      电话本
*/
void PhoneBook_Read(void)
{
  u8 i=0;

  for(i=0;i<8;i++)
  {    
      	Api_RecordNum_Read(phonebook, i+1, (u8*)&PhoneBook_8[i],sizeof(PhoneBook)); 
	//rt_kprintf("\r\n\r\n 电话本 TYPE: %d   Numlen=%d  Num: %s   UserLen: %d  UserName:%s \r\n",PhoneBook.CALL_TYPE,PhoneBook.NumLen,PhoneBook.NumberStr,PhoneBook.UserLen,PhoneBook.UserStr);  
  }
} 
void PhoneBook_Init(u8  Intype)
{
  u8 i=0;

	  if(Intype==0)
	  {
	    PhoneBook.Effective_Flag=1;  //有效标志位
           PhoneBook.CALL_TYPE=2; //类型定义为输出 
	    PhoneBook.NumLen=5;    // 号码长度
	    memset(PhoneBook.NumberStr,0,sizeof(PhoneBook.NumberStr)); // 号码内容
	    memcpy(PhoneBook.NumberStr,"10086",5);
		PhoneBook.UserLen=8;		// 用户名长度
	    memset(PhoneBook.UserStr,0,sizeof(PhoneBook.UserStr)); // 用户名内容
	    memcpy(PhoneBook.UserStr,"中国移动",8); 
		
	  } 
	  else
	  if(Intype==1)
		{
		   PhoneBook.Effective_Flag=0;  //有效标志位 
		   PhoneBook.CALL_TYPE=2; //类型定义为输出  
	       PhoneBook.NumLen=0;    // 号码长度
	       memset(PhoneBook.NumberStr,0,sizeof(PhoneBook.NumberStr));  
		   PhoneBook.UserLen=0;		
	       memset(PhoneBook.UserStr,0,sizeof(PhoneBook.UserStr)); 	
		}
	  
   for(i=0;i<8;i++)
   {
       	Api_RecordNum_Write(phonebook, i+1, (u8*)&PhoneBook,sizeof(PhoneBook)); 
   }
}

/*
       圆形围栏
*/
//---------------------------------------------------------- 
void  RailCycle_Init(void)
{
  u8 i=0;
  
   Rail_Cycle.Area_ID=0;
   Rail_Cycle.Area_attribute=0; // Bit 0  表示根据时间  该字段为0 表示围栏没有作用
   Rail_Cycle.Center_Latitude=0;
   Rail_Cycle.Center_Longitude=0;   
   Rail_Cycle.Radius=100; // 半径   
   Time2BCD(Rail_Cycle.StartTimeBCD);
   Time2BCD(Rail_Cycle.EndTimeBCD);
   Rail_Cycle.MaxSpd=100; // 最高速度
   Rail_Cycle.KeepDur=30; // 超速持续时间
   Rail_Cycle.Rail_Num=8;
   Rail_Cycle.Effective_flag=0; 

   for(i=0;i<8;i++)
   {
     Rail_Cycle.Area_ID=i+1;
     Api_RecordNum_Write(Rail_cycle,Rail_Cycle.Area_ID, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 	 
   }
}
void  RailCycle_Read(void)   
{
  u8 i=0;

   for(i=0;i<8;i++)
   {
          Api_RecordNum_Read(Rail_cycle,i+1, (u8*)&Rail_Cycle,sizeof(Rail_Cycle)); 		  
		//  rt_kprintf("\r\n\r\n 圆形围栏 TYPE: %d    atrri=%d  lati: %d  longiti:%d  radicus:%d	maxspd: %d  keepdur:%d \r\n",Rail_Cycle.Area_ID,Rail_Cycle.Area_attribute,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude,Rail_Cycle.Radius,Rail_Cycle.MaxSpd,Rail_Cycle.KeepDur);  
   }
}

/*
       矩形围栏
*/
void  RailRect_Init(void)
{
  u8 i=0;
  
   Rail_Rectangle.Area_ID=0;
   Rail_Rectangle.Area_attribute=0; // Bit 0  表示根据时间  该字段为0 表示围栏没有作用
   Rail_Rectangle.LeftUp_Latitude=0; // 左上
   Rail_Rectangle.LeftUp_Longitude=0;   
   Rail_Rectangle.RightDown_Latitude=0; //  右下 
   Rail_Rectangle.RightDown_Longitude=0;  
   Time2BCD(Rail_Rectangle.StartTimeBCD);
   Time2BCD(Rail_Rectangle.EndTimeBCD);
   Rail_Rectangle.MaxSpd=100; // 最高速度
   Rail_Rectangle.KeepDur=30; // 超速持续时间
   Rail_Rectangle.Rail_Num=8;  
   Rail_Rectangle.Effective_flag=0;

   for(i=0;i<8;i++)
   {
     Rail_Rectangle.Area_ID=i+1;
     Api_RecordNum_Write(Rail_rect,Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); 	 
   }      
   
}

void  RailRect_Read(void)
{
  u8 i=0;

   for(i=0;i<8;i++)
   {
	 Api_RecordNum_Read(Rail_rect,i+1, (u8*)&Rail_Rectangle,sizeof(Rail_Rectangle)); 	 	  
        //  rt_kprintf("\r\n\r\n 矩形形围栏 TYPE: %d    atrri=%d  leftlati: %d  leftlongiti:%d    rightLati:%d   rightLongitu: %d  \r\n",Rail_Rectangle.Area_ID,Rail_Rectangle.Area_attribute,Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude,Rail_Rectangle.RightDown_Latitude,Rail_Rectangle.RightDown_Longitude);  
   }  
}  

/*
       多边形围栏
*/
void  RailPolygen_Init(void)
{
  u8 i=0;
  
   Rail_Polygen.Area_ID=0;
   Rail_Polygen.Area_attribute=0; // Bit 0  表示根据时间  该字段为0 表示围栏没有作用
   Time2BCD(Rail_Polygen.StartTimeBCD);
   Time2BCD(Rail_Polygen.EndTimeBCD);
   Rail_Polygen.MaxSpd=100; // 最高速度
   Rail_Polygen.KeepDur=30; // 超速持续时间
   Rail_Polygen.Acme_Num=3;   
   Rail_Polygen.Acme1_Latitude=10; //顶点1 
   Rail_Polygen.Acme1_Longitude=10;   
   Rail_Polygen.Acme2_Latitude=20; //顶点2
   Rail_Polygen.Acme2_Longitude=20;     
   Rail_Polygen.Acme3_Latitude=30; //顶点3
   Rail_Polygen.Acme3_Longitude=30;  
   Rail_Polygen.Effective_flag=0;

   for(i=0;i<8;i++)
   {
     Rail_Polygen.Area_ID=i+1;
     Api_RecordNum_Write(Rail_polygen,Rail_Polygen.Area_ID, (u8*)&Rail_Polygen,sizeof(Rail_Polygen)); 	 
   }
}

void  RailPolygen_Read(void)
{
  u8 i=0;

   for(i=0;i<8;i++)
   {
       Api_RecordNum_Read(Rail_polygen,i+1, (u8*)&Rail_Polygen,sizeof(Rail_Polygen)); 	 		  
       //  rt_kprintf("\r\n\r\n 多边形围栏 TYPE: %d   1lat: %d  1long:%d    2lat:%d   2long: %d  3lat:%d 3long:%d \r\n",Rail_Polygen.Area_ID,Rail_Polygen.Acme1_Latitude,Rail_Polygen.Acme1_Longitude,Rail_Polygen.Acme2_Latitude,Rail_Polygen.Acme2_Longitude,Rail_Polygen.Acme3_Latitude,Rail_Polygen.Acme3_Longitude);   
   }
} 

/*
        拐点设置    (Maybe Null)
*/
      




/*
        路线设置围栏 
*/
void  RouteLine_Obj_init(void)
{
  u8 i=0;
  
   ROUTE_Obj.Route_ID=0;
   ROUTE_Obj.Route_attribute=0; // Bit 0  表示根据时间  该字段为0 表示围栏没有作用
   Time2BCD(ROUTE_Obj.StartTimeBCD);
   Time2BCD(ROUTE_Obj.EndTimeBCD);
   ROUTE_Obj.Points_Num=3;   
   for(i=0;i<3;i++)
   	{
   	  ROUTE_Obj.RoutePoints[i].POINT_ID=i+1;
	  ROUTE_Obj.RoutePoints[i].Line_ID=i;
	  ROUTE_Obj.RoutePoints[i].POINT_Latitude=i+300;
	  ROUTE_Obj.RoutePoints[i].POINT_Longitude=i+500;
	  ROUTE_Obj.RoutePoints[i].Width=20;
	  ROUTE_Obj.RoutePoints[i].Atribute=0; // 0 表示未启用
	  ROUTE_Obj.RoutePoints[i].TooLongValue=100;
	  ROUTE_Obj.RoutePoints[i].TooLessValue=50;
	  ROUTE_Obj.RoutePoints[i].MaxSpd=60;
	  ROUTE_Obj.RoutePoints[i].KeepDur=3;
	  ROUTE_Obj.Effective_flag=0; 
   	}

}
 
 void  RouteLine_Init(void)
{
  u8 i=0;
  
    RouteLine_Obj_init();

   for(i=0;i<8;i++)
   {
     ROUTE_Obj.Route_ID=i+1;
     Api_RecordNum_Write(route_line,ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj)); 	 
   }
}

void  RouteLine_Read(void)
{
  u8 i=0;

   for(i=0;i<8;i++)
   {
       Api_RecordNum_Read(route_line,i+1, (u8*)&ROUTE_Obj,sizeof(ROUTE_Obj)); 	 
	//  rt_kprintf("\r\n\r\n 路线 TYPE: %d   pointsNum:%d   p1lat: %d  p1long:%d    p2lat:%d   p2long: %d  p3lat:%d p3long:%d \r\n",ROUTE_Obj.Route_ID,ROUTE_Obj.Points_Num,ROUTE_Obj.RoutePoints[0].POINT_Latitude,ROUTE_Obj.RoutePoints[0].POINT_Longitude,ROUTE_Obj.RoutePoints[1].POINT_Latitude,ROUTE_Obj.RoutePoints[1].POINT_Longitude,ROUTE_Obj.RoutePoints[2].POINT_Latitude,ROUTE_Obj.RoutePoints[2].POINT_Longitude);   
   }
}    

/*
       提问
*/
#if 0
void Question_Read(void)
{
	//提问消息	 读出
	  Api_RecordNum_Read(ask_quesstion,1, (u8*)&ASK_Centre,sizeof(ASK_Centre)); 	
	  rt_kprintf("\r\n标志位:%d  流水号:%d  信息长度:%d 回复ID:%d",ASK_Centre.ASK_SdFlag,ASK_Centre.ASK_floatID,ASK_Centre.ASK_infolen,ASK_Centre.ASK_answerID); 
	if(ASK_Centre.ASK_SdFlag==1)
	  {
	  ASK_Centre.ASK_SdFlag=0;
	  rt_kprintf("\r\n信息内容: %s",ASK_Centre.ASK_info); 
	  rt_kprintf("\r\n候选答案: %s",ASK_Centre.ASK_answer); 
	  }
}
 #endif
 

/*
        文本信息
*/

void TEXTMsg_Read (void)  
{
  u8 i=0,min=0,max=0;
  
		  for(i=0;i<=7;i++)
		    {
                      Api_RecordNum_Read(text_msg,i+1, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 			
			memcpy((u8*)&TEXT_Obj_8bak[i],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));     
		    //rt_kprintf("\r\n文本信息 最新:%d  消息TYPE:%d  长度:%d",TEXT_Obj_8bak[i].TEXT_mOld,TEXT_Obj_8bak[i].TEXT_TYPE,TEXT_Obj_8bak[i].TEXT_LEN);  
		    }

		  //最新一条数据
		  max=TEXT_Obj_8bak[0].TEXT_TYPE;    
		  for(i=0;i<=7;i++)
		  	{
		  	if(TEXT_Obj_8bak[i].TEXT_TYPE>max)
				max=TEXT_Obj_8bak[i].TEXT_TYPE;
		  	}
		  TextInforCounter=max;
		  //rt_kprintf("\r\n  最新数据序号  max=%d,TextInforCounter=%d",max,TextInforCounter); 
		  

		  //找出最早的一条信息
		  min=TEXT_Obj_8bak[0].TEXT_TYPE;
		  //rt_kprintf("\r\n  排序前  最老一条信息序号  min=%d",min); 
		  for(i=0;i<=7;i++)
		  	{
		  	if((TEXT_Obj_8bak[i].TEXT_TYPE<min)&&(TEXT_Obj_8bak[i].TEXT_TYPE>0))
				min=TEXT_Obj_8bak[i].TEXT_TYPE;
		  	}
		  //rt_kprintf("\r\n	排序后	最老一条信息序号  min=%d",min); 
  
  if(max<1)return;
  
  if(max<=8)
  	{
  	for(i=1;i<=max;i++)
		{
		Api_RecordNum_Read(text_msg,max-i+1, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 	
		memcpy((u8*)&TEXT_Obj_8[i-1],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));
		//rt_kprintf("\r\n(<8)消息TYPE:%d  长度:%d",TEXT_Obj_8[i-min].TEXT_TYPE,TEXT_Obj_8[i-min].TEXT_LEN);   
		}
  	}
  else
  	{
  	  max=max%8;
	  //rt_kprintf("\r\n max%8=%d",max);   

	  if(max==0)
	  	{
  		  for(i=0;i<8;i++)
			{
			Api_RecordNum_Read(text_msg,(7-i), (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 	
			memcpy((u8*)&TEXT_Obj_8[i],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));
			//rt_kprintf("\r\n(8*n)i=%d 消息TYPE:%d  长度:%d",i,TEXT_Obj_8[i].TEXT_TYPE,TEXT_Obj_8[i].TEXT_LEN);   
			}
	  	}
	  else
	  	{
		  for(i=0;i<max;i++)
			{
			Api_RecordNum_Read(text_msg,(max-i), (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 
			memcpy((u8*)&TEXT_Obj_8[i],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));
			//rt_kprintf("\r\n(1)消息TYPE:%d  长度:%d",TEXT_Obj_8[i].TEXT_TYPE,TEXT_Obj_8[i].TEXT_LEN);   
			}
		  for(i=7;i>=max;i--)
			{
			Api_RecordNum_Read(text_msg,i+1, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 
			memcpy((u8*)&TEXT_Obj_8[max+7-i],(u8*)&TEXT_Obj,sizeof(TEXT_Obj));
			//rt_kprintf("\r\n(2)消息TYPE:%d  长度:%d",TEXT_Obj_8[max+7-i].TEXT_TYPE,TEXT_Obj_8[max+7-i].TEXT_LEN);  
			}
	  	}
	  }
}

void TEXTMSG_Write(u8 num,u8 new_state,u8 len,u8 *str)   
{     //  写单条信息
u8 pos_1_8=0;//,i=0;
    //事件写入
       TEXT_Obj.TEXT_mOld=new_state;//是否是最新信息
	TEXT_Obj.TEXT_TYPE=num;// 1
	TEXT_Obj.TEXT_LEN=len; //消息长度
	memset(TEXT_Obj.TEXT_STR,0,sizeof(TEXT_Obj.TEXT_STR));
	memcpy(TEXT_Obj.TEXT_STR,str,TEXT_Obj.TEXT_LEN); 
	
	if(num%8)
		pos_1_8=TEXT_Obj.TEXT_TYPE%8;
	else
		pos_1_8=8; 
        Api_RecordNum_Write(text_msg,pos_1_8, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 
}

void TEXTMSG_Write_Init(void)
{
    u8  i=0;

       TEXT_Obj.TEXT_mOld=1;//是否是最新信息
	TEXT_Obj.TEXT_TYPE=1;// 1
	TEXT_Obj.TEXT_LEN=0; //消息长度
	memset(TEXT_Obj.TEXT_STR,0,sizeof(TEXT_Obj.TEXT_STR));   

	for(i=0;i<8;i++)
	{
	     //-------------------------------------------------------
             if(i==0)
			  TEXT_Obj.TEXT_mOld=1;//是否是最新信息 	
	      else
		  	    TEXT_Obj.TEXT_mOld=0;//是否是最新信息
           //-------------------------------------------------------
           TEXT_Obj.TEXT_TYPE=i+1;// 1
           Api_RecordNum_Write(text_msg,TEXT_Obj.TEXT_TYPE, (u8*)&TEXT_Obj,sizeof(TEXT_Obj)); 

	}
}


/*
     多媒体索引，用的时候再操作 write / read   初始化话不用处理
*/




void  BD_EXT_initial(void)
{
      //    北斗设置
    BD_EXT.GNSS_Mode=0x02;     //   双模
    BD_EXT.GNSS_Baud=0x01;     //   9600
    BD_EXT.BD_OutputFreq=0x01;  // 1000ms
    BD_EXT.BD_SampleFrea=1; // 
    BD_EXT.GNSS_Baud=0x01;  //  9600
    //-----  车台相关 ----------------------
    BD_EXT.Termi_Type=0x0001;   //  终端类型
    BD_EXT.Software_Ver=0x0100; //  Ver  1.00
    BD_EXT.GNSS_Attribute=0x54444244;// TDBD
    BD_EXT.GSMmodule_Attribute=0x48554157;// HUAW
    BD_EXT.Device_Attribute=0x00000001; //  终端属性

     //   CAN   相关设置
     BD_EXT.CAN_1_Mode=0xC0000014;    //  CAN  模式  01:还回模式  10  : 普通模式   11:  静默模式  
          /*  bit31  开启  bit 30 -29   模式   bit 15-0   波特率0x14   <=> 20k*/
     BD_EXT.CAN_1_ID=0x01;
     BD_EXT.CAN_1_Type=0;// 扩展帧
//     BD_EXT.CAN_1_Duration=1;  // 0   表示停止
     BD_EXT.CAN_2_Mode=0xC0000014;   //   CAN  模式 
     BD_EXT.CAN_2_ID=0x02;
     BD_EXT.CAN_2_Type=0;// 扩展帧	 
    // BD_EXT.CAN_2_Duration=1;  // 0   表示停止
     BD_EXT.Collision_Check=0x0101;     //     关闭震动  0.1g    4ms

  //   位置附加信息
       // 1. 信号强度
      BD_EXT.FJ_SignalValue=0x0000;  //  信号强度   高字节 0 ；低字节  高4为 X2 gprs强度 ，低4位 卫星颗数
      //  2. 自定义状态机模拟量上传  
       BD_EXT.FJ_IO_1=0x00;
	BD_EXT.FJ_IO_2=0x00;
	BD_EXT.AD_0=0x00;  //  2 Byte
	BD_EXT.AD_1=0x00; //   2 Byte
 
       BD_EXT.Close_CommunicateFlag=disable; 
	BD_EXT.Trans_GNSS_Flag=disable;

       Api_Config_Recwrite_Large(BD_ext_config,0,(u8*)&BD_EXT,sizeof(BD_EXT));   
       DF_delay_ms(5);
 
	
}

void BD_EXT_Write(void)
{
    Api_Config_Recwrite_Large(BD_ext_config,0,(u8*)&BD_EXT,sizeof(BD_EXT));   
    DF_delay_ms(15);
}

void BD_EXT_Read(void)
{
    Api_Config_read(BD_ext_config,0,(u8*)&BD_EXT,sizeof(BD_EXT));  	 
}

void  BD_list(void)
{ 
     u8  BD_str[20];
	 
     //-----list -----
     rt_kprintf("\r\n -------北斗扩展信息相关------\r\n ");
     rt_kprintf("\r\n\r\n	 终端类型:    0x%08X      \r\n       软件版本:   0x%08X  \r\n",BD_EXT.Termi_Type,BD_EXT.Software_Ver);  
     rt_kprintf("\r\n\r\n	 GNNS属性:   %04s      \r\n       GSM属性:  %04s    \r\n   终端属性:  0x%08X \r\n\r\n    ",(char*)&BD_EXT.GNSS_Attribute,(char*)&BD_EXT.GSMmodule_Attribute,BD_EXT.Device_Attribute);   

     memset(BD_str,0,sizeof(BD_str));
     switch(BD_EXT.CAN_1_Mode&0x60000000)
     	{
     	    case 0x20000000: 
				memcpy(BD_str,"还回模式",8);
			        break;
	    case 0x40000000:
                           	memcpy(BD_str,"普通模式",8);
			        break;
	    case 0x6000000:
                           	memcpy(BD_str,"静默模式",8);
				break; 

     	}
     rt_kprintf("\r\n   CAN1 :\r\n            CAN1  Mode:    %s      \r\n             CAN1  ID:    0x%08X      \r\n\r\n ",BD_str,BD_EXT.CAN_1_ID);  

      memset(BD_str,0,sizeof(BD_str));
     switch(BD_EXT.CAN_2_Mode&0x60000000)
     	{
     	    case 0x20000000: 
				memcpy(BD_str,"还回模式",8);
			        break;
	    case 0x40000000:
                           	memcpy(BD_str,"普通模式",8);
			        break;
	    case 0x6000000:
                           	memcpy(BD_str,"静默模式",8);
				break; 

     	}
  //   rt_kprintf("\r\n   CAN2 :\r\n            CAN2  Mode:    %s      \r\n             CAN2  ID:    0x%08X      \r\n\r\n ",BD_str,BD_EXT.CAN_2_ID);

     

}


void  SendMode_ConterProcess(void)         //  定时发送处理程序
{   //   发送方式计数器处理
   //  1. 心跳包计数器
    JT808Conf_struct.DURATION.Heart_SDCnter++;       
    if(JT808Conf_struct.DURATION.Heart_SDCnter>JT808Conf_struct.DURATION.Heart_Dur)  //超过心跳包设置的间隔
      	{
            JT808Conf_struct.DURATION.Heart_SDCnter=0;     
            JT808Conf_struct.DURATION.Heart_SDFlag=1; 
    	}
   //  2. 发送超时判断
   #if 0
    if(1==JT808Conf_struct.DURATION.TCP_SD_state)
    {
      JT808Conf_struct.DURATION.TCP_ACK_DurCnter++;
	  if(JT808Conf_struct.DURATION.TCP_ACK_DurCnter>JT808Conf_struct.DURATION.TCP_ACK_Dur) //发送应答定时
	  	{
          JT808Conf_struct.DURATION.TCP_ACK_DurCnter=0;
		  JT808Conf_struct.DURATION.Heart_SDFlag=1;         //重新发送
		  JT808Conf_struct.DURATION.TCP_ReSD_cnter++;
		  if(JT808Conf_struct.DURATION.TCP_ReSD_cnter>JT808Conf_struct.DURATION.TCP_ReSD_Num)  //重新发送次数判断
		  	{
               JT808Conf_struct.DURATION.TCP_ReSD_cnter=0;
			   Close_DataLink();   // AT_End();	   //挂断GPRS链接    

		  	}
	  	}
 
    }
   #endif  
}


//----------  
void  Rails_Routline_Read(void)
{
   u16  i=0;
   
            //-----------读取围栏状态-------
           
		   for(i=0;i<8;i++)
		   {
				Api_RecordNum_Read(Rail_rect,i+1, (u8*)&Rail_Rectangle_multi[i], sizeof(Rail_Rectangle));
				delay_ms(2);
				Api_RecordNum_Read(Rail_cycle,i+1, (u8*)&Rail_Cycle_multi[i],sizeof(Rail_Cycle));	

				
		   }  

} 


//-----------------------------------------------------------------

void  FirstRun_Config_Write(void)
{     //   程序首次更新是写配置操作 

       //  rt_kprintf("\r\n  sizeof(sysconfig): %d   sizeof(jt808): %d    sizeof(tiredconfig): %d   \r\n",sizeof(SysConf_struct),sizeof(JT808Conf_struct),sizeof(TiredConf_struct)); 
                  SysConfig_init();   //  写入系统配置信息
                  TIRED_CONF_Init(); //  写入疲劳驾驶相关配置信息
                 JT808_Conf_init();   //  写入 JT808   配置信息
                  Api_WriteInit_var_rd_wr();	
		    BD_EXT_initial(); 		  

			     
				 Vehicleinfo_Init();// 写入车辆信息
                 Event_Write_Init();
		   MSG_BroadCast_Write_Init();
		   PhoneBook_Init(0);  
		   RailCycle_Init();		   
		   RailRect_Init();
		   RailPolygen_Init();	
		   RouteLine_Init(); 
                 TEXTMSG_Write_Init();	   
				 
 		  //---- add special -----------  
 		  Login_Menu_Flag=0;     //  输入界面为0 
		  DF_WriteFlashSector(DF_LOGIIN_Flag_offset,0,&Login_Menu_Flag,1); 
		  Limit_max_SateFlag=1; //使能
		  DF_WriteFlashSector(DF_LimitSPEED_offset,0,&Limit_max_SateFlag,1); 
		  

}
//-----------------------------------------------------------------
void SetConfig(void)
{
	//u8 i=0;//,len_write=0;
//	u32 j=0;
	
       rt_kprintf("\r\nSave Config\r\n");
	// 1.  读取config 操作      0 :成功    1 :  失败
	Api_Config_read(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));           
       //rt_kprintf("\r\nRead Save SYSID\r\n");
       //  2. 读取成功  ，判断  版本ID 
	if(SysConf_struct.Version_ID!=SYSID)//SYSID)   //  check  wether need  update  or not 
	{
	       rt_kprintf("\r\n ID not Equal   Saved==0x%X ,  Read==0x%X !\r\n",SYSID,SysConf_struct.Version_ID);	
	        SysConf_struct.Version_ID=SYSID;  // update  ID 
         //  2.1  播报使能
         
		 DF_WriteFlashSector(DF_WARN_PLAY_Page,0,&Warn_Play_controlBit,1); 
              
	     //  2.2  重新写入		 
           FirstRun_Config_Write();   // 里边更新了 SYSID           
		   vdr_erase(); 

	}
	else			
		   rt_kprintf("\r\n Config Already Exist!\r\n"); 
}

 void ReadConfig(void) 
{
    u16   res[3];
	
           DF_delay_ms(500);  

		  //  1.   read  read to  compare  judge
         //-------- JT808 参数配置读取测试，操作频繁而且重要所以需要特殊处理 
           DF_ReadFlash(JT808Start_offset, 0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct)); 
		   DF_delay_ms(80); 	// large content delay	

		   DF_ReadFlash(JT808_BakSetting_offset, 0,(u8*)&JT808_struct_Bak,sizeof(JT808_struct_Bak)); 
		   DF_delay_ms(80); 	// large content delay	 

		   DF_ReadFlash(JT808_Bak2Setting_offset, 0,(u8*)&JT808_struct_Bak2,sizeof(JT808_struct_Bak2)); 
		   DF_delay_ms(80); 	// large content delay	

		   // 2. compare
		   /*
		           note:   res[0] == org cmp  bak    res[1]== bak  cmp  bak2    res[2]== bak2  cmp  org		  

		           ---org --<seg1>--  bak ---<seg2>----bak2 ---<seg3>---
		           |-----------<---------------<----------------------|
		    */
		   res[0]=memcmp((u8*)&JT808Conf_struct,(u8*)&JT808_struct_Bak,sizeof(JT808_struct_Bak));
		   res[1]=memcmp((u8*)&JT808_struct_Bak,(u8*)&JT808_struct_Bak2,sizeof(JT808_struct_Bak));
		   res[2]=memcmp((u8*)&JT808_struct_Bak2,(u8*)&JT808Conf_struct,sizeof(JT808_struct_Bak));

           // 3. judge 
           if(res[0]&&res[1]&&res[2])   // 全有问题
           	{
           	   rt_kprintf("\r\n JT808 全部失败!  need all recover\r\n"); 	
	           JT808_Conf_init();	
           	}
		   else
		   if(res[0]&&res[1])   //    seg1  seg2  有问题说明  BAK error
		   	{    
		   	    // org  bak2 ---ok      bak---error
		   	   if((u8)(JT808Conf_struct.DURATION.Default_Dur>>24)!=0xFF) // 判断正确的是不是 FF
		   	   	{ 
		   	   	 
				  DF_WriteFlashSector(JT808_BakSetting_offset,0,(u8*)&JT808Conf_struct,sizeof(JT808_struct_Bak));
				  rt_kprintf("\r\n JT808 BAK error ,correct ok"); 			

		   	   	}
			   else
			   	{ 
			   	  rt_kprintf("\r\n need all recover 1"); 
				  JT808_Conf_init();
			   	}

		   	}
		   else
		   if(res[0]&&res[2])  //  seg1  seg3    有问题说明 BAK2  error
		   	{
		   	   // org  bak  ---ok       bak2 -----error
		   	   if((u8)(JT808Conf_struct.DURATION.Default_Dur>>24)!=0xFF) // 判断正确的是不是 FF
		   	   	{ 
		   	   	 
				  DF_WriteFlashSector(JT808_Bak2Setting_offset,0,(u8*)&JT808Conf_struct,sizeof(JT808_struct_Bak));
				  rt_kprintf("\r\n JT808 BAK2 error ,correct ok"); 			

		   	   	}
			   else
			   	{ 
			   	  rt_kprintf("\r\n need all recover 2"); 
				  JT808_Conf_init();
			   	}

		   	}
		   else
		   if(res[1]&&res[2])  //  seg2  seg3	 有问题说明 org  error
			{
			   //  bak  bak2 --ok     org---error
		         if((u8)(JT808_struct_Bak.DURATION.Default_Dur>>24)!=0xFF) // 判断正确的是不是 FF
		   	   	{ 
		   	   	 
				  DF_WriteFlashSector(JT808Start_offset,0,(u8*)&JT808_struct_Bak,sizeof(JT808_struct_Bak));
				  rt_kprintf("\r\n JT808 org error ,correct ok"); 	 		

		   	   	}
			   else
			   	{ 
			   	  rt_kprintf("\r\n need all recover 3"); 
				  JT808_Conf_init();
			   	}
		   
			} 
		   else
		   	rt_kprintf("\r\n JT808 读取校验成功! \r\n"); 
         //-------------------------------------------------------------------------------------
		   
		   SysConfig_Read();  //读取系统配置信息	                 
           TIRED_DoorValue_Read();    
		   
           Event_Read();
		   MSG_BroadCast_Read();
		   PhoneBook_Read();  
		  // RailCycle_Read();
 		   //RailPolygen_Read();
 		   //RailRect_Read();
 		   //RouteLine_Read();   
                 TEXTMsg_Read();	 
                 BD_EXT_Read();   
		   Api_Read_var_rd_wr();	   	  	   

           //  Vechicle  compare
		   DF_ReadFlash(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));   
		   
		   WatchDog_Feed();
		   DF_ReadFlash(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info_BAK,sizeof(Vechicle_Info_BAK)); 

		   WatchDog_Feed();
		   DF_ReadFlash(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_info_BAK2,sizeof(Vechicle_info_BAK2)); 

		   //  compare
		   /*
		           note:   res[0] == org cmp  bak    res[1]== bak  cmp  bak2    res[2]== bak2  cmp  org		  

		           ---org --<seg1>--  bak ---<seg2>----bak2 ---<seg3>---
		           |-----------<---------------<----------------------|
		    */
		   res[0]=memcmp((u8*)&Vechicle_Info,(u8*)&Vechicle_Info_BAK,sizeof(Vechicle_Info_BAK));	
		   res[1]=memcmp((u8*)&Vechicle_Info_BAK,(u8*)&Vechicle_info_BAK2,sizeof(Vechicle_Info_BAK));
		   res[2]=memcmp((u8*)&Vechicle_info_BAK2,(u8*)&Vechicle_Info,sizeof(Vechicle_Info_BAK));

			// 3. judge 
			if(res[0]&&res[1]&&res[2])	 // 全有问题
			 {
				rt_kprintf("\r\n Vechicle全部失败! \r\n");	 
				rt_kprintf("\r\n need all recover"); 
				Vehicleinfo_Init();// 写入车辆信息	 
			 }
			else
			if(res[0]&&res[1])	 //    seg1  seg2  有问题说明  BAK error
			 {	  
				 // org  bak2 ---ok 	 bak---error
				if((u8)(Vechicle_Info.Dev_CityID>>8)!=0xFF) // 判断正确的是不是 FF
				{ 
				  DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
				  rt_kprintf("\r\n Vehicle BAK error ,correct ok");   
		   	   	}
			   else
			   	{ 
			   	  rt_kprintf("\r\n Vehicle need all recover 1"); 
				  Vehicleinfo_Init();
			   	}			
			 }
			else
			if(res[0]&&res[2])	//	seg1  seg3	  有问题说明 BAK2  error
			 {
				// org	bak  ---ok		 bak2 -----error
			    if((u8)(Vechicle_Info.Dev_CityID>>8)!=0xFF) // 判断正确的是不是 FF
				{ 
				  DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
				  rt_kprintf("\r\n Vehicle BAK2 error ,correct ok");   
		   	   	}
			   else
			   	{ 
			   	  rt_kprintf("\r\n Vehicle need all recover 2"); 
				  Vehicleinfo_Init();
			   	}
			
			 }
			else
			if(res[1]&&res[2])	//	seg2  seg3	  有问题说明 org  error
			 {
				//	bak  bak2 --ok	   org---error
			    	if((u8)(Vechicle_Info.Dev_CityID>>8)!=0xFF) // 判断正确的是不是 FF
				{ 
				  DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info_BAK,sizeof(Vechicle_Info_BAK)); 
				  rt_kprintf("\r\n Vehicle BAK error ,correct ok");   
		   	   	}
			   else
			   	{ 
			   	  rt_kprintf("\r\n Vehicle need all recover 3"); 
				  Vehicleinfo_Init();
			   	}
			
			 }
			else
				rt_kprintf("\r\n Vehicle 读取校验成功! \r\n");  
			//---- 设备ID  --------	 
		   memset(DeviceNumberID,0,sizeof(DeviceNumberID));
		   DF_ReadFlash(DF_DeviceID_offset,0,(u8*)DeviceNumberID,12);  
		   //读出车牌号是否设置标志
		   DF_ReadFlash(DF_License_effect,0,&License_Not_SetEnable,1); 
		   
		   if(License_Not_SetEnable==1) 
		   	rt_kprintf("\r\n无牌照\r\n");  
		   //  ------ SIM ID  入网ID --------
		   memset(SimID_12D,0,sizeof(SimID_12D));
		   DF_ReadFlash(DF_SIMID_12D,0,(u8*)SimID_12D,12);  

		   //------  读取 录入状态-----------
		   DF_ReadFlash(DF_LOGIIN_Flag_offset,0,&Login_Menu_Flag,1); 

		   //  读取播报状态		   
		   DF_ReadFlash(DF_WARN_PLAY_Page,0,&Warn_Play_controlBit,1); 
		   
		   if(JT808Conf_struct.DF_K_adjustState)  
		   {
                      ModuleStatus|=Status_Pcheck; 
		   }
           else 
		   {
		              ModuleStatus&=~Status_Pcheck;
           } 
		    DF_ReadFlash(DF_LimitSPEED_offset,0,&Limit_max_SateFlag,1); 
		   rt_kprintf("\r\n  Limit_max_stateflag=%d",Limit_max_SateFlag); 
           Rails_Routline_Read();
                 
    rt_kprintf("\r\n Read Config Over \r\n");   
}
void DefaultConfig(void)
{
   u32 DriveCode32=0;
   u8  reg_str[30],i=0;

       rt_kprintf("\r\n         SYSTEM ID=0x%X ",SysConf_struct.Version_ID);   
	rt_kprintf("\r\n		   设置的鉴权码为: ");
       rt_kprintf(" %s\r\n		   鉴权码长度: %d\r\n",JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode));   
					   
  if(JT808Conf_struct.Regsiter_Status)
  	   rt_kprintf("\r\n		   该终端已经注册过!    %d \r\n",JT808Conf_struct.Regsiter_Status);  
  else
  	   rt_kprintf("\r\n		   该终端被尚未被注册!\r\n");   
  
  
	  rt_kprintf("\r\n中心控制断电= %d (1 : 断电  0 : 正常)\r\n", JT808Conf_struct.relay_flag);     
   if(JT808Conf_struct.relay_flag==1)
		  {
	  Enable_Relay();
	   Car_Status[2]|=0x08; 	// 需要控制继电器
	   }
  else
	  {
	  Disable_Relay();
	  Car_Status[2]&=~0x08;    // 需要控制继电器
	  rt_kprintf("\r\n继电器闭合");
	  }
        
        // APN 设置
	     rt_kprintf("\r\n		   APN 设置 :%s	 \r\n",APN_String); 
         DataLink_APN_Set(APN_String,1); 
            //     域名
          memset(reg_str,0,sizeof(reg_str));
          memcpy(reg_str,DomainNameStr,strlen((char const*)DomainNameStr));
          rt_kprintf("\r\n		  域名设置 :	 %s\r\n",reg_str); 
          KorH_check();  
		 
		  
	      //    域名aux
          memset(reg_str,0,sizeof(reg_str));
          memcpy(reg_str,DomainNameStr_aux,strlen((char const*)DomainNameStr_aux));
          rt_kprintf("\r\n		aux  域名设置 :	 %s\r\n",reg_str); 	    
					  
         // 数据中心IP 地址(4Bytes)  UDP端口号码(2Bytes) TCP端口号码(2Bytes)
         rt_kprintf("\r\n		  主IP: %d.%d.%d.%d : %d \r\n",RemoteIP_main[0],RemoteIP_main[1],RemoteIP_main[2],RemoteIP_main[3],RemotePort_main);   
	  DataLink_MainSocket_set(RemoteIP_main, RemotePort_main,0);
	//  rt_kprintf("\r\n		   备用IP: %d.%d.%d.%d : %d \r\n",RemoteIP_aux[0],RemoteIP_aux[1],RemoteIP_aux[2],RemoteIP_aux[3],RemotePort_aux);   
         DataLink_AuxSocket_set(RemoteIP_aux, RemotePort_main,0);

	 // rt_kprintf("\r\n		   IC 卡IP: %d.%d.%d.%d  TCP: %d  UDP:%d\r\n",SysConf_struct.BD_IC_main_IP[0],SysConf_struct.BD_IC_main_IP[1],SysConf_struct.BD_IC_main_IP[2],SysConf_struct.BD_IC_main_IP[3],SysConf_struct.BD_IC_TCP_port,SysConf_struct.BD_IC_UDP_port);   
	 // rt_kprintf("\r\n		   IC 卡备用IP: %d.%d.%d.%d \r\n",SysConf_struct.BD_IC_Aux_IP[0],SysConf_struct.BD_IC_Aux_IP[1],SysConf_struct.BD_IC_Aux_IP[2],SysConf_struct.BD_IC_Aux_IP[3]);   
	  DataLink_IC_Socket_set(SysConf_struct.BD_IC_main_IP,SysConf_struct.BD_IC_TCP_port,0); 
	  	 
	  //  ACC On 上报间隔(2Bytes)  ACC Off 上报间隔(2Bytes)
	  rt_kprintf("\r\n		   ACC on 发送间隔为: %d S\r\n		   ACC Off 发送间隔为: %d S\r\n",ACC_on_sd_Duration,ACC_off_sd_Duration);
	  rt_kprintf("\r\n  播放状态: 0x%2X 最小休息时间: %d s   疲劳驾驶门限: %d s  \r\n",Warn_Play_controlBit,TiredConf_struct.TiredDoor.Door_MinSleepSec,TiredConf_struct.TiredDoor.Door_DrvKeepingSec);  		  
	  
          //  超长停车报警(2Bytes)	
	//  rt_kprintf("\r\n		   超长停车报警: %d S\r\n",StopLongDuration);
	   //  定距离回传使能标志位(1Bytes) ， 定距离上传距离(4Bytes)  单位: m 米  
         rt_kprintf("\r\n         监听号码: %s \r\n",JT808Conf_struct.LISTEN_Num);    


	   
          //   记录仪唯一ID
         rt_kprintf("\r\n		   车辆唯一性编号: %35s \r\n",JT808Conf_struct.DeviceOnlyID);    
         rt_kprintf("\r\n		   上报速度获取方式: %d :",JT808Conf_struct.Speed_GetType);	  
         if(JT808Conf_struct.Speed_GetType)							
                rt_kprintf("车速通过车辆速度传感器获取!\r\n");
         else
                rt_kprintf("车速通过GPS模块获取!\r\n"); 

         rt_kprintf("\r\n		   特征系数校准状态: %d :",JT808Conf_struct.DF_K_adjustState);	    
         if(JT808Conf_struct.DF_K_adjustState)					  	 
                  rt_kprintf(" 特征系数--校准成功!\r\n"); 
         else
                  rt_kprintf("特征系数--尚未校准!\r\n");   

	  //   里程
		  JT808Conf_struct.DayStartDistance_32=DayStartDistance_32;
		  JT808Conf_struct.Distance_m_u32=Distance_m_u32;		
         rt_kprintf("\r\n		   累计里程: %d  米   ,  当日里程:   %d米\r\n",JT808Conf_struct.Distance_m_u32,JT808Conf_struct.Distance_m_u32-JT808Conf_struct.DayStartDistance_32);  	
         //  速度限制
         rt_kprintf("		   允许最大速度: %d  Km/h    超速报警持续时间门限: %d  s \r\n", JT808Conf_struct.Speed_warn_MAX,JT808Conf_struct.Spd_Exd_LimitSeconds);  		 

     //    rt_kprintf("\r\n		  国家标准版本: %14s \r\n",JT808Conf_struct.StdVersion.stdverStr);					  
       //  rt_kprintf("\r\n		  国家标准版本修改单号: %d \r\n",JT808Conf_struct.StdVersion.MdfyID); 


         rt_kprintf("\r\n		  特征系数(速度脉冲系数): %d \r\n",JT808Conf_struct.Vech_Character_Value); 					  
         rt_kprintf("\r\n		  初次安装日期: %X-%X-%X %X:%X:%X \r\n",JT808Conf_struct.FirstSetupDate[0],JT808Conf_struct.FirstSetupDate[1],JT808Conf_struct.FirstSetupDate[2],JT808Conf_struct.FirstSetupDate[3],JT808Conf_struct.FirstSetupDate[4],JT808Conf_struct.FirstSetupDate[5]);  


         DriveCode32=(JT808Conf_struct.Driver_Info.Effective_Date[0]<<16)+(JT808Conf_struct.Driver_Info.Effective_Date[1]<<8)+JT808Conf_struct.Driver_Info.Effective_Date[2];
         rt_kprintf("\r\n		  驾驶员代码: %d \r\n",DriveCode32);  					  
         rt_kprintf("\r\n		  机动车驾驶证号: %18s \r\n",JT808Conf_struct.Driver_Info.DriverCard_ID);  					  
         rt_kprintf("\r\n		  驾驶员姓名: %s \r\n",JT808Conf_struct.Driver_Info.DriveName); 					  
         rt_kprintf("\r\n		  驾驶员从业资格证: %20s \r\n",JT808Conf_struct.Driver_Info.Drv_CareerID); 
         rt_kprintf("\r\n		  发证机构: %s \r\n",JT808Conf_struct.Driver_Info.Comfirm_agentID);   



         rt_kprintf("\r\n		  车辆VIN号: %17s \r\n",Vechicle_Info.Vech_VIN);   
         rt_kprintf("\r\n		  车牌号码: %12s \r\n",Vechicle_Info.Vech_Num);  
         rt_kprintf("\r\n		  车牌分类: %12s \r\n",Vechicle_Info.Vech_Type);  
         rt_kprintf("\r\n        车辆所在省ID: %d \r\n",Vechicle_Info.Dev_ProvinceID);
         rt_kprintf("\r\n        车辆所在市ID: %d \r\n",Vechicle_Info.Dev_CityID); 
         rt_kprintf("\r\n        车辆颜色:   JT415    1  蓝 2 黄 3 黑 4 白 9其他----当前颜色 %d \r\n",Vechicle_Info.Dev_Color);  


         rt_kprintf("\r\n        触发上报传感器为  TriggerSDsatus=%X    \r\n",TriggerSDsatus);   
         rt_kprintf("\r\n        Max_picNum =  %d   Max_CycleNum = %d   Max_DrvRcdNum=%d \r\n",Max_PicNum,Max_CycleNum,Max_RecoderNum); 

         rt_kprintf("\r\n\r\n	  车辆负载状态: ");
         switch(JT808Conf_struct.LOAD_STATE)
		{  
		case 1:
		   rt_kprintf("空车\r\n"); 
		  break;
		case 2:
		   rt_kprintf("半载\r\n"); 
		  break;		  
		case 3:
		   rt_kprintf("满载\r\n"); 
		  break;
		default:
		       JT808Conf_struct.LOAD_STATE=1;
		       Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
		   rt_kprintf("空车2\r\n");  
		  break;
		}
         rt_kprintf("\r\n\r\n  起始流水号: %d \r\n", JT808Conf_struct.Msg_Float_ID); 
	     rt_kprintf("\r\n\r\n             cyc_read:   %d ,     cyc_write :%d\r\n  \r\n",cycle_read,cycle_write);     		
         
         //=====================================================================
         //API_List_Directories();
         //-----------  北斗模块相关  ---------
	     BD_list(); 

		 // ----    首次连接类型  --------------------
		 if(Vechicle_Info.Link_Frist_Mode==1)  
		 	{
                 rt_kprintf("\r\n\r\n   首次连接模式:   MainLink");    
		 	}
		 else
		 	{
                  rt_kprintf("\r\n\r\n   首次连接模式:  DNSR域名");     
		 	}

		if(DeviceNumberID[0]==0xFF)
			rt_kprintf("\r\n  =======> 尚未设置设备编号，请重新设置\r\n" );  
		else
			{
			rt_kprintf("\r\n 读取终端ID为 : "); 
			for(i=0;i<12;i++)
				rt_kprintf("%c",DeviceNumberID[i]);
			rt_kprintf("\r\n");
			}  
       //---------- SIM ID -----------------------------  
        	if(SimID_12D[0]==0xFF)
			rt_kprintf("\r\n  =======> 尚未设置SIMID_入网编号，请重新设置\r\n" );  
		else
			{
			rt_kprintf("\r\n 读取设备SIMID 入网ID 为 : "); 
			for(i=0;i<12;i++)
				rt_kprintf("%c",SimID_12D[i]);
			rt_kprintf("\r\n");
			} 
	  	// 短息中心号码-----------------------------
	  	// rt_kprintf("\r\n		   短息中心号码 :%s	 \r\n",JT808Conf_struct.SMS_RXNum);

		 // ---  硬件版本信息-------------
		  HardWareVerion=HardWareGet();		 
		  rt_kprintf("\r\n		        -------硬件版本:%X        B : %d %d %d\r\n",HardWareVerion,(HardWareVerion>>2)&0x01,(HardWareVerion>>1)&0x01,(HardWareVerion&0x01));   
		  if(HardWareVerion==3) 
		  	  rt_kprintf("\r\n		       车台型号: TW705   \r\n"); 

          //------- 自动接听方式 -----------           
		  rt_kprintf("\r\n		        -------自动接听方式:%d    \r\n",JT808Conf_struct.Auto_ATA_flag);      

}
//FINSH_FUNCTION_EXPORT(DefaultConfig, DefaultConfig);     


void KorH_check(void)  // 客运货运显示状态查询
{
     // 类型判断  221.204.240.66
		   if(((Vechicle_Info.Link_Frist_Mode==1)&&(RemoteIP_main[0]==221)&&(RemoteIP_main[0]==204)&&(RemoteIP_main[0]==240)&&(RemoteIP_main[0]==66)))
		      Vechicle_Info.Vech_Type_Mark=1; 
		   else 
		   if((strcmp(DomainNameStr,"jt1.gghypt.net")==0)&&(Vechicle_Info.Link_Frist_Mode==0)) 
		      Vechicle_Info.Vech_Type_Mark=2; 
}


/*
读参数配置文件
*/
	void SysConfiguration(void) 
	{
			SetConfig();  
			ReadConfig();
			DefaultConfig();     
	}

void product_type(u8 *instr)
{
  u8 len=strlen(instr);

  if(len!=5)
   {
     rt_kprintf("\r\n len error\r\n");
     return;
  	}  

  if((strncmp(instr,"TW703",5)==0)||(strncmp(instr,"TW705",5)==0))
  {
	  memcpy(Vechicle_Info.ProType,instr,5);  
	   
	  
	  DF_WriteFlashSector(DF_Vehicle_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info));
	  WatchDog_Feed();
	  DF_WriteFlashSector(DF_VehicleBAK_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 
	  
	  WatchDog_Feed();
	  DF_WriteFlashSector(DF_VehicleBAK2_Struct_offset,0,(u8*)&Vechicle_Info,sizeof(Vechicle_Info)); 

	  rt_kprintf("\r\n 设置车台终端: %s\r\n",Vechicle_Info.ProType); 
	  
	  //--------	清除鉴权码 -------------------
		  idip("clear");		   
  }

}
FINSH_FUNCTION_EXPORT(product_type, type set);  

void  idip(u8 *str)
{
   u8 Reg_buf[25];
   
    if (strlen((const char*)str)==0){
	  	rt_kprintf("\r\n		   当前鉴权码为: ");
              rt_kprintf(" %s\r\n		   鉴权码长度: %d\r\n",JT808Conf_struct.ConfirmCode,strlen((const char*)JT808Conf_struct.ConfirmCode));   

		return ;
	}
   else
   	{	
   	    if(strncmp((const char*)str,"clear",5)==0)
   	    	{
                   JT808Conf_struct.Regsiter_Status=0; 
		           rt_kprintf("     手动清除 鉴权码 !\r\n");        	     
			       DEV_Login.Operate_enable=0; 
				   DEV_Login.Enable_sd=0; 
				   DEV_Login.Sd_counter=0;
   	    	}
		else
	      {
                  memset(JT808Conf_struct.ConfirmCode,0,sizeof(JT808Conf_struct.ConfirmCode));
                  memcpy(JT808Conf_struct.ConfirmCode,str,strlen((const char*)str));
		     JT808Conf_struct.Regsiter_Status=1; 
		    rt_kprintf("     手动设置  鉴权码: %s\r\n",JT808Conf_struct.ConfirmCode);   
	 
		}  
		    memset(Reg_buf,0,sizeof(Reg_buf));
		    memcpy(Reg_buf,JT808Conf_struct.ConfirmCode,20);
		    Reg_buf[20]=JT808Conf_struct.Regsiter_Status;		
                  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));   
      }
}
FINSH_FUNCTION_EXPORT(idip, id code set);



void deviceid(u8 *str)
{

	  u8 i=0,value=0;
	  u8 reg_str[20];
	 
	    memset(reg_str,0,sizeof(reg_str));
	     if (strlen((const char*)str)==0){
		   rt_kprintf("\r\n 终端ID为 : "); 
		  for(i=0;i<12;i++)
		  	rt_kprintf("%c",DeviceNumberID[i]); 
		  rt_kprintf("\r\n");
			return ;
		}
		else 
		{	   
          //---- check -------
          memcpy(reg_str,str,strlen((const char*)str));	
          if(strlen((const char*)reg_str)==12)  //  长度判断
          {
             for(i=0;i<12;i++)
             	{ 
             	   if(!((reg_str[i]>='0')&&(reg_str[i]<='9')))
				   {
				       value=1;
					   break;
             	   } 
             	}

			 if(value)
			 	{
			 	  rt_kprintf("\r\ndevice_ContentError\r\n");
			 	  rt_kprintf("\r\n 手动设置终端ID不合法!  \r\n");   
                  return ;
			 	} 

          }
		  else
		  	{
		  	    rt_kprintf("\r\ndevice_LenError\r\n");
		  	    rt_kprintf("\r\n 手动设置终端ID 长度不正确!  \r\n");   
                return ;
		  	}	
		 
		  memset(DeviceNumberID,0,sizeof(DeviceNumberID));
		  memcpy(DeviceNumberID,reg_str,12);	
		  DF_WriteFlashSector(DF_DeviceID_offset,0,DeviceNumberID,13); 
		  delay_ms(80);		  
		  DF_ReadFlash(DF_DeviceID_offset,0,DeviceNumberID,13);    
		  rt_kprintf("\r\ndevice_OK(");  
		  for(i=0;i<12;i++)
		  	rt_kprintf("%c",DeviceNumberID[i]);
		  rt_kprintf(")\r\n"); 
		  rt_kprintf("\r\n手动终端ID设置为 :%s\r\n",DeviceNumberID);
		  rt_kprintf("\r\n");
	      return ;
		}
}
FINSH_FUNCTION_EXPORT(deviceid, deviceid set); 



void simid(u8 *str)
{

	  u8 i=0,value=0;
	  u8 reg_str[20];
	 
	    memset(reg_str,0,sizeof(reg_str));
	     if (strlen((const char*)str)==0){
		   rt_kprintf("\r\n 入网 SIM_ID为 : ");  
		  for(i=0;i<12;i++)
		  	rt_kprintf("%c",SimID_12D[i]); 
		  rt_kprintf("\r\n");
			return ;
		}
		else 
		{	   
          //---- check -------
          memcpy(reg_str,str,strlen((const char*)str));	
          if(strlen((const char*)reg_str)==12)  //  长度判断
          {
             for(i=0;i<12;i++)
             	{ 
             	   if(!((reg_str[i]>='0')&&(reg_str[i]<='9')))
				   {
				       value=1;
					   break;
             	   } 
             	}

			 if(value)
			 	{
			 	  rt_kprintf("\r\n 手动设置入网SIM_ID不合法!  \r\n");   
                  return ;
			 	} 

          }
		  else
		  	{
		  	    rt_kprintf("\r\n 手动设置入网SIM_ID 长度不正确!  \r\n");   
                return ;
		  	}	
		 
		  memset(SimID_12D,0,sizeof(SimID_12D));
		  memcpy(SimID_12D,reg_str,12);								 
		  DF_WriteFlashSector(DF_SIMID_12D,0,SimID_12D,13); 
		  delay_ms(80); 		  
		  rt_kprintf("\r\n 手动设备入网Sim_ID设置为 : ");   
		  DF_ReadFlash(DF_SIMID_12D,0,SimID_12D,13);     
		  SIMID_Convert_SIMCODE();  // 转换 
		  for(i=0;i<12;i++)
		  	rt_kprintf("%c",SimID_12D[i]);
		  rt_kprintf("\r\n");
	         return ;
		}
}
FINSH_FUNCTION_EXPORT(simid, simid set); 













