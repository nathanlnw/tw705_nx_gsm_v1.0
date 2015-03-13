
//================================================================
/*         读写AT45DB16函数的头文件
MSP430 IAR application builder : 2007-04-15 9:00:00
Target : MSP430F149
Crystal: 3.6864Mhz
*/
//================================================================
//#include "common.h"
//#include "71x_type.h"

#ifndef _H_AT45
#define _H_AT45

#define    PageSIZE     512
#define  DFBakSize   150//50 

//================================================================
/*
  Flash Chip : SSST25VF032B-50-4I-S2AF
  ChipSize       : 4MBytes       PageSize(vitual): 512Bytes  SectorSize:4K<=>8 Pages    Chip: 1024Sectors<=>8192Pages

  Regulation :



<一>   系统参数 以及 应用参数  行车记录相关地址存储部分
*/


/*               Dataflash Page  规划   ------->    Start          */

/*  0. Page 0~9     Producet Info */
#define DF_ProInfo_Page      0

/*  1. page 10 -903	  ISP	*/
#define ISP_StartArea                                 0x1000        // 起始地址 
#define DF_APP1_PageNo		                          8             /*
start :   0x1000---- 和Boot 程序对应  8 page
size        60 sector     480 page

DF_APP_flah run PageNo:   50  ~ 903  page        */
/* 512K  -->1072 Page */


//   2.     config   information
#define    ConfigStart_offset                         808        //   Block   起始位置  Conifg  Struct Save      Sector 1 
#define    TiredCondifg_offset                        864        //   Block   起始位置  Conifg  Struct Save      Sector 3  
#define    JT808_BakSetting_offset                    960        //   Block   起始位?
#define    JT808_Bak2Setting_offset                   6240        //   Block   起始位?
#define    JT808Start_offset                          1000        //   Block   起始位置  Conifg  Struct Save    Sector 2 





/*  2. Page  904 - 912           状态信息    */


/*  3. Page  920 - 943        	GPS应用 最基本配置    */   //快速读写
#define  DF_socket_all                          1080     // Block起始    socket 1 ,2, 3 
#define  DF_APN_page			                 1088	 //
#define  DF_ACC_ON_dur_Page                     1092     // ACC 开时发送间隔     
#define  DF_ACC_OFF_dur_Page					 1093 	 // ACC 关时发送间隔 		   
#define  DF_TCP_sd_Dur_Page                     1094     // TCP 发送间隔
#define  DF_TrigSDStatus_Page                   1095     // 传感器触发上报状态
#define  DF_CycleAdd_Page                       1096     // Block 起始-- 记录循环存储读写偏移地址的page
#define  DF_PhotoAdd_Page                       1104     // Block 起始--记录照片存储读写偏移地址的page 


/*  4. 不同客户产品应用特有功能参数           */
#define  DF_DevConfirmCode_Page                 1112     // Block 起始-- 车辆伪IP
#define  DF_ListenNum_Page                      1113     // 中心监听号码    
#define  DF_Distance_Page                       1120     // Block 起始-- 车辆累计行驶里程
#define  DF_LoadState_Page                      1128     // Block 起始-- 车辆负载状态 
#define  DF_Speed_GetType_Page               1136     // Block 起始--存储速度获取方式 1为 速度传感器 0为GPS
#define  DF_K_adjust_Page                          1144     // Block 起始--存储标识是否特征系数已经被自动校准   1.校准过  0:尚未校准
#define  DF_ACCONFFcounter_Page              1152     // Block 起始--异常复位时存储ACCON_Off的计数数值
/*
               Byte1 Flag :   0 :停车  1:停车但没触发 2:触发了还没结束
               Byte2 TiredDrvStatus  Tired_drive.Tireddrv_status
               Byte3 On->off Flag
               Byte4~8: starttimeBCD
         */
#define  DF_OutGPS_Page                          1168    // Block 起始 -- 接外部GPS信号源状态标志   
#define  DF_BD_Extend_Page                    1176   //  北斗扩展

//该区域结束 至 1023  ( 共1024Page)


/*  5.行车记录仪相关参数  */
#define       DF_VehicleID_Page                        1192                           // Block 起始-车牌号码
#define       DF_VehicleType_Page                      1200                           // Block 起始-车辆类型
#define       DF_PropertiValue_Page                    1208                           // Block 起始-- 特征系数
#define       DF_DriverID_Page                         1216                           // Block 起始--驾驶员ID 及编码
#define       DF_ExpSpdAdd_Page                        1232                           // Block 起始--超速报警记录偏移地址
#define       DF_DayDistance_Page                      1288                           // Block 起始--当天里程  
#define       DF_Minpos_Page                           1328                           // Block 起始-每分钟位置存储
#define       DF_WARN_PLAY_Page                        1332                           // Block 起始--播报使能状态--2014  TW705 add 
/*
                连续驾驶时间、当天累计驾驶时间、最小休息时间、最长停车时间
               */
#define       DF_SDTime_Page                           1392                           // Block 起始-定时方式间隔                
#define       DF_SDDistance_Page                       1400                           // Block 起始-定距发送距离
#define       DF_SDMode_Page                           1408                           // Block 起始- 终端数据发送方式
#define       DF_RTLock_Page                           1416                           // Block 起始- 实时上报 --
#define       DF_Event_Page                            1424                           // Block 起始- 事件相关  
#define       DF_Msg_Page                              1432                           // Block 起始- 消息相关  
#define       DF_PhoneBook_Page                        1440                           // Block 起始- 电话本相关
#define       DF_CircleRail_Page                       1448                           // Block 起始- 圆形围栏

#define       DF_RectangleRail_Page                    4056                           // Block 起始- 矩形围栏  1288  --过检验就用7000 了 24个 

#define       DF_PolygenRail_Page                      1464                           // Block 起始- 多边形围栏
#define       DF_PicIndex_Page                         1480                           // Block 起始- 图像检索
#define       DF_SoundIndex_Page                       1488                           // Block 起始- 音频检索        
#define       DF_FlowNum_Page                          1496                           // Block 起始- 流水号

// 16  文本信息
#define       TextStart_offdet                         1504

//17.  域名
#define       DF_DomainName_Page                        1512


//该区域结束

/*
<二>   循环存储上报  行车记录仪相关功能 数据存储区
*/

/*  I.  Function App Area                  注: 以下Page 规划基于   W25Q64FVSSIP     用于临时Test      */

//                     Name                                     PageNum                	 	                     Description
//     2048*3  个 Page         8192+2048*3=14336          3*2048*4=24567   条记录
// 1.  Cycle Save Send Area
#define       CycleStart_offset                       8192  //1768                          // 循环存储上报存储区域(Basic 基本必备)        1 record=128 Bytes

// 5. Exp  Speed  Record
#define       ExpSpdStart_offset                      4016                          //  超速报警偏移 

// 6. Average 15 min  spd  recrod
#define      Avrg15minSpeedt_offset                     4032                         // 车辆每小时内每分钟位置记录   1 record =512 Bytes

// 14. Picture   Area
/*
                                 filename            cameraNum    size
                                    19                         1             4
                      */
#define       PicStart_offset                          4096                          // Block 起始位置 图片存储区域(Current Save) 将来要放到TF卡中
#define       PicStart_offset2                        4424                          // Block 起始位置 图片2区域 
#define       PicStart_offset3                        4752                          // Block 起始位置 图片3区域 
#define       PicStart_offset4                        5080                          // Block 起始位置 图片4区域  



// 15  Sound  Area
#define       SoundStart_offdet                      5248      //4200                 32K 空间        // Block 起始位置 15s声音存储区域(Current Save) 将来要放到TF卡中
/*
             filesize              filename
                4  Bytes          5thstart
*/
#define       SoundFileMax_Sectors                   5                              //  5 sect=5*8 pages =20s data



#define       DF_DeviceID_offset                      5400                 // Block 起始位置   车辆ID  12  位 BCD   
#define       DF_License_effect                       5416
#define       DF_Vehicle_Struct_offset                5424                 // block 起始位置   
#define       DF_VehicleBAK_Struct_offset             6200                 // block 起始位置   
#define       DF_VehicleBAK2_Struct_offset            6216                 // block 起始位置      

#define       DF_SIMID_12D                            6000                 // Block  	起始位置     
#define       DF_LOGIIN_Flag_offset                          6040                 // Block      起始位置 
#define       DF_LimitSPEED_offset                    6048                 // Block   起始位置    




//----  补充
#define    DF_Broadcast_offset                      5300       //  Block   起始位置  播报起始地址
#define    DF_Route_Page                               5400      // 1304                           // Block 起始- 路线
#define    DF_turnPoint_Page                         5500       //  拐点
#define   DF_AskQuestion_Page                    5600       //  中心提问    
/*                Dataflash     <------------   End              */



//  =================  行车记录仪 相关 ============================
/*
    StartPage :    6320          Start Address offset :   0x316000

    Area Size :
                          213   Sector       = 1704  pages
                           ----------------------

				扇区
				1                                      00-07H
				135                                   08H
				64                                     09H
				7                                      10H
				2                                      11H
				2                                      12H
				1                                      13H
				1                                      14H
				1                                      15H

          ----------  只是在这里做了---  注释 ，具体操作在 Vdr.C

*/



//-------------------------------------------------------


extern  u8   DF_initOver;    //     Dataflash  Lock


extern void DF_delay_us(u16 j);
extern void DF_delay_ms(u16 j);
extern void DF_ReadFlash(u16 page_counter, u16 page_offset, u8 *p, u16 length);
extern void DF_WriteFlash(u16 page_counter, u16 page_offset, u8 *p, u16 length);
extern void DF_ReadFlash(u16 page_counter, u16 page_offset, u8 *p, u16 length);
extern void DF_WriteFlashSector(u16 page_counter, u16 page_offset, u8 *p, u16 length); //512bytes 直接存储
extern void DF_WriteFlashDirect(u16 page_counter, u16 page_offset, u8 *p, u16 length);
extern void DF_Erase(void);
extern void DF_init(void);

#endif
