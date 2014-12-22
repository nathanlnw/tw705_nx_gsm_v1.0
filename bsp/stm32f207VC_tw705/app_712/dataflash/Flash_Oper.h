#ifndef Flash_Nand
#define  Flash_Nand
#include "App_moduleConfig.h"

//-----------------------------------------------------------------------------------------------------------------------------
//==============================================================================================================================
//   Add Type define
#define   Type_Idle                            0                     // 空闲   
#define   TYPE_CycleAdd                        1                     // 循环
#define   TYPE_PhotoAdd                        2                     // 图片 
#define   TYPE_ExpSpdAdd                       4                     // 超速报警记录偏移地址
#define   TYPE_15minSpd                        5                      // 停车前15分钟平均速度
#define   TYPE_DayDistancAdd                   6                    // 每天里程起始数目 
//-----------------------------------------------------------------------------------------------------------------------------

//---------  顺序读取发送 相关 define  -----------
#define   RdCycle_Idle                 0     // 空闲
#define   RdCycle_RdytoSD              1     // 准备发送
#define   RdCycle_SdOver               2     // 发送完毕等待中心应答


//--------   顺序读取发送相关  ------------
extern u8       ReadCycle_status;   
extern u8	    ReadCycle_timer;   // 超时判断 

extern u32    cycle_write, cycle_read,delta_0704_rd,mangQu_read_reg;  // 循环存储记录  delta 用于批量数据上传
#ifdef SPD_WARN_SAVE 
extern u32    ExpSpdRec_write, ExpSpdRec_read;  // 超速报警存储记录
#endif
extern u32    pic_current_page,pic_PageIn_offset,pic_size;       // 图片存储记录 
extern u32    Distance_m_u32;	 // 行车记录仪运行距离	  单位米
extern u32    DayStartDistance_32; //每天起始里程数目    



extern u8  SaveCycleGPS(u32 cycle_wr,u8 *content ,u16 saveLen);   
extern u8  ReadCycleGPS(u32 cycleread,u8 *content ,u16 ReadLen);    
#ifdef SPD_WARN_SAVE 
extern u8  Common_WriteContent(u32 In_write,u8 *content ,u16 saveLen, u8 Type); 
extern u8  Common_ReadContent(u32 In_read,u8 *content ,u16 ReadLen, u8 Type);    
#endif


#endif 

