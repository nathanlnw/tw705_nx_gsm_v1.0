/*
   vdr Vichle Driver Record 车辆行驶记录
 */

#include <rtthread.h>
#include <finsh.h>
#include "stm32f2xx.h"

#include <rtdevice.h>
#include <dfs_posix.h>

#include <time.h>

#include "SST25VF.h"
#include "Vdr.h"
#include "App_moduleConfig.h"


#define   MONI   //模拟产生DRV 数据，格式和正式的一样
 


//  =================  行车记录仪 相关 ============================
/*
    StartPage :    6320          Start Address offset :   0x316000       

    Area Size :
                          213  Sector       = 1704 pages
                           ----------------------
                           
				扇区               
				1                                     00-07H
				90                                    08H               
				85                                    09H
				7                                      10H
				2                                      11H 
				2                                      12H
				1                                      13H
				1                                      14H  
				1                                      15H   

          ----------  只是在这里做了---  注释 ，具体操作在 Vdr.C  				
*/

#define       SECTORSIZE                  4096
#define       VDR_START_ADDRESS           0x316000
#define       MarkByte                    0x0F          // 表示写入过相关信息

//-------------------------- Sectors-----------  total   195 +8 Sectors --------------
#define  VDR_07_SIZE          1
#define  VDR_08_SIZE         90    //   128 rec_size               1sector=32 recrods    2880/32=90
#define  VDR_09_SIZE         72    //  666=>>768 rec_size      1sector=5 recrods     360/5=72
#define  VDR_10_SIZE         7     //   234=>> 256 rec_size    1sector=16              100/16 =7
#define  VDR_11_SIZE         2     //   50 =>> 64                  1 sector=64             100/64 =2 
#define  VDR_12_SIZE         2     //    25 =>> 32                 1 sector=128            200/128=2
#define  VDR_13_SIZE         1     //     7==>>  8                 1 sector= 512             100/512=1
#define  VDR_14_SIZE         1     //     7==>>  8                 1 sector= 512             100/512=1
#define  VDR_15_SIZE         1     //  133==>256                  1 sector =16               10/16 =1


//----------------------absolutely  address   --------------  
#define VDR_00_to_07H    VDR_START_ADDRESS 

#define VDR_08H_START	 VDR_START_ADDRESS+VDR_07_SIZE*SECTORSIZE
#define VDR_08H_END		 VDR_START_ADDRESS+VDR_07_SIZE*SECTORSIZE-1

#define VDR_09H_START	 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE)*SECTORSIZE
#define VDR_09H_END		 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE)*SECTORSIZE-1


#define VDR_10H_START	 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE)*SECTORSIZE
#define VDR_10H_END		 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE)*SECTORSIZE-1

#define VDR_11H_START	 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE)*SECTORSIZE
#define VDR_11H_END		 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE)*SECTORSIZE-1

#define VDR_12H_START	 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE)*SECTORSIZE
#define VDR_12H_END		 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE)*SECTORSIZE-1

#define VDR_13H_START	 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE)*SECTORSIZE
#define VDR_13H_END		 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE)*SECTORSIZE-1

#define VDR_14H_START	 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE)*SECTORSIZE
#define VDR_14H_END		 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE)*SECTORSIZE-1

#define VDR_15H_START	 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE+VDR_14_SIZE)*SECTORSIZE
#define VDR_15H_END		 VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE+VDR_14_SIZE)*SECTORSIZE-1 

//  ----08~ 15   每个区域存储满了的状态记录  前8 个字节  need15  个sector
#define VDR_WRITEINDEX  VDR_START_ADDRESS+(VDR_07_SIZE+VDR_08_SIZE+VDR_09_SIZE+VDR_10_SIZE+VDR_11_SIZE+VDR_12_SIZE+VDR_13_SIZE+VDR_14_SIZE)*SECTORSIZE




//  -   no  use  actually----
#define VDR_08H_09H_START	 VDR_START_ADDRESS+VDR_07_SIZE*SECTORSIZE
#define VDR_08H_09H_END		 VDR_START_ADDRESS+VDR_07_SIZE*SECTORSIZE-1




/*
   实现每条记录128字节，便于定位。要做一定的处理。没有使用delta编码

   记录格式

   000--003  yy-mm-dd hh:mm   可以把时间转成UTC格式,便于比较查找
      又可以节省一个字节，无效的时间格式FFFFFFFFFF
   004--056  秒速度记录 速度要求+/- 1kmh
          原先60*8=480bit 现在用7bit保存需要 60*7=420bit=53byte
          7bit的 0b1111111 表示速度无效(gps未定位)
   057--116  状态信息
   117--126  单位分钟位置,参见行车记录仪 GBT19065 首个有效位置

 */


/*
   4MB serial flash 0x400000
 */

/*转换hex到bcd的编码*/
#define HEX_TO_BCD( A ) ( ( ( ( A ) / 10 ) << 4 ) | ( ( A ) % 10 ) )



/*基于小时的时间戳,主要是为了比较大小使用
   byte0 year
   byte1 month
   byte2 day
   byte3 hour
 */
typedef unsigned int YMDH_TIME;

typedef struct
{
	uint8_t cmd;

	uint32_t	ymdh_start;
	uint8_t		minute_start;

	uint32_t	ymdh_end;
	uint8_t		minute_end;

	uint32_t	ymdh_curr;
	uint8_t		minute_curr;
	uint32_t	addr;

	uint16_t	blocks;         /*定义每次上传多少个数据块*/
	uint16_t	blocks_remain;  /*当前组织上传包是还需要的的blocks*/
}VDR_CMD;

VDR_CMD		vdr_cmd;

uint8_t		vdr_tx_info[1024];
uint16_t	vtr_tx_len = 0;


/*传递写入文件的信息
   0...3  写入SerialFlash的地址
   4...31 文件名
 */

//=====================================================
VDR_INDEX         Vdr_Wr_Rd_Offset;      //  记录仪写索引 位置
VDR_DATA          VdrData;               //  行驶记录仪的databuf
VDR_TRIG_STATUS   VDR_TrigStatus;









//======================================================
/*
      遍历获取当前行驶记录的操作位置

      type :    0  代表遍历所有      08  09  10  11 12 13  14 
      u8    :     0   :遍历所有  
      
*/
u8  Vdr_PowerOn_getWriteIndex(u8 type)   
{
   
       u16    i;
	   u8	  c;  
      
	  //	 1.  get  address	


     switch(type)
     	{
		   case 0x08:
		   	           // 1.1  先读取第一个字符做判断
		   	           //rt_kprintf("\r\n 08H write=0x%X  \r\n",VDR_08H_START); 
		   	           for(i=0;i<VDR_08_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_08H_START+i*128);
						    if(c==0xFF)
						    {
                              break;
						    }
		   	           	}
					   // 1.2. 没有找到2880 那么 擦除第一个  从0 开始
					   if(i==VDR_08_MAXindex)
					   	{
						   Vdr_Wr_Rd_Offset.V_08H_Write=VDR_08_MAXindex;
						   Vdr_Wr_Rd_Offset.V_08H_full=1;// enable
					   	}							   	
		   	            if(GB19056.workstate==0)
					   rt_kprintf("\r\n 08H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_08H_Write); 
		   	           break;
		   case 0x09:
		   	           //  2.1 
		   	           // rt_kprintf("\r\n 09H write=0x%X  \r\n",VDR_09H_START); 
		   	          for(i=0;i<VDR_09_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_09H_START+i*768);
						    if(c==0xFF)
						    {
                              break;
						    }
		   	           	}
					   // 2. 没有找到360 那么 擦除第一个  从0 开始
					   if(i==VDR_09_MAXindex)
					   	{
						   Vdr_Wr_Rd_Offset.V_09H_Write=VDR_09_MAXindex;
						   Vdr_Wr_Rd_Offset.V_09H_full=1;						   
					   	} 
					    if(GB19056.workstate==0)
					   rt_kprintf("\r\n 09H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_09H_Write); 
					   break;
		   
		   case 0x10:
		   	          //  rt_kprintf("\r\n 10H write=0x%X  \r\n",VDR_10H_START);  
		   	           for(i=0;i<VDR_10_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_10H_START+i*256);
						    if(c==0xFF)
						    {						     
                              break;
						    }
		   	           	}
					   // 2. 没有找到100 那么 擦除第一个  从0 开始
					   if(i==VDR_10_MAXindex)
					   	{
						   Vdr_Wr_Rd_Offset.V_10H_Write=VDR_10_MAXindex; 
						   Vdr_Wr_Rd_Offset.V_10H_full=1;   
					   	}  
					    if(GB19056.workstate==0)
					   rt_kprintf("\r\n 10H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_10H_Write);	
					   break;
		   
		   case 0x11:
		   	            for(i=0;i<VDR_11_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_11H_START+i*64);
						    if(c==0xFF)
						    {
                              break;
						    }
		   	           	}
					   // 2. 没有找到100 那么 擦除第一个  从0 开始
					   if(i==VDR_11_MAXindex)
					   	{
						   Vdr_Wr_Rd_Offset.V_11H_Write=VDR_11_MAXindex; 
						   Vdr_Wr_Rd_Offset.V_11H_full=1;       
					   	}
					    if(GB19056.workstate==0)
					    rt_kprintf("\r\n 11H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_11H_Write);	 
					   break;

		   case 0x12: 
		   	            for(i=0;i<VDR_12_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_12H_START+i*32);
						    if(c==0xFF)
						    {
                              break;
						    }
		   	           	}
					    // 2. 没有找到200 那么 擦除第一个  从0 开始 
					    if(i==VDR_12_MAXindex)
					   	{
						   Vdr_Wr_Rd_Offset.V_12H_Write=VDR_12_MAXindex; 
						   Vdr_Wr_Rd_Offset.V_12H_full=1;           
					   	}  
						 if(GB19056.workstate==0)
						rt_kprintf("\r\n 12H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_12H_Write);   
						break;

		   case 0x13:   for(i=0;i<VDR_13_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_13H_START+i*8); 
						    if(c==0xFF)
						    {
                              break;
						    }
		   	           	}
					   // 2. 没有找到100 那么 擦除第一个  从0 开始
					   if(i==VDR_13_MAXindex)
					   	{
						   Vdr_Wr_Rd_Offset.V_13H_Write=VDR_13_MAXindex;    
						   Vdr_Wr_Rd_Offset.V_13H_full=1;  
					   	} 

					    if(GB19056.workstate==0)
					    rt_kprintf("\r\n 13H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_13H_Write);	  
						break;

		   case 0x14:   
		   	             for(i=0;i<VDR_14_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_14H_START+i*8); 
						    if(c==0xFF)
						    {
                              break;
						    }
		   	           	}
					   // 2. 没有找到100 那么 擦除第一个  从0 开始
					   if(i==VDR_14_MAXindex)
					   	{
						   Vdr_Wr_Rd_Offset.V_14H_Write=VDR_14_MAXindex;  
						   Vdr_Wr_Rd_Offset.V_14H_full=1;
					   	}  
					    if(GB19056.workstate==0)
					    rt_kprintf("\r\n 14H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_14H_Write);	  
						break;

		   case 0x15:
		   	            for(i=0;i<VDR_15_MAXindex;i++)
		   	           	{
                            c=SST25V_ByteRead(VDR_15H_START+i*256);  
						    if(c==0xFF)
						    {
                              break;
						    }
		   	           	}
					   // 2. 没有找到10 那么 擦除第一个  从0 开始
					   if(i==VDR_15_MAXindex)
					   	{
						   Vdr_Wr_Rd_Offset.V_15H_Write=VDR_15_MAXindex;   
						   Vdr_Wr_Rd_Offset.V_15H_full=1;
					   	}  
					    if(GB19056.workstate==0)
					   rt_kprintf("\r\n 15H write=%d  \r\n",Vdr_Wr_Rd_Offset.V_15H_Write);	  
						break;

     	}
  return true;
}


//    遍历所有行车记录写入的起始地址   from  08H
void total_ergotic(void)    
{
   
   Vdr_Wr_Rd_Offset.V_08H_Write=vdr_cmd_writeIndex_read(0x08);
   Vdr_Wr_Rd_Offset.V_09H_Write=vdr_cmd_writeIndex_read(0x09);
   Vdr_Wr_Rd_Offset.V_10H_Write=vdr_cmd_writeIndex_read(0x10);
   Vdr_Wr_Rd_Offset.V_11H_Write=vdr_cmd_writeIndex_read(0x11);
   Vdr_Wr_Rd_Offset.V_12H_Write=vdr_cmd_writeIndex_read(0x12);
   Vdr_Wr_Rd_Offset.V_13H_Write=vdr_cmd_writeIndex_read(0x13);
   Vdr_Wr_Rd_Offset.V_14H_Write=vdr_cmd_writeIndex_read(0x14);
   Vdr_Wr_Rd_Offset.V_15H_Write=vdr_cmd_writeIndex_read(0x15);
   
   
    Vdr_PowerOn_getWriteIndex(0x08);
    Vdr_PowerOn_getWriteIndex(0x09);
	Vdr_PowerOn_getWriteIndex(0x10);
    Vdr_PowerOn_getWriteIndex(0x11);
	Vdr_PowerOn_getWriteIndex(0x12);
    Vdr_PowerOn_getWriteIndex(0x13);
	Vdr_PowerOn_getWriteIndex(0x14); 
	Vdr_PowerOn_getWriteIndex(0x15);     

}
//FINSH_FUNCTION_EXPORT( total_ergotic, total_ergotic ); 

void index_write(u8 cmd,u32 value)
{
    vdr_cmd_writeIndex_save(cmd,value); 
	
   total_ergotic();

}
//FINSH_FUNCTION_EXPORT( index_write, index_write(cmd,value)); 

//   只能在初始化是用到
void vdr_erase(void) 
{
   u16  i=0;
    // start address        0x316000        start sector :790th        total: 195 sector 

    //  1. erase  to  839          840-790=50 
{
	//  32K
	 rt_kprintf("\r\n   <------------  vdr  area  erase  0 ------------------>\r\n");   
   for(i=0;i<10;i++)
   {
	 WatchDog_Feed();
	 SST25V_SectorErase_4KByte(VDR_START_ADDRESS+i*SECTORSIZE);  	 
	// rt_kprintf("\r\n  addr=0x%X \r\n",VDR_START_ADDRESS+i*SECTORSIZE);    
     delay_ms(150);     
	 
   }	 
   
   rt_kprintf("\r\n   <------------  vdr  area  erase  1 ------------------>\r\n");   
   
   #if 1
   // 195-50=145         145-64*2=17             0x320000
   for(i=0;i<22;i++) 
   {
	 WatchDog_Feed();
	// SST25V_BlockErase_64KByte(VDR_START_ADDRESS+0xA000+i*0x10000);   // 32k  <=> 0x8000   
      
	 SST25V_BlockErase_32KByte(VDR_START_ADDRESS+0xA000+i*0x8000);   // 32k  <=> 0x8000   8sector
     DF_delay_ms(700); 
	 WatchDog_Feed();
	 
	// rt_kprintf("\r\n  addr=0x%X \r\n",VDR_START_ADDRESS+0xA000+i*0x8000);   
	//  rt_kprintf("\r\n   <------------ erase 32K : %d\r\n",i+1);    
   }
  #endif
   Vdr_Wr_Rd_Offset.V_08H_Write=0;
   vdr_cmd_writeIndex_save(0x08,Vdr_Wr_Rd_Offset.V_08H_Write);
   Vdr_Wr_Rd_Offset.V_09H_Write=0;
   vdr_cmd_writeIndex_save(0x09,Vdr_Wr_Rd_Offset.V_09H_Write);
   	Vdr_Wr_Rd_Offset.V_10H_Write=0;
   vdr_cmd_writeIndex_save(0x10,Vdr_Wr_Rd_Offset.V_10H_Write);
    Vdr_Wr_Rd_Offset.V_11H_Write=0;
   vdr_cmd_writeIndex_save(0x11,Vdr_Wr_Rd_Offset.V_11H_Write); 
   	Vdr_Wr_Rd_Offset.V_12H_Write=0;
   vdr_cmd_writeIndex_save(0x12,Vdr_Wr_Rd_Offset.V_12H_Write);
   	Vdr_Wr_Rd_Offset.V_13H_Write=0;
   vdr_cmd_writeIndex_save(0x13,Vdr_Wr_Rd_Offset.V_13H_Write);
   	Vdr_Wr_Rd_Offset.V_14H_Write=0;
   vdr_cmd_writeIndex_save(0x14,Vdr_Wr_Rd_Offset.V_14H_Write);
   	Vdr_Wr_Rd_Offset.V_15H_Write=0;
   vdr_cmd_writeIndex_save(0x15,Vdr_Wr_Rd_Offset.V_15H_Write);
 }   
  rt_kprintf("\r\n   <------------  vdr  area  erase   over  ------------------>\r\n");  
}
FINSH_FUNCTION_EXPORT( vdr_erase, vdr_erase ); 

//    创建行驶记录仪数据  

void  VDR_product_08H_09H_10H(void)
{
  
 // u16 j=0,cur=0;
  u8 spd_reg=0,i=0,spd_usefull_counter=0,pos_V=0,avrg_spd=0;
  u8 reg8[6]; 

  //time_now=Get_RTC(); 	//	RTC  相关 
  //  行驶记录相关数据产生 触发,  且定位的情况下   
 // if(VDR_TrigStatus.Running_state_enable==1)  	    
  { 	//	  必须在行驶状态下
			
			//-------------------  08H	 --------------------------------------------------	 		
			// 1.	save	 Minute 		1-59   s		
			         spd_reg=Spd_Using%10;
					 if(spd_reg>=5)
					 	  spd_reg=Spd_Using/10+1;
					 else
					 	  spd_reg=Spd_Using/10; 
            //Vehicle_sensor=0xf0; //debug
			VdrData.H_08[6+2*VdrData.H_08_counter]=spd_reg; //km/h  
			VdrData.H_08[7+2*VdrData.H_08_counter]=Vehicle_sensor;   
			if(VdrData.H_08_counter==56)
				Time2BCD(VdrData.H_08);  
			
			if(VdrData.H_08_counter==59)   
			{  // 秒为0 
				// 0.  判断是否是第一次上电
			   //  1.  年月日 时分秒	 秒是0
	
				reg8[0]=((time_now.year/10)<<4)+(time_now.year%10);	    
				reg8[1]=((time_now.month/10)<<4)+(time_now.month%10); 
				reg8[2]=((time_now.day/10)<<4)+(time_now.day%10);
				reg8[3]=((time_now.hour/10)<<4)+(time_now.hour%10);
				reg8[4]=((time_now.min/10)<<4)+(time_now.min%10);
                 if((reg8[0]==VdrData.H_08[0])&&(reg8[1]==VdrData.H_08[1])&&(reg8[2]==VdrData.H_08[2])&&(reg8[3]==VdrData.H_08[3]))
				 	;
				 else
				 	{
				 	  
					  VdrData.H_08[0]=((time_now.year/10)<<4)+(time_now.year%10);	  
					  VdrData.H_08[1]=((time_now.month/10)<<4)+(time_now.month%10); 
					  VdrData.H_08[2]=((time_now.day/10)<<4)+(time_now.day%10);
					  VdrData.H_08[3]=((time_now.hour/10)<<4)+(time_now.hour%10);
					  VdrData.H_08[4]=((time_now.min/10)<<4)+(time_now.min%10);

				 	} 
				 	
				 
				 VdrData.H_08[5]=0; 	

				 
				memcpy(VdrData.H_08_BAK,VdrData.H_08,126); 

				//--- Check  save  or not 
				spd_usefull_counter=0;
				for(i=0;i<60;i++)
					{
					   if(VdrData.H_08[6+2*i])
					   	   spd_usefull_counter++;
					}
			   if(spd_usefull_counter)		// 有一个速度不为0  ，则存储
				   VdrData.H_08_saveFlag=1;  //	保存  当前分钟的数据记录 					
  
			   // 2.  initial 
			   memset(VdrData.H_08+6,0,120);    // 默认是 0x0  
			}  
			VdrData.H_08_counter++;
			if(VdrData.H_08_counter>=60)
				VdrData.H_08_counter=0;


  
		  //------------------------09H ---------  min=0;	sec=0	----------------------------
		     //  1 .	填写常规位置	1-59 min
		   if(VdrData.H_09_seconds_counter>=56)  // 59 秒时候做处理，如果是0 那么分钟就变了
		   {  
			  //   表 A.20		  (当前分钟的位置  +	当前分钟的平均速度) 
			  memcpy( VdrData.H_09+6+time_now.min*11,VdrData.Longi,4);  // 经度 
			  memcpy( VdrData.H_09+6+time_now.min*11+4,VdrData.Lati,4); //纬度
			  VdrData.H_09[6+time_now.min*11+8]=(GPS_Hight>>8);
			  VdrData.H_09[6+time_now.min*11+9]=GPS_Hight; 		   
			  //  当前分钟的平均速度	 从AvrgSpd_MintProcess()  引用的变量
			  VdrData.H_09[6+time_now.min*11+10]= VdrData.H_09_spd_accumlate/60;  		

              //  停车前15 分钟平均速度记录
              if(VdrData.H_09_seconds_counter==58) 
              {
				  avrg_spd=VdrData.H_09_spd_accumlate/58;
				  if(avrg_spd>5)
				      Avrg15_min_generate(VdrData.H_09_spd_accumlate/58); 
				  else
				  	{
					  	  if(Avrg_15minSpd.savefull_state==1)
					  	  	{
					  	  	   Avrg15_min_save();							   
					  	  	}
					  	  Avrg_15minSpd.Ram_counter=0;
					      Avrg_15minSpd.savefull_state=0;
				  	}
              }
		  
		   }
		     //   提前几秒存否则就跨小时了
		   if((time_now.min==59)&&(VdrData.H_09_seconds_counter==57)) 
		   {			   
			   //  2.  年月日 时分秒	 |	 分  秒是0
				 VdrData.H_09[0]=((time_now.year/10)<<4)+(time_now.year%10);	 	   
				 VdrData.H_09[1]=((time_now.month/10)<<4)+(time_now.month%10);  
				 VdrData.H_09[2]=((time_now.day/10)<<4)+(time_now.day%10);
				 VdrData.H_09[3]=((time_now.hour/10)<<4)+(time_now.hour%10);
				 VdrData.H_09[4]=0;
				 VdrData.H_09[5]=0;
				 //  最后一分钟的判断         速度已经
                // VdrData.H_09[6+time_now.min*11+10]=Spd_Using/10;//VdrData.H_09_spd_accumlate/(VdrData.H_09_seconds_counter+1);	
				 // 1.	判断是否是需要存储
                 spd_usefull_counter=0;
				 pos_V=0;
				for(i=0;i<60;i++)
				{
				  if(VdrData.H_09[6+i*11+10])  // 有速度累加
				  	  spd_usefull_counter++;
				  if((VdrData.H_09[6+i*11]==0x7F)&&(VdrData.H_09[6+i*11+4]==0x7F))
				  	  pos_V++;   // 位置无效就累加
				}

                 //  除了  60 分钟速度都为0  且 都没有定位外其他都保存
				 if(!((spd_usefull_counter==0)&&(pos_V==60)))
			        VdrData.H_09_saveFlag=1;	//	 保存  当前分钟的数据记录   
		   }  

            //---------09  tick --------------
            VdrData.H_09_seconds_counter++;
			if(VdrData.H_09_seconds_counter>=60)
			 {	
			     VdrData.H_09_seconds_counter=0;
				 VdrData.H_09_spd_accumlate=0;
			 }
             VdrData.H_09_spd_accumlate+=Spd_Using/10; 

		   // 每个小时process
		    if((time_now.min==0)&&(time_now.sec==2)) 
		   {   //   新一小时的数据需要初始化一下
                 for(i=0;i<60;i++)
				  {
				         VdrData.H_09[6+i*11]=0x7F;
						 VdrData.H_09[7+i*11]=0xFF;
						 VdrData.H_09[8+i*11]=0xFF;
						 VdrData.H_09[9+i*11]=0xFF;
						 VdrData.H_09[10+i*11]=0x7F;
						 VdrData.H_09[11+i*11]=0xFF;
						 VdrData.H_09[12+i*11]=0xFF;
						 VdrData.H_09[13+i*11]=0xFF;
						 VdrData.H_09[14+i*11]=0x7F;
						 VdrData.H_09[15+i*11]=0xFF;
						 VdrData.H_09[16+i*11]=0x00;
				  } 
		   }

		 memcpy(XinhaoStatusBAK,XinhaoStatus,20);
  
   }
  
}


void VDR_product_11H_Start(void)
{
           //    11 H   连续驾驶开始时间   和起始位置     从60s 开始算驾驶起始
                   	{ 
                   	    //  1.   机动车驾驶证号
                   	    memcpy(VdrData.H_11,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
					    //   2.   起始时间
						time_now=Get_RTC();     //  RTC  相关 
			            Time2BCD(VdrData.H_11+18); 
                        //   3.  起始位置                        
						memcpy( VdrData.H_11+30,VdrData.Longi,4);  // 经度
						memcpy( VdrData.H_11+30+4,VdrData.Lati,4); //纬度
						VdrData.H_11[30+8]=(GPS_Hight>>8);
						VdrData.H_11[30+9]=GPS_Hight;	
						VdrData.H_11_start=1; //  start  
						VdrData.H_11_lastSave=0;

                   	}
}

void VDR_product_11H_End(u8 value)
{
    #if 0
         if((TiredConf_struct.TiredDoor.Door_DrvKeepingSec<=0) ||(VdrData.H_11_start==0))
		 	 return;
                 //         11 H     相关
			
                    //   2.   结束时间 
                      /*
                                             如果是之前没有存储过lastsave==0，那么需要RTC 时间
                                             如果 存储数值为2 那么需要RTC 
                                             如果存储过，且不为2  ，那么不用替换RTC ，存储并累加索引
                                        */
                    if((VdrData.H_11_lastSave==0)||(value==2))
                    {
						time_now=Get_RTC();     //  RTC  相关 
			             Time2BCD((VdrData.H_11+24)); 
                    }
                        //   3.  起始位置                        
						memcpy( VdrData.H_11+40,VdrData.Longi,4);  // 经度
						memcpy( VdrData.H_11+40+4,VdrData.Lati,4); //纬度
						VdrData.H_11[40+8]=(GPS_Hight>>8);
						VdrData.H_11[40+9]=GPS_Hight;	
		#endif				
                    //    4.   save 
                    VdrData.H_11_saveFlag=value;   // 1: 累加  2: 不累加
        #if 1            
                    //     5. clear  state
					if(value==1)
					{
						VdrData.H_11_lastSave=0; // clear  						
						//VdrData.H_11_start=0;
					}	 
					else
					if(value==2)
						VdrData.H_11_lastSave=1; // 已经存储过
	   #endif				
}




void VDR_product_12H(u8  value)    // 插拔卡记录
{
     //   1. 时间发生的时间
		time_now=Get_RTC();     //  RTC  相关 
        Time2BCD(VdrData.H_12); 

     //  2.   机动车驾驶证号
         memcpy(VdrData.H_12+6,Read_ICinfo_Reg.DriverCard_ID,18); 
     //  3.  结果 
         VdrData.H_12[24]=value;

	 // 4.  save 
	     VdrData.H_12_saveFlag=1; 
}


//  Note :  定义了但是没有用，不带电池情况下，断电操作SPI DF  危害会很大
void VDR_product_13H(u8  value)    //  外部供电记录   01  通电    02  断电
{
       //   1. 时间发生的时间
		time_now=Get_RTC();     //  RTC  相关 
        Time2BCD(VdrData.H_13); 
       //  2.  value
         VdrData.H_13[6]=value;
       //  3. save
       VdrData.H_13_saveFlag=1; 
}


void VDR_product_14H(u8 cmdID)    // 记录仪参数修改记录
{

    //   1. 时间发生的时间
		time_now=Get_RTC();     //  RTC  相关 
        Time2BCD(VdrData.H_14); 
       //  2.  value
         VdrData.H_14[6]=cmdID;
       //  3. save
       VdrData.H_14_saveFlag=1;
}


 
// note  :    需要索引检索   08 H 的数据记录 ，这里不需要填充。
void VDR_product_15H(u8  value)    //  采集指定速度记录 
{  // 速度异常 提示时间    5  min   
    




}

void  VDR_get_15H_StartEnd_Time(u8  *Start, u8 *End)  
{
    u8  i=0;

	for(i=0;i<6;i++)
	{
      VdrData.H_15_Set_StartDateTime[i]=Start[i];
	  VdrData.H_15_Set_EndDateTime[i]=End[i];
	}
	
}

//-------------模拟测试记录仪数据----------------------------------------
#ifdef MONI
void  moni_drv(u8 CMD,u16 delta)
{
  
  u16 j=0,cur=0,i=0,icounter=0;	 
  
  //  行驶记录相关数据产生 触发,  且定位的情况下   
 switch(CMD) 
 {
   #if 0
   case  0x08:
			
			//-------------------  08H	 --------------------------------------------------
	 
			   //  1.  年月日 时分秒	 秒是0
				 VdrData.H_08[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);	   
				 VdrData.H_08[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10); 
				 VdrData.H_08[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);
				 VdrData.H_08[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);
				 VdrData.H_08[4]=((Temp_Gps_Gprs.Time[1]/10)<<4)+(Temp_Gps_Gprs.Time[1]%10);
				 VdrData.H_08[5]=0; 	


		       for(i=0;i<60;i++)
		       {
					 // 3.	 save	  Minute		 1-59	s		  
					 VdrData.H_08[6+2*i]=20; //km/h
					 VdrData.H_08[7+2*i]=23;  
		       }
			   
			    // last
			   memcpy(VdrData.H_08_BAK,VdrData.H_08,126);
			   VdrData.H_08_saveFlag=1;  //   保存	当前分钟的数据记录		
			   OutPrint_HEX("08 ",VdrData.H_08_BAK,126);
        break;
		case 0x09:
		  //------------------------09H ---------  min=0;	sec=0	----------------------------
		        // 起始时间
          	     VdrData.H_09[0]=((Temp_Gps_Gprs.Date[0]/10)<<4)+(Temp_Gps_Gprs.Date[0]%10);	   
				 VdrData.H_09[1]=((Temp_Gps_Gprs.Date[1]/10)<<4)+(Temp_Gps_Gprs.Date[1]%10); 
				 VdrData.H_09[2]=((Temp_Gps_Gprs.Date[2]/10)<<4)+(Temp_Gps_Gprs.Date[2]%10);
				 VdrData.H_09[3]=((Temp_Gps_Gprs.Time[0]/10)<<4)+(Temp_Gps_Gprs.Time[0]%10);
				 VdrData.H_09[4]=0;
				 VdrData.H_09[5]=0;

				//
				for(i=0;i<60;i++)
				{
					   //  4 .	填写常规位置	1-59 min
					   {  
						  //   表 A.20		  (当前分钟的位置  +	当前分钟的平均速度) 
						  memcpy( VdrData.H_09+6+i*11,VdrData.Longi,4);  // 经度
						  memcpy( VdrData.H_09+6+i*11+4,VdrData.Lati,4); //纬度
						  VdrData.H_09[6+i*11+8]=(GPS_Hight>>8);
						  VdrData.H_09[6+i*11+9]=GPS_Hight; 		   
						  //  当前分钟的平均速度	 从AvrgSpd_MintProcess()  引用的变量
						  VdrData.H_09[6+i*11+10]= Spd_Using;  		  
					   }
				}
  		  

				// 1.  判断是否是需要存储
				 VdrData.H_09_saveFlag=1;  //   保存  当前分钟的数据记录     
  
                   OutPrint_HEX("09 ",VdrData.H_09,666);
		break;
	#endif	
	case 0x10:	
		 //-------------------- 10H    事故疑点数据  ---------------------------------
		      if(delta!=10)
		      	{
					  //  1.  行驶结束时间
					 time_now=Get_RTC();	 //  RTC  相关 
					 Time2BCD(VdrData.H_10);   
		      	}
			 //   2.   机动车驾驶证号码
			 memcpy(VdrData.H_10+6,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
			 //   3.  速度和状态信息   
					  //-----------------------  Status Register   --------------------------------
					cur=save_sensorCounter;  //20s的事故疑点 
					//------------------------------------------
					if(cur>0)	  // 从当前往前读取
						   cur--; 
						else
						   cur=200; 	 	
					//-----------------------------------------	
					icounter=100+delta;
					for(j=0;j<icounter;j++) 
					{
					   if(j>=delta)
					   {
						VdrData.H_10[24+(j-delta)*2]=Sensor_buf[cur].DOUBTspeed;	//速度
						//    rt_kprintf("%d-",Sensor_buf[cur].DOUBTspeed);  
						VdrData.H_10[24+(j-delta)*2+1]=Sensor_buf[cur].DOUBTstatus;   //状态  	   
					   }
						
						if(cur>0)	  // 从当前往前读取
						   cur--; 
						else
						   cur=200; 	 					   
					}  
			 // 	4.	位置信息
			  memcpy( VdrData.H_10+224,VdrData.Longi,4);  // 经度
			  memcpy( VdrData.H_10+224+4,VdrData.Lati,4); //纬度
			  VdrData.H_10[224+8]=(GPS_Hight>>8);
					VdrData.H_10[224+9]=GPS_Hight;	
  
			 //--------------------------------------------------------------------------			  
			 VdrData.H_10_saveFlag=1;
			// if(GB19056.workstate==0)
			  // OutPrint_HEX("10 ",VdrData.H_10,234); 
     break;
#if 1			
   case 0x11:
                //  1.   机动车驾驶证号
                
                       if( VdrData.H_12[24]==0x01)	 //已登录
				         memcpy(VdrData.H_11,Read_ICinfo_Reg.DriverCard_ID,18);
                       else
                   	    memcpy(VdrData.H_11,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
					    //   2.   起始时间
						time_now=Get_RTC();     //  RTC  相关 
			            Time2BCD(VdrData.H_11+18); 
                        //   3.  起始位置                        
						memcpy( VdrData.H_11+30,VdrData.Longi,4);  // 经度
						memcpy( VdrData.H_11+30+4,VdrData.Lati,4); //纬度
						VdrData.H_11[30+8]=(GPS_Hight>>8);
						VdrData.H_11[30+9]=GPS_Hight;	
						 //   2.   结束时间
						time_now=Get_RTC();     //  RTC  相关 
			            Time2BCD((VdrData.H_11+24)); 
                        //   3.  起始位置                        
						memcpy( VdrData.H_11+40,VdrData.Longi,4);  // 经度
						memcpy( VdrData.H_11+40+4,VdrData.Lati,4); //纬度
						VdrData.H_11[40+8]=(GPS_Hight>>8);
						VdrData.H_11[40+9]=GPS_Hight;	
                    //    4.   save 
                    VdrData.H_11_saveFlag=1; 
					  OutPrint_HEX("11 ",VdrData.H_11,50);
          break;
   case 0x12:    // 插卡拔卡时间
				     //   1. 时间发生的时间
						time_now=Get_RTC();     //  RTC  相关 
				        Time2BCD(VdrData.H_12); 

				     //  2.   机动车驾驶证号
				         memcpy(VdrData.H_12+6,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
				     //  3.  结果 
				         VdrData.H_12[24]=((rt_tick_get()%2)+1);   
					 // 4.  save 
					     VdrData.H_12_saveFlag=1; 
					   OutPrint_HEX("12 ",VdrData.H_12,25);
		  break;
   case 0x13:   //  充放电
                         //   1. 时间发生的时间
							time_now=Get_RTC();     //  RTC  相关 
					        Time2BCD(VdrData.H_13); 
					       //  2.  value
					         VdrData.H_13[6]=((rt_tick_get()%2)+1);
					       //  3. save
					       VdrData.H_13_saveFlag=1; 
						    OutPrint_HEX("13 ",VdrData.H_13,7); 
          break;
   case 0x14:              // 记录仪参数修改
                         //   1. 时间发生的时间
							time_now=Get_RTC();     //  RTC  相关 
					        Time2BCD(VdrData.H_14); 
					       //  2.  value
					         VdrData.H_14[6]=((rt_tick_get()%10));     
					       //  3. save
					       VdrData.H_14_saveFlag=1;
						     OutPrint_HEX("14 ",VdrData.H_14,7);
          break;
   case 0x15:            //  速度状态日志
                        //   1. 时间发生的时间
                            VdrData.H_15[0]=(rt_tick_get()%2)+1; // 状态      
							time_now=Get_RTC();     // 起始时间
					        Time2BCD(VdrData.H_15+1); 
							time_now=Get_RTC();     //  结束时间
					        Time2BCD(VdrData.H_15+7); 
					       //  2.  value
					        for(i=0;i<60;i++)
					        	{
					        	  VdrData.H_15[13+2*i]=Spd_Using/10;
								  VdrData.H_15[14+2*i]=40;
					        	}
					       //  3. save
					       VdrData.H_15_saveFlag=1;
						   OutPrint_HEX("15 ",VdrData.H_15,133); 
          break;
#endif 		  
      default:
	  	         //rt_kprintf("\r\n  CMD =%d   not  support ",CMD); 
		       break;		  
  	
  
 	}
}
//FINSH_FUNCTION_EXPORT(moni_drv, moni_drv(u8 CMD,u16 delta));  

  void  drv_query(void) 
  	{
       rt_kprintf("\r\n       08   max=%d  index=%d    full=%d",VDR_08_MAXindex,Vdr_Wr_Rd_Offset.V_08H_Write,Vdr_Wr_Rd_Offset.V_08H_full); 
	   rt_kprintf("\r\n       09   max=%d  index=%d    full=%d",VDR_09_MAXindex,Vdr_Wr_Rd_Offset.V_09H_Write,Vdr_Wr_Rd_Offset.V_09H_full);  
	   rt_kprintf("\r\n       10   max=%d  index=%d    full=%d",VDR_10_MAXindex,Vdr_Wr_Rd_Offset.V_10H_Write,Vdr_Wr_Rd_Offset.V_10H_full); 
	   rt_kprintf("\r\n       11   max=%d  index=%d    full=%d",VDR_11_MAXindex,Vdr_Wr_Rd_Offset.V_11H_Write,Vdr_Wr_Rd_Offset.V_11H_full); 
	   rt_kprintf("\r\n       12   max=%d  index=%d    full=%d",VDR_12_MAXindex,Vdr_Wr_Rd_Offset.V_12H_Write,Vdr_Wr_Rd_Offset.V_12H_full); 
	   rt_kprintf("\r\n       13   max=%d  index=%d    full=%d",VDR_13_MAXindex,Vdr_Wr_Rd_Offset.V_13H_Write,Vdr_Wr_Rd_Offset.V_13H_full); 
	   rt_kprintf("\r\n       14   max=%d  index=%d    full=%d",VDR_14_MAXindex,Vdr_Wr_Rd_Offset.V_14H_Write,Vdr_Wr_Rd_Offset.V_14H_full); 
	   rt_kprintf("\r\n       15   max=%d  index=%d    full=%d",VDR_15_MAXindex,Vdr_Wr_Rd_Offset.V_15H_Write,Vdr_Wr_Rd_Offset.V_15H_full);   

  	}
  FINSH_FUNCTION_EXPORT(drv_query, drv_query ); 	







#endif
//---------------------------------------------------------------------------------

//------- stuff     user data       ---------------
u16  stuff_drvData(u8 type,u16 Start_recNum,u16 REC_nums,u8 *dest)
{
    u16  res_len=0,get_len=0;
    u16  i=0;
	u16  get_indexnum=0;

   DF_TAKE;
   if(Start_recNum>=1)  //没存满
				get_indexnum=Start_recNum-1-i;
   
	for(i=0;i<REC_nums;i++)
	{
	  if(get_indexnum>=(Start_recNum-1-i))
	      get_indexnum=Start_recNum-1-i;	  
	  else
	  	  continue;
	  
	  switch(type)
	  	{
	  	   case 0x08:
                     get_len=get_08h(get_indexnum,dest+res_len);
					 break;
		   case 0x09:
                     get_len=get_09h(get_indexnum,dest+res_len);    
					 break;
		   case 0x10:
                     get_len=get_10h(get_indexnum,dest+res_len);
					 break;
		   case 0x11:
                     get_len=get_11h(get_indexnum,dest+res_len);
					 break;
		   case 0x12:
                     get_len=get_12h(get_indexnum,dest+res_len);
					 break;	
		   case 0x13:
                     get_len=get_13h(get_indexnum,dest+res_len);
					 break;		
		   case 0x14:
                     get_len=get_14h(get_indexnum,dest+res_len);
					 break;		
		   case 0x15:
                     get_len=get_15h(get_indexnum,dest+res_len); 
					 break;	
		   default:
		   	         DF_RELEASE;
		   	         return 0;

	  	}
	   
	   res_len+=get_len;
	}  
	DF_RELEASE;
    return   res_len;
}



/*
   行驶速度记录
   不管他，自己填数据，自己报
   单条126字节每分钟
   48*60=2880包

    每一条   128 字节

 */

u16 get_08h( u16 indexnum,u8 *p)    
{
	int			i;
	uint8_t		buf[128];
    u32  addr=0;
	u8  FCS=0;


    
	//	  1.  get  address	 
	 addr=VDR_08H_START+indexnum*128;

   //    2. read   
      SST25V_BufferRead(buf, addr, 128 );
      delay_ms(1);
      // OutPrint_HEX("08H content",buf,128); // debug    
   //     3.  FCS  check
           FCS=0;
       for(i=0;i<127;i++)
	 	   FCS^=buf[i];
	   if(buf[127]!=FCS)
	   	{
	   	    // rt_kprintf("\r\n  08H read fcs error  save:%X  cacu: %X \r\n",buf[127],FCS);
             return  0;
	   	}
     if(buf[0]==MarkByte)
      {  
         memcpy(p,buf+1,126);
	     return  126;
      }	
	 else 
	 	 return 0; 
}



//FINSH_FUNCTION_EXPORT( get_08h, get_08 );


/*
   位置信息
   360小时   ，每小时666字节
 */

u16  get_09h( u16 indexnum,u8 *p)  
{
	 int		 i;
	 uint8_t	 buf[669];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_09H_START+indexnum*768;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 669 ); //2+content 
	   delay_ms(10);
	  // OutPrint_HEX("09H content",buf,669); // debug 
	//	   3.  FCS	check
	          FCS=0;
		for(i=0;i<667;i++)
			FCS^=buf[i];
		if(buf[667]!=FCS)
		 {
			  //rt_kprintf("\r\n	09H read fcs error	save:%X  cacu: %X \r\n",buf[667],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,666); 
		  return  666;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_09h, get_09 );


/*
   事故疑点
   234Byte
   共100个
 */
u16 get_10h( u16 indexnum,u8 *p)  
{
	 int		 i;
	 uint8_t	 buf[237];
	 u32  addr=0;
	 u8   FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_10H_START+indexnum*256;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 236 );	
	   delay_ms(4); 
	   //OutPrint_HEX("10H content",buf,236); // debug 
	//	   3.  FCS	check
	         FCS=0;
		for(i=0;i<235;i++)
			FCS^=buf[i];
		if(buf[235]!=FCS)
		 {
			  //rt_kprintf("\r\n	10H read fcs error	save:%X  cacu: %X \r\n",buf[235],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,234); 
		  return  234;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_10h, get_10 );


/*超时驾驶记录
   50Bytes   100条

      每个记录   64   个字节
 */

u16 get_11h( u16 indexnum,u8 *p)   
{
	
	 int		 i;
	 uint8_t	 buf[60];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_11H_START+indexnum*64;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 52 );	
	   delay_ms(3);
	   //OutPrint_HEX("11H content",buf,52); // debug  
	//	   3.  FCS	check
	        FCS=0;
		for(i=0;i<51;i++)
			FCS^=buf[i];
		if(buf[51]!=FCS)
		 {
			  //rt_kprintf("\r\n	11H read fcs error	save:%X  cacu: %X \r\n",buf[51],FCS);
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,50); 
		  return  50;
	   } 
	  else 
		  return 0; 

	
}

//FINSH_FUNCTION_EXPORT( get_11h, get_11 );


/*驾驶员身份登录
   25Bytes   200条
 */

u16 get_12h( u16 indexnum,u8 *p)  
{
     int		 i;
	 uint8_t	 buf[30];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_12H_START+indexnum*32;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 27 ); 
	   delay_ms(4);
	  // OutPrint_HEX("12H content",buf,27); // debug 
	//	   3.  FCS	check
	       FCS=0;
		for(i=0;i<26;i++)
			FCS^=buf[i];
		if(buf[26]!=FCS)
		 {
			  //rt_kprintf("\r\n	11H read fcs error	save:%X  cacu: %X \r\n",buf[26],FCS); 
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,25); 
		  return  25;
	   } 
	  else 
		  return 0; 
}

//FINSH_FUNCTION_EXPORT( get_12h, get_12 );


/*外部供电记录
   7字节 100条**/
u16 get_13h( u16 indexnum,u8 *p)   
{
	 uint8_t	 buf[10];
	 u32  addr=0;

	
	 
	 //    1.  get	address   
	  addr=VDR_13H_START+indexnum*8;	
	//	  2.  read	
	   SST25V_BufferRead(buf, addr, 8 );	
	  // OutPrint_HEX("13H content",buf,8); // debug  
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,7); 
		  return  7;
	   } 
	  else 
		  return 0;  

}
//FINSH_FUNCTION_EXPORT( get_13h, get_13 );


/*记录仪参数
   7字节 100条    8 */
u16 get_14h( u16 indexnum,u8 *p)  
{
	 uint8_t	 buf[10];
	 u32  addr=0;
	
	 
	 //    1.  get	address   
	  addr=VDR_14H_START+indexnum*8;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 8 );	
	  // OutPrint_HEX("14H content",buf,8); // debug 
	  if(buf[0]==MarkByte)   
	   {  
		  memcpy(p,buf+1,7); 
		  return  7;
	   } 
	  else 
		  return 0; 

}

//FINSH_FUNCTION_EXPORT( get_14h, get_14 );


/*
   速度状态日志
   133Byte    256
   共10个
 */
u16 get_15h( u16 indexnum,u8 *p)  
{
	
     int		 i;
	 uint8_t	 buf[150];
	 u32  addr=0;
	 u8  FCS=0;
	
	
	 
	 //    1.  get	address   
	  addr=VDR_15H_START+indexnum*256;
	
	//	  2. read	
	   SST25V_BufferRead(buf, addr, 135 );
	   //OutPrint_HEX("15H content",buf,135); // debug 
	//	   3.  FCS	check
		for(i=0;i<134;i++)
			FCS^=buf[i];
		if(buf[134]!=FCS)
		 {
			  //rt_kprintf("\r\n	15H read fcs error	save:%X  cacu: %X \r\n",buf[134],FCS); 
			  return  0;
		 }
	  if(buf[0]==MarkByte)
	   {  
		  memcpy(p,buf+1,133); 
		  return  133;
	   } 
	  else 
		  return 0;

}

//FINSH_FUNCTION_EXPORT( get_15h, get_15 );







//==========  write   ------------------------------------------------------------------------------
u16  vdr_creat_08h( u16 indexnum,u8 *p, u16 inLen) 
{
    u8  inbuf[128];
	u8  FCS=0,i=0;
	u32 inaddress=0;//  写入地址

    //  1.  Stuff  Head 
	inbuf[0]=MarkByte;           //
    //  2.   Index  get  address  
    if(indexnum>=VDR_08_MAXindex)
    	{
             Vdr_Wr_Rd_Offset.V_08H_Write=0; 
			 indexnum=0;
    	}
	memcpy(inbuf+1,p,inLen);   //  126 bytes
   //   3.   caculate  fcs
     for(i=0;i<127;i++)
	 	FCS^=inbuf[i];
	 inbuf[127]=FCS;   
   //    4.  get  address 	
	inaddress=VDR_08H_START+indexnum*128;

	//  判断擦除相关区域
	DF_TAKE;
	/*if( ( inaddress & 0x0FFF ) == 0 )
			{
			    WatchDog_Feed();
				SST25V_SectorErase_4KByte( inaddress );
				delay_ms(100);
			}
   //     5 .  write
     WatchDog_Feed(); 
    SST25V_BufferWrite(inbuf, inaddress, 128 );  
    delay_ms(50);*/
	SST25V_OneSector_Write(inbuf, inaddress, 128 );
	WatchDog_Feed(); 
    DF_RELEASE;
	return  indexnum;
}


//	666=>>768 rec_size   
u16  vdr_creat_09h( u16 indexnum,u8 *p, u16 inLen) 
{
    u8  inbuf[669];     
	u8  FCS=0;
	u16 i=0;
	u32 inaddress=0;//  写入地址
   
    //  1.  Stuff  Head 
	inbuf[0]=MarkByte;           //
    //  2.   Index  get  address  
    if(indexnum>=VDR_09_MAXindex)
    	{
             Vdr_Wr_Rd_Offset.V_09H_Write=0; 
			 indexnum=0;
    	}
	memcpy(inbuf+1,p,inLen);   // 666 bytes   
   //   3.   caculate  fcs
       for(i=0;i<667;i++)
	    	FCS^=inbuf[i];
	   
	   inbuf[667]=FCS;   
   //    4.  get  address 	
	inaddress=VDR_09H_START+indexnum*768;

	//  判断擦除相关区域
	DF_TAKE;
/*	if( ( inaddress&0x0FFF)==0 )
			{
			    WatchDog_Feed();
				SST25V_SectorErase_4KByte( inaddress );   
				delay_ms(100);
			}
   //     5 .  write
    WatchDog_Feed(); 
    SST25V_BufferWrite(inbuf, inaddress, 668 ); // 2+content
    delay_ms(80); 
    */
	SST25V_OneSector_Write(inbuf, inaddress, 668 ); // 2+content
		WatchDog_Feed(); 
   DF_RELEASE;
     return  indexnum; 

} 

/*
   事故疑点 Accident Point    234
*/ 
u16  vdr_creat_10h( u16 indexnum,u8 *p, u16 inLen) 
{
   
      u8  inbuf[256];
	  u8  FCS=0,i=0;
	  u32 inaddress=0;//  写入地址
	 
	  //  1.  Stuff  Head 
	  inbuf[0]=MarkByte	;	   //
	  //  2.   Index  get  address	
	  if(indexnum>=VDR_10_MAXindex)
		  {
			   Vdr_Wr_Rd_Offset.V_10H_Write=0; 
			   indexnum=0;
		  }
	  memcpy(inbuf+1,p,inLen);	 // 666 bytes
	 //   3.   caculate  fcs
	   for(i=0;i<235;i++)
		  FCS^=inbuf[i];
	   inbuf[235]=FCS;	 
	 //    4.  get	address   
	  inaddress=VDR_10H_START+indexnum*256;
   
	  //  判断擦除相关区域
	  DF_TAKE;
	 /* if( ( inaddress & 0x0FFF ) == 0 )
			  {
				  WatchDog_Feed();
				  SST25V_SectorErase_4KByte( inaddress );
				  delay_ms(100);
			  }
	 // 	5 .  write
	  WatchDog_Feed(); 
	  SST25V_BufferWrite(inbuf, inaddress, 236 ); // 2+content 
	  delay_ms(10);*/
	  SST25V_OneSector_Write(inbuf, inaddress, 236 ); // 2+content
		WatchDog_Feed(); 
      DF_RELEASE;
	   return  indexnum;

}

/***********************************************************
* Function:
* Description:       recordsize 5==>> 64 size 
* Input:             100   条
* Input:
* Output:
* Return:
* Others:
***********************************************************/

u16  vdr_creat_11h( u16 indexnum,u8 *p, u16 inLen) 
{

   
	u8	inbuf[60];
	u8	FCS=0,i=0;
	u32 inaddress=0;//	写入地址
   
	//	1.	Stuff  Head 
	inbuf[0]=MarkByte;		 //
	//	2.	 Index	get  address  
	if(indexnum>=VDR_11_MAXindex)
		{
			 Vdr_Wr_Rd_Offset.V_11H_Write=0; 
			 indexnum=0;
		}
	memcpy(inbuf+1,p,inLen);   // 666 bytes
   //	3.	 caculate  fcs
	 for(i=0;i<51;i++)
		FCS^=inbuf[i];
	 inbuf[51]=FCS;   
   //	 4.  get  address	
	inaddress=VDR_11H_START+indexnum*64;
   
   //	  5 .  write   
   DF_TAKE;
	SST25V_OneSector_Write(inbuf, inaddress, 52 ); // 2+content     读擦写  
     WatchDog_Feed(); 
   DF_RELEASE;	 
	 return  indexnum;

}
/*
   外部供电记录
   都存在4k的记录里，1次读出

      25 =>>   32
 */

u16  vdr_creat_12h( u16 indexnum,u8 *p, u16 inLen) 
{
  
   u8  inbuf[30];
   u8  FCS=0,i=0;
   u32 inaddress=0;//  写入地址
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>=VDR_12_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_12H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
  //   3.	caculate  fcs
	for(i=0;i<26;i++)
	   FCS^=inbuf[i];
	inbuf[26]=FCS;	 
  //	4.	get  address   
   inaddress=VDR_12H_START+indexnum*32; 
  
  //	 5 .  write   
  DF_TAKE;
   SST25V_OneSector_Write(inbuf, inaddress, 27 ); // 2+content	   读擦写   
    WatchDog_Feed(); 
  DF_RELEASE;	
   return	indexnum;

}


/*都存在4k的记录里，1次读出
     7 bytes    >> 8       record size     

*/
u16  vdr_creat_13h( u16 indexnum,u8 *p, u16 inLen) 
{
   u8  inbuf[10];
   u32 inaddress=0;//  写入地址
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>=VDR_13_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_13H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
  //	4.	get  address   
   inaddress=VDR_13H_START+indexnum*8; 
  
  //	 5 .  write   
  DF_TAKE;
   SST25V_OneSector_Write(inbuf, inaddress, 8 ); // 2+content	   读擦写    
   WatchDog_Feed();  
  DF_RELEASE; 
   return	indexnum;
    

}

/*都存在4k的记录里，1次读出
     7 bytes    >> 8 	  record size 

*/
u16  vdr_creat_14h( u16 indexnum,u8 *p, u16 inLen) 
{
   u8  inbuf[10];
  // u8  FCS=0,i=0;
   u32 inaddress=0;//  写入地址
  
   //  1.  Stuff  Head 
   inbuf[0]=MarkByte;		//
   //  2.	Index  get	address  
   if(indexnum>=VDR_14_MAXindex)
	   {
			Vdr_Wr_Rd_Offset.V_14H_Write=0; 
			indexnum=0;
	   }
   memcpy(inbuf+1,p,inLen);   // 666 bytes
  //	4.	get  address   
   inaddress=VDR_14H_START+indexnum*8; 
  
  //	 5 .  write   
  DF_TAKE;
   SST25V_OneSector_Write(inbuf, inaddress, 8 ); // 2+content	   读擦写   
   WatchDog_Feed(); 
  DF_RELEASE; 
   return	indexnum;
  
}

/*都存在4k的记录里，1次读出
       133   >>  256 
*/
u16  vdr_creat_15h( u16 indexnum,u8 *p, u16 inLen) 
{
	
	 u8  inbuf[260];
	 u8  FCS=0,i=0;
	 u32 inaddress=0;//  写入地址
	
	 //  1.  Stuff	Head 
	 inbuf[0]=MarkByte; 	  //
	 //  2.   Index  get  address  
	 if(indexnum>=VDR_15_MAXindex)
		 {
			  Vdr_Wr_Rd_Offset.V_15H_Write=0; 
			  indexnum=0;
		 }
	 memcpy(inbuf+1,p,inLen);	// 666 bytes
	//	 3.   caculate	fcs
	  for(i=0;i<134;i++)
		 FCS^=inbuf[i];
	  inbuf[134]=FCS;   
	//	  4.  get  address	 
	 inaddress=VDR_15H_START+indexnum*256; 
	
	//	   5 .	write	
	DF_TAKE;
	 SST25V_OneSector_Write(inbuf, inaddress, 135); // 2+content	 读擦写    
	 WatchDog_Feed(); 
	 DF_RELEASE;
	 return   indexnum;  

}

/*
   存储08-15  每种类型数据存储到什么位置了 
*/
u8  vdr_cmd_writeIndex_save(u8 CMDType,u32 value) 
{
  u8     head[448];
  u16    offset=0;  
  u32    Addr_base=0; 
  u32    Save_Addr=0;     
 // u16    InpageOffset=0;  // 页内偏移 
  u8     reg[9];  
  u8     Flag_used=0x01;


    DF_TAKE;
  //  1.   Classify
    switch(CMDType)
    {
		case 0x08:
							 Addr_base=VDR_WRITEINDEX;
							 break;
		case 0x09:
							 Addr_base=VDR_WRITEINDEX+SECTORSIZE;
							 break;
		case 0x10:
							 Addr_base=VDR_WRITEINDEX+2*SECTORSIZE;
							 break;
		case 0x11:
							 Addr_base=VDR_WRITEINDEX+3*SECTORSIZE;
							 break; 					 
		case 0x12:
							 Addr_base=VDR_WRITEINDEX+4*SECTORSIZE;
							 break;
		case 0x13:
							 Addr_base=VDR_WRITEINDEX+5*SECTORSIZE;
							 break;
		case 0x14:
							 Addr_base=VDR_WRITEINDEX+6*SECTORSIZE;
							 break;
		case 0x15:
                             Addr_base=VDR_WRITEINDEX+7*SECTORSIZE;
			                 break;
		default :			                 
							 DF_RELEASE;
							 return false;							 
    }
  //  2 .  Excute     
    SST25V_BufferRead(head,Addr_base,448);   //读取前448 个字节	
    DF_delay_ms(5);
	/*
	     448 字节(占512个字节)，是状态表示是否被写过。
	     后3584 是内容 addr=base+512+n*8
	*/ 		
     for(offset=0;offset<448;offset++)
     {
       if(0xFF==head[offset])
	   	  break;
     }

	 if(offset==448)
	 	{     
		   SST25V_SectorErase_4KByte(Addr_base); // Erase block
		   offset=0; 		   
		   DF_delay_ms(50);
	    }   
	 Save_Addr=Addr_base+512+(offset<<3);  

     memset(reg,0xff,sizeof(reg));  
      // 只用4个字节
	 reg[0]=(value>>24);
	 reg[1]=(value>>16);
	 reg[2]=(value>>8);
	 reg[3]=(value);
	 
	 SST25V_BufferWrite(&Flag_used,Addr_base+offset,1); //  更新状态位
	 DF_delay_us(150); 
	 SST25V_BufferWrite(reg,Save_Addr,4);  
	 DF_delay_us(50);
	 
	 DF_RELEASE;
	 return true;
   //                 The  End     	     
}


u32 vdr_cmd_writeIndex_read(u8 CMDType) 
{
	u8	   head[448];
	u16    offset=0;  
	u32    Addr_base=0; 
	u32    Read_Addr=0; 
	u32    value=0;
	u8     reg[9]; 
	

   //Wr_Address, Rd_Address  , 没有什么用只是表示输入，便于写观察而已    
  //  1.   Classify
  
   //  1.	Classify
	 switch(CMDType)
	 {
		 case 0x08:
							  Addr_base=VDR_WRITEINDEX;
							  break;
		 case 0x09:
							  Addr_base=VDR_WRITEINDEX+SECTORSIZE;
							  break;
		 case 0x10:
							  Addr_base=VDR_WRITEINDEX+2*SECTORSIZE;
							  break;
		 case 0x11:
							  Addr_base=VDR_WRITEINDEX+3*SECTORSIZE;
							  break;					  
		 case 0x12:
							  Addr_base=VDR_WRITEINDEX+4*SECTORSIZE;
							  break;
		 case 0x13:
							  Addr_base=VDR_WRITEINDEX+5*SECTORSIZE;
							  break;
		 case 0x14:
							  Addr_base=VDR_WRITEINDEX+6*SECTORSIZE;
							  break;
		 case 0x15:
							  Addr_base=VDR_WRITEINDEX+7*SECTORSIZE;
							  break;
		 default :
							  return false; 						  
	 }
   //  2 .	Excute	   
	 SST25V_BufferRead(head,Addr_base,448);   //读取前448 个字节 	 
	 DF_delay_ms(5);
	 /*
		  448 字节(占512个字节)，是状态表示是否被写过。
		  后3584 是内容 addr=base+512+n*8
	 */ 	 
   
		for(offset=0;offset<448;offset++)
		{
		  if(0xFF==head[offset])
			 break;
		}
		offset--;  // 第一个不会为0xFF
        Read_Addr=Addr_base+512+(offset<<3);  	 
	    SST25V_BufferRead(reg,Read_Addr,4); 
		delay_us(3);
		value=(reg[0]<<24)+(reg[1]<<16)+(reg[2]<<8)+reg[3]; 
	    return value; 
}

//---------------------------------------------------------------------------------

/************************************** The End Of File **************************************/
