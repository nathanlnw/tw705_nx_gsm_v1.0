#include "App_moduleConfig.h"




//--------   顺序读取发送相关  ------------
u8    ReadCycle_status=RdCycle_Idle;  
u8    ReadCycle_timer=0;   // 超时判断



u32     cycle_write=0, cycle_read=0,delta_0704_rd=0,mangQu_read_reg=0;  // 循环存储记录
#ifdef SPD_WARN_SAVE 
u32    ExpSpdRec_write=0, ExpSpdRec_read=0;  // 超速报警存储记录
#endif
u32    pic_current_page=0,pic_PageIn_offset=0,pic_size=0;;       // 图片存储记录 
u32   	Distance_m_u32=0;	 // 行车记录仪运行距离	  单位米
u32     DayStartDistance_32=0; //每天起始里程数目


//-----------------------------------------------------------------------------------------------------------------------------
u8 SaveCycleGPS(u32 cyclewr,u8 *content ,u16 saveLen) 
{
  /*
         //old  NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
         NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 128 Bytes ;  1page= 4Records   1Sector=8Page=32Records
  */
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
//	u8   reg[1]={0};
	u8   rd_back[128];
	u16  i=0,retry=0;

  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page 
     pageoffset=(u32)(cycle_write>>2);                // 计算出 Page 偏移  除以4 (每个page能放4条记录)
     InPageoffset=cycle_write-(u32)(pageoffset<<2);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<7);           // 计算出页内 字节   乘以 128 (每个记录128个字节)
     if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Sector  被移除到下一个Sector  1Sector=8Page  
     {
        WatchDog_Feed();
		SST25V_SectorErase_4KByte((pageoffset+CycleStart_offset)*PageSIZE);      // erase Sector 	 	
		DF_delay_ms(60); 
		if(GB19056.workstate==0)
	    rt_kprintf("\r\n Erase Cycle Sector : %d\r\n",(pageoffset>>3));       
	 }
  //	   2. write  and read back    
  SV_RTRY:
      if(retry>=2)
      	{
	  	  return false;
      	}
	   delay_ms(5);
	   WatchDog_Feed(); 
	   DF_WriteFlashDirect(pageoffset+CycleStart_offset,InPageAddr,content,saveLen);  //   写入信息
	   DF_delay_us(500);     
       DF_ReadFlash(pageoffset+CycleStart_offset,InPageAddr,rd_back,saveLen);  //   读取信息
  //  compare 
       for(i=0;i<saveLen;i++)
       	{
             if(content[i]!=rd_back[i])
			 {
			     cycle_write++;		  
		         if(cycle_write>=Max_CycleNum)
			       cycle_write=0; 
				 if(retry==0)
				 {
				     retry++;
			         goto SV_RTRY;
				 }
				 else
				 {
				   //---------------------------
				  PositionSD_Enable(); 
				  Current_State=1; // 使能即时上报 		
				  Current_UDP_sd=1;
				  if(GB19056.workstate==0)
				     rt_kprintf("\r\n wrte error-> current\r\n");
				    return false;
				 }
             }
       	}
		return true;  
  //-------------------------------------------------------------------------------------------- 
}  


u8 ReadCycleGPS(u32 cycleread,u8 *content ,u16 ReadLen)
{
  /*
     NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
  */
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8  i=0,FCS=0;
	u8  Len_read=0;  // 信息长度

  /*
      上报的每一包数第一个字节是有效信息的长度，从第二个字节是信息内容，
      信息内容的后边是一个字节额校验(校验从长度开始到内容最后一个字节)
  */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
     pageoffset=(u32)(cycle_read>>2);                 // 计算出 Page 偏移  除以4 (每个page能放4条记录)
     InPageoffset=cycle_read-(u32)(pageoffset<<2);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<7);            // 计算出页内 字节   乘以 128 (每个记录128个字节)
  //   2. Write Record Content 
     DF_TAKE;
     DF_ReadFlash(pageoffset+CycleStart_offset,InPageAddr,content,ReadLen); 
     DF_delay_ms(10); 
	 DF_RELEASE;
	 //  获取信息长度
	 Len_read=content[0];
	 
  if(DispContent==2)
  {
   	 OutPrint_HEX("读取CycleGPS 内容为 ",content,Len_read+1); 
  }	 
  //  3. Judge FCS	
	//--------------- 过滤已经发送过的信息 ------- 
	  FCS = 0;
	   for ( i = 0; i < Len_read; i++ )   //计算读取信息的异或和
	   {
			   FCS ^= *( content + i );  
	   } 
	  if(((content[Len_read]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))  // 判断异或和   
	    { 	      
		  if(content[0]==0xFF)
		  {
			// rt_kprintf("\r\n  content[0]==0xFF   read=%d,  write=%d  \r\n",cycle_read,cycle_write);   
                       
			  cycle_read++;	
		        if(cycle_read>=Max_CycleNum)
		  	      cycle_read=0; 
			 ReadCycle_status=RdCycle_Idle;  
			 return false;	
		  }  
		  //------------------------------------------------
          cycle_read++;	
		  if(cycle_read>=Max_CycleNum)
		  	cycle_read=0;
		  ReadCycle_status=RdCycle_Idle; 
		  return false; 
     	}	  
	//------------------------------------------------------------------------------------------   	 
	    return true;     
  //-------------------------------------------------------------------------------------------- 
} 

#ifdef SPD_WARN_SAVE 

//-----------------------------------------------------------------------------------------------------------
u8  Common_WriteContent(u32 In_write,u8 *content ,u16 saveLen, u8 Type) 
{
  //-----------------------------------------------------
  u8     reg[1];
  //u8   regStr[25];   
  //-----------------------------------------------------
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u32  Start_offset=0; 

   //--------------------------------------------------	
    //memset(regStr,0,sizeof(regStr)); 
  //  1.   Classify
    switch(Type)
    {
	
		case TYPE_ExpSpdAdd:
							 Start_offset=ExpSpdStart_offset;
							// memcpy(regStr,"超速报警",25);
							 break; 					 
		default :
							 return false;		  					 
    }
	//----------------------------------------------------------------------------------------------
	//	 2. caculate address 
	 
		pageoffset=(u32)(In_write>>4);				 // 计算出 Page 偏移  除以16  
		InPageoffset=In_write-(u32)(pageoffset<<4);	 // 计算出 页内偏移地址 
		InPageAddr=(u16)(InPageoffset<<5);			 // 计算出页内 字节   乘以 32 (每个记录32个字节)
		if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Block  被移除到下一个Block	1Block=8Page  
		{
        SST25V_SectorErase_4KByte((pageoffset+Start_offset)*PageSIZE);      // erase Sector	
        DF_delay_ms(70);
		}
	 // 	  2. Filter write  area    
		DF_ReadFlash(pageoffset+Start_offset,InPageAddr,reg,1); 
		  if(reg[0]!=0xff)	// 如果要写入的区域 dirty  ，则地址增然后从新开始寻找知道找到为止
		   {
			 In_write++;
			 if(In_write>=Max_CommonNum)
				  In_write=0;  
				 return false;
		   }	   
	 //   3. Write Record Content 
        DF_WriteFlashDirect(pageoffset+Start_offset,InPageAddr,content,saveLen);  //   写入信息
		DF_delay_ms(10);

     //   4. end  
        switch(Type) 
    {
	
		case TYPE_ExpSpdAdd:
							 ExpSpdRec_write=In_write;
							 break; 					 
		default :
							 return false;	 						 
    }

	return true; 
	     	     
}


u8  Common_ReadContent(u32 In_read,u8 *content ,u16 ReadLen, u8 Type) 
{
  //-----------------------------------------------------
  //u8    regStr[25];   
  //-----------------------------------------------------
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u32  Start_offset=0;     
	u8   i=0,FCS=0;;
   //--------------------------------------------------	
 
	// memset(regStr,0,sizeof(regStr)); 
   //  1.	Classify
	 switch(Type)
	 {
	 
		 case TYPE_ExpSpdAdd:
							  Start_offset=ExpSpdStart_offset;
							//  memcpy(regStr,"超速报警",25);
							  break;					  
		 default :
							  return false; 						  
	 } 
 
	//----------------------------------------------------------------------------------------------
	//	 2. caculate address 
	 
		   pageoffset=(u32)(In_read>>4);				  // 计算出 Page 偏移  除以64  
		   InPageoffset=In_read-(u32)(pageoffset<<4);   // 计算出 页内偏移地址 
		   InPageAddr=(u16)(InPageoffset<<5);			 // 计算出页内 字节   乘以 32 (每个记录32个字节)
		  //	 2. Write Record Content 
		   DF_ReadFlash(pageoffset+Start_offset,InPageAddr,content,ReadLen);    
		   DF_delay_us(10);
		 if(DispContent)
		 {
			   OutPrint_HEX("读取Common 内容为 ",content,ReadLen);
		 }   
		//	3. Judge FCS  
		  //--------------- 过滤已经发送过的信息 -------
			FCS = 0;
			 for ( i = 0; i < ReadLen-1; i++ )	 //计算读取信息的异或和
			 {
					 FCS ^= *( content + i );  
			 }				
			 if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))  // 判断异或和 
			  { 
			   
			   if(content[0]==0xFF)
			   {
				  //如果是内容是0xFF ，读指针和写指针相等，不再触发上报。	
				         switch(Type)
					    {
						
							case TYPE_ExpSpdAdd:
												 ExpSpdRec_read=ExpSpdRec_write;
												 break; 					 
							default :
												 return false;							 
					    
				         	}	
						return false; 
			   }
				In_read++; 
				if(In_read>=Max_CommonNum)
				  In_read=0;
				//rt_kprintf("\r\n   record info error  \r\n");  
				return false;
			  }
			 if(content[0]==0xFF)
			 {
              // rt_kprintf("\r\n  read  0xFF \r\n");
			        switch(Type)
					    {
						
							case TYPE_ExpSpdAdd:
												 ExpSpdRec_read=ExpSpdRec_write;
												 break; 					 
							default :
												 return false;							 
					    
				         	}	
               return false;
			 } 
			 
			 	
     //   4. end  
        switch(Type)
    {
	
		case TYPE_ExpSpdAdd:
							 ExpSpdRec_read=In_read;
							 break; 					 
		default :
							 return false;							 
    }

	return true; 
	     	     
}
#endif
//----------------------------------------------------------------------  

