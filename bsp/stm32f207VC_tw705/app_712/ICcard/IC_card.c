/*
       IC  card
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
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include "App_moduleConfig.h"
#include "Vdr.h"
#include"IS2401.h"


u8   IC_loginState = 0; //  IC  插入且正确识别:   1    否则   0
unsigned char IC_CardInsert = 0; //1:IC卡插入正确  2:IC卡插入错误
unsigned char IC_Check_Count = 0;
unsigned int      read_counter = 0, flag_8024off = 1;
unsigned char   Init8024Flag = 0;
unsigned int      DelayCheckIc = 0;
unsigned char administrator_card = 0;
u8        powerOn_first = 0; //    首次上电后不判断拔卡

// -----IC  卡信息相关---
DRV_INFO    Read_ICinfo_Reg;   // 临时读取的IC 卡信息
DRIVE_STRUCT     Drivers_struct[MAX_DriverIC_num]; // 预留5 个驾驶员的插卡对比




void KeyBuzzer(unsigned char num)
{

    if(num == 1)
    {
        BuzzerFlag++;
        if(BuzzerFlag == 2)
            buzzer_onoff(1);
        if(BuzzerFlag == 4)
        {
            buzzer_onoff(0);
            BuzzerFlag = 0;
            IC_CardInsert = 0;
        }

    }
    else if(num == 2)
    {
        BuzzerFlag++;
        if((BuzzerFlag == 12) || (BuzzerFlag == 16))
            buzzer_onoff(1);
        if((BuzzerFlag == 14) || (BuzzerFlag == 18))
            buzzer_onoff(0);
        if(BuzzerFlag == 18)
        {
            BuzzerFlag = 0;
            IC_CardInsert = 0;
        }
    }
}

void IC_info_default(void)
{
    memcpy(Read_ICinfo_Reg.DriverCard_ID, "000000000000000000", 18);
    memset(Read_ICinfo_Reg.Effective_Date, 0, sizeof(Read_ICinfo_Reg.Effective_Date));
    Read_ICinfo_Reg.Effective_Date[0] = 0x20;
    Read_ICinfo_Reg.Effective_Date[1] = 0x08;
    Read_ICinfo_Reg.Effective_Date[2] = 0x08;
    memcpy(Read_ICinfo_Reg.Drv_CareerID, "000000000000000000", 18);
}
//   判断多个驾驶员，插卡后信息的更新
void  Different_DriverIC_InfoUpdate(void)
{
    u8  i = 0, j = 0, compare = 0; // compare 是否有匹配的
    s8  cmp_res = 0; // 匹配为0
    u8  selected_num = 0; // 表示第几个被选择了

    //  0 .    先遍历是否和既有的匹配
    for(i = 0; i < MAX_DriverIC_num; i++)
    {
        if(Drivers_struct[i].Working_state)
        {
            // check compare
            cmp_res = memcmp(Drivers_struct[i].Driver_BASEinfo.DriverCard_ID, Read_ICinfo_Reg.DriverCard_ID, 18);
            if(cmp_res == 0) // 匹配
            {

                Drivers_struct[i].Working_state = 2;
                selected_num = MAX_DriverIC_num + 2; //   表完全匹配
                compare = 1; //enable
                if(GB19056.workstate == 0)
                    rt_kprintf("\r\n  有完全匹配的 i=%d\r\n", i);
            }
            else
            {
                if(Drivers_struct[i].Working_state == 2)
                {
                    Drivers_struct[i].Working_state = 1;
                    selected_num = i; //  记录以前为2 的 下标
                    if(GB19056.workstate == 0)
                        rt_kprintf("\r\n  上次插过卡 2bian1  i=%d\r\n", i);
                }
            }
        }
    }

    if(compare)         //  找到匹配的了，且更新状态了
        return;
    //--------------------------------------------------------------
    // 1. 遍历驾驶员状态，找出当前要用的位置
    for(i = 0; i < MAX_DriverIC_num; i++)
    {
        // 如果有当前的，那么把它当前状态转换成激活状态

        //    状态为2 的下一个即为 当前位置
        if((Drivers_struct[i].Working_state == 1) && (selected_num == 0x0F))
        {
            selected_num = i;
            break;
        }
        if(Drivers_struct[i].Working_state == 1)
        {
            if(selected_num == i) // 判断当前记录是否是上次为2 的记录
            {
                Drivers_struct[i].Working_state = 1;
                selected_num = 0x0F;
            }
        }
        if(Drivers_struct[i].Working_state == 0)
        {
            selected_num = i;
            break;
        }

    }

    if(i >= MAX_DriverIC_num) //   全满了，且最后一个是2，那么第一个就是当前新的
    {
        selected_num = 0;
    }

    // 2. update new  info
    memcpy((u8 *)(&Drivers_struct[selected_num].Driver_BASEinfo), (u8 *)(&Read_ICinfo_Reg), sizeof(Read_ICinfo_Reg));
    Drivers_struct[selected_num].Working_state = 2; //
    Drivers_struct[selected_num].Running_counter = 0;
    Drivers_struct[selected_num].Stopping_counter = 0;

    if(GB19056.workstate == 0)
    {
        rt_kprintf("\r\n  更新新内容 i=%d  ID=%s\r\n", selected_num, Drivers_struct[selected_num].Driver_BASEinfo.DriverCard_ID);


        rt_kprintf("\r\n  1--更新新内容 i=%d  ID=%s\r\n", 0, Drivers_struct[0].Driver_BASEinfo.DriverCard_ID);
        rt_kprintf("\r\n  2--更新新内容 i=%d  ID=%s\r\n", 1, Drivers_struct[1].Driver_BASEinfo.DriverCard_ID);
        rt_kprintf("\r\n  3--更新新内容 i=%d  ID=%s\r\n", 2, Drivers_struct[2].Driver_BASEinfo.DriverCard_ID);
    }
}



//  判断 不同驾驶员 ,连续驾驶的开始时间
void  Different_DriverIC_Start_Process(void)
{
    u8  i = 0;
    // 判断各个驾驶员的连续驾驶的开始时间
    if((Spd_Using > 60) && (Sps_larger_5_counter > 10))
    {
        for(i = 0; i < MAX_DriverIC_num; i++)
        {
            if((Drivers_struct[i].Working_state == 2) && (Drivers_struct[i].Running_counter == 0))
            {
                time_now = Get_RTC();   //  RTC  相关
                Time2BCD(Drivers_struct[i].Start_Datetime);
                //   3.  起始位置
                memcpy(Drivers_struct[i].Longi, VdrData.Longi, 4); // 经度
                memcpy(Drivers_struct[i].Lati, VdrData.Lati, 4); //纬度
                Drivers_struct[i].Hight = GPS_Hight;
                Drivers_struct[i].H_11_start = 1; //  start
                Drivers_struct[i].H_11_lastSave_state = 0;
            }
        }
    }

}

//  判断不同驾驶员，连续驾驶的结束时间 // 速度为0  时候执行
void  Different_DriverIC_End_Process(void)
{
    u8 i = 0;
    u8  value = 0;

    for(i = 0; i < MAX_DriverIC_num; i++)
    {
        if((Drivers_struct[i].Working_state) && (Drivers_struct[i].Running_counter >= TiredConf_struct.TiredDoor.Door_DrvKeepingSec))
        {
            if((TiredConf_struct.TiredDoor.Door_DrvKeepingSec <= 0) && (Drivers_struct[i].H_11_start == 0))
                break;

            //  1.   机动车驾驶证号
            memcpy(VdrData.H_11, Drivers_struct[i].Driver_BASEinfo.DriverCard_ID, 18);
            if(GB19056.workstate == 0)
                rt_kprintf("\r\n    drivernum=%d drivercode-=%s\r\n", i, Drivers_struct[i].Driver_BASEinfo.DriverCard_ID);
            //   2.   起始时间
            memcpy(VdrData.H_11 + 18, Drivers_struct[i].Start_Datetime, 6); // 起始时间
            time_now = Get_RTC();   //  RTC  相关
            Time2BCD(Drivers_struct[i].End_Datetime);
            memcpy(VdrData.H_11 + 24, Drivers_struct[i].End_Datetime, 6); // 结束时间
            //   3.  起始位置
            memcpy( VdrData.H_11 + 30, Drivers_struct[i].Longi, 4); // 经度
            memcpy( VdrData.H_11 + 30 + 4, Drivers_struct[i].Lati, 4); //纬度
            VdrData.H_11[30 + 8] = (Drivers_struct[i].Hight >> 8);
            VdrData.H_11[30 + 9] = Drivers_struct[i].Hight;
            //VdrData.H_11_start=1; //  start
            //VdrData.H_11_lastSave=0;

            //   4.   结束位置
            memcpy( VdrData.H_11 + 40, VdrData.Longi, 4); // 经度
            memcpy( VdrData.H_11 + 40 + 4, VdrData.Lati, 4); //纬度
            VdrData.H_11[40 + 8] = (GPS_Hight >> 8);
            VdrData.H_11[40 + 9] = GPS_Hight;

            value = 2;

            VDR_product_11H_End(2);

            Drivers_struct[i].H_11_start = 2; // end over

            if(value == 2)
            {
                Drivers_struct[i].H_11_lastSave_state = 1;
            }
        }

    }
}

//  判断不同驾驶员 疲劳状态
void  Different_DriverIC_Checking(void)
{
    u8 i = 0;

    if((Spd_Using > 60 ) && (Sps_larger_5_counter > 10))	// Spd_Using 单位为0.1 km/h  速度大于6 km/h  认为是行驶
    {
        // 行驶状态
        for(i = 0; i < MAX_DriverIC_num; i++)
        {
            if( Drivers_struct[i].Working_state == 2)
            {
                Drivers_struct[i].Running_counter++;

                //       提前30 min  语音提示
                if(TiredConf_struct.TiredDoor.Door_DrvKeepingSec > 1800)
                {
                    if(Drivers_struct[i].Running_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec - 1800 ) ) //提前5分钟蜂鸣器提示注意疲劳驾驶 14100
                    {
                        //   疲劳报警预提示 开始
                        if(GB19056.SPK_PreTired.Warn_state_Enable == 0)
                            GB19056.SPK_PreTired.Warn_state_Enable = 1;
                    }
                }
                //--   判断疲劳驾驶--------------------------------------------------------------------
                if(  Drivers_struct[i].Running_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec) )//14400
                {
                    if(GB19056.workstate == 0)
                        rt_kprintf( "\r\n   疲劳驾驶触发了!  on\r\n" );
                    //  TTS_play( "您已经疲劳驾驶，请注意休息" );


                    //	超时驾驶 警示触发了
                    if(GB19056.SPK_DriveExceed_Warn.Warn_state_Enable == 0)
                        GB19056.SPK_DriveExceed_Warn.Warn_state_Enable = 1;
                    //	疲劳报警预提示 结束
                    GB19056.SPK_PreTired.Warn_state_Enable = 0;



                    //tts_bro_tired_flag    = 1;
                    Warn_Status[3]	   |= 0x04; //BIT(2)  疲劳驾驶
                    //---- 触发即时上报数据-------
                    PositionSD_Enable( );
                    Current_UDP_sd = 1;
                    //-------------------------------------
                }
                //----------------------------------------------------------------------------------------
                Drivers_struct[i].Stopping_counter = 0;
            }
            else if(Drivers_struct[i].Working_state == 1) //  状态为1  也算休息
            {
                Drivers_struct[i].Stopping_counter++; // 状态为1 的车算休息

                if(Drivers_struct[i].Stopping_counter >= TiredConf_struct.TiredDoor.Door_MinSleepSec )	//1200	 // ACC 关20分钟视为休息
                {
                    if(Drivers_struct[i].Stopping_counter == TiredConf_struct.TiredDoor.Door_MinSleepSec )
                    {
                        Drivers_struct[i].Stopping_counter = 0;
                        Drivers_struct[i].Running_counter = 0;

                        // 	只有触发过疲劳驾驶的且开始时间赋过数值才会存储判断
                        if((Drivers_struct[i].Running_counter >= TiredConf_struct.TiredDoor.Door_DrvKeepingSec) && \
                                (Drivers_struct[i].H_11_start == 2));
                        {
                            Drivers_struct[i].Working_state = 0; //
                            Warn_Status[3] &= ~0x04; //BIT(2)	 疲劳驾驶

                            //---- 触发即时上报数据-------
                            PositionSD_Enable( );
                            Current_UDP_sd = 1;
#if 0
                            //-------------------------------------
                            //    超时结束  存储，索引累加
                            //  1.   机动车驾驶证号
                            memcpy(VdrData.H_11, Drivers_struct[i].Driver_BASEinfo.DriverCard_ID, 18);
                            //   2.   起始时间
                            memcpy(VdrData.H_11 + 18, Drivers_struct[i].Start_Datetime, 6); // 起始时间
                            memcpy(VdrData.H_11 + 24, Drivers_struct[i].End_Datetime, 6);	// 结束时间
                            //   3.  起始位置
                            memcpy( VdrData.H_11 + 30, Drivers_struct[i].Longi, 4); // 经度
                            memcpy( VdrData.H_11 + 30 + 4, Drivers_struct[i].Lati, 4); //纬度
                            VdrData.H_11[30 + 8] = (Drivers_struct[i].Hight >> 8);
                            VdrData.H_11[30 + 9] = Drivers_struct[i].Hight;

                            //   4.   结束位置
                            memcpy( VdrData.H_11 + 40, VdrData.Longi, 4);	// 经度
                            memcpy( VdrData.H_11 + 40 + 4, VdrData.Lati, 4); //纬度
                            VdrData.H_11[40 + 8] = (GPS_Hight >> 8);
                            VdrData.H_11[40 + 9] = GPS_Hight;

                            VDR_product_11H_End(1);
#endif

                            //	只有不为当前才清除，当前为2 不清除
                            if(Drivers_struct[i].Working_state == 1)
                                memset((u8 *)&Drivers_struct[i], 0, sizeof(DRIVE_STRUCT)); // clear


                            //  超时驾驶结束
                            GB19056.SPK_DriveExceed_Warn.Warn_state_Enable = 0;
                            if(GB19056.workstate == 0)
                                rt_kprintf( "\r\n	i=%d 您的疲劳驾驶报警已经解除-on 1 \r\n", i);
                        }
                    }
                }
            }

        }

    }
    else
    {
        // 停止状态
        for(i = 0; i < MAX_DriverIC_num; i++)
        {
            if( Drivers_struct[i].Working_state)
            {
                if(Drivers_struct[i].Running_counter)
                {
                    //-- ACC 没有休息前还算AccON 的状态  ---------------
                    Drivers_struct[i].Running_counter++;

                    //---------------------------------------------------------------
                    //       提前30 min  语音提示
                    if(TiredConf_struct.TiredDoor.Door_DrvKeepingSec > 1800)
                    {
                        if(Drivers_struct[i].Running_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec - 1800 ) ) //提前5分钟蜂鸣器提示注意疲劳驾驶 14100
                        {
                            //   疲劳报警预提示 开始
                            if(GB19056.SPK_PreTired.Warn_state_Enable == 0)
                                GB19056.SPK_PreTired.Warn_state_Enable = 1;
                        }
                    }


                    //      ACC off    but     Acc on  conintue  run
                    if( Drivers_struct[i].Running_counter >= ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec ) )       //14400 // 连续驾驶超过4小时算疲劳驾驶
                    {
                        if(Drivers_struct[i].Running_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec  ) )  //14400
                        {
                            if(GB19056.workstate == 0)
                                rt_kprintf( "\r\n	 速度小，但未满足休息门限时间 疲劳驾驶触发了! \r\n");
                            //TTS_play( "您已经疲劳驾驶，请注意休息" );




                            //	 超时驾驶 触发了
                            if(GB19056.SPK_DriveExceed_Warn.Warn_state_Enable == 0)
                                GB19056.SPK_DriveExceed_Warn.Warn_state_Enable = 1;
                            //   疲劳报警预提示 结束
                            GB19056.SPK_PreTired.Warn_state_Enable = 0;


                            Warn_Status[3]		|= 0x04;                                                                                    //BIT(2)  疲劳驾驶
                            //---- 触发即时上报数据-------
                            PositionSD_Enable( );
                            Current_UDP_sd = 1;
                        }
                        Warn_Status[3] |= 0x04;                                                                                             //BIT(2)  疲劳驾驶
                    }


                    //     ACC  off   counter
                    Drivers_struct[i].Stopping_counter++;
                    if(Drivers_struct[i].Stopping_counter >= TiredConf_struct.TiredDoor.Door_MinSleepSec ) //1200	// ACC 关20分钟视为休息
                    {
                        if(Drivers_struct[i].Stopping_counter == TiredConf_struct.TiredDoor.Door_MinSleepSec )
                        {
                            Drivers_struct[i].Stopping_counter = 0;
                            Drivers_struct[i].Running_counter = 0;

                            //     只有触发过疲劳驾驶的且开始时间赋过数值才会存储判断
                            if((Drivers_struct[i].Running_counter >= TiredConf_struct.TiredDoor.Door_DrvKeepingSec) && \
                                    (Drivers_struct[i].H_11_start == 2));
                            {
                                Drivers_struct[i].Working_state = 0; //
                                Warn_Status[3] &= ~0x04; //BIT(2)	疲劳驾驶

                                //---- 触发即时上报数据-------
                                PositionSD_Enable( );
                                Current_UDP_sd = 1;
                                //-------------------------------------
                                //    超时结束  存储，索引累加
                                //  1.   机动车驾驶证号
                                memcpy(VdrData.H_11, Drivers_struct[i].Driver_BASEinfo.DriverCard_ID, 18);
                                //   2.   起始时间
                                memcpy(VdrData.H_11 + 18, Drivers_struct[i].Start_Datetime, 6); // 起始时间
                                memcpy(VdrData.H_11 + 24, Drivers_struct[i].End_Datetime, 6); // 结束时间
                                //   3.  起始位置
                                memcpy( VdrData.H_11 + 30, Drivers_struct[i].Longi, 4); // 经度
                                memcpy( VdrData.H_11 + 30 + 4, Drivers_struct[i].Lati, 4); //纬度
                                VdrData.H_11[30 + 8] = (Drivers_struct[i].Hight >> 8);
                                VdrData.H_11[30 + 9] = Drivers_struct[i].Hight;

                                //   4.   结束位置
                                memcpy( VdrData.H_11 + 40, VdrData.Longi, 4); // 经度
                                memcpy( VdrData.H_11 + 40 + 4, VdrData.Lati, 4); //纬度
                                VdrData.H_11[40 + 8] = (GPS_Hight >> 8);
                                VdrData.H_11[40 + 9] = GPS_Hight;

                                if((Drivers_struct[i].End_Datetime[0] == 0x00) && (Drivers_struct[i].End_Datetime[1] == 0x00) && (Drivers_struct[i].End_Datetime[2] == 0x00))
                                    ; // 年月日 为0  过滤
                                else
                                    VDR_product_11H_End(1);

                                //  只有不为当前才清除，当前为2 不清除
                                if(Drivers_struct[i].Working_state == 1)
                                    memset((u8 *)&Drivers_struct[i], 0, sizeof(DRIVE_STRUCT)); // clear
                                else



                                    //  超时驾驶结束
                                    GB19056.SPK_DriveExceed_Warn.Warn_state_Enable = 0;
                                if(GB19056.workstate == 0)
                                    rt_kprintf( "\r\n	您的疲劳驾驶报警已经解除 \r\n");
                            }

                        }
                    }
                    //-------------------------------------------------------------------------------------
                }
            }
        }


    }
}



#if 1
//  orginal setting
void CheckICInsert(void)
{
    unsigned char write_flag = 0;
    u8 result0 = 0, result1 = 1, result2 = 2, result3 = 4, result4 = 5; //i=0;
    u8 reg_record_liu[13];
    u32 DriveCode32 = 0;
    u8  red_byte[128];
    u8  fcs = 0, i = 0;


    //===================测试IC卡读写==================================================
    if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))
    {
        IC_Check_Count++;
        if(IC_Check_Count >= 10)
        {
            IC_Check_Count = 0;
            //带卡上电开8024的电
            if(flag_8024off == 1)     //  上电不读取IC 卡
            {
                R_Flag |= b_CardEdge;
                Init8024Flag = 2;
                flag_8024off = 0;
            }

            //8024的off从低变高
            if(Init8024Flag == 1)
            {
                Init8024Flag = 2;
                R_Flag |= b_CardEdge;
            }
            //检测到卡后初始化ic卡
            if((R_Flag & b_CardEdge) && (Init8024Flag == 2))
            {
                Init8024Flag = 3;
                _CardCMDVCC_LOW;
                for(DelayCheckIc = 0; DelayCheckIc < 500; DelayCheckIc++)
                    DELAY5us();
                _CardSetPower_HIGH;
                _CardSetRST_LOW;
                for(DelayCheckIc = 0; DelayCheckIc < 15; DelayCheckIc++)
                {
                    _CardSetCLK_LOW;
                    DELAY5us();
                    DELAY5us();
                    DELAY5us();
                    _CardSetCLK_HIGH;
                    DELAY5us();
                    DELAY5us();
                    DELAY5us();
                    _CardSetCLK_LOW;
                }
                R_Flag &= ~b_CardEdge;
                write_flag = 1;
                //----------------------------------------------------
            }
        }
    }
    else
    {

        IC_Check_Count = 0;
        _CardSetRST_HIGH;
        _CardSetPower_LOW;
        _CardCMDVCC_HIGH;
        if(Init8024Flag == 0)
        {
            Init8024Flag = 1;
            //if(powerOn_first==0)
            // powerOn_first=1;
            //else
            {
                //---------------------------------------------------
                // rt_kprintf("   拔卡 pc7  为 低---触发\r\n");
                if((Spd_Using < 40) && (IC_loginState == 1)) // 在行驶结束状态下才开始 判断插拔卡
                {
                    VDR_product_12H(0x02);  //  登出
                    TTS_play("驾驶员退出");
                    IC_loginState = 0;
                }
                //---------------------------------------------------

            }
        }
    }

    if(write_flag == 1)
    {
        write_flag = 0;
        if((Spd_Using < 40) && (Sps_larger_5_counter == 0)) // 在行驶结束状态下才开始 判断插拔卡
        {
            /*  result2=Rx_4442(0,32,red_byte);  //读驾驶证号码
            	  rt_kprintf("\r\n机动车驾驶证号码:result1=%d",result2);
            	{
            		OutPrint_HEX("国标IC信息 1",red_byte,32);

            	}
            */
            // 0  ------------- 管理员卡---
            Rx_4442(241, 13, red_byte);	//管理员卡
            if(strncmp((char *)red_byte, "administrator", 13) == 0)
            {
                rt_kprintf("\r\n管理员卡");
                administrator_card = 1;

                pMenuItem = &Menu_0_loggingin; // 管理员卡进入界面
                pMenuItem->show();
                BuzzerFlag = 1; //响一声提示
                Init8024Flag = 0;
                GpsIo_Init();
                return;
            }

            // 1 .---------获取国标信息-----------
            result2 = Rx_4442(32, 96, red_byte); //读驾驶证号码
            if(result2 == 0)
            {
                //OutPrint_HEX("国标IC信息 2",red_byte,96);
                fcs = 0;
                for(i = 0; i < 95; i++)
                {
                    fcs ^= red_byte[i];
                }
            }
            else
            {
                //  读卡失败
                BuzzerFlag = 11; //响一声提示
                IC_CardInsert = 2; //IC	卡插入错误
                Init8024Flag = 0;
                GpsIo_Init();
                Menu_txt_state = 3;
                pMenuItem = &Menu_TXT;
                pMenuItem->show();
                pMenuItem->timetick( 10 );
                pMenuItem->keypress( 10 );
                return;
            }

            // 2.计算FCS
            if((red_byte[95] == fcs) && (result2 == 0))
            {
                memset(Read_ICinfo_Reg.DriverCard_ID, 0, sizeof(Read_ICinfo_Reg.DriverCard_ID));
                result1 = Rx_4442(32, 18, (unsigned char *)Read_ICinfo_Reg.DriverCard_ID);	//读驾驶证号码
                //if( GB19056.workstate==0)
                //{
                //rt_kprintf("\r\n机动车驾驶证号码:%s,result1=%d",Read_ICinfo_Reg.DriverCard_ID,result1);
                //OutPrint_HEX("机动车驾驶证号码HEX",Read_ICinfo_Reg.DriverCard_ID,18);
                //}

                memset(Read_ICinfo_Reg.Effective_Date, 0, sizeof(Read_ICinfo_Reg.Effective_Date));
                result2 = Rx_4442(50, 3, (unsigned char *)Read_ICinfo_Reg.Effective_Date);	//读驾驶员代码
                //if( GB19056.workstate==0)
                //rt_kprintf("\r\n有效日期:%2X-%2X-%2X,result2=%d",Read_ICinfo_Reg.Effective_Date[0],Read_ICinfo_Reg.Effective_Date[1],Read_ICinfo_Reg.Effective_Date[2],result2);

                memset(Read_ICinfo_Reg.Drv_CareerID, 0, sizeof(Read_ICinfo_Reg.Drv_CareerID));
                result4 = Rx_4442(53, 18, (unsigned char *)Read_ICinfo_Reg.Drv_CareerID);	//从业资格证
                //if( GB19056.workstate==0)
                //rt_kprintf("\r\n从业资格证:%s,result4=%d",Read_ICinfo_Reg.Drv_CareerID,result4);

                //------  驾驶员姓名(选用)
                memcpy(red_byte, 0, sizeof(red_byte));
                result0 = Rx_4442(128, 10, (unsigned char *)red_byte); //读驾驶员姓名
                if(result0 == 0)
                {
                    if(strlen(red_byte))
                    {
                        if(GB19056.workstate == 0)
                            rt_kprintf("\r\n驾驶员姓名:%s,result0=%d", red_byte, result0);
                        memset(JT808Conf_struct.Driver_Info.DriveName, 0, sizeof(JT808Conf_struct.Driver_Info.DriveName));
                        memcpy(JT808Conf_struct.Driver_Info.DriveName, red_byte, strlen(red_byte));
                    }
                }
                //-----------  停车插卡 且识别 -------
                if((result1 == 0) && (result2 == 0) && (result4 == 0)) //读结果正确
                {
                    Different_DriverIC_InfoUpdate();  // 更新IC 驾驶员信息
                    IC_loginState = 1;
                    VDR_product_12H(0x01);	//	登录
                    if(powerOn_first)
                        TTS_play("驾驶员登录"); // 第一次上电播读，但要记录
                    powerOn_first = 1;
                    GB19056.SPK_UnloginWarn.Warn_state_Enable = 0; // clear
                }
            }
            else
            {
                // 校验错误
                BuzzerFlag = 11; //响一声提示
                IC_CardInsert = 2; //IC	卡插入错误
                Menu_txt_state = 2;
                pMenuItem = &Menu_TXT;
                pMenuItem->show();
                pMenuItem->timetick( 10 );
                pMenuItem->keypress( 10 );

            }
            //------------------------------------------
        }
        else
        {
            result1 = 1; // 不全为0  让判断错误
            // if( GB19056.workstate==0)
            // rt_kprintf("\r\n行驶中插卡不记录!");
        }


        //--------- IC 卡声音提示-----------------
        if((result1 == 0) && (result2 == 0) && (result4 == 0)) //读结果正确
        {
            IC_CardInsert = 1; //IC	卡插入正确
            BuzzerFlag = 1; //响一声提示
            pMenuItem = &Menu_3_4_DriverInfor;
            pMenuItem->show();
        }
        else
        {
            BuzzerFlag = 11; //响一声提示
            IC_CardInsert = 2; //IC	卡插入错误
        }
        //-------------------------------------------------------------------
        Init8024Flag = 0;
        GpsIo_Init();
    }

    //===================测试IC卡读写完成==================================================

}
#endif


