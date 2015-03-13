/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>

#include "stm32f2xx.h"
#include <board.h>
#include <rtthread.h>
#include "App_moduleConfig.h"

#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#include "stm32_eth.h"
#endif


#ifdef   TFCARD
#include "ffconf.h"
#include "ff.h"
#include "SPI_SD_driver.h"
#include <finsh.h>
#include "diskio.h"

//----  TF  卡相关 ---
uint8_t sd_ok = 0;
static FATFS fs;
static FIL 	file;
u32  TimeTriggerPhoto_counter = 0; // 定时触发拍照计时器
u8	dir_name[32];



void sd_open(char *name)
{
    FRESULT res;
    int  len;
    char buffer[512];

    DF_TAKE;
    res = f_open(&file, name, FA_READ | FA_WRITE);
    if(res == 0)
    {
        rt_kprintf("\r\n file\"%s:\"\r\n", name);
        while(1)
        {
            memset(buffer, 0, sizeof(buffer));
            res = f_read(&file, buffer, sizeof(buffer), &len);
            if(res || len == 0)
            {
                rt_kprintf("\r\n f_read = %d,len = %d", res, len);
                break;
            }
            rt_kprintf("%s", buffer);
        }
        rt_kprintf("\r\n");
        f_close(&file);
    }
    else
    {
        rt_kprintf("\r\n f_open = %d", res);
    }
    DF_RELEASE;
}
FINSH_FUNCTION_EXPORT( sd_open, open a tf card );

void sd_writefile(char *name, char *str, int len_str)
{
    FRESULT res;
    int  len;
    //u8  dir_name[32];
    u8  path[80];
    //	 FILINFO fno;
    DIR dj;

    SPI_SetSpeed(1);  // 高速


    memset(path, 0, sizeof(path));
    //=======================================
    sprintf(path, "0:/%s/%s", dir_name, name);
    //=======================================

    res = f_open(&file, path, FA_READ | FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_EXISTING );
    if(res == FR_OK)
    {
        f_lseek( &file, file.fsize);
        res = f_write(&file, str, len_str, &len);
        if(res)
        {
            rt_kprintf("\r\n f_write = %d", res);
            sd_ok = 0;
        }
        else
        {
            //rt_kprintf("%s",str);
        }
        res = f_close(&file);
        if(res)
        {
            rt_kprintf("\r\n f_close = %d", res);
            sd_ok = 0;
        }

    }
    else
    {
        rt_kprintf("\r\n f_open = %d", res);
        sd_ok = 0;
    }

    SPI_SetSpeed(0);  // 低速
}

void sd_write( char *name, char *str)
{
    FRESULT res;
    int  len;

    //rt_kprintf("\r\n F_WR(%s):",name);
    if(DF_TAKE != RT_EOK)
    {
        return;
    }
    sd_writefile(name, str, strlen(str));
    DF_RELEASE;
}
FINSH_FUNCTION_EXPORT( sd_write, open a tf card );


void sd_write_console(char *str)
{
    static uint8_t	time_wr = 0;
    FRESULT res;
    int  len;
    char buffer[256];
    char file_name[32];

    if(sd_ok == 0)
    {
        rt_kprintf( "\n%d>%s", rt_tick_get( ), str);
        return;
    }
    if(time_wr == 0)
    {

        // sprintf(file_name, "0:RESET.txt");
        sprintf(file_name, "RESET.txt");
        sprintf( buffer, "20%02d-%02d-%02d-%02d:%02d:%02d->%s\r\n",
                 time_now.year,
                 time_now.month,
                 time_now.day,
                 time_now.hour,
                 time_now.min,
                 time_now.sec,
                 "Reset system!"
               );
        sd_write(file_name, buffer);
    }
#if  0
    ///比较当前时间和最后一次的时间是否在同一天
    sprintf(file_name, "0:TW%02d%02d%02d.txt", \
            time_now.year,
            time_now.month,
            time_now.day
           );
    sd_write(file_name, buffer);
}

///比较当前时间和最后一次的时间是否在同一天
sprintf(file_name, "0:TW%02d%02d%02d.txt", \
        time_now.year,
        time_now.month,
        time_now.day
       );

sprintf( buffer, "20%02d-%02d-%02d-%02d:%02d:%02d->%s\r\n",
         time_now.year,
         time_now.month,
         time_now.day,
         time_now.hour,
         time_now.min,
         time_now.sec,
         str
       );
sd_write(file_name, buffer);
#endif
time_wr = 1;
}
FINSH_FUNCTION_EXPORT_ALIAS( sd_write_console, sd_write, debug sd write_console );


void sd_init_process(void)
{
    FRESULT res;
    char tempbuf[64];
    u32 capcity = 0, free_Capcity = 0;

    DIR dj;

    if(DF_TAKE != RT_EOK)
        return;

    //SPI_Configuration();
    res = SD_Init();
    if(0 == res)
    {

        capcity = SD_GetCapacity();

        if(SD_Type == SD_TYPE_V2)
            rt_kprintf("\r\n SD CARD INIT OK!  SD 容量  HEX=0x%00000008X      DEC=%d 字节  \r\n", capcity, capcity);

        if(SD_Type == SD_TYPE_V2HC)
            rt_kprintf("\r\n SD CARD INIT OK!  SD 容量  HEX=0x%00000008X      DEC=%d 扇区(512字节)   SIZE=%d  MB \r\n", capcity, capcity, (capcity / 2048)); //

        /*
        		 free_Capcity=sd_freesize();

        		 rt_kprintf("\r\n  剩余 SD 容量  HEX=0x%00000008X	   DEC=%d 扇区(512字节)   SIZE=%d  MB \r\n", free_Capcity,free_Capcity,(free_Capcity/2048)); //

                memset(tempbuf,0,sizeof(tempbuf));
                SD_GetCID(tempbuf);
                OutPrint_HEX("\r\n SID=",tempbuf, 16);
                SD_GetCSD(tempbuf);
                OutPrint_HEX("\r\n CSD=",tempbuf, 16);
        */
        if(0 == f_mount(MMC, &fs))
            //if(0 == f_mount( (&fs),"0:",1))
        {
            //rt_kprintf("\r\n f_mount SD OK!");
            sd_ok = 1;
        }
    }
    else
    {
        rt_kprintf("\r\n SD CARD INIT ERR = %d", res);
    }
    //rt_sem_init( &sem_sd, "sem_sd", 0, RT_IPC_FLAG_FIFO );
    //  创建文件夹或打开文件
    memset(dir_name, 0, sizeof(dir_name));
    //======================================
    sprintf(dir_name, "%02d%02d%02d", time_now.year,time_now.month, time_now.day);
    // sprintf(dir_name, "lnw");
    res = f_mkdir(dir_name);
    if(res)
    {
        rt_kprintf("\r\n f_mkdir = %d", res);
        res = f_opendir(&dj, dir_name);
        if(res)
            rt_kprintf("\r\n f_opendir = %d", res); 

    }
    DF_RELEASE;
    sd_write_console("\r\nInit_SD_card! \r\n");

}

void TF_Init(void)
{
    u8  i = 0;

#if 0
    for(i = 0; i < 3; i++)		//多次初始化时因为曾经发现没有正常初始化，导致误认为没有装卡
    {
        if(sd_ok == 0)
        {
            sd_init_process();
        }
    }
#endif
    sd_init_process();

}

void  TakePhoto_timerISR_1S(void)
{
    // acc on  且有速度
    if(ACC_StatusGet() && (Speed_gps > 50))
    {
        TimeTriggerPhoto_counter++;
        if(TimeTriggerPhoto_counter > JT808Conf_struct.take_Duration)
            // if(TimeTriggerPhoto_counter>50)
        {
            TimeTriggerPhoto_counter = 0;
            if(DataLink_Status())
            {
                takephoto("1");
                //  rt_kprintf("\r\n 定时拍照触发 \r\n");
            }
        }
    }
    else
        TimeTriggerPhoto_counter = 0;

}

#endif


void rt_init_thread_entry(void *parameter)
{
    /* LwIP Initialization */

    //FS

    //GUI
}

int rt_application_init()
{
    rt_thread_t init_thread;


    Device_CAN2_regist();    //  Device CAN2 Init

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   256, 8, 20);  // thread null
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;

}
/*@}*/
