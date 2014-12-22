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

void rt_init_thread_entry(void* parameter)
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
