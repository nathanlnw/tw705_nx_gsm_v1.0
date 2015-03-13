/*
      App_moduleConig   ,   Comon use   Fuctions         according to  712  Rule
*/


#include <rtthread.h>
#include <rthw.h>
#include "stm32f2xx.h"

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"


//  1.      MsgQueue    TX
rt_err_t rt_712Rule_MsgQue_Post(rt_mq_t Dest_mq, u8 *buffer, rt_size_t size)
{
    MSG_Q_TYPE  msg_struct;

    msg_struct.len = (u16)size;
    msg_struct.info = buffer;

    /* send message to usb message queue */
    rt_mq_send(Dest_mq, (u8 *)(&msg_struct),  size + 2);
    return RT_EOK;
}
