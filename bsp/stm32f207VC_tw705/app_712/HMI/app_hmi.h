/*
     app_hmi.h
*/
#ifndef  _RT_HMI
#define _RT_HMI

#include <rthw.h>
#include <rtthread.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>
#include "App_moduleConfig.h"


typedef struct    _HMI_COM
{
    u16    Msg_ID;  //JT808  ID
    u16    Info_Len;
    u8   *Content;
} HMI_COM;

extern void  HMI_app_init(void);
extern void Dayin_Fun(u8 dayin_par);

#endif
