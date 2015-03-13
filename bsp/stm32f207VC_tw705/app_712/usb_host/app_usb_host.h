/*
     App_usb_host.h
*/
#ifndef  _RT_USBHOST
#define _RT_USBHOST

#include <rthw.h>
#include <rtthread.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>


extern void usbhost_app_init(void);
extern void usbhost_set_device(char *device_name);
extern rt_device_t rt_usbhost_set_device(const char *name);







#endif

