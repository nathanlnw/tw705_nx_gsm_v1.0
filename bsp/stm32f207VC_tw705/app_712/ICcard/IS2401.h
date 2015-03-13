#ifndef _IS2401_H
#define _IS2401_H

#include <rtthread.h>
#include <rthw.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//Êý×Ö×ª»»³É×Ö·û´®
#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include "App_moduleConfig.h"



extern unsigned char IC_Rx_2401(unsigned char addr, unsigned char num, unsigned char *buf);



#endif
