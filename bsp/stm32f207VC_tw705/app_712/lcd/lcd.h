#ifndef LCDDis_H_
#define LCDDis_H_

#include "LCD_Driver.h"
#include <stdio.h>


// note    703    按键顺序:       菜单   确认  上翻  下翻(打印)
/*
#define KEY_MENU_PORT	GPIOC
#define KEY_MENU_PIN	GPIO_Pin_8

#define KEY_OK_PORT		GPIOA
#define KEY_OK_PIN		GPIO_Pin_8

#define KEY_UP_PORT		GPIOC
#define KEY_UP_PIN		GPIO_Pin_9

#define KEY_DOWN_PORT	GPIOD
#define KEY_DOWN_PIN	GPIO_Pin_3
*/

// note    705    按键顺序:       菜单   上    下   确认

#define KEY_MENU_PORT	GPIOC
#define KEY_MENU_PIN	GPIO_Pin_8

#define KEY_OK_PORT		GPIOC
#define KEY_OK_PIN		GPIO_Pin_9

#define KEY_UP_PORT		GPIOD
#define KEY_UP_PIN		GPIO_Pin_3

#define KEY_DOWN_PORT	GPIOA
#define KEY_DOWN_PIN	GPIO_Pin_8



extern void KeyCheckFun(void);
extern void Init_lcdkey(void);


#endif
