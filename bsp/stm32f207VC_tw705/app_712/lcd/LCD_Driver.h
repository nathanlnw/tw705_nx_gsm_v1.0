#ifndef SED1520_H_
#define SED1520_H_

// define to disable busy-delays (useful for simulation)
//// #define LCD_SIM (1)
// define to enable additional debugging functions
//// #define LCD_DEBUG (1)

#include <rtthread.h>
//#include <rtdevice.h>
#include <rthw.h>
#include "stm32f2xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>


#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include  "App_moduleConfig.h"
#include  "rtdevice.h "




#include <stdio.h>
#include <rtthread.h>
#include "stm32f2xx.h"
#include "board.h"
//#include "ringbuffer.h"
#include "Device_printer.h"
#include <finsh.h>


#include <stdint.h>
#include "bmp.h"



/* draw modes */
#define LCD_MODE_CLEAR     0
#define LCD_MODE_SET       1
#define LCD_MODE_XOR       2
#define LCD_MODE_INVERT		3

/* command function equates for ST7565R LCD Display Controller */
#define LCD_DISP_OFF       0xAE	/* turn LCD panel OFF */
#define LCD_DISP_ON        0xAF	/* turn LCD panel ON */
#define LCD_SET_LINE       0xC0	/* set line for COM0 (4 lsbs = ST3:ST2:ST1:ST0) */
#define LCD_SET_PAGE_ST7565R       0xB0	/* set page address (2 lsbs = P1:P0) */
#define LCD_SET_PAGE_SED1520       0xB8	/* set page address (2 lsbs = P1:P0) */

#define LCD_SET_COLH       0x10 /*  列地址高字节*/
#define LCD_SET_COL        0x00	/* set column address (6 lsbs = Y4:Y4:Y3:Y2:Y1:Y0) */
#define LCD_SET_ADC_NOR    0xA0	/* ADC set for normal direction */
#define LCD_SET_ADC_REV    0xA1	/* ADC set for reverse direction */
#define LCD_STATIC_OFF     0xA6	/* normal drive */
#define LCD_STATIC_ON      0xA7	/* static drive (power save) */
#define LCD_DISPALL_NORMAL 0xA4 /*0:disp all points  normal --new lcd*/
#define LCD_DISPALL_ENABLE 0xA5 /*1:disp all points  ON---new lcd*/
#define LCD_BIAS_1D9       0xA2 /* bias    1/9 bias*/
#define LCD_BIAS_1D7       0xA3 /* bias    1/7 bias*/
#define LCD_OUTPUT_NORMAL  0xC0 /* output  scann  direction   0:  normal direction--new lcd*/
#define LCD_OUTPUT_RVS     0xC8 /* output  scann  direction   1:  reverse direction--new lcd*/
#define LCD_SET_MODIFY     0xE0	/* start read-modify-write mode */
#define LCD_CLR_MODIFY     0xEE	/* end read-modify-write mode */
#define LCD_RESET          0xE2	/* soft reset command */

/* LCD screen and bitmap image array consants */
#define LCD_X_BYTES		122
#define LCD_Y_BYTES		4
#define SCRN_LEFT		0
#define SCRN_TOP		0
#define SCRN_RIGHT		121
#define SCRN_BOTTOM		31

/* LCD_Driver is used with reverse direction (ADC_REV).
   This value is the address of the leftmost column: */
#define LCD_STARTCOL_REVERSE	19

/* LCD Global data arrays */
//extern const unsigned char l_mask_array[8];
//extern unsigned char l_display_array[LCD_Y_BYTES][LCD_X_BYTES];

/* LCD function prototype list */
/*    Close  Disp         -- add  by  nathan */

extern u8	cByte;// 关闭背光



extern void lcd_init(void);
extern void lcd_out_dat(const unsigned char dat, const unsigned char nctrl);
extern void lcd_out_ctl(const unsigned char cmd, const unsigned char nctrl);
extern void lcd_fill(const unsigned char pattern);

extern void lcd_update(const unsigned char top, const unsigned char bottom);
extern void lcd_update_all(void);

void lcd_dot(const unsigned char x, const unsigned char y, const unsigned char mode);
#define lcd_dot_set(x,y)    lcd_dot(x, y, LCD_MODE_SET)
#define lcd_dot_clear(x,y)  lcd_dot(x, y, LCD_MODE_CLEAR);
#define lcd_dot_switch(x,y) lcd_dot(x, y, LCD_MODE_XOR);

extern void lcd_line( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, const uint8_t mode );
#if 0
extern void lcd_rect( const uint8_t x, const uint8_t y, uint8_t width, uint8_t height, const uint8_t mode);
extern void lcd_box(const uint8_t x, const uint8_t y, uint8_t width, const uint8_t height, const uint8_t mode);
extern void lcd_circle(const uint8_t xCenter, const uint8_t yCenter, const uint8_t radius, const uint8_t mode);
#endif
extern void lcd_glyph(uint8_t left, uint8_t top, uint8_t width, uint8_t height, uint8_t *glyph_ptr, uint8_t store_width);
extern void lcd_bitmap(const uint8_t left, const uint8_t top, const struct IMG_DEF *img_ptr, const uint8_t mode);
extern void lcd_text12(char left, char top , char *p, char len, const char mode);
extern void lcd_text12_local(char left, char top , char *p, char len, const char mode);
extern void lcd_RstLow(void);




extern void Lcd_hardInit_timer(void);

#endif
