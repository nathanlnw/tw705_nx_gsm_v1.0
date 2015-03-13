/**************************************************************************
 *
 *   LCD_Driver.c
 *   LCD display controller interface routines for graphics modules
 *   with onboard LCD_Driver controller(s) in "write-only" setup
 *
 *   Version 1.02 (20051031)
 *
 *   For Atmel AVR controllers with avr-gcc/avr-libc
 *   Copyright (c) 2005
 *     Martin Thomas, Kaiserslautern, Germany
 *     <eversmith@heizung-thomas.de>
 *     http://www.siwawi.arubi.uni-kl.de/avr_projects
 *
 *   Permission to use in NON-COMMERCIAL projects is herbey granted. For
 *   a commercial license contact the author.
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *   FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *   COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *   partly based on code published by:
 *   Michael J. Karas and Fabian "ape" Thiele
 *
 *
 ***************************************************************************/

/*
   An Emerging Display EW12A03LY 122x32 Graphics module has been used
   for testing. This module only supports "write". There is no option
   to read data from the LCD_Driver RAM. The LCD_Driver R/W line on the
   module is bound to GND according to the datasheet. Because of this
   Read-Modify-Write using the LCD-RAM is not possible with the 12A03
   LCD-Module. So this library uses a "framebuffer" which needs
   ca. 500 bytes of the AVR's SRAM. The libray can of cause be used
   with read/write modules too.
*/

/* tab-width: 4 */

//#include <LPC213x.H>
//#include <includes.h>

#include <stdint.h>
#include "LCD_Driver.h"
#include "bmp.h"
#include "board.h"
#include "stm32f2xx.h"
#include "App_moduleConfig.h"

/* pixel level bit masks for display */
/* this array is setup to map the order */
/* of bits in a byte to the vertical order */
/* of bits at the LCD controller */
const unsigned char l_mask_array[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; /* TODO: avoid or PROGMEM */

/* the LCD display image memory */
/* buffer arranged so page memory is sequential in RAM */
static unsigned char l_display_array[LCD_Y_BYTES][LCD_X_BYTES];

//  new   Lcd MMD
unsigned char ADJUST = 0x23;	//对比度寄存器初始值	，vop调整设置  当前设置数值VDD=3.0V,VOP=0X10=9V


/* control-lines hardware-interface (only "write") */
#define LCD_CMD_MODE()     LCDCTRLPORT &= ~(1<<LCDCMDPIN)
#define LCD_DATA_MODE()    LCDCTRLPORT |=  (1<<LCDCMDPIN)
#define LCD_ENABLE_E1()    LCDCTRLPORT &= ~(1<<LCDE1PIN)
#define LCD_DISABLE_E1()   LCDCTRLPORT |=  (1<<LCDE1PIN)
#define LCD_ENABLE_E2()    LCDCTRLPORT &= ~(1<<LCDE2PIN)
#define LCD_DISABLE_E2()   LCDCTRLPORT |=  (1<<LCDE2PIN)




#define MR		(1<<15)
#define SHCP	(1<<12)
#define DS		(1<<14)
#define STCP1	(1<<15)
#define STCP2	(1<<13)

//--- ST7565R --------
#define RST0	(1<<0)
#define CS_		(1<<1)   // new  LCD     /CS
#define RD_		(1<<2)   // new LCD      /RD
#define WR_		(1<<3)
#define A0		(1<<4)
//----- SED1520  ---------------
#define E1		(1<<1)
#define E2		(1<<2)
#define RW		(1<<3)




#define pgm_read_byte(a)	(*(a))
#define pgm_read_word(a) (*(a))


//  设C9E8
u8 Dot_She[24] = {0x10, 0x00, 0x11, 0x00, 0xF2, 0x07, 0x00, 0x02, 0x50, 0x08, 0xCF, 0x08, 0x41, 0x05, 0x41, 0x02, 0x4F, 0x05, 0xD0, 0x08, 0x10, 0x08, 0x00, 0x00};


// 备B1B8
u8 Dot_Bei[24] = {0x20, 0x00, 0x24, 0x00, 0xD2, 0x0F, 0x57, 0x05, 0x4A, 0x05, 0xCA, 0x07, 0x4A, 0x05, 0x56, 0x05, 0xD2, 0x0F, 0x20, 0x00, 0x20, 0x00, 0x00, 0x00};

//自 D7D4
u8 Dot_Zi[24] = {0x00, 0x00, 0xFC, 0x0F, 0x24, 0x09, 0x24, 0x09, 0x26, 0x09, 0x25, 0x09, 0x24, 0x09, 0x24, 0x09, 0x24, 0x09, 0xFC, 0x0F, 0x00, 0x00, 0x00, 0x00};

//检 BCEC
u8 Dot_Jian[24] = {0x88, 0x00, 0x68, 0x00, 0xFF, 0x0F, 0x48, 0x00, 0x10, 0x09, 0x28, 0x0E, 0xA4, 0x08, 0x23, 0x0B, 0x24, 0x0C, 0xA8, 0x0B, 0x10, 0x08, 0x00, 0x00};


// 开BFAA
u8  Dot_Kai[24] = {0x40, 0x00, 0x42, 0x08, 0x42, 0x06, 0xFE, 0x01, 0x42, 0x00, 0x42, 0x00, 0x42, 0x00, 0xFE, 0x0F, 0x42, 0x00, 0x42, 0x00, 0x40, 0x00, 0x00, 0x00};



// 正D5FD
u8 Dot_Zheng[24] = {0x00, 0x08, 0x02, 0x08, 0xE2, 0x0F, 0x02, 0x08, 0x02, 0x08, 0xFE, 0x0F, 0x42, 0x08, 0x42, 0x08, 0x42, 0x08, 0x42, 0x08, 0x00, 0x08, 0x00, 0x00};

//  常 B3A3
u8 Dot_Chang[24] = {0x0C, 0x00, 0x04, 0x07, 0x75, 0x01, 0x56, 0x01, 0x54, 0x01, 0xD7, 0x0F, 0x54, 0x01, 0x56, 0x01, 0x75, 0x05, 0x04, 0x07, 0x0C, 0x00, 0x00, 0x00};

// 天CCEC
u8 Dot_Tian[24] = {0x20, 0x08, 0x22, 0x08, 0x22, 0x04, 0x22, 0x02, 0xA2, 0x01, 0x7E, 0x00, 0xA2, 0x01, 0x22, 0x02, 0x22, 0x04, 0x22, 0x08, 0x20, 0x08, 0x00, 0x00};

// 线CFDF
u8 Dot_Xian[24] = {0x98, 0x04, 0xD4, 0x04, 0xB3, 0x02, 0x88, 0x02, 0x00, 0x08, 0x48, 0x08, 0x48, 0x04, 0xFF, 0x03, 0x24, 0x05, 0xA5, 0x08, 0x26, 0x0E, 0x00, 0x00};

//短C2B7
u8 Dot_Duan [24] = {0x9E, 0x0F, 0x12, 0x08, 0xF2, 0x07, 0x9E, 0x04, 0x48, 0x00, 0xC4, 0x0F, 0xAB, 0x04, 0x92, 0x04, 0xAA, 0x04, 0xC6, 0x0F, 0x40, 0x00, 0x00, 0x00};

//路 B6CC
u8 Dot_Lu[24] = {0x48, 0x08, 0x47, 0x06, 0xFC, 0x01, 0x44, 0x06, 0x02, 0x08, 0x7A, 0x09, 0x4A, 0x0A, 0x4A, 0x08, 0x4A, 0x0A, 0x7A, 0x09, 0x02, 0x08, 0x00, 0x00};

//  内C4DA
u8 Dot_Nei[24] = {0x00, 0x00, 0xFC, 0x0F, 0x04, 0x00, 0x84, 0x00, 0x64, 0x00, 0x1F, 0x00, 0x24, 0x00, 0xC4, 0x08, 0x04, 0x08, 0xFC, 0x0F, 0x00, 0x00, 0x00, 0x00};

// 部B2BF
u8 Dot_Bu[24] = {0x20, 0x00, 0xAA, 0x0F, 0xB2, 0x04, 0xA3, 0x04, 0xB2, 0x04, 0xAA, 0x0F, 0x20, 0x00, 0xFE, 0x0F, 0x02, 0x02, 0x32, 0x02, 0xCE, 0x01, 0x00, 0x00};

// 电B5E7
u8 Dot_Dian[24] = {0xFC, 0x03, 0x24, 0x01, 0x24, 0x01, 0x24, 0x01, 0xFF, 0x07, 0x24, 0x09, 0x24, 0x09, 0x24, 0x09, 0xFC, 0x09, 0x00, 0x08, 0x00, 0x0E, 0x00, 0x00};

// 池B3D8
u8 Dot_Chi[24] = {0x22, 0x04, 0x44, 0x02, 0x40, 0x00, 0xFC, 0x07, 0x20, 0x08, 0x10, 0x08, 0xFF, 0x0B, 0x08, 0x08, 0x04, 0x09, 0xFC, 0x09, 0x00, 0x0C, 0x00, 0x00};

// 供B9A9
u8 Dot_Gong[24] = {0x10, 0x00, 0xFC, 0x0F, 0x83, 0x08, 0x88, 0x04, 0xFF, 0x02, 0x88, 0x00, 0x88, 0x00, 0x88, 0x00, 0xFF, 0x02, 0x88, 0x04, 0x80, 0x08, 0x00, 0x00};

// .
u8 Dot_dot[24] = {0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x20, 0x01, 0x20, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


u8   lcd_hard_init = 0; // 每屏幕  显示器硬件初始话
u32  lcd_hardIint_couter = 0;


void ControlBitShift(unsigned char data)
{
    unsigned char i;

    //IOSET0	= STCP1;
    //IOCLR0 = STCP2;
    GPIO_SetBits(GPIOE, GPIO_Pin_15);  // 关闭data 操作
    GPIO_ResetBits(GPIOE, GPIO_Pin_13); // 启动对 CMD 写入

    for(i = 0; i < 8; i++)
    {
        //IOCLR0=SHCP;
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);  //clk
        if(data & 0x80)
            //IOSET0 = DS;
            GPIO_SetBits(GPIOE, GPIO_Pin_14); // data
        else
            //IOCLR0 = DS;
            GPIO_ResetBits(GPIOE, GPIO_Pin_14); // data
        //IOSET0 = SHCP;
        GPIO_SetBits(GPIOE, GPIO_Pin_12); //clk
        data <<= 1;
    }
    //IOSET0 = STCP2;
    GPIO_SetBits(GPIOE, GPIO_Pin_13);

}


void DataBitShift(unsigned char data)
{
    unsigned char i;
    //IOCLR0 = STCP1;
    GPIO_SetBits(GPIOE, GPIO_Pin_13); //  关闭CMD 操作
    GPIO_ResetBits(GPIOE, GPIO_Pin_15); // 启动写入data
    for(i = 0; i < 8; i++)
    {
        //IOCLR0=SHCP;
        GPIO_ResetBits(GPIOE, GPIO_Pin_12); // clk
        if(data & 0x80)
            //IOSET0 = DS;
            GPIO_SetBits(GPIOE, GPIO_Pin_14); // data
        else
            //IOCLR0 = DS;
            GPIO_ResetBits(GPIOE, GPIO_Pin_14); //data
        //IOSET0 = SHCP;
        GPIO_SetBits(GPIOE, GPIO_Pin_12);
        data <<= 1;
    }
    //IOSET0 = STCP1;
    GPIO_SetBits(GPIOE, GPIO_Pin_15);

}

void lcd_clear(void)
{
    unsigned char i, j;
    for(i = 0; i < 4; i++)
    {
        lcd_out_ctl(0xb0 + i, 1);
        lcd_out_ctl(0x10, 1);
        lcd_out_ctl(0x00, 1);
        for(j = 0; j < 122; j++)
        {
            lcd_out_dat(0x00, 1);
        }
    }
}
/*
**
** low level routine to send a byte value
** to the LCD controller control register.
** entry argument is the data to output
** and the controller to use
** 1: IC 1, 2: IC 2, 3: both ICs
**
*/
void lcd_out_ctl(const unsigned char cmd, const unsigned char ncontr)
{
    unsigned char ctr = 0x00;
    unsigned int i;
    u8 cByte = 0x60;  // Q6 是背光


    if(Q7_enable == 1)
        cByte |= 0x80;
    else
        cByte &= ~0x80;
    //0.  SED1520
    if(0x03 == HardWareVerion)
    {
        ControlBitShift(RST0 | 0x0 | cByte);
        DataBitShift(cmd);
        ctr = RST0;
        if(ncontr & 0x01)
        {
            ctr |= E1;
        }
        if(ncontr & 0x02)
        {
            ctr |= E2;
        }
        ControlBitShift(ctr | cByte);
        //delay(1);
        for(i = 0; i < 0xf; i++) {}
        ControlBitShift(RST0 | 0 | cByte);

    }
    // 1. ST7565R LCD        ncontr 已经没有任何意义
    else if(0x01 == HardWareVerion)
    {
        ControlBitShift(RST0 | RD_ | 0x0 | cByte); // WR   0  RD 1
        DataBitShift(cmd);

        ctr = RST0 | WR_ | RD_; //WR_  1
        ControlBitShift(ctr | cByte);
        ControlBitShift(RST0 | WR_ | RD_ | CS_ | 0 | cByte);
    }

    //   end
}

/*
**
** low level routine to send a byte value
** to the LCD controller data register. entry argument
** is the data to output and the controller-number
**
*/
void lcd_out_dat(const unsigned char dat, const unsigned char ncontr)
{
    unsigned char ctr = 0x00;
    unsigned int i;
    u8 cByte = 0x60;

    if(Q7_enable == 1)
        cByte |= 0x80;
    else
        cByte &= ~0x80;
    // 0.  SED1520
    if(0x03 == HardWareVerion)
    {
        ctr = RST0 | A0;
        ControlBitShift(ctr | cByte);
        DataBitShift(dat);
        if(ncontr & 0x01)
        {
            ctr |= E1;
        }
        if(ncontr & 0x02)
        {
            ctr |= E2;
        }
        ControlBitShift(ctr | cByte);
        //delay(1);
        for(i = 0; i < 0xf; i++) {}
        ControlBitShift(RST0 | A0 | cByte);
    }

    // 1. ST7565R LCD
    else if(0x01 == HardWareVerion)
    {
        ctr = RST0 | A0 | RD_;
        ControlBitShift(ctr | cByte);
        DataBitShift(dat);
        ctr = RST0 | A0 | WR_ | RD_;
        ControlBitShift(ctr | cByte);
        ControlBitShift(RST0 | A0 | WR_ | RD_ | CS_ | cByte);
    }
    //  end
}


/*
**
** routine to initialize the operation of the LCD display subsystem
**
*/
void lcd_init(void)
{

    // 1. ST7565R LCD
    if(0x01 == HardWareVerion)
    {
        //-----------------------------
        ControlBitShift(0x0 | 0x60); //
        ControlBitShift(RST0 | 0x0 | 0x60); //


        lcd_out_ctl(0xe2, 3);	//soft reset开始进行软复位
        lcd_out_ctl(0xa2, 3);	//bias select确定为1/9BIAS,1/33duty
        lcd_out_ctl(0xa1, 3); //lcd_out_ctl(0xa0,3);	// ADC select确定SEG反扫seg0--->seg131      列扫描
        lcd_out_ctl(0xc0, 3); //lcd_out_ctl(0xc8,3);	//SHL select确定COM为正扫com63--->com0  行扫描
        lcd_out_ctl(0x26, 3);	//regulator resistor select V0调整电阻设定(1 + Rb/Ra)=6.5		0X26+0X41		vdd=5.0v,vop=10.0v						 0x26原来的数值
        lcd_out_ctl(0x81, 3);	//The Electronic Volume (Double Byte Command)数字电位器调节VO输出电压(也就是对比度)两字节				 0x81原来的数值
        lcd_out_ctl(ADJUST, 3);	 //vop 设置
        lcd_out_ctl(0x2c, 3);	//power control1 on内部倍压电路开启,需分以下三步打开电路
        lcd_out_ctl(0x2e, 3);	//power control2
        lcd_out_ctl(0x2f, 3);	//power control3
        lcd_out_ctl(0xf1, 3);	//The Booster Ratio (Double Byte Command)设定倍压数,两字节
        lcd_out_ctl(0x00, 3);	//"00"最大4倍压,"01"最大5倍压,"03"最大6倍压
        lcd_out_ctl(0x40, 3);	//lcd_out_ctl(0x40,3);//设定起始行第1行  32
        lcd_out_ctl(0xa4, 3);	//Display All Points ON/OFF to Normal display mode正常显示模式选择
        lcd_out_ctl(0xaf, 3);	//Display ON/OFF开显示设定
        //lcd_delay(2);
        lcd_clear();
    }
    else if(0x03 == HardWareVerion)
    {
        // 2. SED1520  LCD

        lcd_out_ctl(0, 3);
        lcd_out_ctl(LCD_RESET, 3);
        //delay_ms(1);//3

        lcd_out_ctl(LCD_DISP_ON, 3);
        lcd_out_ctl(LCD_SET_ADC_NOR, 3); // !
        lcd_out_ctl(LCD_SET_LINE + 16, 3);
        lcd_out_ctl(LCD_SET_PAGE_SED1520 + 0, 3);
        lcd_out_ctl(LCD_SET_COL, 3);
    }

}
void lcd_RstLow(void)
{
    ControlBitShift(0x60); //  开背光
}


/* fill buffer and LCD with pattern */
void lcd_fill(const unsigned char pattern)
{
    unsigned char page, col;

    if((lcd_hard_init) || (0x03 == HardWareVerion))
    {
        lcd_init();
        lcd_out_ctl(LCD_DISP_OFF, 3);
    }
    for (page = 0; page < LCD_Y_BYTES; page++)
    {
        for (col = 0; col < LCD_X_BYTES; col++)
            l_display_array[page][col] = pattern;
    }
    lcd_update_all();

    if((lcd_hard_init) || (0x03 == HardWareVerion))
        lcd_out_ctl(LCD_DISP_ON, 3);

    if(0x01 == HardWareVerion)
        lcd_hard_init = 0;
}


/*
**
** Updates area of the display. Writes data from "framebuffer"
** RAM to the lcd display controller RAM.
**
** Arguments Used:
**    top     top line of area to update.
**    bottom  bottom line of area to update.
**    from MJK-Code
**
*/
void lcd_update(const unsigned char top, const unsigned char bottom)
{
    unsigned char x;
    unsigned char y;
    unsigned char yt;
    unsigned char yb;
    unsigned char *colptr;

    /* setup bytes of range */
    yb = bottom >> 3;
    yt = top >> 3;

    for(y = yt; y <= yb; y++)
    {
        if(0x03 == HardWareVerion)
        {

            lcd_out_ctl(LCD_SET_PAGE_SED1520 + y, 3);	/* set page */
            //	   lcd_out_ctl(LCD_SET_COL+LCD_STARTCOL_REVERSE,3);
            lcd_out_ctl(LCD_SET_COL + 0, 3);

        }
        else if(0x01 == HardWareVerion)
        {

            lcd_out_ctl(LCD_SET_PAGE_ST7565R + y, 3); /* set page */
            lcd_out_ctl(LCD_SET_COLH + 0, 3); //  add by nathan
            lcd_out_ctl(LCD_SET_COL + 10, 3); // old  :lcd_out_ctl(LCD_SET_COL+0,3);   modify by nathan
        }
        colptr = &l_display_array[y][0];

        for (x = 0; x < LCD_X_BYTES; x++)
        {
            if ( x < LCD_X_BYTES / 2 )
                lcd_out_dat(*colptr++, 1);
            else
                lcd_out_dat(*colptr++, 2);
        }
    }
}



void lcd_update_all(void)
{
    lcd_update(SCRN_TOP, SCRN_BOTTOM);
}



/* sets/clears/switchs(XOR) dot at (x,y) */
void lcd_dot(const unsigned char x, const unsigned char y, const unsigned char mode)
{
    unsigned char bitnum, bitmask, yByte;
    unsigned char *pBuffer; /* pointer used for optimisation */

    if ( ( x > SCRN_RIGHT ) || ( y > SCRN_BOTTOM ) ) return;

    yByte   = y >> 3;
    bitnum  = y & 0x07;
    bitmask = l_mask_array[bitnum]; // bitmask = ( 1 << (y & 0x07) );
    pBuffer = &(l_display_array[yByte][x]);
    switch (mode)
    {
    case LCD_MODE_SET:
        *pBuffer |= bitmask;
        break;
    case LCD_MODE_CLEAR:
        *pBuffer &= ~bitmask;
        break;
    case LCD_MODE_XOR:
        *pBuffer ^= bitmask;
        break;
    case LCD_MODE_INVERT:
        if((*pBuffer) & bitmask > 0)
        {
            *pBuffer &= ~bitmask;
        }
        else
        {
            *pBuffer |= bitmask;
        }
        break;
    default:
        break;
    }
}


/* line- and circle-function from a KS0108-library by F. Thiele */

void lcd_line( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, const uint8_t mode )
{
    uint8_t length, xTmp, yTmp, i, y, yAlt;
    int16_t m;

    if(x1 == x2)
    {
        // vertical line
        // x1|y1 must be the upper point
        if(y1 > y2)
        {
            xTmp = x1;
            yTmp = y1;
            x1 = x2;
            y1 = y2;
            x2 = xTmp;
            y2 = yTmp;
        }
        length = y2 - y1;
        for(i = 0; i <= length; i++)
            lcd_dot(x1, y1 + i, mode);
    }
    else if(y1 == y2)
    {
        // horizontal line
        // x1|y1 must be the left point
        if(x1 > x2)
        {
            xTmp = x1;
            yTmp = y1;
            x1 = x2;
            y1 = y2;
            x2 = xTmp;
            y2 = yTmp;
        }

        length = x2 - x1;
        for(i = 0; i <= length; i++)
            lcd_dot(x1 + i, y1, mode);

    }
    else
    {
        // x1 must be smaller than x2
        if(x1 > x2)
        {
            xTmp = x1;
            yTmp = y1;
            x1 = x2;
            y1 = y2;
            x2 = xTmp;
            y2 = yTmp;
        }

        if((y2 - y1) >= (x2 - x1) || (y1 - y2) >= (x2 - x1))
        {
            // angle larger or equal 45?
            length = x2 - x1;								// not really the length :)
            m = ((y2 - y1) * 200) / length;
            yAlt = y1;
            for(i = 0; i <= length; i++)
            {
                y = ((m * i) / 200) + y1;
                if((m * i) % 200 >= 100)
                    y++;
                else if((m * i) % 200 <= -100)
                    y--;

                lcd_line(x1 + i, yAlt, x1 + i, y, mode ); /* wuff wuff recurs. */
                if(length <= (y2 - y1) && y1 < y2)
                    yAlt = y + 1;
                else if(length <= (y1 - y2) && y1 > y2)
                    yAlt = y - 1;
                else
                    yAlt = y;
            }
        }
        else
        {
            // angle smaller 45?
            // y1 must be smaller than y2
            if(y1 > y2)
            {
                xTmp = x1;
                yTmp = y1;
                x1 = x2;
                y1 = y2;
                x2 = xTmp;
                y2 = yTmp;
            }
            length = y2 - y1;
            m = ((x2 - x1) * 200) / length;
            yAlt = x1;
            for(i = 0; i <= length; i++)
            {
                y = ((m * i) / 200) + x1;

                if((m * i) % 200 >= 100)
                    y++;
                else if((m * i) % 200 <= -100)
                    y--;

                lcd_line(yAlt, y1 + i, y, y1 + i, mode); /* wuff */
                if(length <= (x2 - x1) && x1 < x2)
                    yAlt = y + 1;
                else if(length <= (x1 - x2) && x1 > x2)
                    yAlt = y - 1;
                else
                    yAlt = y;
            }
        }
    }
}

#if  0
void lcd_circle(const uint8_t xCenter, const uint8_t yCenter, const uint8_t radius, const uint8_t mode)
{
    int16_t tSwitch, y, x = 0;
    uint8_t d;

    d = yCenter - xCenter;
    y = radius;
    tSwitch = 3 - 2 * radius;

    while (x <= y)
    {
        lcd_dot(xCenter + x, yCenter + y, mode);
        lcd_dot(xCenter + x, yCenter - y, mode);

        lcd_dot(xCenter - x, yCenter + y, mode);
        lcd_dot(xCenter - x, yCenter - y, mode);

        lcd_dot(yCenter + y - d, yCenter + x, mode);
        lcd_dot(yCenter + y - d, yCenter - x, mode);

        lcd_dot(yCenter - y - d, yCenter + x, mode);
        lcd_dot(yCenter - y - d, yCenter - x, mode);

        if (tSwitch < 0)
            tSwitch += (4 * x + 6);
        else
        {
            tSwitch += (4 * (x - y) + 10);
            y--;
        }
        x++;
    }
}


void lcd_rect(const uint8_t x, const uint8_t y, uint8_t width, uint8_t height, const uint8_t mode)
{
    width--;
    height--;
    lcd_line(x, y, x + width, y, mode);	// top
    lcd_line(x, y, x, y + height, mode);	// left
    lcd_line(x, y + height, x + width, y + height, mode);	// bottom
    lcd_line(x + width, y, x + width, y + height, mode);		// right
}

void lcd_box(const uint8_t x, const uint8_t y, uint8_t width, const uint8_t height, const uint8_t mode)
{
    uint8_t i;
    if (!width) return;

    width--;

    for (i = y; i < y + height; i++)
        lcd_line(x, i, x + width, i, mode);
}
#endif

/*
 Writes a glyph("letter") to the display at location x,y
 (adapted function from the MJK-code)
 Arguments are:
    column    - x corrdinate of the left part of glyph
    row       - y coordinate of the top part of glyph
    width     - size in pixels of the width of the glyph
    height    - size in pixels of the height of the glyph
    glyph     - an unsigned char pointer to the glyph pixels
                to write assumed to be of length "width"
*/

void lcd_glyph(uint8_t left, uint8_t top, uint8_t width, uint8_t height, uint8_t *glyph_ptr, uint8_t store_width)
{
    uint8_t bit_pos;
    uint8_t byte_offset;
    uint8_t y_bits;
    uint8_t remaining_bits;
    uint8_t mask;
    uint8_t char_mask;
    uint8_t x;
    uint8_t *glyph_scan;
    uint8_t glyph_offset;

    bit_pos = top & 0x07;		/* get the bit offset into a byte */
    glyph_offset = 0;			/* start at left side of the glyph rasters */
    char_mask = 0x80;			/* initial character glyph mask */

    for (x = left; x < (left + width); x++)
    {
        byte_offset = top >> 3;        	/* get the byte offset into y direction */
        y_bits = height;		/* get length in y direction to write */
        remaining_bits = 8 - bit_pos;	/* number of bits left in byte */
        mask = l_mask_array[bit_pos];	/* get mask for this bit */
        glyph_scan = glyph_ptr + glyph_offset;	 /* point to base of the glyph */
        /* boundary checking here to account for the possibility of  */
        /* write past the bottom of the screen.                        */
        while((y_bits) && (byte_offset < LCD_Y_BYTES)) /* while there are bits still to write */
        {
            /* check if the character pixel is set or not */
            //if(*glyph_scan & char_mask)
            if(pgm_read_byte(glyph_scan) & char_mask)
                l_display_array[byte_offset][x] |= mask;	/* set image pixel */
            else
                l_display_array[byte_offset][x] &= ~mask;	/* clear the image pixel */

            if(l_mask_array[0] & 0x80)
                mask >>= 1;
            else
                mask <<= 1;

            y_bits--;
            remaining_bits--;
            if(remaining_bits == 0)
            {
                /* just crossed over a byte boundry, reset byte counts */
                remaining_bits = 8;
                byte_offset++;
                mask = l_mask_array[0];
            }
            /* bump the glyph scan to next raster */
            glyph_scan += store_width;
        }

        /* shift over to next glyph bit */
        char_mask >>= 1;
        if(char_mask == 0)				/* reset for next byte in raster */
        {
            char_mask = 0x80;
            glyph_offset++;
        }
    }
}



void lcd_bitmap(const uint8_t left, const uint8_t top, const struct IMG_DEF *img_ptr, const uint8_t mode)
{
    uint8_t width, heigth, h, w, pattern, mask;
    uint8_t *ptable;

    uint8_t bitnum, bitmask;
    uint8_t page, col, vdata;

    width  = img_ptr->width_in_pixels;
    heigth = img_ptr->height_in_pixels;
    ptable  = (uint8_t *)(img_ptr->char_table);


    mask = 0x80;
    pattern = *ptable;

    for ( h = 0; h < heigth; h++ ) /**/
    {
        page = (h + top) >> 3;
        bitnum = (h + top) & 0x07;
        bitmask = (1 << bitnum);
        for ( w = 0; w < width; w++ )
        {
            col = left + w;
            vdata = l_display_array[page][col];
            switch(mode)
            {
            case LCD_MODE_SET:		/*不管原来的数据，直接设置为pattern的值*/
                if (pattern & mask)
                    vdata |= bitmask;
                else
                    vdata &= ~bitmask;
                break;
            case LCD_MODE_CLEAR:	/*不管原来的数据，清除原来的值=>0*/
                vdata &= ~bitmask;
                break;
            case LCD_MODE_XOR:		/*原来的数据，直接设置为pattern的值*/
                if(vdata & bitmask)
                {
                    if(pattern & mask)	vdata &= ~bitmask;
                    else vdata |= bitmask;
                }
                else
                {
                    if(pattern & mask)	vdata |= bitmask;
                    else vdata &= ~bitmask;
                }
                break;
            case LCD_MODE_INVERT:	/*不管原来的数据，直接设置为pattern的值*/
                if (pattern & mask)
                    vdata &= ~bitmask;
                else
                    vdata |= bitmask;
                break;

            }
            l_display_array[page][col] = vdata;
            mask >>= 1;
            if ( mask == 0 )
            {
                mask = 0x80;
                ptable++;
                pattern = *ptable;

            }
        }
        if(mask != 0x80) 		/*一行中的列已处理完*/
        {
            mask = 0x80;
            ptable++;
            pattern = *ptable;
        }
    }
}


/*
绘制12点阵的字符，包括中文和英文


*/
void lcd_text12(char left, char top , char *p, char len, const char mode)
{
    int charnum = len;
    int i;
    char msb, lsb;

    int addr = 0;
    unsigned char start_col = left;
    unsigned int  val_old, val_new, val_mask;

    unsigned int glyph[12];   /*保存一个字符的点阵信息，以逐列式*/
    unsigned char 	font_buf[32];

    DF_TAKE;
    while( charnum )
    {
        for( i = 0; i < 12; i++ )
        {
            glyph[i] = 0;
        }
        msb = *p++;
        charnum--;
        if( msb <= 0x80 ) //ascii字符 0612
        {

            // select      font  on chip
            //addr = ( msb - 0x20 ) * 12 + FONT_ASC0612_ADDR;

            //  slect    font  on   8Mbytes  Dataflash
            addr = ( msb - 0x20 ) * 12 + FONT_ASC0612_ADDR;
            SST25V_BufferRead( font_buf, addr, 12);
            addr = (int)font_buf;

            for( i = 0; i < 3; i++ )
            {
                val_new				= *(__IO uint32_t *)addr;
                glyph[i * 2 + 0]	= ( val_new & 0xffff );
                glyph[i * 2 + 1]	= ( val_new & 0xffff0000 ) >> 16;
                addr						+= 4;
            }

            val_mask = ((0xfff) << top); /*12bit*/

            /*加上top的偏移*/
            for( i = 0; i < 6; i++ )
            {
                glyph[i] <<= top;

                val_old = l_display_array[0][start_col] | (l_display_array[1][start_col] << 8) | (l_display_array[2][start_col] << 16) | (l_display_array[3][start_col] << 24);
                if(mode == LCD_MODE_SET)
                {
                    val_new = val_old & (~val_mask) | glyph[i];
                }
                else if(mode == LCD_MODE_INVERT)
                {
                    val_new = (val_old | val_mask) & (~glyph[i]);
                }
                l_display_array[0][start_col] = val_new & 0xff;
                l_display_array[1][start_col] = (val_new & 0xff00) >> 8;
                l_display_array[2][start_col] = (val_new & 0xff0000) >> 16;
                l_display_array[3][start_col] = (val_new & 0xff000000) >> 24;
                start_col++;
            }
        }
        else
        {
            lsb = *p++;
            charnum--;
            if( ( msb >= 0xa1 ) && ( msb <= 0xa3 ) && ( lsb >= 0xa1 ) )
            {
                // OUT flash
                addr = FONT_HZ1212_ADDR + ( ( ( (unsigned long)msb ) - 0xa1 ) * 94 + ( ( (unsigned long)lsb ) - 0xa1 ) ) * 24;
                SST25V_BufferRead( font_buf, addr, 24);
                addr = (int)font_buf;
                ///CPU FLASH
                //addr = FONT_HZ1212_ADDR + ( ( ( (unsigned long)msb ) - 0xa1 ) * 94 + ( ( (unsigned long)lsb ) - 0xa1 ) ) * 24;
            }
            else if( ( msb >= 0xb0 ) && ( msb <= 0xf7 ) && ( lsb >= 0xa1 ) )
            {
                ///OUT FLASH
                addr = FONT_HZ1212_ADDR + ( ( ( (unsigned long)msb ) - 0xb0 ) * 94 + ( ( (unsigned long)lsb ) - 0xa1 ) ) * 24 + 282 * 24;
                SST25V_BufferRead( font_buf, addr, 24);
                addr = (int)font_buf;
                ///CPU FLASH
                //addr = FONT_HZ1212_ADDR + ( ( ( (unsigned long)msb ) - 0xb0 ) * 94 + ( ( (unsigned long)lsb ) - 0xa1 ) ) * 24 + 282 * 24;
            }
            for( i = 0; i < 6; i++ )
            {
                val_new				= *(__IO uint32_t *)addr;
                glyph[i * 2 + 0]	= ( val_new & 0xffff );
                glyph[i * 2 + 1]	= ( val_new & 0xffff0000 ) >> 16;
                addr				+= 4;
            }
            val_mask = ((0xfff) << top); /*12bit*/

            /*加上top的偏移*/
            for( i = 0; i < 12; i++ )
            {
                glyph[i] <<= top;
                /*通过start_col映射到I_display_array中，注意mask*/
                val_old = l_display_array[0][start_col] | (l_display_array[1][start_col] << 8) | (l_display_array[2][start_col] << 16) | (l_display_array[3][start_col] << 24);
                if(mode == LCD_MODE_SET)
                {
                    val_new = val_old & (~val_mask) | glyph[i];
                }
                else if(mode == LCD_MODE_INVERT)
                {
                    val_new = (val_old | val_mask) & (~glyph[i]);
                }
                l_display_array[0][start_col] = val_new & 0xff;
                l_display_array[1][start_col] = (val_new & 0xff00) >> 8;
                l_display_array[2][start_col] = (val_new & 0xff0000) >> 16;
                l_display_array[3][start_col] = (val_new & 0xff000000) >> 24;
                start_col++;
            }

        }
    }


    DF_RELEASE;

}

void lcd_text12_local(char left, char top , char *p, char len, const char mode)
{
    int charnum = len;
    int i;
    char msb, lsb;
    u16  GB_Code = 0;

    int addr = 0;
    unsigned char start_col = left;
    unsigned int  val_old, val_new, val_mask;

    unsigned int glyph[12];   /*保存一个字符的点阵信息，以逐列式*/
    unsigned char 	font_buf[32];

    DF_TAKE;
    while( charnum )
    {
        for( i = 0; i < 12; i++ )
        {
            glyph[i] = 0;
        }
        msb = *p++;
        charnum--;
        if( msb <= 0x80 ) //ascii字符 0612
        {
            ;
        }
        else
        {
            lsb = *p++;
            charnum--;
            GB_Code = (msb << 8) + lsb;
            //----------------------------------------------------------
            switch(GB_Code)
            {
            case  0xC9E8:
                //  设C9E8
                memcpy(font_buf, Dot_She, 24);
                break;
            case  0xB1B8:
                // 备B1B8
                memcpy(font_buf, Dot_Bei, 24);
                break;

            case  0xBCEC:
                //检 BCEC
                memcpy(font_buf, Dot_Jian, 24);
                break;

            case  0xD7D4:
                //自 D7D4
                memcpy(font_buf, Dot_Zi, 24);
                break;
            case  0xD5FD:
                // 正D5FD
                memcpy(font_buf, Dot_Zheng, 24);
                break;

            case  0xB3A3:
                //  常 B3A3
                memcpy(font_buf, Dot_Chang, 24);
                break;

            case  0xCCEC:
                // 天CCEC
                memcpy(font_buf, Dot_Tian, 24);
                break;

            case  0xCFDF:
                // 线CFDF
                memcpy(font_buf, Dot_Xian, 24);
                break;
            case  0xBFAA:
                // 开BFAA
                memcpy(font_buf, Dot_Kai, 24);
                break;
            case  0xB6CC:
                //路 B6CC
                memcpy(font_buf, Dot_Lu, 24);
                break;

            case  0xC2B7:
                //短C2B7
                memcpy(font_buf, Dot_Duan, 24);
                break;

            case  0xC4DA:
                //  内C4DA
                memcpy(font_buf, Dot_Nei, 24);
                break;

            case  0xB2BF:
                // 部B2BF
                memcpy(font_buf, Dot_Bu, 24);
                break;

            case  0xB5E7:
                // 电B5E7
                memcpy(font_buf, Dot_Dian, 24);
                break;

            case  0xB3D8:
                // 池B3D8
                memcpy(font_buf, Dot_Chi, 24);
                break;

            case  0xB9A9:
                // 供B9A9
                memcpy(font_buf, Dot_Gong, 24);
                break;

            default:
                // .
                memcpy(font_buf, Dot_dot, 24);
                break;
            }

            addr = (int)font_buf;
            // OutPrint_HEX("汉字 点阵x",font_buf,24);
            //-----------------------------------------------------------
            for( i = 0; i < 6; i++ )
            {
                val_new				= *(__IO uint32_t *)addr;
                glyph[i * 2 + 0]	= ( val_new & 0xffff );
                glyph[i * 2 + 1]	= ( val_new & 0xffff0000 ) >> 16;
                addr				+= 4;
            }
            val_mask = ((0xfff) << top); /*12bit*/

            /*加上top的偏移*/
            for( i = 0; i < 12; i++ )
            {
                glyph[i] <<= top;
                /*通过start_col映射到I_display_array中，注意mask*/
                val_old = l_display_array[0][start_col] | (l_display_array[1][start_col] << 8) | (l_display_array[2][start_col] << 16) | (l_display_array[3][start_col] << 24);
                if(mode == LCD_MODE_SET)
                {
                    val_new = val_old & (~val_mask) | glyph[i];
                }
                else if(mode == LCD_MODE_INVERT)
                {
                    val_new = (val_old | val_mask) & (~glyph[i]);
                }
                l_display_array[0][start_col] = val_new & 0xff;
                l_display_array[1][start_col] = (val_new & 0xff00) >> 8;
                l_display_array[2][start_col] = (val_new & 0xff0000) >> 16;
                l_display_array[3][start_col] = (val_new & 0xff000000) >> 24;
                start_col++;
            }

        }
    }


    DF_RELEASE;

}


/*
      ST7565R  LCD  定时初始化液晶一次     放到1s 定时器那里
*/
void Lcd_hardInit_timer(void)
{
    if(0x01 == HardWareVerion) // 新的硬件版本
    {
        lcd_hardIint_couter++;
        if(lcd_hardIint_couter > 3600) //3600
        {
            lcd_hard_init = 1;
            lcd_hardIint_couter = 0;
        }
    }
    else if(0x03 == HardWareVerion)
        lcd_hard_init = 1;
}

