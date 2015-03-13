#include "Menu_Include.h"
#include <string.h>


#define  DIS_Dur_width_inter 10


unsigned char noselect_recorder[] = {0x3C, 0x7E, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x3C}; //空心
unsigned char select_recorder[] = {0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C}; //实心
DECL_BMP(8, 8, select_recorder);
DECL_BMP(8, 8, noselect_recorder);


static unsigned char menu_pos = 0;

static PMENUITEM psubmenu[8] =
{
    &Menu_4_1_pilao,     //超时驾驶记录
    &Menu_5_5_CarInfor,  //车辆信息
    &Menu_5_4_ICcard,    //驾驶员信息
    &Menu_5_6_speedlog,  //速度日志信息
    &Menu_5_2_ExportData,//记录仪数据导出
    &Menu_5_1_PulseCoefficient,//脉冲设置
    &Menu_5_3_print,     //打印输出
    &Menu_2_6_Mileage,   //里程查看
};


static void menuswitch(void)
{
    unsigned char i = 0;

    lcd_fill(0);
    lcd_text12(0, 3, "记录仪", 6, LCD_MODE_SET);
    lcd_text12(6, 17, "信息", 4, LCD_MODE_SET);
    for(i = 0; i < 8; i++)
        lcd_bitmap(40 + i * DIS_Dur_width_inter, 5, &BMP_noselect_recorder, LCD_MODE_SET);
    lcd_bitmap(40 + menu_pos * DIS_Dur_width_inter, 5, &BMP_select_recorder, LCD_MODE_SET);
    lcd_text12(37, 19, (char *)(psubmenu[menu_pos]->caption), psubmenu[menu_pos]->len, LCD_MODE_SET);
    lcd_update_all();
}


static void msg( void *p)
{
}
static void show(void)
{
    MenuIdle_working = 0; //clear

    menu_pos = 0;
    menuswitch();
}



static void keypress(unsigned int key)
{
    switch(KeyValue)
    {
    case KeyValueMenu:
        Mileage_02_05_flag = 0;
        CounterBack = 0;
        pMenuItem = &Menu_1_menu; //
        pMenuItem->show();
        break;
    case KeyValueOk:
        if(menu_pos == 7)
            Mileage_02_05_flag = 1; //跳转到里程
        else if(menu_pos)
            Mileage_02_05_flag = 2; //跳转到超时驾驶
        pMenuItem = psubmenu[menu_pos]; //
        pMenuItem->show();
        break;
    case KeyValueUP:
        if(menu_pos == 0)
            menu_pos = 7;
        else
            menu_pos--;
        menuswitch();
        break;
    case KeyValueDown:
        menu_pos++;
        if(menu_pos > 7)
            menu_pos = 0;
        menuswitch();
        break;
    }
    KeyValue = 0;
}




static void timetick(unsigned int systick)
{

    CounterBack++;
    if(CounterBack != MaxBankIdleTime)
        return;
    pMenuItem = &Menu_1_Idle;
    pMenuItem->show();
    CounterBack = 0;

}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_5_recorder =
{
    "记录仪信息",
    10,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};


