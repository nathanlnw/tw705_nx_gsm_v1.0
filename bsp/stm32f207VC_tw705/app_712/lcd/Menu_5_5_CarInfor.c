#include "Menu_Include.h"
#include <string.h>


u8 Car_infor_num = 0;

static void Car_infor_fun(u8 par)
{
    u8 color_disp[6];


    if(par == 1)
    {
        lcd_fill(0);
        lcd_text12(0, 3, "车辆识别代码:", 13, LCD_MODE_SET);
        lcd_text12(10, 18, Vechicle_Info.Vech_VIN, 17, LCD_MODE_SET);
        lcd_update_all();
    }
    else if(par == 2)
    {
        lcd_fill(0);
        lcd_text12(0, 3, "机动车号牌号码:", 15, LCD_MODE_SET);

        if(License_Not_SetEnable == 1)
            lcd_text12(0, 18, "无牌照  ", 8, LCD_MODE_SET);
        else
            lcd_text12(0, 18, Vechicle_Info.Vech_Num, 8, LCD_MODE_SET);
        lcd_update_all();
    }
    else if(par == 3)
    {
        //车辆颜色
        memset(color_disp, 0, sizeof(color_disp));
        switch(Vechicle_Info.Dev_Color)
        {
        case 1:
            memcpy(color_disp, "蓝色", 4);
            break;
        case 2:
            memcpy(color_disp, "黄色", 4);
            break;
        case 3:
            memcpy(color_disp, "黑色", 4);
            break;
        case 4:
            memcpy(color_disp, "白色", 4);
            break;
        case 9:
            memcpy(color_disp, "其他", 4);
            break;
        default:
            memcpy(color_disp, "黄色", 4);
            break;
        }
        lcd_fill(0);
        lcd_text12(0, 3, "机动车号牌分类:", 15, LCD_MODE_SET);
        if(License_Not_SetEnable == 1) //====  车牌号未设置=====
            lcd_text12(64, 19, (char *)"0", 1, LCD_MODE_SET);
        else
            lcd_text12(64, 19, (char *)color_disp, 4, LCD_MODE_SET);
        lcd_update_all();
    }
    lcd_update_all();
}

static void msg( void *p)
{
}
static void show(void)
{
    MenuIdle_working = 0; //clear

    Car_infor_num = 1;
    Car_infor_fun(Car_infor_num);
}



static void keypress(unsigned int key)
{
    switch(KeyValue)
    {
    case KeyValueMenu:
        Car_infor_num = 0;
        CounterBack = 0;
        pMenuItem = &Menu_5_recorder;
        pMenuItem->show();
        break;
    case KeyValueOk:
        break;
    case KeyValueUP:
        if(Car_infor_num == 1)
            Car_infor_num = 3;
        else
            Car_infor_num--;
        Car_infor_fun(Car_infor_num);
        break;
    case KeyValueDown:
        if(Car_infor_num >= 3)
            Car_infor_num = 1;
        else
            Car_infor_num++;
        Car_infor_fun(Car_infor_num);
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
MENUITEM	Menu_5_5_CarInfor =
{
    "车辆信息",
    8,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};

