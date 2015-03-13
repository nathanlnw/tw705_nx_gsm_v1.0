#include "Menu_Include.h"

u8 menu_screen = 0;
u8 menu_flag = 0;

static void menu_sel(u8 par)
{
    lcd_fill(0);
    switch(par)
    {
    case 1:
        lcd_text12(0, 3, "1.记录仪信息", 12, LCD_MODE_INVERT);
        lcd_text12(0, 18, "2.信息查看", 10, LCD_MODE_SET);
        break;
    case 2:
        lcd_text12(0, 3, "1.记录仪相关", 12, LCD_MODE_SET);
        lcd_text12(0, 18, "2.信息查看", 10, LCD_MODE_INVERT);
        break;
    case 3:
        lcd_text12(0, 3, "3.信息交互", 10, LCD_MODE_INVERT);
        lcd_text12(0, 18, "4.车辆设置", 10, LCD_MODE_SET);
        break;
    case 4:
        lcd_text12(0, 3, "3.信息交互", 10, LCD_MODE_SET);
        lcd_text12(0, 18, "4.车辆设置", 10, LCD_MODE_INVERT);
        break;
    }
    lcd_update_all();
}



static void msg( void *p)
{

}
static void show(void)
{
    MenuIdle_working = 0; //clear
    if(menu_screen == 0)
    {
        menu_screen = 1;
        menu_flag = 1;
        menu_sel(menu_flag);
    }

}


static void keypress(unsigned int key)
{

    switch(KeyValue)
    {
    case KeyValueMenu:
        pMenuItem = &Menu_1_Idle;
        pMenuItem->show();
        CounterBack = 0;

        menu_screen = 0;
        menu_flag = 0;
        break;
    case KeyValueOk:
        if(menu_screen == 1)
        {
            menu_screen = 0;
            if(menu_flag == 1)
            {
                menu_flag = 0;
                pMenuItem = &Menu_5_recorder;
                pMenuItem->show();
            }
            else if(menu_flag == 2)
            {
                menu_flag = 0;
                pMenuItem = &Menu_2_InforCheck;
                pMenuItem->show();
            }
            else if(menu_flag == 3)
            {
                menu_flag = 0;
                pMenuItem = &Menu_3_InforInteract;
                pMenuItem->show();
            }
            else if(menu_flag == 4)
            {
                menu_flag = 0;
                pMenuItem = &Menu_6_SetInfor;
                pMenuItem->show();
            }

        }
        break;
    case KeyValueUP:
        if(menu_screen == 1)
        {
            if(menu_flag == 1)
                menu_flag = 4;
            else
                menu_flag--;
            menu_sel(menu_flag);
        }
        break;
    case KeyValueDown:
        if(menu_screen == 1)
        {
            if(menu_flag >= 4)
                menu_flag = 1;
            else
                menu_flag++;
            menu_sel(menu_flag);
        }
        break;
    }
    KeyValue = 0;
}


static void timetick(unsigned int systick)
{
    CounterBack++;
    if(CounterBack != MaxBankIdleTime * 5)
        return;
    CounterBack = 0;
    pMenuItem = &Menu_1_Idle;
    pMenuItem->show();



}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_1_menu =
{
    "一级菜单",
    8,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};
