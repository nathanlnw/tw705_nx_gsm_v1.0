#include  <string.h>
#include "Menu_Include.h"


static void msg( void *p)
{

}
static void show(void)
{
    MenuIdle_working = 0; //clear
    //rt_kprintf("\r\n------------------打印缺纸----------");
    lcd_fill(0);
    switch(Menu_txt_state)
    {
    case 1:
        lcd_text12(20, 10, "打印缺纸", 8, LCD_MODE_SET);
        break;
    case 2:
        lcd_text12(20, 10, "IC卡类型不匹配", 14, LCD_MODE_SET);
        break;
    case 3:
        lcd_text12(20, 10, "非标准IC卡", 10, LCD_MODE_SET);
        break;
    case 4:
        lcd_text12(0, 10, "USB所有数据导出完成!", 20, LCD_MODE_SET);
        break;
    case 5:
        lcd_text12(20, 10, "打印中...", 9, LCD_MODE_SET);
        break;
    }
    lcd_update_all();

    Menu_txt_state = 0; // clear
}


static void keypress(unsigned int key)
{
    switch(KeyValue)
    {
    case KeyValueMenu:
    case KeyValueOk:
    case KeyValueUP:
    case KeyValueDown:
        pMenuItem = &Menu_1_Idle;
        pMenuItem->show();
        break;
    }
    KeyValue = 0;
}


static void timetick(unsigned int systick)
{
    if(Menu_txt_state != 5)
    {
        CounterBack++;
        if(CounterBack != 20)
            return;
        pMenuItem = &Menu_1_Idle;
        pMenuItem->show();
        CounterBack = 0;
    }
}


MENUITEM	Menu_TXT =
{
    "Menu_TXT",
    8,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};


