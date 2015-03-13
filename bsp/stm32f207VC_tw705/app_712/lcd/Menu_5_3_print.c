#include "Menu_Include.h"
#include <string.h>

u8 printf_screen = 0;
u16 printf_num = 0;
static void msg( void *p)
{
}
static void show(void)
{
    MenuIdle_working = 0; //clear

    lcd_fill(0);
    lcd_text12(24, 10, "按确认键打印", 12, LCD_MODE_SET);
    lcd_update_all();
}



static void keypress(unsigned int key)
{
    switch(KeyValue)
    {
    case KeyValueMenu:
        printf_screen = 0;
        printf_num = 0;
        CounterBack = 0;

        pMenuItem = &Menu_5_recorder;
        pMenuItem->show();
        break;
    case KeyValueOk:
        Dayin_Fun(1);
        break;
    case KeyValueUP:
        break;
    case KeyValueDown:
        break;
    }
    KeyValue = 0;
}




static void timetick(unsigned int systick)
{

}

ALIGN(RT_ALIGN_SIZE)
MENUITEM	Menu_5_3_print =
{
    "打印输出",
    8,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};

