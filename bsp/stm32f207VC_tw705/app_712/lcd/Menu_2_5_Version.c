#include "Menu_Include.h"

u8 version_screen = 0;
static void msg( void *p)
{

}
static void show(void)
{
    MenuIdle_working = 0; //clear
    version_disp(0);

}


static void keypress(unsigned int key)
{

    switch(KeyValue)
    {
    case KeyValueMenu:
        pMenuItem = &Menu_2_InforCheck;
        pMenuItem->show();
        CounterBack = 0;

        version_screen = 0;
        break;
    case KeyValueOk:
        version_disp(0);
        break;
    case KeyValueUP:
        version_disp(0);
        break;
    case KeyValueDown:
        version_disp(1);
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
MENUITEM	Menu_2_5_Version =
{
    "∞Ê±æœ‘ æ",
    8,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};

