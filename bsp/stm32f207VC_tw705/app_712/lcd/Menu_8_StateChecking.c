#include  <string.h>
#include "Menu_Include.h"

static u8 Dis_spd[20] = {"      000km/h     "};
static u8 tickcount = 0;
static void msg( void *p)
{

}
static void disp_checking(void)
{
    u16 disp_spd = 0;


    Dis_spd[0] = ' ';
    disp_spd = Speed_cacu / 10;
    if((disp_spd >= 100) && (disp_spd < 999))
    {
        Dis_spd[6] = disp_spd / 100 + '0';
        Dis_spd[7] = (disp_spd % 100) / 10 + '0';
        Dis_spd[8] = disp_spd % 10 + '0';


    }
    else if((disp_spd >= 10) && (disp_spd < 100))
    {
        Dis_spd[6] = ' ';
        Dis_spd[7] = (disp_spd / 10) + '0';
        Dis_spd[8] = disp_spd % 10 + '0';
    }
    else if(disp_spd < 10)
    {
        Dis_spd[6] = ' ';
        Dis_spd[7] = ' ';
        Dis_spd[8] = disp_spd % 10 + '0';
    }
    //-----------------------------------------------------
    lcd_fill(0);
    lcd_text12(0, 3, "ÖÕ¶Ë¼ì¶¨×´Ì¬", 12, LCD_MODE_SET);
    lcd_text12(0, 20, (char *)Dis_spd, 18, LCD_MODE_SET);
    lcd_update_all();
}

static void show(void)
{
    MenuIdle_working = 0; //clear
    disp_checking();
}

static void keypress(unsigned int key)
{
    /*
    switch(KeyValue)
    	{
    	case KeyValueMenu:
    	case KeyValueOk:
    	case KeyValueUP:
    	case KeyValueDown:
    		   pMenuItem=&Menu_1_Idle;
    		   pMenuItem->show();
    		   break;
    	}
    KeyValue=0;*/
}


static void timetick(unsigned int systick)
{
    //Ñ­»·ÏÔÊ¾´ý»ú½çÃæ
    tickcount++;
    if(tickcount >= 5)
    {
        tickcount = 0;
        disp_checking();
    }

}


MENUITEM	Menu_8_statechecking =
{
    "¼ì¶¨×´Ì¬",
    8,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};


