#include "Menu_Include.h"
#include <string.h>

u8 IC_card_num = 0;

static void driver_IC_Card(u8 par)
{
    u8  reg_str[12];
    lcd_fill(0);
    if(par == 1)
    {
        lcd_text12(0, 3, "驾驶员姓名:", 11, LCD_MODE_SET);
        lcd_text12(45, 18, (char *)JT808Conf_struct.Driver_Info.DriveName, strlen((char *)JT808Conf_struct.Driver_Info.DriveName), LCD_MODE_SET);
    }
    else if(par == 2)
    {
        lcd_text12(0, 3, "机动车驾驶证号码:", 17, LCD_MODE_SET);
        lcd_text12(6, 18, (char *)Read_ICinfo_Reg.DriverCard_ID, 18, LCD_MODE_SET);
    }
    else if(par == 3)
    {
        lcd_text12(0, 3, "驾驶证有效期:", 13, LCD_MODE_SET);
        memset(reg_str, 0, sizeof(reg_str));
        sprintf(reg_str, "20%d%d-%d%d-%d%d", (Read_ICinfo_Reg.Effective_Date[0] >> 4), (Read_ICinfo_Reg.Effective_Date[0] & 0x0f), (Read_ICinfo_Reg.Effective_Date[1] >> 4), (Read_ICinfo_Reg.Effective_Date[1] & 0x0f), (Read_ICinfo_Reg.Effective_Date[2] >> 4), (Read_ICinfo_Reg.Effective_Date[2] & 0x0f));
        lcd_text12(0, 18, reg_str, 10, LCD_MODE_SET);
    }
    else if(par == 4)
    {
        lcd_text12(0, 3, "从业资格证号:", 13, LCD_MODE_SET);
        lcd_text12(6, 19, (char *)Read_ICinfo_Reg.Drv_CareerID, 18, LCD_MODE_SET);
    }
    lcd_update_all();
}

static void msg( void *p)
{
}
static void show(void)
{
    MenuIdle_working = 0; //clear
    MenuIdle_working = 0; //clear

    IC_card_num = 1;
    driver_IC_Card(IC_card_num);
}



static void keypress(unsigned int key)
{
    switch(KeyValue)
    {
    case KeyValueMenu:
        IC_card_num = 0;
        CounterBack = 0;
        pMenuItem = &Menu_5_recorder;
        pMenuItem->show();
        break;
    case KeyValueOk:
        break;
    case KeyValueUP:
        if(IC_card_num == 1)
            IC_card_num = 4;
        else
            IC_card_num--;
        driver_IC_Card(IC_card_num);
        break;
    case KeyValueDown:
        if(IC_card_num >= 4)
            IC_card_num = 1;
        else
            IC_card_num++;
        driver_IC_Card(IC_card_num);
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
MENUITEM	Menu_5_4_ICcard =
{
    "驾驶员信息",
    10,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};

