#include  <string.h>
#include "Menu_Include.h"
#include "Lcd.h"
#include <string.h>


static u8 Dis_num = 0;
static u8 Dis_maxNum = 0;
static u8 Dis_status[25] = {"1.超时驾驶记录:"};
static u8 Dis_start_time[25] = {"14-06-02 10:20:15"};
static u8 Dis_end_time[25]  = {"14-06-02 15:18:32"};
static u8 get_indexnum = 0;
static u8 H11_reg[50];


void  disp_record_11h(u8 record_num)
{

    get_11h(get_indexnum - record_num, H11_reg);

    memcpy(Dis_status + 2, H11_reg, 18);
    sprintf(Dis_start_time, "st:%2X-%2X-%2X %2X:%2X:%2X", H11_reg[18], H11_reg[19], H11_reg[20], H11_reg[21], H11_reg[22], H11_reg[23]);
    sprintf(Dis_end_time, "ed:%2X-%2X-%2X %2X:%2X:%2X", H11_reg[24], H11_reg[25], H11_reg[26], H11_reg[27], H11_reg[28], H11_reg[29]);
    lcd_fill(0);
    lcd_text12(0, 0, Dis_status, strlen(Dis_status), LCD_MODE_SET);
    lcd_text12(0, 12, Dis_start_time, strlen(Dis_start_time), LCD_MODE_SET);
    lcd_text12(0, 22, Dis_end_time, strlen(Dis_end_time), LCD_MODE_SET);
    lcd_update_all();
}

static void msg( void *p)
{
}
static void show(void)
{
    u8 datatime_str[6];
    u8  illegal_driveNum = 0;
    u32  current_u32time = 0; //  当前的时间
    u32  old2day_u32time = 0; //  前2个日历天的时间    current-2x86400 (172800)
    u32  read_u32time = 0;
    u8  read_flag = 0;    // 是否需要读取
    u8  show_state = 0;
    u8   i = 0, readusefull_num = 0, read_index = 0;

    MenuIdle_working = 0; //clear
    //0.   caculate time  seconds

    Time2BCD(datatime_str);
    current_u32time = Time_sec_u32(datatime_str, 6);
    old2day_u32time = current_u32time - 172800; // 2个日历天内的时间

    //  1.   flag  state
    if(Vdr_Wr_Rd_Offset.V_11H_Write >= 1) //没存满
    {
        get_indexnum = Vdr_Wr_Rd_Offset.V_11H_Write - 1;
        read_flag = 1;
    }
    else if(Vdr_Wr_Rd_Offset.V_11H_full == 1)
    {
        get_indexnum = 0;
        read_flag = 1;
    }
    else
        get_indexnum = 0;

    //   2. read
    if(read_flag)
    {
        if(get_11h(get_indexnum, H11_reg) == 0)							//50  packetsize	  num=100
            show_state = 0;
        read_u32time = Time_sec_u32(H11_reg + 18, 6);
        if(read_u32time >= old2day_u32time)
        {
            show_state = 1;
        }
    }
    //  1.
    if(show_state)
    {
        Dis_num = 0;
        Dis_status[0] = Dis_num + 1 + '0';
        disp_record_11h(Dis_num);
    }
    else
    {
        Dis_num = 0;
        lcd_fill(0);
        lcd_text12(18, 10, "无超时驾驶记录", 14, LCD_MODE_SET);
        lcd_update_all();
    }
    //读疲劳驾驶记录
}


static void keypress(unsigned int key)
{
    unsigned char temp = 0;
    unsigned char tired_num = 0;



    switch(KeyValue)
    {
    case KeyValueMenu:
        CounterBack = 0;
        /*	ErrorRecord=0;//疲劳超速记录错误清0
        	StartDisTiredExpspeed=0;
        	tire_Flag=0;//查看疲劳报警记录过程标志清0;
        	if(Mileage_02_05_flag==2)
        		pMenuItem=&Menu_5_recorder;
        	else
        		pMenuItem=&Menu_4_1_pilao;
        */
        pMenuItem = &Menu_5_recorder;
        pMenuItem->show();
        break;
    case KeyValueOk:
        break;
    case KeyValueUP:
        if(get_indexnum)
        {
            if(Dis_num == 0)
                Dis_num = Dis_maxNum;
            else
                Dis_num--;
            Dis_status[0] = Dis_num + 1 + '0';
            disp_record_11h(Dis_num);
        }
        break;
    case KeyValueDown:
        if(get_indexnum)
        {
            if(Dis_num > (Dis_maxNum - 1))
                Dis_num = 0;
            else
                Dis_num++;

            Dis_status[0] = Dis_num + 1 + '0';
            disp_record_11h(Dis_num);
        }
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
MENUITEM	Menu_4_1_pilao =
{
    "超时驾驶记录",
    12,
    &show,
    &keypress,
    &timetick,
    &msg,
    (void *)0
};


