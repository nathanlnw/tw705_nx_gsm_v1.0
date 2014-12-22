#include  <string.h>
#include "Menu_Include.h"

u8 self_checking_screen=0;
u16 self_checking_counter=0;//自检时间计数
u16 self_checking_pro=0;//自检过程中有异常项+1

u16 Auto_exit=0;
static void msg( void *p)
{

}
static void show(void)
{
  MenuIdle_working=0;//clear
if(self_checking_screen==0)
	{
	self_checking_screen=1;
	self_checking_counter=0;
	CounterBack=0;
	//rt_kprintf("\r\n------------------开始自检-----------");
	lcd_fill(0);
	lcd_text12_local(20,10,"设备自检。。。",14,LCD_MODE_SET);
	lcd_update_all();
	}
  
}


static void keypress(unsigned int key)
{
	switch(KeyValue)
		{
		case KeyValueMenu:		
			//pMenuItem=&Menu_0_loggingin;
			//pMenuItem->show();
			break;
		case KeyValueOk:
			if(self_checking_result==2)
				{
				self_checking_result=30;
				
				self_checking_screen=0;
				self_checking_counter=0;//自检时间计数
				self_checking_pro=0;//自检过程中有异常项+1
				}
			break;
		case KeyValueUP:
			break;
		case KeyValueDown:
			break;
		}
	KeyValue=0;
}


static void timetick(unsigned int systick)
{
if(Auto_exit==0)
	{
	self_checking_counter++;//250ms
    if(self_checking_counter>=30) 
    	{
    	self_checking_counter=0;
    	//rt_kprintf("\r\n-------***********------查看自检结果-------*************----");
    	if(self_checking_result==1) 
			{
			self_checking_result=10;
			//rt_kprintf("\r\n------自检正常-----------");
			lcd_fill(0);
			lcd_text12_local(36,10,"自检正常",8,LCD_MODE_SET);
			lcd_update_all();
			}
		else if(self_checking_result==2)
			{
			Auto_exit=1;
			//rt_kprintf("\r\n------自检异常-----------");
			lcd_fill(0);
			if(self_checking_Antenna==1)//天线开路
				{
			     // rt_kprintf("\r\n  天线开路 \r\n");
				  lcd_text12_local(36,3,"天线开路",8,LCD_MODE_SET); 
				}
			else if(self_checking_Antenna==2)//天线短路
			   {
			     // rt_kprintf("\r\n  天线短路 \r\n");
				  lcd_text12_local(36,3,"天线短路",8,LCD_MODE_SET);
			    }
			if(self_checking_PowerCut==1)
				{
				  // rt_kprintf("\r\n  内部电池供电 \r\n");
				   lcd_text12_local(24,18,"内部电池供电",12,LCD_MODE_SET);
				}
			lcd_update_all();
			}
		}
	}
else if(Auto_exit==1)
	{
	self_checking_counter++;
	if(self_checking_counter>=20)
		{
		if(self_checking_result==2)
			{
			self_checking_result=30;
			
			self_checking_screen=0;
			self_checking_counter=0;//自检时间计数
			self_checking_pro=0;//自检过程中有异常项+1

			}
		}
	}
}


MENUITEM	Menu_0_0_self_Checking=
{
    "设备自检",
	8,
	&show,
	&keypress,
	&timetick,
	&msg,
	(void*)0
};


