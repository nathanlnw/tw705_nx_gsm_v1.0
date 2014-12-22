#ifndef _H_SMS
#define _H_SMS

#include "App_moduleConfig.h"
#include "SMS_PDU.h"


#define  SMS_ENABLE  

#define  SMS_ACK_msg          1      // 需哟返回短息
#define  SMS_ACK_none         0      // 不需要返回短息



typedef  struct _SMS
{
   	u8  SMIndex;    // 短信记录
	u8  SMS_read;   // 读取短信标志位
	u8  SMS_delALL; // 删除所有短信标志位
	u8  SMS_come;   // 有短信过来了
	u8  SMS_delayCounter; //短信延时器
	u8  SMS_waitCounter;	///短信等待
	u8  SMSAtSend[45];    //短信AT命令寄存器   

	u8  SMS_destNum[15];  //  发送短息目的号码 
	u8  SMS_sendFlag;  //  短息发送标志位
	u8  SMS_sd_Content[150];  // 短息发送内容

	//------- self sms protocol  ----
	u8  MsgID[4];    //  自定义短息ID 
	SmsType Sms_Info;	//解析的PDU消息的参数信息
} SMS_Style;

extern SMS_Style SMS_Service;
/////////////////////////////////////////////////////////////////////////////////////////////////
///借口函数
/////////////////////////////////////////////////////////////////////////////////////////////////
extern void SMS_timer(void);
extern void SMS_Process(void);
extern void SMS_protocol(u8 *instr,u16 len, u8  ACKstate);
extern u8 SMS_Rx_PDU(char *instr,u16 len);
extern u8 SMS_Rx_Text(char *instr,char *strDestNum);
//extern u8 SMS_Tx_PDU(char *strDestNum,char *s);
//extern u8 SMS_Tx_Text(char *strDestNum,char *s);
extern u8 SMS_Rx_Notice(u16 indexNum);
#endif

