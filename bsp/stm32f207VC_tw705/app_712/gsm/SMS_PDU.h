#ifndef _H_SMS_PDH
#define _H_SMS_PDH

#define SMS_TYPE_PDU


#define GSM_7BIT	0x00
#define GSM_UCS2	0x08


#define TRUE	1
#define FALSE	0

typedef struct 
{
 char SCA[15];		///短信中心号码
 char TPA[15];		///短信发送方号码
 char TP_SCTS[14];	///事件戳信息
 u8  TP_PID;			///协议标识
 u8  TP_DCS; 		///协议编码类型
}SmsType;


extern u16 SetPhoneNumToPDU(u8 *pDest,char *pSrc,u16 len);
extern u16 GetPhoneNumFromPDU(char *pDest,u8 *pSrc,u16 len);
extern u16 GsmDecodePdu(const u8* pSrc,u16 pSrcLength,SmsType *pSmstype,u8 *DataDst);
extern u16 GsmEncodePdu_NoCenter(const SmsType pSrc,const u8 *DataSrc,u16 datalen, u8* pDst);
extern u16 AnySmsEncode_NoCenter(const u8 *SrcNumber,u8 type,const u8 *DataSrc,u16 datalen, u8* pDst);
extern u16 GsmEncodePdu_Center(const SmsType pSrc,const u8 *DataSrc,u16 datalen, u8* pDst);
#endif
