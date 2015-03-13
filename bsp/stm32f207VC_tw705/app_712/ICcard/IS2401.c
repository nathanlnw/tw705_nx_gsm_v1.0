//=======================================



#include"IS2401.h"

void IIC_Delay(void);



#define IC_SDA_H   _CardPutIO_HIGH()
#define IC_SDA_L   _CardPutIO_LOW()

#define IC_SCL_H   _CardSetCLK_HIGH
#define IC_SCL_L   _CardSetCLK_LOW

#define  FAILED    0
#define  PASS      1



u8 IIC_Start(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    IC_SCL_H;
    IC_SDA_H;

    IIC_Delay();

    if(!_CardReadIO())
        return FAILED;
    IC_SDA_L;
    IIC_Delay();

    if(_CardReadIO())
        return FAILED;

    IC_SCL_L;
    IIC_Delay();

    return PASS;
}


void IIC_Stop(void)
{

    IC_SCL_H;

    IC_SDA_L;

    IIC_Delay();

    IC_SDA_H;
    IIC_Delay();
}



u8 IIC_WriteByte(uint8_t byte)
{

    uint8_t i = 8;

    uint8_t mask = 0x80;

    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    while(i--)
    {

        IC_SCL_L;

        IIC_Delay();

        if(byte & mask)
        {
            IC_SDA_H;
        }
        else
        {
            IC_SDA_L;
        }
        mask >>= 1;

        IIC_Delay();
        IC_SCL_H;

        IIC_Delay();

    }

    IC_SCL_L;
    IC_SDA_H;
    IIC_Delay();
    IC_SCL_H;
    IIC_Delay();

    if(_CardReadIO())
    {
        IC_SCL_L;
        return FAILED;
    }
    IC_SCL_L;
    return PASS;
}



uint8_t IIC_ReadByte_ACK(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;


    GPIO_InitTypeDef	GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    IC_SDA_H;
    while(i--)
    {
        byte <<= 1;
        IC_SCL_L;
        IIC_Delay();

        IC_SCL_H;

        IIC_Delay();

        if(_CardReadIO())
        {
            byte |= 0x01;
        }
    }
    IC_SCL_L;
    IIC_Delay();
    IC_SDA_L;
    IIC_Delay();
    IC_SCL_H;
    IIC_Delay();
    IC_SCL_L;
    return byte;
}



uint8_t IIC_ReadByte_NACK(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);


    IC_SDA_H;
    while(i--)
    {
        byte <<= 1;
        IC_SCL_L;
        IIC_Delay();
        IC_SCL_H;
        IIC_Delay();
        if(_CardReadIO())
        {
            byte |= 0x01;
        }
    }
    IC_SCL_L;
    IIC_Delay();
    IC_SDA_H;
    IIC_Delay();
    IC_SCL_H;
    IIC_Delay();
    IC_SCL_L;
    return byte;
}



u8  IIC_Write(uint8_t addr, uint8_t byte)
{
    if(!IIC_Start())return FAILED;
    if(!IIC_WriteByte(0xa0))
    {
        IIC_Stop();
        return FAILED;
    }
    if(!IIC_WriteByte(addr))
    {
        IIC_Stop();
        return FAILED;
    }
    if(!IIC_WriteByte(byte))
    {
        IIC_Stop();
        return FAILED;
    }
    IIC_Stop();
    return PASS;

}



uint8_t IIC_Read(uint8_t addr)
{
    uint8_t Data = 0;
    if(!IIC_Start())return FAILED;
    if(!IIC_WriteByte(0xa0))
    {
        IIC_Stop();
        return FAILED;
    }
    if(!IIC_WriteByte(addr))
    {
        IIC_Stop();
        return FAILED;
    }
    if(!IIC_Start())
    {
        IIC_Stop();
        return FAILED;
    }
    if(!IIC_WriteByte(0xa1))
    {
        IIC_Stop();
        return FAILED;
    }
    rt_kprintf(" + ");
    Data = IIC_ReadByte_NACK();
    IIC_Stop();
    return Data;

}


void IIC_Delay(void)
{
    uint8_t time = 100;
    while(time)
    {
        time--;
    }
}

u8  IC_RX_Buf(unsigned char addr, unsigned char num, unsigned char *buf)
{
    u16  i = 0;
    unsigned char inaddr = addr;

    for(i = 0; i < num; i++)
    {
        *buf = IIC_Read(inaddr++);
        buf++;
    }

    return PASS;
}


unsigned char IC_Rx_2401(unsigned char addr, unsigned char num, unsigned char *buf)
{
    //  if(addr<32)
    //  return 0x03;
    if((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) == 0)
        return 0x01;

    if(IC_RX_Buf(addr, num, buf) == 0)
        return 0x02;
    return 0x00;
}











