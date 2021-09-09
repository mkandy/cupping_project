#include "delay.h"
//****************************************************
//HTI_Hx711驱动程序；包括气压传感器XGZP6847和温度NTC 3950 10K %1
//****************************************************


void Delay__hx711_us(void)
{
	NOP();
	NOP();
}
//****************************************************
//HX711驱动：设置channel A 或 B
//****************************************************
unsigned long HX711_Read(u8 channel)	//
{
	unsigned long count; 
	unsigned char i; 
  	HX711_DOUT=1; 
	Delay__hx711_us();
  	HX711_SCK=0; 
  	count=0; 
  	while(HX711_DOUT); 
  	for(i=0;i<channel;i++)				
	{ 
	  	HX711_SCK=1; 
	  	count=count<<1; 
		HX711_SCK=0; 
	  	if(HX711_DOUT)
			count++; 
	} 
 	HX711_SCK=1; 
    count=count^0x800000;//设置channel A 或 B
	Delay__hx711_us();
	HX711_SCK=0;  
	if(count&0x800000)
	{
		count=0-((~count+1)&0x7fffff);
	}

	return(count);
}
/***********************************************************************
* 功能：将一个十六进制字节串转换成ASCII码表示的十六进制字符串
* 输入参数：pHex----十六进制数字节串首地址
*                   pAscii---转换后ASCII码表示的十六进制字符串首地址
*                   nLen---要转换的十六进制数长度（字节数）
* 注：转换后的结果全部是大写ASCII码表示的十六进制数
************************************************************************/
void HexToAscii(unsigned char *pHex, unsigned char *pAscii, unsigned char nLen)
{
    unsigned char Nibble[2];
    unsigned char i,j;
    for (i = 0; i < nLen; i++){
        Nibble[0] = (pHex[i] & 0xF0) >> 4;
        Nibble[1] = pHex[i] & 0x0F;
        for (j = 0; j < 2; j++){
            if (Nibble[j] < 10){            
                Nibble[j] += 0x30;
            }
            else{
                if (Nibble[j] < 16)
                    Nibble[j] = Nibble[j] - 10 + 'A';
            }
            *pAscii++ = Nibble[j];
        }               // for (int j = ...)
    }           // for (int i = ...)
}