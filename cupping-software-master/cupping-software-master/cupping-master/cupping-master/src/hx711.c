#include "delay.h"
//****************************************************
//HTI_Hx711驱动程序；包括气压传感器XGZP6847和温度NTC 3950 10K %1
//****************************************************
#define	 HX711_DOUT P1_3
#define	 HX711_SCK P1_4

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
	return(count);
}
