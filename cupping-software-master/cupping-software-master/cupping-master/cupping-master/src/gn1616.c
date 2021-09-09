#include "start.h"

#define DIO		P1_0				//数据总线
#define	CLK		P1_1				//数据时钟
#define STB		P1_2				//片选端
//********************************************//
//	实际使用时仅更改以上头文件和端口定义即可	//
//********************************************//
//********************************************//
//****************主函数**********************//
//********************************************//
void gn1616_start(void)
{

       // display(0xf0);

}
//********************************************//
//***************子函数***********************//
//********************************************//

void delay(void)					//几us的延时
{
	u8 i = 2;
	while(i--);



}

void gn1616_ms(u16 ms)		//大约ms级的延时
{
	u16 i,j;
	for(j=0;j<ms;j++)
		for(i=0;i<1000;i++);
}

void write_data(u8 dat)					//写数据
{
	u8 i;
	STB = 0;
	delay();
	for(i=0;i<8;i++)
	{
		CLK = 0;
		delay();
		DIO = dat & 0x01;							//低位先传
		CLK = 1;
		delay();
		dat >>=1;
	}
	delay();
}

void write_cmd(u8 cmd)							//写指令
{
	STB = 1;
	delay();
	STB = 0;
	delay();
	write_data(cmd);								//STB下降沿后的DATA输入的第一个字节为一条指令
}
void display(u8 dat,u8 channel)
{
//u8 i;
		write_cmd(0x00);							//显示模式设置：4位*7段
	write_cmd(0x44);							//数据设置：普通模式，地址自加，写数据到显示寄存器
	write_cmd(channel);							//地址设定：00H
	write_data(dat);
	write_cmd(0x8f);	


}