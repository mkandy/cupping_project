#ifndef __UART_H
#define __UART_H
#include "stdio.h"
#include <string.h>
#define S1_S0 0x40              
#define S1_S1 0x80             
extern char UAR[20],rec_flag;//全局量
void Uart1_Init(void); //@11.0592MHz 9600hz
void send1_Byte(unsigned char c);
void Send1_String(char *s);
void UART1_config();
void HexToAscii(unsigned char *pHex, unsigned char *pAscii, unsigned char nLen);
extern void Dump_value(u8 val);
extern void Dump_AD(unsigned long ad_value);

extern unsigned char unicode_number[4];
extern unsigned char consumer_number[]; 



#endif