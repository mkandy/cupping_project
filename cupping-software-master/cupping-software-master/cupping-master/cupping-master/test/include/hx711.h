#ifndef __HX711_H
#define __HX711_H

void Delay__hx711_us(void);
unsigned long HX711_Read(u8 channel)	;
void HexToAscii(unsigned char *pHex, unsigned char *pAscii, unsigned char nLen);
#endif