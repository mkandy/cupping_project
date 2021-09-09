#ifndef __GN1616_H
#define __GN1616_H
#define GIRD1 0xc0
#define GIRD2 0xc2
void gn1616_start(void);
void gn1616_ms(u16 ms);
void delay(void);
void write_data(u8  dat);
void write_cmd(u8  cmd);
void display(u8 dat,u8 channel);
#endif