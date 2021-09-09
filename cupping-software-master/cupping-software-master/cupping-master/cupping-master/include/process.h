#ifndef __PROCESS_H
#define __PROCESS_H
#include <start.h>


void Time_handler(void);
void Key_handler(void);
void IO_handler(void);
void Display_handler(void);
extern unsigned char LED1,LED2;

#endif