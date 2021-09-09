#ifndef __INTERRUPT_H
#define __INTERRUPT_H
#include <start.h>

void LoadPWM(u16 i);
void KeyTimer(void) __interrupt TIMER0_VECTOR;
void PWMTimer(void)__interrupt TIMER2_VECTOR;
void INT0_int(void)__interrupt INT0_VECTOR;
void INT1_int(void)__interrupt INT1_VECTOR;
void INT2_int(void)__interrupt INT2_VECTOR;
void INT3_int(void)__interrupt INT3_VECTOR;



#endif