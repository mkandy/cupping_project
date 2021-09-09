#ifndef __START_H
#define __START_H
typedef unsigned char u8;
typedef unsigned int	u16;
//GN1616 Driver
#define DIO		P1_0				//数据总线
#define	CLK		P1_1				//数据时钟
#define STB		P1_2				//片选端
//loader Driver
#define Power_switch P1_5
#define Pump_motor P5_4
#define PTC P5_5
#define Vibration_motor P3_0
#define Valve P3_1
//HX711 Driver
#define	 HX711_DOUT P1_3
#define	 HX711_SCK P1_4

#define MAIN_Fosc 11059200L //晶振频率，每秒
#define Timer0_Reload (MAIN_Fosc / 100) //Timer 0 中断频率, 100次/秒
#define Seril_Debug 1
#include<string.h>
#include "stc12.h"
#include "8051.h"

#include "delay.h"
#include "hx711.h"
#include "gn1616.h"
#if(Seril_Debug==1) 
#include "uart.h"
void UART1_Interrupt(void)__interrupt UART1_VECTOR;
#endif


void DeviceInit(void);
void Start(void);
void DeviceInit(void);
void Timer0_init(void);
void Int0_init(void);
void Int1_init(void);
void Int2_init(void);
void Int3_init(void);
void Timer2Init(void);
void Timer2Off(void);
void KeyTimer(void) __interrupt TIMER0_VECTOR;
void PwmTimer(void) __interrupt TIMER2_VECTOR;
void INT0_int(void) __interrupt INT0_VECTOR;
void INT1_int(void) __interrupt INT1_VECTOR;
void INT2_int(void) __interrupt INT2_VECTOR;
void INT3_int(void) __interrupt INT3_VECTOR;

//功能函数
void TimerOutPut(unsigned char KeyLeve);
void PumpOutPut(unsigned char KeyLeve);
void PTCOutPut(unsigned char KeyLeve);
void VibrationOutPut(unsigned char KeyLeve);
void PumpPWM(unsigned char pwm);
void VibrationPWM(unsigned char pwm);
void HeatPTCPWM(unsigned char pwm);
void TimerHandler(void);
#endif
