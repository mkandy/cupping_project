#ifndef __START_H
#define __START_H
typedef unsigned char u8;
typedef unsigned int u16;
#define MAIN_Fosc 11059200L //晶振频率，每秒
#define Seril_Debug 1

#include "stc12.h"
#include "8051.h"
#include "delay.h"
#include "hx711.h"
#include "gn1616.h"
#include "init.h"
#include "interrupt.h"
#include "process.h"

#if (Seril_Debug == 1)
#include "uart.h"
void UART1_Interrupt(void) __interrupt UART1_VECTOR;
#endif

//

#define Timer0_Reload (MAIN_Fosc /40)       //Timer 0 中断频率, 50 ms period
#define T2KHZ (65536 - MAIN_Fosc / 12 / 6000) //Timer2

#define IO_Power P1_5
#define IO_Pump P5_4
#define IO_PTC P5_5
#define IO_Vibration P3_0
#define IO_Valve P3_1


#define IN_Power P3_2
#define IN_Pump P3_3
#define IN_Vibration P3_6
#define IN_PTC P3_7

#define Max_key 4

#define PWM_DUTY 6000 //??PWM???,????????,????24.576MHZ???,?PWM???6000HZ?
#define PWM_DUTY1 4000
#define PWM_DUTY2 5000
#define PWM_HIGH_MIN 32                        //??PWM????????????????
#define PWM_HIGH_MAX (PWM_DUTY - PWM_HIGH_MIN) //??PWM????????????????

//自定义一个int结构体位域来解析按键：单次，双击，长按，等功能
typedef struct Button_Setting
{
    unsigned char update : 1;
    unsigned char pressed : 1;
    unsigned char long_press : 4;
    unsigned char long_press_state : 1;
    unsigned char debounce;
    unsigned char which_press;

} BUTTON_STAT;

//program setting
typedef struct Program_Setting
{
    unsigned char on : 1;
    unsigned char level : 3;
    unsigned char timer : 4;

} LEVEL_STAT;

//define 4 buttons
typedef enum Button_type
{
    Key_Power = 0x00,     //0x00
    Key_Pump = 0x01,      //0x01
    Key_Vibration = 0x02, //0x02
    Key_PTC = 0x03        //0x03

} BUTTON;

//State machine
typedef enum State_name
{
    idle_mode,
    normal_mode,
    Timer_mode,
    Power_down,
    Power_on,
    BT_mode,

} STATE;

typedef enum Treatment_time
{
    Time0 = 0,
    Time1 = 1,
    Time2 = 1,
    Time3 = 1

} COUNTER;

typedef struct PWM_Setting
{
    unsigned char on : 1;
    unsigned int low;
    unsigned int high;
    unsigned int value;
} PWM_STAT;

typedef struct Timer_Setting
{
   
    unsigned char sec;
    unsigned char min;
    unsigned char count;
    unsigned char update : 1;
    unsigned char quartersec : 1;
    unsigned char halfsec:1;
    unsigned char onesec:1;

} TIMER_STAT;

//Sensor_init
typedef struct sensor_in
{
unsigned long pressure;
unsigned long ntc;
}SENSOR;





void Start(void);


//初始化按键
extern BUTTON_STAT Key;
extern LEVEL_STAT Power, Vibration, Suction, Heating;
extern BUTTON Key_pressed;
extern PWM_STAT PWM;
extern TIMER_STAT Time;
extern STATE state;
extern COUNTER duration;
extern SENSOR sensor;



#endif
