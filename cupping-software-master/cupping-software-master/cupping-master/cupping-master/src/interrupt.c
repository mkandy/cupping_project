#include <start.h>



void TimeReset(void)

{
    Key.pressed=1;
    Key.update=0;
    Time.sec=0;
    Time.min=0;

}

void KeyTimer(void) __interrupt TIMER0_VECTOR      //150ms period
{
    Time.update=1;
   
    if (Key.pressed==1)
        Key.debounce++;
/* 
    
       
    } */

   
}

void PWMTimer(void) __interrupt TIMER2_VECTOR
{
    PWM.on = !PWM.on;
   
    if (PWM.on == 1)
    {

        T2H = (PWM.low >> 8); //LED on time is 10%
        T2L = PWM.low;
    }
    else
    {

        T2H = (PWM.high >> 8); //LED off time is 90%
        T2L = PWM.high;
    }
}

void LoadPWM(u16 i)
{
    u16 j;
    AUXR &= ~(1 << 4); //stop counter
    if (i > PWM_HIGH_MAX)
        i = PWM_HIGH_MAX; //max range for PWM
    if (i < PWM_HIGH_MIN)
        i = PWM_HIGH_MIN;       //min range for PWM
    j = 65536UL - PWM_DUTY + i; //low value for PWM
    i = 65536UL - i;            //high value for PWM
                                //    EA = 0;
    PWM.high = i;               //
    PWM.low = j;                //
    //	EA=1;
}

/********************* INT0中断函数 *************************/
void INT0_int(void) __interrupt INT0_VECTOR //进中断时已经清除标志
{

     TimeReset();
    Key_pressed = Key_Power;
    // LED1=!LED1;
    //EX0 = 0;	//INT0 Disable
    //IE0 = 0;	//外中断0标志位
}

/********************* INT1中断函数 *************************/
void INT1_int(void) __interrupt INT1_VECTOR //进中断时已经清除标志
{
    
    TimeReset();
    Key_pressed = Key_Pump;
    //LED2=!LED2;
    //EX1 = 0;	//INT1 Disable
    //IE1 = 0;	//外中断1标志位
}
/********************* INT2中断函数 *************************/
void INT2_int(void) __interrupt INT2_VECTOR //进中断时已经清除标志
{
    
     TimeReset();
    Key_pressed = Key_Vibration;
    // LED2=!LED2;
    //EX1 = 0;	//INT1 Disable
    //IE1 = 0;	//外中断1标志位
}

/********************* INT2中断函数 *************************/
void INT3_int(void) __interrupt INT3_VECTOR //进中断时已经清除标志
{
  
    TimeReset();
    
    Key_pressed = Key_PTC;
    // LED2=!LED2;
    //EX1 = 0;	//INT1 Disable
    //IE1 = 0;	//外中断1标志位
}
