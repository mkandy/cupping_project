
#include <init.h>

void ParameterReset(void)

{

    //变量初始化
    IO_Pump = 0;
    IO_Vibration = 1;
    IO_PTC = 0;
    IO_Valve = 1;
    IO_Power = 0;

    Key.update = 0;
    Key.long_press_state = 0;

    Suction.on = 0;
    Heating.on = 0;
    Vibration.on = 0;

    Suction.level = 0;
    Heating.level = 0;
    Vibration.level = 0;

    Time.sec = 0;
    Time.min = 0;
    Time.count = 0;

    LED1 = 0;
    LED2 = 0;
}

void DeviceInit(void)
{

    //IO init

    P0M1 = 0;
    P0M0 = 0; //设置为准双向口
    P1M1 = 0;
    P1M0 = 0x20; //设置为准双向口
    P2M1 = 0;
    P2M0 = 0; //设置为准双向口
    P3M1 = 0;
    P3M0 = 0x03; //设置为准双向口
    P4M1 = 0;
    P4M0 = 0; //设置为准双向口
    P5M1 = 0;
    P5M0 = 0x30; //设置为准双向口

    ParameterReset();

    state = Power_on;

    Key.debounce = 0; //按键延时
    TR0 = 0;          //停止计数
    ET0 = 0;          //停止计数中断

    Int0_init();
    Int1_init();
    Int2_init();
    Int3_init();
    Timer0_init();

#if (Seril_Debug == 0)
    Timer2_init(); //use timer2 for PWM if UART is not use for debug
#endif
}

void Int0_init(void)
{
    IE0 = 0; //外中断0标志位
    EX0 = 1; //INT0 Enable
    IT0 = 1; //INT0 下降沿中断
    //	IT0 = 0;		//INT0 上升,下降沿中断
}
void Int1_init(void)
{
    IE1 = 0; //外中断1标志位
    EX1 = 1; //INT1 Enable
    IT1 = 1; //INT1 下降沿中断
    //	IT1 = 0;		//INT1 上升,下降沿中断
}
void Int2_init(void)
{
    WAKE_CLKO |= 0x10; //INT3 Enable 	//INT2 Enable
                       //INT2 只能下降沿中断
}
void Int3_init(void)
{
    WAKE_CLKO |= 0x20; //INT3 Enable
                       //INT3 只能下降沿中断
}

//========================================================================
// 函数: void	Timer0_init(void)
// 描述: timer0初始化函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2015-1-12
//========================================================================
  void Timer0_init(void)
{
    TR0 = 0; //停止计数

#if (Timer0_Reload < 64) // 如果用户设置值不合适， 则不启动定时器
#error "Timer0设置的中断过快!"

#elif ((Timer0_Reload / 12) < 65536UL) // 如果用户设置值不合适， 则不启动定时器
    ET0 = 1; //允许中断
             //	PT0 = 1;	//高优先级中断
    TMOD &= ~0x03;
    TMOD |= 0;          //工作模式, 0: 16位自动重装, 1: 16位定时/计数, 2: 8位自动重装, 3: 16位自动重装, 不可屏蔽中断
                        //	TMOD |=  0x04;	//对外计数或分频
    TMOD &= ~0x04;      //定时
                        //	INT_CLKO |=  0x01;	//输出时钟
    WAKE_CLKO &= ~0x01; //不输出时钟

#if (Timer0_Reload < 65536UL)
    AUXR |= 0x80;       //1T mode
    TH0 = (u8)((65536UL - Timer0_Reload) / 256);
    TL0 = (u8)((65536UL - Timer0_Reload) % 256);
#else
    AUXR &= ~0x80; //12T mode
    TH0 = (u8)((65536UL - Timer0_Reload / 12) / 256);
    TL0 = (u8)((65536UL - Timer0_Reload / 12) % 256);
#endif

    TR0 = 1; //开始运行

#else
#error "Timer0设置的中断过慢!"
#endif

} 
 




void Timer2_init(void)
{
    AUXR &= ~(1 << 4); //stop counter
    IE2 |= (1 << 2);   //enable timer2 interrupt
    AUXR |= (1 << 2);  //set 1T
    AUXR &= ~(1 << 3); //set timer mode
                       //    INT_CLKO |=  0x04;  //output clock

    T2H = 0;
    T2L = 0;
    AUXR |= (1 << 4); //start timer2
}
