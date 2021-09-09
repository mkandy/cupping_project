/***************************************************************************
# Cupping_software
MIT License

Copyright (c) 2021 Joel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/
#include <start.h>

unsigned char PWM_ON, timer_on; //定义按键时间
unsigned char unicode_number[4] = {0};
unsigned char consumer_number[] = "";
unsigned char button_number[] = "";


//驱动宏定义：
//hx711通道；
enum
{
    Pressure_A128 = 24,
    NTC_B32 = 25,
    Pressure_A64 = 26 /*channel data */
} Sensor_channel;
//自定义枚举4个按键按下时的值；发热键，电源键，气泵键，振动键
enum
{
    Power_button,     //0x00
    Pump_button,      //0x01
    Vibration_button, //0x02
    PTC_button        //0x03

} Key_pressed;

//自定义枚举按键档位
enum
{
    OFF,        //0x00
    LEVEL_LOW,  //0x01
    LEVEL_MIDD, //0x02
    LEVEL_HIGH  //0x03

} Key_level;
//GN1616驱动显示位 :定义led1－led8 每个位的点亮位置
enum
{
    ledoff = 0b00000000,
    led1 = 0b00000001,
    led2 = 0b00000010,
    led3 = 0b00000100,
    led4 = 0b00001000,
    led5 = 0b00010000,
    led6 = 0b00100000,
    led7 = 0b01000000,
    led8 = 0b10000000
} Gn1616_led;
//因为LED接的是2组8位：自定义一个显示体结体
typedef struct
{
    unsigned char G2display;
    unsigned char G1display;
} Gn1616_display;

//自定义一个int结构体位域来解析按键：单次，双击，长按，等功能
typedef struct
{
    unsigned int stat : 1;
    unsigned int times : 2;
    unsigned int which_press : 8;
    unsigned int long_press : 3;
    unsigned int long_press_state : 1;
    unsigned int button_press_flag : 1;
} HTI_button;
//气压和NTC、Sensor_init
struct
{
    unsigned long pressure;
    unsigned long ntc;
} HTI_sensor;
//气泵档位Init
typedef struct
{
    unsigned char pump_keyleve;
    unsigned char pump_timer;
    unsigned char pump_pwm_duty;
} Pump_control;
//振动档位Init
typedef struct
{
    unsigned char vibration_keyleve;
    unsigned char vibration_timer;
    unsigned char vibration_pwm_duty;
} Vibration_control;
//加热档位Init
typedef struct
{
    unsigned char ptc_keyleve;
    unsigned char ptc_timer;
    unsigned char ptc_pwm_duty;
} PTC_control;
//定时档位Init
typedef struct
{
    unsigned int timer_keyleve : 3;
    unsigned int timer_flag : 1;
    unsigned int timer_timer : 6;
    unsigned int timer_pwm_duty : 6;

} Timer_control;
//初始化按键
HTI_button copping_button, *button;

//时间控制
Timer_control how_set_timer, *set_timer;
//气泵控制
Pump_control how_set_pump, *set_pump;
//振动控制
Vibration_control how_set_vibration, *set_vibration;
//加热控制
PTC_control how_set_ptc, *set_ptc;
//控制显示
Gn1616_display control_display, *set_display;
//变量初始化
void DeviceInit(void)
{

    Pump_motor = 0;          //气泵输出初始化
    Vibration_motor = 0;     //振动马达输出初始化
    PTC = 0;                 //加热输出初始化
    Valve = 1;               //电磁阀输出初始化
    Power_switch = 0;        //电源开关输出初始化
    copping_button.stat = 0; //按键标志位初始化
    PWM_ON = 0;              //按键延时
    timer_on = 0;
    TR0 = 0;                  //停止计数
    ET0 = 0;                  //停止计数中断
    button = &copping_button; //按键指针;

    set_display = &control_display;                           //初始化显示指针
    set_pump = &how_set_pump;                                 //初始化气泵指针
    set_vibration = &how_set_vibration;                       //初始化振动指针
    set_ptc = &how_set_ptc;                                   //初始化加热指针
    set_timer = &how_set_timer;                               //初始化定时指针
    memset(&copping_button, 0, sizeof(copping_button));       //清零copping所有按键状态输出
    memset(&how_set_pump, 0, sizeof(how_set_pump));           //清零pump所有pwm输出
    memset(&how_set_vibration, 0, sizeof(how_set_vibration)); //清零vibration所有pwm输出
    memset(&how_set_ptc, 0, sizeof(how_set_ptc));             //清零ptc所有pwm输出
    memset(&control_display, 0, sizeof(control_display));     //将LED所有显示输出灯灭
    memset(&how_set_timer, 0, sizeof(how_set_timer));         //将定时结体构初始化
}
//scheduler and handler进度和处理
void Start(void)
{

    P0M1 = 0;
    P0M0 = 0; //设置为准双向口
    P1M1 = 0;
    P1M0 = 0x30; //设置
    P2M1 = 0;
    P2M0 = 0; //设置为准双向口
    P3M1 = 0;
    P3M0 = 0x03; //设置
    P4M1 = 0;
    P4M0 = 0; //设置为准双向口
    P5M1 = 0;
    P5M0 = 0x30; //设置

    Int0_init();
    Int1_init();
    Int2_init();
    Int3_init();
    display(set_display->G1display, GIRD1); //初始化后的led all清零显示全灭
    display(set_display->G2display, GIRD2);

#if (Seril_Debug == 1)
    Send1_String("STC15W204S\r\nUart is ok !\r\n");      //发送字符串检测是否初始化成功
    Send1_String("gn1616_start\r\ndelay_ms(1000)!\r\n"); //发送字符串检测是否初始化成功
#endif
    EA = 0;
    DeviceInit();
    EA = 1;

    while (1)
    {

        HTI_sensor.pressure = HX711_Read((Pressure_A128)); //获取气压sensor输入通道增益128的电压值
        //  HTI_sensor.pressure = 0x12345678;

        unicode_number[3] = HTI_sensor.pressure;

        unicode_number[2] = HTI_sensor.pressure >> 8;

        unicode_number[1] = HTI_sensor.pressure >> 16;
        // send1_Byte(unicode_number[2]);
        unicode_number[0] = HTI_sensor.pressure >> 24;
        //  send1_Byte(unicode_number[3]);
#if (Seril_Debug == 1)
        HexToAscii(unicode_number, consumer_number, 4);
        Send1_String("\r\nL+Hpressure="); //发送气压传感器 低 8位
        Send1_String(consumer_number);
        // send1_Byte(consumer_number[0]);
        gn1616_ms(500);
#endif

        if (copping_button.stat) //按键中断flag;
        {
            EA = 0;
            copping_button.stat = 0;
            copping_button.which_press = Key_pressed;

#if (Seril_Debug == 1)
            //这里定义的数组是将16进制的数转换为字符输出，方便打印调试
            Send1_String("\r\nButton="); //发送按键值
            button_number[0] = copping_button.which_press;
            HexToAscii(button_number, button_number, 1);
            Send1_String(button_number);

#endif
            switch (copping_button.which_press)
            {

            case PTC_button:
                if (how_set_ptc.ptc_keyleve < 3)
                    how_set_ptc.ptc_keyleve++;
                else
                {
                    how_set_ptc.ptc_keyleve = 0; /* code */
                }
                PTCOutPut(set_ptc->ptc_keyleve);
                break;
            case Vibration_button:
                if (how_set_vibration.vibration_keyleve < 3)
                    how_set_vibration.vibration_keyleve++;
                else
                {
                    how_set_vibration.vibration_keyleve = 0; /* code */
                }
                VibrationOutPut(set_vibration->vibration_keyleve);
                break;
            case Pump_button:
                if (how_set_pump.pump_keyleve < 3)
                    how_set_pump.pump_keyleve++;
                else
                {
                    how_set_pump.pump_keyleve = 0; /* code */
                }
                PumpOutPut(set_pump->pump_keyleve); //取按键值计算档位
                break;
            case Power_button:
                if (how_set_timer.timer_keyleve < 3)
                {
                    how_set_timer.timer_keyleve++;
                }
                else
                {
                    how_set_timer.timer_keyleve = 0; /* code */
                }
                TimerOutPut(set_timer->timer_keyleve);
                Timer0_init();
                break;
                // -------------------------------
                // Default event handler.
            default:
                break;
            }
            display(set_display->G2display, GIRD1); //保持每个按键独立档位LED显示
            display(set_display->G1display, GIRD2); //保持每个按键独立档位LED显示
            EA = 1;
#ifdef BUTTON_TIMES //定义按键次数
            switch (copping_button.times)
            {
            case 1:

                break;
            case 2:

                break;
            case 3:

                break;

                // -------------------------------
                // Default event handler.
            default:
                break;
            }
#endif
        }
        if (copping_button.long_press_state) //如果检测到长时间按键则打开电源及升压
        {
            // DeviceInit();
            EA = 0;
            copping_button.long_press_state = 0; //reset press long time flag;
            Power_switch = !Power_switch;        //trun on the power switch;
            if (Power_switch)
            {

                display(0xff, GIRD1);
                display(0xff, GIRD2);
            }
            else
            {

                display(0x00, GIRD1);
                display(0x00, GIRD2);
            }
            EA = 1;
#if (Seril_Debug == 1)
            Send1_String("long_press_state=1\r\n"); //发送字符串检测是否初始化成功
#endif
        } //
          //气泵

        //振动
        //   VibrationOutPut(set_vibration->vibration_keyleve); //取按键值计算档位
        // VibrationPWM(set_vibration->vibration_pwm_duty);   //设置档位推送到振动工作
        //display((u8)leddisplay1, GIRD1);                   //推送到GN1616显示
        //加热
        // PTCOutPut(set_ptc->ptc_keyleve);                   //取按键值计算档位
        //HeatPTCPWM(set_ptc->ptc_pwm_duty);                 //设置档位推送到加热工作
        //display((u8)leddisplay1, GIRD1);                   //推送到GN1616显示
    }
}

void KeyTimer(void) __interrupt TIMER0_VECTOR
{
    if (button->button_press_flag)
    {

        if ((!INT0) || (!INT1) || (!INT2) || (!INT3))
            PWM_ON++;
        else
        {
            copping_button.stat = 0;
            PWM_ON = 0;
            TR0 = 0; //停止计数
            ET0 = 0; //停止计数中断
            copping_button.long_press = 0;
            button->button_press_flag = 0;
        }

        if (PWM_ON % 20 == 0)
        {
            if (!INT0)
                copping_button.long_press++;
            else
            {
                copping_button.times++;
                copping_button.stat = 1;
                copping_button.long_press = 0;
                copping_button.long_press_state = 0;
                button->button_press_flag = 0;
                PWM_ON = 0;
                TR0 = 0; //停止计数
                ET0 = 0; //停止计数中断
            }
            if (copping_button.long_press > 5)
            {
                copping_button.times++;
                copping_button.stat = 0;
                copping_button.long_press = 0;
                copping_button.long_press_state = 1;
                button->button_press_flag = 0;
                PWM_ON = 0;
                TR0 = 0; //停止计数
                ET0 = 0; //停止计数中断
            }
        }
    }
    else if (set_timer->timer_flag)

    {
        timer_on++;
        if (timer_on % 40 == 0)
        {

            set_timer->timer_timer++; /* code */
            if ((set_timer->timer_timer % set_timer->timer_pwm_duty) == 0)
                TimerHandler(); //自动定时关机
        }
    }
}

void PwmTimer(void) __interrupt TIMER2_VECTOR
{

    how_set_ptc.ptc_timer++;                         //ptc_pwm
    how_set_vibration.vibration_timer++;             //vibration_pwm
    how_set_pump.pump_timer++;                       //pump_pwm
    PumpPWM(set_pump->pump_pwm_duty);                //设置档位推送到气泵电机工作
    VibrationPWM(set_vibration->vibration_pwm_duty); //设置档位推送到振动电机工作
    HeatPTCPWM(set_ptc->ptc_pwm_duty);               //设置档位推送到PTC发热片
}
/********************* INT0中断函数 *************************/
void INT0_int(void) __interrupt INT0_VECTOR //进中断时已经清除标志
{
    Timer0_init(); //启动定时器：作用于消抖按键，判断按键功能
    button->stat = 0;
    button->button_press_flag = 1;
    Key_pressed = Power_button;
}

/********************* INT1中断函数 *************************/
void INT1_int(void) __interrupt INT1_VECTOR //进中断时已经清除标志
{
    Timer0_init(); //启动定时器：作用于消抖按键，判断按键功能
    button->stat = 0;
    button->button_press_flag = 1;
    Key_pressed = Pump_button;
}
/********************* INT2中断函数 *************************/
void INT2_int(void) __interrupt INT2_VECTOR //进中断时已经清除标志
{
    Timer0_init(); //启动定时器：作用于消抖按键，判断按键功能
    button->stat = 0;
    button->button_press_flag = 1;
    Key_pressed = Vibration_button;
}
/********************* INT2中断函数 *************************/
void INT3_int(void) __interrupt INT3_VECTOR //进中断时已经清除标志
{
    Timer0_init(); //启动定时器：作用于消抖按键，判断按键功能
    button->stat = 0;
    button->button_press_flag = 1;
    Key_pressed = PTC_button;
}
void Int0_init(void)
{
    IE0 = 1; //外中断0标志位
    EX0 = 1; //INT0 Enable
    IT0 = 1; //INT0 下降沿中断
    //	IT0 = 0;		//INT0 上升,下降沿中断
}
void Int1_init(void)
{
    IE1 = 1; //外中断1标志位
    EX1 = 1; //INT1 Enable
    IT1 = 1; //INT1 下降沿中断
    //	IT1 = 0;		//INT1 上升,下降沿中断
}
void Int2_init(void)
{
    WAKE_CLKO |= 0x10; //INT2 Enable
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
// 版本: V2.0, 2021-8-12
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

void Timer2Init(void) //50微秒@11.0592MHz
{
#if (Seril_Debug == 0)
    AUXR &= ~0x1c;          ////停止计数, 定时模式, 1T模式
    AUXR |= 0x04;           //定时器时钟1T模式
    WAKE_CLKO &= ~(1 << 2); //第3位是t2映射的时钟输出脚P3.0,所以将1左移2位取反运算为0:不允许P3.0输出T2时钟；
    T2L = 0x33;             //设置定时初始值
    T2H = 0xF5;             //设置定时初始值
    AUXR |= 0x10;           //定时器2开始计时	等于1左移4位，AUXR |= (1<<4);
    IE2 |= 0x04;            //定时器2允许中断
#endif
}

void Timer2Off(void) //停止Timer2计时
{
#if (Seril_Debug == 0)
    AUXR &= ~0x1c;          ////停止计数, 定时模式, 1T模式
    WAKE_CLKO &= ~(1 << 2); //第3位是t2映射的时钟输出脚P3.0,所以将1左移2位取反运算为0:不允许P3.0输出T2时钟；
    IE2 &= ~(1 << 2);       //定时器2允许中断
#endif
}

//设定定时档位
void TimerOutPut(unsigned char KeyLeve)
{
    switch (KeyLeve)
    {
    case OFF:                                                           //关气泵
        memset(&how_set_timer, 0, sizeof(how_set_timer));               //清零气泵pwm输出
        set_display->G1display = (set_display->G1display & 0b11101111); //把第4位置零
        set_display->G1display = (set_display->G1display & 0b11011111); //把第5位置零
        set_display->G1display = (set_display->G1display & 0b10111111); //把第6位置零
        set_timer->timer_flag = 0;
        timer_on = 0;
        set_timer->timer_pwm_duty = 1; //设置－中－档pwm占空比
        TR0 = 0;                       //停止计数
        ET0 = 0;                       //停止计数中断
        break;
    case LEVEL_LOW:
        set_display->G1display |= led7;
        set_timer->timer_pwm_duty = 20; //设置－低－档pwm占空比
        set_timer->timer_flag = 1;
        break;
    case LEVEL_MIDD:
        set_display->G1display |= (led7 + led6);
        set_timer->timer_pwm_duty = 40; //设置－中－档pwm占空比
        set_timer->timer_flag = 1;
        break;
    case LEVEL_HIGH:
        set_display->G1display |= (led5 + led6 + led7);
        set_timer->timer_pwm_duty = 60; //设置－高－档pwm占空比
        set_timer->timer_flag = 1;
        break;
        // -------------------------------
    // Default event handler.
    default:
        break;
    }
}
//设定气泵档位
void PumpOutPut(unsigned char KeyLeve)
{
    //设置气阀先关阀，再扫描气泵档位
    Valve = 1;
    switch (KeyLeve)
    {
    case OFF:
        Valve = 0;                                                      //气阀打开，放气;
        Pump_motor = OFF;                                               //关气泵
        memset(&how_set_pump, 0, sizeof(how_set_pump));                 //清零气泵pwm输出
        set_display->G2display = (set_display->G2display & 0b11101111); //把第4位置零
        set_display->G2display = (set_display->G2display & 0b11011111); //把第5位置零
        set_display->G2display = (set_display->G2display & 0b10111111); //把第6位置零
        break;
    case LEVEL_LOW:
        Timer2Init();
        set_display->G2display |= led7;
        how_set_pump.pump_pwm_duty = 30; //设置－低－档pwm占空比
        break;
    case LEVEL_MIDD:
        Timer2Init();
        set_display->G2display |= (led7 + led6);
        how_set_pump.pump_pwm_duty = 60; //设置－中－档pwm占空比
        break;
    case LEVEL_HIGH:
        Timer2Init();
        set_display->G2display |= (led5 + led6 + led7);
        how_set_pump.pump_pwm_duty = 90; //设置－高－档pwm占空比
        break;
        // -------------------------------
    // Default event handler.
    default:
        break;
    }
}

//设定振动档位
void VibrationOutPut(unsigned char KeyLeve)
{
    switch (KeyLeve)
    {
    case OFF:
        // Timer2Off();                                    //关pwm
        set_display->G2display = (set_display->G2display & 0b11111101); //把第1位置零
        set_display->G2display = (set_display->G2display & 0b11111011); //把第2位置零
        set_display->G2display = (set_display->G2display & 0b11110111); //把第3位置零
        Vibration_motor = OFF;                                          //关振动
        memset(&how_set_vibration, 0, sizeof(how_set_vibration));       //清零振动pwm输出
        break;
    case LEVEL_LOW:
        Timer2Init();
        set_display->G2display |= led4;
        how_set_vibration.vibration_pwm_duty = 30; //设置－低－档pwm占空比
        break;
    case LEVEL_MIDD:
        Timer2Init();
        set_display->G2display |= (led3 + led4);
        how_set_vibration.vibration_pwm_duty = 60; //设置－中－档pwm占空比
        break;
    case LEVEL_HIGH:
        Timer2Init();
        set_display->G2display |= (led2 + led3 + led4);
        how_set_vibration.vibration_pwm_duty = 90; //设置－高－档pwm占空比
        break;
        // -------------------------------
    // Default event handler.
    default:
        break;
    }
}
//设定加热档位
void PTCOutPut(unsigned char KeyLeve)
{
    switch (KeyLeve)
    {
    case OFF:
        // Timer2Off();                                    //关pwm
        set_display->G1display = (set_display->G1display & 0b11111101); //把第1位置零
        set_display->G1display = (set_display->G1display & 0b11111011); //把第2位置零
        set_display->G1display = (set_display->G1display & 0b11110111); //把第3位置零
        PTC = OFF;                                                      //关气泵
        memset(&how_set_ptc, 0, sizeof(how_set_ptc));                   //清零加热pwm输出
        break;
    case LEVEL_LOW:
        set_display->G1display |= led4;
        Timer2Init();
        how_set_ptc.ptc_pwm_duty = 30; //设置－低－档pwm占空比
        break;
    case LEVEL_MIDD:
        set_display->G1display |= (led3 + led4);
        Timer2Init();
        how_set_ptc.ptc_pwm_duty = 60; //设置－中－档pwm占空比
        break;
    case LEVEL_HIGH:
        set_display->G1display |= (led2 + led3 + led4);
        Timer2Init();
        how_set_ptc.ptc_pwm_duty = 90; //设置－高－档pwm占空比
        break;
        // -------------------------------
    // Default event handler.
    default:
        break;
    }
}
//气泵控制
void PumpPWM(unsigned char pwm)
{
    if (how_set_pump.pump_timer < pwm) //高电平时间
        Pump_motor = 1;
    else
    {
        Pump_motor = 0; /* 低电平时间code */
    }
    if (how_set_pump.pump_timer > 100) //pwm占空比设定为100%
        how_set_pump.pump_timer = 0;
}
//振动控制
void VibrationPWM(unsigned char pwm)
{
    if (how_set_vibration.vibration_timer < pwm) //高电平时间
        Vibration_motor = 1;
    else
    {
        Vibration_motor = 0; /* 低电平时间code */
    }
    if (how_set_vibration.vibration_timer > 100) //pwm占空比设定为100%
        how_set_vibration.vibration_timer = 0;
}
//加热控制
void HeatPTCPWM(unsigned char pwm)
{
    if (how_set_ptc.ptc_timer < pwm) //高电平时间
        PTC = 1;
    else
    {
        PTC = 0; /* 低电平时间code */
    }
    if (how_set_ptc.ptc_timer > 100) //pwm占空比设定为100%
        how_set_ptc.ptc_timer = 0;
}
void TimerHandler(void)
{
    if (how_set_timer.timer_keyleve > 0)
        set_timer->timer_keyleve--;
    if (set_timer->timer_keyleve == 0)
        Power_switch = 0;                                           //auto trun off the power switch;
    set_display->G1display = (set_display->G1display & 0b11101111); //把第4位置零
    set_display->G1display = (set_display->G1display & 0b11011111); //把第5位置零
    set_display->G1display = (set_display->G1display & 0b10111111); //把第6位置零
    TimerOutPut(set_timer->timer_keyleve);
    display(set_display->G1display, GIRD2);

    if (set_timer->timer_timer > 60)
        set_timer->timer_timer = 0;
}
