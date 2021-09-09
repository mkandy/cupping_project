
#include <start.h>

unsigned char LED1, LED2;

void Timer_Reset(void)
{
    Time.count = 0;
    Time.sec = 0;
    Time.min = 0;
}

void Time_handler(void) //Timer 0 is 50ms period,
{
    Time.update = 0;
    Time.count++;

    if ((Time.count % 10) == 0)
        Time.quartersec = !Time.quartersec;

    if (Time.count > 19)
    {
        Time.sec++;
        Time.count = 0;
    }

    if (Time.sec > 59)
    {
        Time.sec = 0;

        Time.min++;
    }

    if (Time.min > 59)
    {
        Time.min = 0;
    }
      if ((Power.level==3) &&(Time.min >= Time3))
        {
            Power.level=2;

            Time.min=0;
            Display_handler();
        }
   if ((Power.level==2) &&(Time.min >= Time2))
        {
            Power.level=1;
           
            Time.min=0;
            Display_handler();
        }
 

     if ((Power.level==1) &&(Time.min >= Time1))
        {
            Power.level=0;

            state=Power_down;
            Display_handler();
            Time.min=0;
        }  

    if ((!INT0) || (!INT1) || (!INT2) || (!INT3)) //if one of the key is release timer is reset
    {
        Key.debounce++;
    }

    else
    {
        Key.update = 0;
        Key.pressed = 0;
        Key.debounce = 0;
        Key.long_press = 0;
    }

  
   /*  if (Key.debounce > 50) //debounce
    {
        if ((!INT0) || (!INT1) || (!INT2) || (!INT3))
        {
            Key.long_press++;
            Key.debounce = 0;
            
        }

        else
        {

            Key.update = 1;
            Key.pressed = 0;
            Key.long_press = 0;
            Key.long_press_state = 0;
            Key.debounce = 0;
        }
        
    if (Key.long_press > 3)
        {
            
            state=Power_down;
           
        } 
    }*/
    

      if (Key.debounce > 4)

    {

        
        Key.update=1;
        Key.debounce = 0;
        Key.long_press = 0;
    }

}

void key_up(LEVEL_STAT *this_key)
{

    this_key->level++;
    if (this_key->level > (Max_key - 1))
        this_key->level = 0;
    if (this_key->level == 0)
        this_key->on = 0;
    else
        this_key->on = 1;
}

void Key_handler(void)

{
    Timer_Reset();


    Key.which_press = Key_pressed;
    if ((Key.which_press == Key_Power) && (Key.long_press_state))

    {

        state = Power_down;

        //如果检测到长时间按键则打开电源及升压

        //

        //

        //
    }
    else
    {

        switch (Key.which_press)
        {

        case Key_PTC:
            // IO_PTC_LED=!IO_PTC_LED;
            // PWM.value = 2000;
            // LoadPWM(PWM.value);
            // Timer2_init();
            // Heating.on = !Heating.on; //启动IO_PTC加热
            key_up(&Heating);

            break;
        case Key_Vibration:
            // Vibration_LED=!Vibration_LED;

            // IO_Vibration = !IO_Vibration;
            key_up(&Vibration);

            break;
        case Key_Pump:

            key_up(&Suction);

            break;
        case Key_Power:
            //Power_LED=!Power_LED;

            // IO_Valve = !IO_Valve;
            key_up(&Power);
            switch (Power.level)
                {
                case 0:
                    state = Power_down;
                    break;
                case 1:
                    duration = Time1;
                   
                    break;

                case 2:
                    duration = Time2+Time1;
                    
                    break;
                case 3:
                    duration = Time3+Time2+Time1;
                   
                    break;

                default:
                    break;
                }
            break;
            // -------------------------------
            // Default event handler.
        default:
            break;
        }
    }
}

void IO_handler(void)
{
    if (Heating.on)
        IO_PTC = 1;
    else
    {
        IO_PTC = 0;
        Heating.level = 0;
    }

    if (Vibration.on)
        IO_Vibration = 0;
    else
    {
        IO_Vibration = 1;
        Vibration.level = 0;
    }

    if (Suction.on)
        IO_Pump = 1;
    else
    {
        IO_Pump = 0;
        Suction.level = 0;
    }

    if (state != Power_down)
    {
        IO_Power = 1;

        if (Power.level == 0)
            Power.level = 1;
    }

    else
        IO_Power = 0;
}

void Display_handler(void)
{
    u8 level_val[Max_key] = {0, 1, 3, 7};
    u8 display_val = 0;

    display_val = (level_val[Vibration.level]) & 0b0000111;
    display_val |= (level_val[Suction.level] << 3) & 0b0111000;

    LED1 = display_val;

    display_val = (level_val[Heating.level]) & 0b0000111;
    display_val |= (level_val[Power.level] << 3) & 0b0111000;

    LED2 = display_val;

    display(LED2, GIRD2);
    display(LED1, GIRD1);
}
