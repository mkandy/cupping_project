#include <stc12.h>
#define MAIN_Fosc 12000000L
#define LED P1_1
void delay_ms(unsigned int ms){
    unsigned int i;
    do{
        i = MAIN_Fosc / 13000;
        while (--i);
    } while (--ms);
}

void main(){
    while (1){
        LED = 0;
        delay_ms(100);
        LED = 1;
        delay_ms(100);
    }
}
