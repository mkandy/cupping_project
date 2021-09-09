
#include "start.h"
#define MAIN_Fosc 11059200L
void main(void)
{
     #if(Seril_Debug==1) 
UART1_config();
#endif


Start();
}


