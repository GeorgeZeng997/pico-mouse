#include "delay.h"

void Delay1ms(void)	//@11.0592MHz
{
	unsigned char data i, j;

	_nop_();
	i = 2;
	j = 199;
	do
	{
		while (--j);
	} while (--i);
}


void delay_ms(unsigned int ms)
{                         
	while(ms--)
    {
        Delay1ms();
    }
}

//void Delay1us(void)	//@11.0592MHz
//{
//	_nop_();
//}

void delay_us(unsigned int us)
{                         
	while(us--);
//    {
//        Delay1us();
//    }
}




















