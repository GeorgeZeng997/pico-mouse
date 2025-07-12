#include "uart.h"

void Uart1_Init(void)	//9600bps@11.0592MHz
{
	PCON &= 0x7F;		//波特率不倍速
	SCON = 0x50;		//8位数据,可变波特率
//	AUXR &= 0xBF;		//定时器时钟12T模式
//	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设置定时器模式
	TMOD |= 0x20;		//设置定时器模式
	TL1 = 0xFD;			//设置定时初始值
	TH1 = 0xFD;			//设置定时重载值
	ET1 = 0;			//禁止定时器中断
	TR1 = 1;			//定时器1开始计时
	ES = 1;				//使能串口1中断
}

void UART_SendByte(uint8_t Byte)
{
    SBUF = Byte;
    while(TI == 0);
    TI = 0;
}

void UART_SendString(uint8_t *str)
{
    while(*str != '\0')
    {
        UART_SendByte(*(str++));
    }   
}

// 获取CH9392芯片版本号测试
void Test1_RecVersion(void)
{
    UART_SendByte(0x57);
    UART_SendByte(0xab);
    UART_SendByte(0x00);
    UART_SendByte(0x01);
    UART_SendByte(0x00);
    UART_SendByte(0x03);    
}

// 普通键盘测试
void Test2_RecKey(void)
{
    //模拟按下A
    UART_SendByte(0x57);
    UART_SendByte(0xab);
    UART_SendByte(0x00);
    UART_SendByte(0x02);
    UART_SendByte(0x08);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x04);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x10);
    //模拟释放A
    UART_SendByte(0x57);
    UART_SendByte(0xab);
    UART_SendByte(0x00);
    UART_SendByte(0x02);
    UART_SendByte(0x08);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x0c);
}

// 相对鼠标位移测试
void Test3_Mouse(signed char delta_x, signed char delta_y)
{
    uint8_t sum;
    
    UART_SendByte(0x57);
    UART_SendByte(0xab);
    UART_SendByte(0x00);
    UART_SendByte(0x05);
    UART_SendByte(0x05);
    UART_SendByte(0x01);// 第 1 个字节：必须为 0x01
    UART_SendByte(0x00);// 第 2 个字节： 1 个字节的鼠标按键值，最低 3 位每位表示 1 个按键
    UART_SendByte((uint8_t)delta_x);// 第 3 个字节： 1 个字节 X 方向(纵坐标，上下方向)移动距离
    UART_SendByte((uint8_t)delta_y);// 第 4 个字节： 1 个字节 Y 方向(纵坐标，上下方向)移动距离
    UART_SendByte(0x00);// 第 5 个字节： 1 个字节滚轮滚动齿数
    
    sum = (uint8_t)(0x57+0xab+0x00+0x05+0x05+0x01+0x00+(uint8_t)delta_x+(uint8_t)delta_y+0x00);
    UART_SendByte(sum);
}




