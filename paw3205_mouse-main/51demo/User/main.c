#include <STC89C5xRC.H>
#include "config.h"
#include "bmp.h" 

/*****************************************任务函数声明***********************************************/

//任务调度机制
void Task_Init(void);
void TaskRun(void);

//具体任务函数
void Led_Task(void);
void key_Task(void);
void Oled_Task(void);
void Uart_Task(void);

/*****************************************外设全局变量***********************************************/

//led专用变量
uint8_t ucled;

//按键专用变量
uint8_t Key_Value;
uint8_t Key_Down;
uint8_t Key_Old;

//任务调度专用变量
uint16_t TaskTimer[TASKNUM_MAX];
TaskStruct Task[] = {
    {Led_Task, 1000},
	{key_Task, 20},
	//{Oled_Task, 2000},
    {Uart_Task, 0}
};
/*****************************************用户全局变量***********************************************/
sbit Beep = P2^5;
char idata str[50];
uint8_t idata uartRev[20];
int count = 0;
int sec = 0;
int i;
uint8_t prodId1 = 0;
uint8_t motion = 0;
signed char delta_x = 0;
signed char delta_y = 0;

/****************************************************************************************************/


/*------------------------------------------主函数--------------------------------------------------*/
int main(void)
{
	Task_Init();
    
    Beep = 0;
    
//	OLED_Init();//初始化OLED
//	OLED_ColorTurn(0);//0正常显示，1 反色显示
//    OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
    delay_ms(1000);
    
	Timer0_Init();
    Uart1_Init();
    
    // 鼠标初始化
    mouseInit();
    prodId1 = readRegister(PROD_ID1);
    UART_SendString((prodId1==0x30)?"\nDevice OK":"\nUnknown Device");
    sprintf(str, "\nprodId1 = %02x\n", (int)prodId1);
    UART_SendString(str);
    
    // 获取CH9392芯片版本号测试
    //Test1_RecVersion();
    
    // 设置外部中断0低电平触发
//    IT0 = 0;  // 设置 INT0 为低电平触发
//    EX0 = 1;  // 使能 INT0 中断
    //PX0 = 1;  // 高优先级
    
    EA = 1;   // 使能全局中断
	
	while(1)
	{
		TaskRun();
        
	}
}

/*-----------------------------------------中断服务函数--------------------------------------------------*/
void Timer0_Isr(void) interrupt 1
{
	uint8_t i;
    
    // 手动重装
    TL0 = 0x66;				//设置定时初始值
	TH0 = 0xFC;				//设置定时初始值
    
	//任务定时器递减
	for(i = 0; i < TASKNUM_MAX; i++)
		if(TaskTimer[i])
			TaskTimer[i]--;
}

void INT0_Isr(void) interrupt 0 
{
    // 中断服务程序，当 INT0 引脚检测到低电平时执行
    motion = readRegister(MOTION_STATUS);
    if(motion==0x82)
    {
        delta_x = (signed char)readRegister(DEL_X); // read delta x 
        delta_y = (signed char)readRegister(DEL_Y); // read delta y
        
//        sprintf(str, "delta_x = %d\ndelta_y = %d\n", (int)delta_x, (int)delta_y);
//        UART_SendString(str);
        Test3_Mouse(delta_x, -delta_y);
        //delay_ms(1);
    }
}

void Uart1_Isr(void) interrupt 4
{
	if (RI)				//检测串口1接收中断
	{
        // 获取CH9392芯片版本号测试
//        uartRev[i] = SBUF;
//        i++;
        
		RI = 0;			//清除串口1接收中断请求位
	}
    
//    if (TI)				//检测串口1发送中断
//    {
//        
//        TI = 0;
//    }
}

/*-----------------------------------------具体任务--------------------------------------------------*/
void Led_Task(void)
{
    P2 ^= 1;
}	

void key_Task(void)
{
	//三行代码
	Key_Value = Key_Read();
	Key_Down = Key_Value&(Key_Value^Key_Old);
	Key_Old = Key_Value;
	
	switch(Key_Down)
	{
		case 0:
			
		break;
	}
}

void Oled_Task(void)
{    
    EA = 0;
    
    sprintf(str, "count = %d", count++);
    OLED_ShowString(0,0,str,16);
    //memset(str, 0, sizeof(str));
    
    sprintf(str, "x: %d,y: %d", (int)delta_x, (int)delta_y);
    OLED_ShowString(0,2,str,16);
    
    // CH9392回复数据包
//    sprintf(str, "%02x,%02x,%02x,%02x,", (int)uartRev[0],(int)uartRev[1],(int)uartRev[2],(int)uartRev[3]);
//    OLED_ShowString(0,2,str,8);
//    sprintf(str, "%02x,%02x,%02x,%02x,", (int)uartRev[4],(int)uartRev[5],(int)uartRev[6],(int)uartRev[7]);
//    OLED_ShowString(0,3,str,8);
//    sprintf(str, "%02x,%02x,%02x,%02x,", (int)uartRev[8],(int)uartRev[9],(int)uartRev[10],(int)uartRev[11]);
//    OLED_ShowString(0,4,str,8);
//    sprintf(str, "%02x,%02x,%02x,%02x,", (int)uartRev[12],(int)uartRev[13],(int)uartRev[14],(int)uartRev[15]);
//    OLED_ShowString(0,5,str,8);
//    sprintf(str, "%02x,%02x,%02x,%02x,", (int)uartRev[16],(int)uartRev[17],(int)uartRev[18],(int)uartRev[19]);
//    OLED_ShowString(0,6,str,8);
    
    EA = 1;
}

void Uart_Task(void)
{
    if(digitalRead(MOTSWK) == LOW)
    {
        motion = readRegister(MOTION_STATUS);
        if(motion==0x82)
        {
            delta_x = (signed char)readRegister(DEL_X); // read delta x 
            delta_y = (signed char)readRegister(DEL_Y); // read delta y
            
//            sprintf(str, "delta_x = %d\ndelta_y = %d\n", (int)delta_x, (int)delta_y);
//            UART_SendString(str);
            
            Test3_Mouse(delta_x, -delta_y);
        }
    }
    
    
}
/*-----------------------------------------任务调度--------------------------------------------------*/
void Task_Init(void)
{
	uint8_t NTask;
	for(NTask = 0; NTask < sizeof(Task)/sizeof(Task[0]); NTask++)
	{
		TaskTimer[NTask] = Task[NTask].TaskPeriod;
	}
}

void TaskRun(void)
{
	uint8_t NTask;
	for(NTask = 0; NTask < sizeof(Task)/sizeof(Task[0]); NTask++)
	{
		if(TaskTimer[NTask] == 0)
		{
			TaskTimer[NTask] = Task[NTask].TaskPeriod;
			(Task[NTask].pTask)();
		}
	}
}















