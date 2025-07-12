#ifndef	__CONFIG_H__
#define __CONFIG_H__

typedef unsigned char 			uint8_t;
typedef unsigned short int 	    uint16_t;
typedef unsigned int 		    uint32_t;

/*************************宏定义*******************************/
#define TASKNUM_MAX	4

/*************************结构体*******************************/
typedef struct{
	void (*pTask)(void);
	uint16_t TaskPeriod;
}TaskStruct;

/*************************常用头文件*******************************/
#include <STC89C5xRC.H>
#include <stdio.h>
#include <string.h>
#include <intrins.h>

/*************************外设头文件*******************************/
#include "bsp_Init.h"
#include "led.h"
#include "key.h"
#include "seg.h"
#include "ds1302.h"
#include "ds18b20.h"
#include "i2c.h"
#include "timer.h"

#include "oled.h"
#include "dht11.h"
#include "uart.h"
#include "paw3205.h"
#include "delay.h"

#endif
















