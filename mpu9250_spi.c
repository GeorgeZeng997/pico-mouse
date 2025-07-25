/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "bsp/board_api.h"
#include "tusb.h"

/* Example code to talk to a PAW3205 optical mouse sensor.
   
   This uses a custom SPI-like protocol specific to the PAW3205 sensor.
   The PAW3205 uses a single bidirectional data line (SDIO) and a clock line (SCLK).
   
   Connections on Raspberry Pi Pico board and PAW3205 sensor board:
   GPIO 4 (pin 6) -> SCLK on PAW3205 board
   GPIO 5 (pin 7) -> SDIO on PAW3205 board  
   GPIO 6 (pin 9) -> MOTSWK on PAW3205 board (optional, for motion detection)
   3.3v (pin 36) -> VCC on PAW3205 board
   GND (pin 38)  -> GND on PAW3205 board
*/

// PAW3205 引脚定义
#define PIN_SCLK  4
#define PIN_SDIO  5
#define PIN_MOTSWK 6

// PAW3205 寄存器地址
#define PROD_ID1 0x00
#define PROD_ID2 0x01
#define MOTION_STATUS 0x02
#define DEL_X 0x03
#define DEL_Y 0x04
#define OP_MODE 0x05
#define CONF 0x06
#define IMG_QUALITY 0x07
#define OP_STATE 0x08
#define W_PROTECT 0x09
#define SLEEP1_SET 0x0A
#define ENTER_TIME 0x0B
#define SLEEP2_SET 0x0C
#define IMG_THRESHOLD 0x0D
#define IMG_RECOG 0x0E

#define NO_SLEEP 0xA0

// 函数声明
void paw3205_init(void);
uint8_t paw3205_read_register(uint8_t address);
void paw3205_write_register(uint8_t address, uint8_t data);
void paw3205_read_motion(int8_t *delta_x, int8_t *delta_y, uint8_t *motion_status);

// PAW3205 初始化函数
void paw3205_init(void) {
    // 初始化时钟引脚
    gpio_init(PIN_SCLK);
    gpio_set_dir(PIN_SCLK, GPIO_OUT);
    gpio_put(PIN_SCLK, 1);
    
    // 初始化数据引脚
    gpio_init(PIN_SDIO);
    gpio_set_dir(PIN_SDIO, GPIO_OUT);
    gpio_put(PIN_SDIO, 0);
    
    // 初始化MOTSWK引脚（可选）
    gpio_init(PIN_MOTSWK);
    gpio_set_dir(PIN_MOTSWK, GPIO_IN);
    gpio_pull_up(PIN_MOTSWK); // 启用内部上拉
    
    // 传感器初始化序列 - 按照datasheet精确时序
    gpio_put(PIN_SCLK, 1);
    gpio_put(PIN_SCLK, 0);
    sleep_us(1); // tRESYNC = 1us
    gpio_put(PIN_SCLK, 1);
    sleep_ms(320); // tSIWTT = 320ms - 等待传感器就绪
    gpio_put(PIN_SCLK, 0);
    
    // 额外等待，确保传感器完全就绪
    sleep_ms(10);
    
    // 禁用睡眠模式
    paw3205_write_register(OP_MODE, NO_SLEEP);
    
    // 等待配置生效
    sleep_ms(10);
    
    // 设置图像质量阈值（可选）
    //paw3205_write_register(IMG_THRESHOLD, 0xff);
    
    sleep_ms(10);

    paw3205_write_register(CONF, 0x04);
    // 最终等待，确保所有配置生效
    sleep_ms(50);
}

// PAW3205 读取寄存器函数
uint8_t paw3205_read_register(uint8_t address) {
    uint8_t i;
    uint8_t result = 0;
    
    // 设置SDIO为输出模式
    gpio_set_dir(PIN_SDIO, GPIO_OUT);
    
    // 发送地址（8位）- 确保时钟从低电平开始
    for (i = 128; i > 0; i >>= 1) {
        gpio_put(PIN_SCLK, 0);
        gpio_put(PIN_SDIO, (address & i) != 0 ? 1 : 0);
        sleep_us(10); // 添加小延时确保信号稳定
        gpio_put(PIN_SCLK, 1);
        sleep_us(10); // 添加小延时确保信号稳定
    }
    
    // 设置SDIO为输入模式，并添加上拉
    gpio_set_dir(PIN_SDIO, GPIO_IN);
    gpio_pull_up(PIN_SDIO);
    
    sleep_us(30); // tHOLD = 3us - 等待传感器响应
    
    // 读取数据（8位）- 确保时钟从低电平开始
    for (i = 128; i > 0; i >>= 1) {
        gpio_put(PIN_SCLK, 0);
        sleep_us(10); // 确保时钟低电平稳定
        gpio_put(PIN_SCLK, 1);
        sleep_us(10); // 确保时钟高电平稳定
        if (gpio_get(PIN_SDIO)) {
            result |= i;
        }
    }
    
    sleep_us(30); // tHOLD = 3us - 最终保持时间
    
    return result;
}

// PAW3205 写入寄存器函数
void paw3205_write_register(uint8_t address, uint8_t data) {
    uint8_t i;
    // 先取消写保护
    if ((address & 0x7F) != W_PROTECT) {
        // 只要不是自己写W_PROTECT寄存器，先解锁
        paw3205_write_register(W_PROTECT, 0x5A);
        sleep_ms(10);
    }
    address |= 0x80; // MSB 表示写模式
    
    // 设置SDIO为输出模式
    gpio_set_dir(PIN_SDIO, GPIO_OUT);
    
    // 发送地址（8位）- 确保时钟从低电平开始
    for (i = 128; i > 0; i >>= 1) {
        gpio_put(PIN_SCLK, 0);
        gpio_put(PIN_SDIO, (address & i) != 0 ? 1 : 0);
        sleep_us(10); // 添加小延时确保信号稳定
        gpio_put(PIN_SCLK, 1);
        sleep_us(10); // 添加小延时确保信号稳定
    }
    
    // 发送数据（8位）- 确保时钟从低电平开始
    for (i = 128; i > 0; i >>= 1) {
        gpio_put(PIN_SCLK, 0);
        gpio_put(PIN_SDIO, (data & i) != 0 ? 1 : 0);
        sleep_us(10); // 添加小延时确保信号稳定
        gpio_put(PIN_SCLK, 1);
        sleep_us(10); // 添加小延时确保信号稳定
    }
    
    // 保持时钟低电平
    gpio_put(PIN_SCLK, 0);
    sleep_us(100); // 写入后等待时间
}
// 全局变量存储运动数据
int8_t mouse_delta_x = 0;
int8_t mouse_delta_y = 0;
uint8_t mouse_motion_status = 0;
bool paw3205_debug_enabled = false;  // 调试开关
// PAW3205 读取运动数据
void paw3205_read_motion(int8_t *delta_x, int8_t *delta_y, uint8_t *motion_status) {
    // 首先检查MOTSWK引脚 - 如果为高电平，没有运动
    if (gpio_get(PIN_MOTSWK)) {
        *motion_status = 0;
        *delta_x = 0;
        *delta_y = 0;
        return;
    }
    // 读取运动状态寄存器
    *motion_status = paw3205_read_register(MOTION_STATUS);
    // 检查是否有有效运动 (Arduino代码中检查0x82)
    if (*motion_status == 0x82) {  // 运动检测位
        // 读取X和Y位移
        int8_t dx = (int8_t)paw3205_read_register(DEL_X);
        int8_t dy = (int8_t)paw3205_read_register(DEL_Y);
        // 增强异常值过滤：极端跳变值直接丢弃
        // if (dx == (int8_t)0x80 || dy == (int8_t)0x80 || dx == 127 || dx == -127 || dy == 127 || dy == -127) {
        //     dx = 0;
        //     dy = 0;
        // }
        // // 滤掉异常大跳变并通过CDC打印
        // if (dx > 50 || dx < -50 || dy > 50 || dy < -50) {
        //     if (paw3205_debug_enabled && tud_cdc_connected()) {
        //         char dbg[64];
        //         snprintf(dbg, sizeof(dbg), "异常跳变: dx=%d dy=%d\r\n", dx, dy);
        //         tud_cdc_write_str(dbg);
        //         tud_cdc_write_flush();
        //     }
        //     dx = 0;
        //     dy = 0;
        // }
        *delta_x = dx;
        *delta_y = dy;
    } else {
        *delta_x = 0;
        *delta_y = 0;
    }
}



void paw3205_task(void) {
    // 初始化PAW3205传感器
    paw3205_init();
    // 确保传感器处于活动模式
    paw3205_write_register(OP_MODE, NO_SLEEP);
    sleep_ms(10);
    // 主循环
    while (1) {
        // 读取运动数据
        paw3205_read_motion(&mouse_delta_x, &mouse_delta_y, &mouse_motion_status);
        // CDC调试输出
        if (paw3205_debug_enabled && tud_cdc_connected()) {
            char dbg[64];
            snprintf(dbg, sizeof(dbg), "PAW3205: status=0x%02X dx=%d dy=%d\r\n", mouse_motion_status, mouse_delta_x, mouse_delta_y);
            tud_cdc_write_str(dbg);
            tud_cdc_write_flush();
        }
        // 延时，避免过度占用CPU（适当加大采样间隔）
        sleep_ms(15);
    }
}
