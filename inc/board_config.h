#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

// ============================================================================
// 硬件引脚定义 (RP2040 / YD-RP2040)
// ============================================================================

// PAW3205DB-TJ3T 光学传感器半双工 SPI 引脚
#define PIN_PAW3205_SCLK        4   // GPIO4 -> SCLK
#define PIN_PAW3205_SDIO        5   // GPIO5 -> SDIO (双向数据线)
#define PIN_PAW3205_MOTSWK      6   // GPIO6 -> MOTSWK (运动唤醒/中断低有效)

// 物理鼠标微动按键引脚 (内部上拉，按下为低电平)
#define PIN_MOUSE_LEFT_BTN      14  // GPIO14 -> 左键
#define PIN_MOUSE_RIGHT_BTN     15  // GPIO15 -> 右键

// ============================================================================
// 采样率、回报率与消抖参数
// ============================================================================

// USB HID 鼠标报文合成周期 (1ms = 1000Hz)
#define HID_REPORT_INTERVAL_MS  1

// Core 1 PAW3205 传感器轮询间隔 (1000us = 1000Hz，远高于旧版 15ms/66Hz)
#define PAW3205_POLL_INTERVAL_US 1000

// 物理微动开关状态稳定消抖时间 (ms)
#define BUTTON_DEBOUNCE_MS      10

// ============================================================================
// 传感器灵敏度定点缩放系数 (默认 1.5x = 3 / 2，带余数累加防截断)
// ============================================================================
#define SENSOR_SCALE_NUM        3
#define SENSOR_SCALE_DEN        2

// 单帧 HID 报文最大位移限幅 (-127 ~ +127)，超出的余量自动保留到下一毫秒发送
#define HID_MAX_DELTA           127

// ============================================================================
// CDC 串口协议配置
// ============================================================================
// 帧头标识：格式为 "55 <btn> <x> <y> <wheel> <pan> <sum>\n"
#define CDC_PROTOCOL_HEADER     55

// 是否在收到每帧串口指令后向主机回传 "ok\n" (默认关闭，避免阻塞高频串口流与塞满主机 RX 缓冲)
#define CDC_ENABLE_ACK          0

// ============================================================================
// USB 设备描述符伪装参数 (Logitech M105 Optical Mouse)
// ============================================================================
#define USB_DEVICE_VID          0x046D
#define USB_DEVICE_PID          0xC077
#define USB_DEVICE_BCD          0x0200

#define USB_STR_MANUFACTURER    "Logitech"
#define USB_STR_PRODUCT         "USB Optical Mouse"

#endif /* BOARD_CONFIG_H_ */
