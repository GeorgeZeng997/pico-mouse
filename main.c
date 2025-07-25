/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "bsp/board_api.h"
#include "tusb.h"

#include "usb_descriptors.h"

// 外部函数声明
extern void paw3205_task(void);
extern int8_t mouse_delta_x, mouse_delta_y;
extern uint8_t mouse_motion_status;

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// 摇杆ADC引脚定义
#define JOYSTICK_X_PIN 26    // ADC0 - GPIO26
#define JOYSTICK_Y_PIN 27    // ADC1 - GPIO27
#define MOUSE_BUTTON_PIN 28  // GPIO28 - 数字输入

// 鼠标按键引脚定义
#define MOUSE_LEFT_BUTTON_PIN 14   // GPIO14 - 左键
#define MOUSE_RIGHT_BUTTON_PIN 15  // GPIO15 - 右键

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static volatile struct {
    uint8_t buttons;
    int8_t x;
    int8_t y;
    int8_t wheel;
    int8_t pan;
    bool has_data;
} mouse_cmd = {0};


static uint32_t mouse_move_x = 0;
static uint32_t mouse_move_y = 0;
static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

// CDC命令阻塞摇杆控制的时间
static uint32_t cdc_command_time = 0;
static const uint32_t CDC_BLOCK_TIMEOUT = 500;  // 500ms

// 灵敏度调节相关变量
static uint8_t sensitivity_level = 2;  // 默认灵敏度等级 (1-3, 2为中等灵敏度)
static uint32_t last_button_press_time = 0;
static uint8_t button_press_count = 0;
static const uint32_t BUTTON_PRESS_TIMEOUT = 1000;  // 1秒超时

// 鼠标按键初始化函数
void mouse_buttons_init(void) {
    // 初始化左键引脚
    gpio_init(MOUSE_LEFT_BUTTON_PIN);
    gpio_set_dir(MOUSE_LEFT_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(MOUSE_LEFT_BUTTON_PIN);  // 上拉，按下时为低电平
    
    // 初始化右键引脚
    gpio_init(MOUSE_RIGHT_BUTTON_PIN);
    gpio_set_dir(MOUSE_RIGHT_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(MOUSE_RIGHT_BUTTON_PIN);  // 上拉，按下时为低电平
}

// 读取鼠标按键状态
uint8_t read_mouse_buttons(void) {
    uint8_t buttons = 0;
    
    // 读取左键状态 (按下时为低电平，所以需要取反)
    if (!gpio_get(MOUSE_LEFT_BUTTON_PIN)) {
        buttons |= 0x01;  // 左键位
    }
    
    // 读取右键状态 (按下时为低电平，所以需要取反)
    if (!gpio_get(MOUSE_RIGHT_BUTTON_PIN)) {
        buttons |= 0x02;  // 右键位
    }
    
    return buttons;
}

// 摇杆初始化函数
void joystick_init(void) {
    // 初始化ADC
    adc_init();
    
    // 初始化ADC引脚
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);
    
    // 初始化按键引脚
    gpio_init(MOUSE_BUTTON_PIN);
    gpio_set_dir(MOUSE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(MOUSE_BUTTON_PIN);  // 上拉，按下时为低电平
}

// 读取摇杆值
void read_joystick(int8_t *x, int8_t *y, uint8_t *button) {
    // 读取X轴 (ADC0)
    adc_select_input(0);
    uint16_t x_raw = adc_read();
    
    // 读取Y轴 (ADC1) 
    adc_select_input(1);
    uint16_t y_raw = adc_read();
    
    // 读取按键状态
    *button = !gpio_get(MOUSE_BUTTON_PIN);  // 取反，因为使用上拉
    
    // 将ADC值(0-4095)转换为鼠标移动值(-127到+127)
    // 2048是中心位置
    const uint16_t center = 2048;
    const uint16_t deadzone = 100;  // 死区，避免漂移
    
    // X轴处理
    if (x_raw > center + deadzone) {
        *x = (int8_t)((x_raw - center - deadzone) * 127 / (4095 - center - deadzone));
    } else if (x_raw < center - deadzone) {
        *x = (int8_t)((x_raw - center + deadzone) * 127 / (center - deadzone));
    } else {
        *x = 0;
    }
    
    // Y轴处理 (反转方向)
    if (y_raw > center + deadzone) {
        *y = (int8_t)((y_raw - center - deadzone) * 127 / (4095 - center - deadzone));
    } else if (y_raw < center - deadzone) {
        *y = (int8_t)((y_raw - center + deadzone) * 127 / (center - deadzone));
    } else {
        *y = 0;
    }
    
    // 限制在合理范围内
    if (*x > 127) *x = 127;
    if (*x < -127) *x = -127;
    if (*y > 127) *y = 127;
    if (*y < -127) *y = -127;
}



void led_blinking_task(void);
void hid_task(void);
// CDC任务：接收并解析串口协议
void cdc_task(void)
{
    if ( tud_cdc_available() )
    {
        char buf[64] = {0};
        uint32_t count = tud_cdc_read(buf, sizeof(buf)-1);
        if (count > 0) {
              // 协议：55 btn x y wheel pan sum
            int head, btn, x, y, wheel, pan, sum;
            if (sscanf(buf, "%d %d %d %d %d %d %d", &head, &btn, &x, &y, &wheel, &pan, &sum) == 7) {
                int calc_sum = btn + x + y + wheel + pan;
                if (head == 55 && calc_sum == sum) {
                    mouse_cmd.buttons = (uint8_t)btn;
                    mouse_cmd.x = (int8_t)x;
                    mouse_cmd.y = (int8_t)y;
                    mouse_cmd.wheel = (int8_t)wheel;
                    mouse_cmd.pan = (int8_t)pan;
                    mouse_cmd.has_data = true;
                    cdc_command_time = board_millis();  // 记录CDC命令时间
                    tud_cdc_write_str("ok\n");
                } else {
                    tud_cdc_write_str("error\n");
                }
            } else {
                tud_cdc_write_str("format\n");
            }
            tud_cdc_write_flush();
        }
    }
}
/*------------- MAIN -------------*/
int main(void)
{
  board_init();
  stdio_init_all();
  
  // 初始化随机数种子
  srand(time(NULL));
  
  // 初始化鼠标按键
  mouse_buttons_init();
  
  // 摇杆初始化已被取消
  // joystick_init();
  
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }
  
  // 启动PAW3205任务在第二个核心
  multicore_launch_core1(paw3205_task);
  
  while (1)
  {
    tud_task(); // tinyusb device task
    led_blinking_task();

    hid_task();
    cdc_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, uint32_t btn)
{
  // skip if hid is not ready yet
  if ( !tud_hid_ready() ) return;

  switch(report_id)
  {
    case REPORT_ID_KEYBOARD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_keyboard_key = false;

      if ( btn )
      {
        uint8_t keycode[6] = { 0 };
        keycode[0] = HID_KEY_A;

        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
        has_keyboard_key = true;
      }else
      {
        // send empty key report if previously has key pressed
        if (has_keyboard_key) tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
        has_keyboard_key = false;
      }
    }
    break;

    case REPORT_ID_MOUSE:
    {
      int8_t const delta = 5;

      // no button, right + down, no scroll, no pan
      tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta, delta, 0, 0);
    }
    break;

    case REPORT_ID_CONSUMER_CONTROL:
    {
      // use to avoid send multiple consecutive zero report
      static bool has_consumer_key = false;

      if ( btn )
      {
        // volume down
        uint16_t volume_down = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
        tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &volume_down, 2);
        has_consumer_key = true;
      }else
      {
        // send empty key report (release key) if previously has key pressed
        uint16_t empty_key = 0;
        if (has_consumer_key) tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &empty_key, 2);
        has_consumer_key = false;
      }
    }
    break;

    case REPORT_ID_GAMEPAD:
    {
      // use to avoid send multiple consecutive zero report for keyboard
      static bool has_gamepad_key = false;

      hid_gamepad_report_t report =
      {
        .x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0,
        .hat = 0, .buttons = 0
      };

      if ( btn )
      {
        report.hat = GAMEPAD_HAT_UP;
        report.buttons = GAMEPAD_BUTTON_A;
        tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));

        has_gamepad_key = true;
      }else
      {
        report.hat = GAMEPAD_HAT_CENTERED;
        report.buttons = 0;
        if (has_gamepad_key) tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
        has_gamepad_key = false;
      }
    }
    break;

    default: break;
  }
}

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
// void hid_task(void)
// {
//   // Poll every 10ms
//   const uint32_t interval_ms = 10;
//   static uint32_t start_ms = 0;

//   if ( board_millis() - start_ms < interval_ms) return; // not enough time
//   start_ms += interval_ms;

//   uint32_t const btn = board_button_read();

//   // Remote wakeup
//   if ( tud_suspended() && btn )
//   {
//     // Wake up host if we are in suspend mode
//     // and REMOTE_WAKEUP feature is enabled by host
//     tud_remote_wakeup();
//   }else
//   {
//     // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
//     send_hid_report(REPORT_ID_KEYBOARD, btn);
//   }
// }
// main.c 片段

void hid_task(void)
{
  static uint32_t start_ms = 0;
  if (board_millis() - start_ms < 1) return;  // 5ms间隔
  start_ms += 1;
  
  // 检查HID设备是否就绪
  if (!tud_hid_ready()) return;
  
  // 优先处理CDC命令
  if (mouse_cmd.has_data){
    tud_hid_mouse_report(REPORT_ID_MOUSE, mouse_cmd.buttons, mouse_cmd.x, mouse_cmd.y, mouse_cmd.wheel, mouse_cmd.pan);
    mouse_cmd.has_data = false;
    cdc_command_time = board_millis();
  }  // 处理PAW3205传感器数据
  else if (board_millis() - cdc_command_time < 50) {
    // 不处理PAW3205等其他输入
    return;
  }
  else if (mouse_motion_status == 0x82) {
    // 不使用缩放系数，直接传递原始数据
    int16_t scaled_x = mouse_delta_x * 1;
    int16_t scaled_y = -mouse_delta_y * 1;  // Y轴反转但不缩放
    
    // 限制范围在int8_t有效范围内 (-127 到 +127)
    int8_t send_x = 0;
    int8_t send_y = 0;
    
    if (scaled_x > 120) send_x = 120;
    else if (scaled_x < -120) send_x = -120;
    else send_x = (int8_t)scaled_x;
    
    if (scaled_y > 120) send_y = 120;
    else if (scaled_y < -120) send_y = -120;
    else send_y = (int8_t)scaled_y;
    
    // 读取鼠标按键状态
    uint8_t buttons = read_mouse_buttons();
    
    // 发送鼠标移动报告
    tud_hid_mouse_report(REPORT_ID_MOUSE, buttons, send_x, send_y, 0, 0);
    
    // 清零运动数据，避免重复发送
    mouse_motion_status = 0;
  }
  // 处理仅按键状态变化（无运动时）
  else {
    static uint8_t last_buttons = 0;
    uint8_t current_buttons = read_mouse_buttons();
    
    // 如果按键状态发生变化，发送鼠标报告
    if (current_buttons != last_buttons) {
      tud_hid_mouse_report(REPORT_ID_MOUSE, current_buttons, 0, 0, 0, 0);
      last_buttons = current_buttons;
    }
  }
  //mouse_delta_x = mouse_delta_y = 0;  // 重置移动数据
}


// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) instance;
  (void) len;
  (void) report;
  
  // 注释掉自动发送链，只在需要时手动发送
  // uint8_t next_report_id = report[0] + 1u;
  // if (next_report_id < REPORT_ID_COUNT)
  // {
  //   send_hid_report(next_report_id, board_button_read());
  // }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD)
    {
      // bufsize should be (at least) 1
      if ( bufsize < 1 ) return;

      uint8_t const kbd_leds = buffer[0];

      if (kbd_leds & KEYBOARD_LED_CAPSLOCK)
      {
        // Capslock On: disable blink, turn led on
        blink_interval_ms = 0;
        board_led_write(true);
      }else
      {
        // Caplocks Off: back to normal blink
        board_led_write(false);
        blink_interval_ms = BLINK_MOUNTED;
      }
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms) return;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
