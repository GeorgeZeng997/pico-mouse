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
#include "hardware/adc.h"
#include "hardware/gpio.h"

#include "bsp/board_api.h"
#include "tusb.h"

#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// 摇杆ADC引脚定义
#define JOYSTICK_X_PIN 26    // ADC0 - GPIO26
#define JOYSTICK_Y_PIN 27    // ADC1 - GPIO27
#define MOUSE_BUTTON_PIN 28  // GPIO28 - 数字输入

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

// 灵敏度等级对应的除数 (值越小越灵敏)
static const uint8_t sensitivity_divisors[] = {5, 10, 30};  // 3个等级: 高/中/低灵敏度

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
                    tud_cdc_write_str("protocol error\n");
                }
            } else {
                tud_cdc_write_str("format error\n");
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
  
  // 初始化摇杆
  joystick_init();
  
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }
  //multicore_launch_core1(core1_task);
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
  // static uint32_t start_ms = 0;
  // if (board_millis() - start_ms < 10) return;
  // start_ms += 10;

  // if (!tud_hid_ready()) return;
  // tud_hid_mouse_report(REPORT_ID_MOUSE, 0, 5, 5, 0, 0);
  static uint32_t start_ms = 0;
  if (board_millis() - start_ms < 1) return; 
  start_ms += 1;

  if (!tud_hid_ready()) return;
  
  // 优先处理CDC命令
  if (mouse_cmd.has_data){
    // 添加-5到+5的随机数
    int8_t rand_x = (rand() % 2) - 2 + mouse_cmd.x * 0.03;  // 生成-1到+1的随机数
    int8_t rand_y = (rand() % 2) - 2 + mouse_cmd.y * 0.03;  // 生成-1到+1的随机数
    tud_hid_mouse_report(REPORT_ID_MOUSE, mouse_cmd.buttons, mouse_cmd.x + rand_x, mouse_cmd.y + rand_y, mouse_cmd.wheel, mouse_cmd.pan);
    mouse_cmd.has_data = false;
  }
  else{
    // 检查是否在CDC命令阻塞期间（500ms内不处理摇杆）
    uint32_t current_time = board_millis();
    if (current_time - cdc_command_time < CDC_BLOCK_TIMEOUT) {
        return;  // 跳过摇杆处理
    }

    // 读取摇杆输入
    int8_t joystick_x, joystick_y;
    uint8_t button_state;
    read_joystick(&joystick_x, &joystick_y, &button_state);
    
    // 处理按键长按调节灵敏度
    static uint8_t last_button_state = 0;
    static uint32_t button_press_start_time = 0;
    static bool sensitivity_changed = false;
    
    // 检测按键按下边沿
    if (button_state && !last_button_state) {
        button_press_start_time = current_time;
        sensitivity_changed = false;
    }
    // 检测按键长按（1秒）
    else if (button_state && !sensitivity_changed && 
             (current_time - button_press_start_time >= BUTTON_PRESS_TIMEOUT)) {
        sensitivity_level++;
        if (sensitivity_level > 3) sensitivity_level = 1;  // 循环回到最高灵敏度
        sensitivity_changed = true;
        
    }
    last_button_state = button_state;
    
    // 使用当前灵敏度等级缩放摇杆输入
    uint8_t current_divisor = sensitivity_divisors[sensitivity_level - 1];
    int8_t mouse_x = joystick_x / current_divisor;
    int8_t mouse_y;
    
    // 只在高灵敏度（等级1）时降低Y轴灵敏度
    if (sensitivity_level == 1 || sensitivity_level == 2) {
        mouse_y = joystick_y / (current_divisor * 4);  // Y轴灵敏度降低
    } else {
        mouse_y = joystick_y / current_divisor;  // 其他等级Y轴与X轴相同
    }
    
    // 如果有移动，发送鼠标报告（灵敏度调节时不发送按键）
    if (mouse_x != 0 || mouse_y != 0) {
        tud_hid_mouse_report(REPORT_ID_MOUSE, 0, mouse_x, mouse_y, 0, 0);
    }
    // 普通按键功能（非灵敏度调节时）
    else if (button_state && !sensitivity_changed) {
        tud_hid_mouse_report(REPORT_ID_MOUSE, 1, 0, 0, 0, 0);  // 左键
    }
    
    // 暂时注释掉ADC调试打印
    /*
    // 同时也打印ADC调试信息（减少频率）
    static uint32_t debug_ms = 0;
    if (board_millis() - debug_ms > 200) {  // 每200ms打印一次调试信息
        debug_ms = board_millis();
        
        // 读取原始ADC值
        adc_select_input(0);
        uint16_t x_raw = adc_read();
        adc_select_input(1);
        uint16_t y_raw = adc_read();
        
        char debug_buf[128];
        snprintf(debug_buf, sizeof(debug_buf), "ADC X:%d Y:%d | Joy X:%d Y:%d | Mouse X:%d Y:%d Btn:%d\n", 
                 x_raw, y_raw, joystick_x, joystick_y, mouse_x, mouse_y, button_state);
        tud_cdc_write_str(debug_buf);
        tud_cdc_write_flush();
    }
    */
  }
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
