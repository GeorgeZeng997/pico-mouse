/**
 * Pico-Mouse 主控调度与 1000Hz HID 双源矢量合成引擎
 *
 * 架构说明:
 *   - Core 1: 独立运行 paw3205_core1_task()，以 1000Hz 轮询 PAW3205 光学传感器并通过硬件自旋锁安全累加位移。
 *   - Core 0: 运行 TinyUSB 协议栈、CDC 流式串口解析器与 1000Hz HID 矢量合成器：
 *       1. 将物理传感器位移 (定点 1.5x 缩放，保留余数) 与上位机 CDC 串口注入位移进行无损矢量叠加；
 *       2. 将物理微动按键 (10ms 消抖) 与上位机按键指令按位合并 (phys_btn | cdc_btn)，彻底消除串口移动导致物理开火断触或锁死手部操作的问题；
 *       3. 单帧超出 [-127, +127] 的剩余位移自动保留至下一毫秒继续发送，杜绝快速甩枪截断丢步。
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#if __has_include("bsp/board_api.h")
#include "bsp/board_api.h"
#define HAS_BOARD_API_H 1
#else
#include "bsp/board.h"
#define HAS_BOARD_API_H 0
#endif
#include "tusb.h"

#include "board_config.h"
#include "usb_descriptors.h"
#include "paw3205.h"
#include "mouse_buttons.h"
#include "cdc_protocol.h"

enum {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED     = 1000,
    BLINK_SUSPENDED   = 2500,
};

static uint32_t g_blink_interval_ms = BLINK_NOT_MOUNTED;

static inline int8_t clamp_hid_delta(int32_t val)
{
    if (val > HID_MAX_DELTA)  return  HID_MAX_DELTA;
    if (val < -HID_MAX_DELTA) return -HID_MAX_DELTA;
    return (int8_t)val;
}

static void hid_compositor_task(void)
{
    static uint32_t last_tick_ms  = 0;
    static int32_t  pending_x     = 0;
    static int32_t  pending_y     = 0;
    static int32_t  scale_rem_x   = 0;
    static int32_t  scale_rem_y   = 0;
    static uint8_t  last_buttons  = 0;
    static uint8_t  cdc_btn_latch = 0;

    uint32_t now = board_millis();
    if (now - last_tick_ms < HID_REPORT_INTERVAL_MS) {
        return;
    }
    last_tick_ms = now;

    if (!tud_hid_ready()) {
        return;
    }

    // 1. 读取消抖后的物理鼠标左右键状态
    uint8_t phys_buttons = mouse_buttons_read();

    // 2. 原子消费 Core 1 采集的 PAW3205 光学传感器位移 (定点 1.5x 缩放，保留余数不丢精度)
    int16_t raw_sx = 0, raw_sy = 0;
    paw3205_consume_delta(&raw_sx, &raw_sy);
    if (raw_sx != 0 || raw_sy != 0) {
        int32_t num_x = (int32_t)raw_sx * SENSOR_SCALE_NUM + scale_rem_x;
        int32_t num_y = (int32_t)(-raw_sy) * SENSOR_SCALE_NUM + scale_rem_y; // Y 轴方向反転

        pending_x  += num_x / SENSOR_SCALE_DEN;
        scale_rem_x = num_x % SENSOR_SCALE_DEN;

        pending_y  += num_y / SENSOR_SCALE_DEN;
        scale_rem_y = num_y % SENSOR_SCALE_DEN;
    }

    // 3. 消费上位机 CDC 串口注入位移并执行矢量叠加
    cdc_mouse_delta_t cdc_cmd;
    int8_t send_wheel = 0;
    int8_t send_pan   = 0;
    bool   force_send = false;

    if (cdc_protocol_consume(&cdc_cmd)) {
        pending_x    += cdc_cmd.dx;
        pending_y    += cdc_cmd.dy;
        send_wheel    = clamp_hid_delta(cdc_cmd.wheel);
        send_pan      = clamp_hid_delta(cdc_cmd.pan);
        cdc_btn_latch = cdc_cmd.buttons;
        force_send    = true;
    }

    // 4. 合并按键状态 (物理按键与串口按键按位或，互不打断)
    uint8_t merged_buttons = phys_buttons | cdc_btn_latch;

    // 5. 限幅当前 1ms 帧可发送的位移，余量留在 pending_x/y 下一毫秒继续发
    int8_t send_x = clamp_hid_delta(pending_x);
    int8_t send_y = clamp_hid_delta(pending_y);

    if (send_x != 0 || send_y != 0 || send_wheel != 0 || send_pan != 0 ||
        merged_buttons != last_buttons || force_send) {
        tud_hid_mouse_report(REPORT_ID_MOUSE, merged_buttons, send_x, send_y, send_wheel, send_pan);
        pending_x   -= send_x;
        pending_y   -= send_y;
        last_buttons = merged_buttons;
    }
}

static void led_blinking_task(void)
{
    static uint32_t start_ms  = 0;
    static bool     led_state = false;

    if (!g_blink_interval_ms) return;
    if (board_millis() - start_ms < g_blink_interval_ms) return;
    start_ms += g_blink_interval_ms;

    board_led_write(led_state);
    led_state = !led_state;
}

int main(void)
{
    board_init();
    stdio_init_all();

    mouse_buttons_init();
    paw3205_sync_init();

    tud_init(BOARD_TUD_RHPORT);
#if HAS_BOARD_API_H
    if (board_init_after_tusb) {
        board_init_after_tusb();
    }
#endif

    // 在 Core 1 上独立启动 PAW3205 光学传感器高频采样任务
    multicore_launch_core1(paw3205_core1_task);

    while (1) {
        tud_task();
        cdc_protocol_task();
        hid_compositor_task();
        led_blinking_task();
    }
}

//--------------------------------------------------------------------+
// TinyUSB Device & HID Callbacks
//--------------------------------------------------------------------+
void tud_mount_cb(void)
{
    g_blink_interval_ms = BLINK_MOUNTED;
}

void tud_umount_cb(void)
{
    g_blink_interval_ms = BLINK_NOT_MOUNTED;
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
    g_blink_interval_ms = BLINK_SUSPENDED;
}

void tud_resume_cb(void)
{
    g_blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
    (void) instance;
    (void) report;
    (void) len;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;
}
