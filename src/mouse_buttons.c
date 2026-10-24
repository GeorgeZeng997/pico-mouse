/**
 * 物理鼠标微动按键采样与状态机消抖模块
 */

#include "mouse_buttons.h"
#include "board_config.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#if __has_include("bsp/board_api.h")
#include "bsp/board_api.h"
#else
#include "bsp/board.h"
#endif

void mouse_buttons_init(void)
{
    gpio_init(PIN_MOUSE_LEFT_BTN);
    gpio_set_dir(PIN_MOUSE_LEFT_BTN, GPIO_IN);
    gpio_pull_up(PIN_MOUSE_LEFT_BTN);

    gpio_init(PIN_MOUSE_RIGHT_BTN);
    gpio_set_dir(PIN_MOUSE_RIGHT_BTN, GPIO_IN);
    gpio_pull_up(PIN_MOUSE_RIGHT_BTN);
}

uint8_t mouse_buttons_read(void)
{
    static uint8_t  last_raw    = 0;
    static uint8_t  stable      = 0;
    static uint32_t last_change = 0;

    uint8_t raw = 0;
    if (!gpio_get(PIN_MOUSE_LEFT_BTN))  raw |= 0x01;
    if (!gpio_get(PIN_MOUSE_RIGHT_BTN)) raw |= 0x02;

    uint32_t now = board_millis();
    if (raw != last_raw) {
        last_change = now;
        last_raw = raw;
    }
    if ((now - last_change) >= BUTTON_DEBOUNCE_MS && stable != last_raw) {
        stable = last_raw;
    }
    return stable;
}
