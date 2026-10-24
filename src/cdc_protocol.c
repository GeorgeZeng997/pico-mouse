/**
 * CDC 虚拟串口流式行协议解析器
 * 协议格式: "55 <btn> <x> <y> <wheel> <pan> <sum>\n"
 * 特性:
 *   1. 逐字节流式按行缓冲解析，彻底解决 USB CDC 粘包 (多帧连发) 与分包截断丢帧问题；
 *   2. 同一 HID 周期内收到的多帧位移自动累加至缓冲池，不覆盖前序微调帧。
 */

#include "cdc_protocol.h"
#include "board_config.h"

#include <stdio.h>
#include <string.h>
#include "tusb.h"

static cdc_mouse_delta_t g_cdc_accum = {0};
static char              g_line_buf[64];
static uint8_t           g_line_pos = 0;

static void cdc_parse_line(const char *line)
{
    int head, btn, x, y, wheel, pan, sum;
    if (sscanf(line, "%d %d %d %d %d %d %d", &head, &btn, &x, &y, &wheel, &pan, &sum) == 7) {
        int calc_sum = btn + x + y + wheel + pan;
        if (head == CDC_PROTOCOL_HEADER && calc_sum == sum) {
            g_cdc_accum.buttons = (uint8_t)btn;
            g_cdc_accum.dx     += (int16_t)x;
            g_cdc_accum.dy     += (int16_t)y;
            g_cdc_accum.wheel  += (int16_t)wheel;
            g_cdc_accum.pan    += (int16_t)pan;
            g_cdc_accum.has_cmd = true;
#if CDC_ENABLE_ACK
            tud_cdc_write_str("ok\n");
            tud_cdc_write_flush();
#endif
        }
    }
}

void cdc_protocol_task(void)
{
    while (tud_cdc_available()) {
        uint8_t rx_buf[64];
        uint32_t count = tud_cdc_read(rx_buf, sizeof(rx_buf));
        for (uint32_t i = 0; i < count; ++i) {
            char c = (char)rx_buf[i];
            if (c == '\n' || c == '\r') {
                if (g_line_pos > 0) {
                    g_line_buf[g_line_pos] = '\0';
                    cdc_parse_line(g_line_buf);
                    g_line_pos = 0;
                }
            } else {
                if (g_line_pos < sizeof(g_line_buf) - 1) {
                    g_line_buf[g_line_pos++] = c;
                } else {
                    // 单行超长溢出保护：重置行指针
                    g_line_pos = 0;
                }
            }
        }
    }
}

bool cdc_protocol_consume(cdc_mouse_delta_t *out)
{
    if (!g_cdc_accum.has_cmd) {
        return false;
    }
    *out = g_cdc_accum;
    memset(&g_cdc_accum, 0, sizeof(g_cdc_accum));
    return true;
}
