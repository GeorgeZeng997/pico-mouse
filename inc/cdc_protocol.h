#ifndef CDC_PROTOCOL_H_
#define CDC_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t dx;
    int16_t dy;
    int16_t wheel;
    int16_t pan;
    uint8_t buttons;
    bool    has_cmd;
} cdc_mouse_delta_t;

// CDC 串口流式接收与行协议解析任务 (在 Core 0 主循环中调用)
// 支持粘包/分包处理，同一毫秒内的多帧位移自动累加不覆盖
void cdc_protocol_task(void);

// 取出并清零当前累积的上位机 CDC 注入位移与按键指令
bool cdc_protocol_consume(cdc_mouse_delta_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CDC_PROTOCOL_H_ */
