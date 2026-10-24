#ifndef PAW3205_H_
#define PAW3205_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化跨核同步原语 (必须在启动 Core 1 之前由 Core 0 调用)
void paw3205_sync_init(void);

// 初始化 PAW3205 硬件引脚与寄存器配置
void paw3205_hw_init(void);

// Core 1 入口任务：高频轮询 PAW3205 并将位移原子累加至跨核缓冲池
void paw3205_core1_task(void);

// Core 0 调用：原子读取并清零自上次消费以来累加的传感器原始位移
void paw3205_consume_delta(int16_t *out_dx, int16_t *out_dy);

#ifdef __cplusplus
}
#endif

#endif /* PAW3205_H_ */
