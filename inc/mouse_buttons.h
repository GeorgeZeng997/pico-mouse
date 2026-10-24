#ifndef MOUSE_BUTTONS_H_
#define MOUSE_BUTTONS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化鼠标左右键 GPIO (上拉输入)
void mouse_buttons_init(void);

// 读取消抖后的物理鼠标按键位掩码 (bit0: 左键, bit1: 右键)
uint8_t mouse_buttons_read(void);

#ifdef __cplusplus
}
#endif

#endif /* MOUSE_BUTTONS_H_ */
