# Pico-Mouse (RP2040 1000Hz USB HID + PAW3205 + CDC Composite Firmware)

基于 **RP2040 (Raspberry Pi Pico / YD-RP2040)** 与 **TinyUSB** 的双核 1000Hz 复合鼠标固件，支持 **PAW3205DB-TJ3T 光学鼠标传感器物理输入** 与 **上位机 USB CDC 虚拟串口位移注入** 的无损矢量合成。

---

## 一、 核心架构特性

1. **双核硬件解耦与自旋锁同步 (`Core 0` + `Core 1`)**
   * **Core 1 (`paw3205_core1_task`)**：以 1000Hz 独立轮询 PAW3205 光学传感器，完整读取 `DEL_X` / `DEL_Y` 后通过硬件自旋锁临界区 (`critical_section_t`) 原子累加至跨核位移池，彻底杜绝跨核半途读取竞争与丢步。
   * **Core 0 (`main` / `hid_compositor_task`)**：运行 TinyUSB 设备栈、CDC 流式行协议解析器与 1000Hz HID 矢量合成器。
2. **双源无损矢量合成与余量保留**
   * 物理传感器位移（定点 `1.5x` 缩放，余数跨帧保留）与上位机串口注入位移在每个 `1ms` HID 周期内进行**矢量叠加**。
   * 物理微动按键（`10ms` 状态机消抖）与串口按键状态**按位或合并 (`phys_btn | cdc_btn`)**，辅瞄/压枪期间绝不锁死手部跟枪操作，按住物理左键扫射绝不断触。
   * 单帧合成位移超出 `[-127, +127]` 时，当前帧发送极值，**余量自动保留至下一毫秒继续发送**。
3. **流式防粘包 CDC 串口协议**
   * 逐字节按行缓冲解析 `"55 <btn> <x> <y> <wheel> <pan> <sum>\n"`（其中 `sum = btn + x + y + wheel + pan`），解决高频串口连发粘包或 USB 分包截断问题。

---

## 二、 硬件引脚接线表

| 模块 | 信号线 | RP2040 引脚 | 说明 |
| :--- | :--- | :--- | :--- |
| **PAW3205DB 传感器** | `SCLK` | `GPIO 4` (Pin 6) | 串行时钟输出 |
| **PAW3205DB 传感器** | `SDIO` | `GPIO 5` (Pin 7) | 半双工双向数据线 |
| **PAW3205DB 传感器** | `MOTSWK` | `GPIO 6` (Pin 9) | 运动检测中断（低电平有效，内部上拉） |
| **物理鼠标按键** | 左键 (`L_BTN`) | `GPIO 14` | 微动开关（低电平有效，内部上拉，10ms 消抖） |
| **物理鼠标按键** | 右键 (`R_BTN`) | `GPIO 15` | 微动开关（低电平有效，内部上拉，10ms 消抖） |

---

## 三、 目录结构

```text
pico-mouse/
├── CMakeLists.txt              # CMake 构建配置 (Pico SDK 2.1.1)
├── pico_sdk_import.cmake       # Pico SDK 导入脚本
├── docs/                       # PAW3205DB 数据手册与测试板原理图
├── inc/
│   ├── board_config.h          # 引脚定义、1000Hz 轮询率、定点灵敏度、VID/PID 配置
│   ├── paw3205.h               # PAW3205 传感器驱动与跨核原子读取接口
│   ├── mouse_buttons.h         # 物理微动按键消抖接口
│   ├── cdc_protocol.h          # CDC 串口流式行解析器接口
│   ├── tusb_config.h           # TinyUSB 配置 (256B CDC RX/TX FIFO)
│   └── usb_descriptors.h       # HID Report ID 定义
└── src/
    ├── main.c                  # 双核启动与 1000Hz HID 矢量合成器
    ├── paw3205.c               # PAW3205 半双工 SPI 驱动与 Core 1 采样任务
    ├── mouse_buttons.c         # 左右键 GPIO 初始化与 10ms 消抖状态机
    ├── cdc_protocol.c          # 防粘包流式行协议解析与位移累加池
    └── usb_descriptors.c       # 1000Hz (bInterval=1) HID 描述符与设备名伪装
```

---

## 四、 分支说明

* **`master` / `pico-paw`**：主力版本（**PAW3205 光学传感器 + 物理按键 + CDC 串口 1000Hz 矢量合成架构**）。
* **`joystick-adc`**：历史硬件存档分支（**ADC 双轴模拟摇杆 + 按键长按切灵敏度 + CDC 串口控制**）。
* **`gyro-mpu6500`**：历史硬件存档分支（**MPU6500 / MPU9250 SPI 陀螺仪体感飞鼠 + CDC 串口控制**）。

---

## 五、 编译方法

```bash
mkdir -p build && cd build
cmake -G Ninja -DPICO_SDK_PATH=/path/to/pico-sdk ..
ninja
```
编译完成后将生成的 `build/dev_hid_composite.uf2` 拖入处于 `BOOTSEL` 模式的 RP2040 磁盘即可。
