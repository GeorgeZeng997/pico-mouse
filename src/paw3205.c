/**
 * PAW3205DB-TJ3T 光学鼠标传感器驱动 (Core 1 高频采样 + 跨核自旋锁安全累加)
 */

#include "paw3205.h"
#include "board_config.h"

#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "hardware/gpio.h"

// PAW3205 寄存器地址定义
#define REG_PROD_ID1        0x00
#define REG_PROD_ID2        0x01
#define REG_MOTION_STATUS   0x02
#define REG_DEL_X           0x03
#define REG_DEL_Y           0x04
#define REG_OP_MODE         0x05
#define REG_CONF            0x06
#define REG_IMG_QUALITY     0x07
#define REG_OP_STATE        0x08
#define REG_W_PROTECT       0x09
#define REG_SLEEP1_SET      0x0A
#define REG_ENTER_TIME      0x0B
#define REG_SLEEP2_SET      0x0C
#define REG_IMG_THRESHOLD   0x0D
#define REG_IMG_RECOG       0x0E

#define PAW3205_NO_SLEEP    0xA0
#define PAW3205_UNLOCK_KEY  0x5A
#define PAW3205_MOTION_MASK 0x80    // bit7 为 MOTION 有效标志位 (0x82 表示存在有效位移)

// 跨核保护临界区与位移累加池
static critical_section_t g_sensor_crit_sec;
static volatile int16_t   g_sensor_accum_x = 0;
static volatile int16_t   g_sensor_accum_y = 0;

void paw3205_sync_init(void)
{
    critical_section_init(&g_sensor_crit_sec);
    g_sensor_accum_x = 0;
    g_sensor_accum_y = 0;
}

static uint8_t paw3205_read_register(uint8_t address)
{
    uint8_t result = 0;

    gpio_set_dir(PIN_PAW3205_SDIO, GPIO_OUT);

    // 发送 8 位寄存器地址 (MSB = 0 表示读操作)
    for (uint8_t mask = 0x80; mask > 0; mask >>= 1) {
        gpio_put(PIN_PAW3205_SCLK, 0);
        gpio_put(PIN_PAW3205_SDIO, (address & mask) ? 1 : 0);
        sleep_us(2);
        gpio_put(PIN_PAW3205_SCLK, 1);
        sleep_us(2);
    }

    // 切换 SDIO 为上拉输入，等待传感器准备数据 (tHOLD >= 3us)
    gpio_set_dir(PIN_PAW3205_SDIO, GPIO_IN);
    gpio_pull_up(PIN_PAW3205_SDIO);
    sleep_us(5);

    // 读取 8 位寄存器数据
    for (uint8_t mask = 0x80; mask > 0; mask >>= 1) {
        gpio_put(PIN_PAW3205_SCLK, 0);
        sleep_us(2);
        gpio_put(PIN_PAW3205_SCLK, 1);
        sleep_us(2);
        if (gpio_get(PIN_PAW3205_SDIO)) {
            result |= mask;
        }
    }

    sleep_us(5);
    return result;
}

static void paw3205_write_register(uint8_t address, uint8_t data)
{
    // 非写保护寄存器写入前先解除写保护
    if ((address & 0x7F) != REG_W_PROTECT) {
        paw3205_write_register(REG_W_PROTECT, PAW3205_UNLOCK_KEY);
        sleep_us(50);
    }

    address |= 0x80; // MSB = 1 表示写操作
    gpio_set_dir(PIN_PAW3205_SDIO, GPIO_OUT);

    // 发送地址
    for (uint8_t mask = 0x80; mask > 0; mask >>= 1) {
        gpio_put(PIN_PAW3205_SCLK, 0);
        gpio_put(PIN_PAW3205_SDIO, (address & mask) ? 1 : 0);
        sleep_us(2);
        gpio_put(PIN_PAW3205_SCLK, 1);
        sleep_us(2);
    }

    // 发送数据
    for (uint8_t mask = 0x80; mask > 0; mask >>= 1) {
        gpio_put(PIN_PAW3205_SCLK, 0);
        gpio_put(PIN_PAW3205_SDIO, (data & mask) ? 1 : 0);
        sleep_us(2);
        gpio_put(PIN_PAW3205_SCLK, 1);
        sleep_us(2);
    }

    gpio_put(PIN_PAW3205_SCLK, 0);
    sleep_us(50);
}

void paw3205_hw_init(void)
{
    gpio_init(PIN_PAW3205_SCLK);
    gpio_set_dir(PIN_PAW3205_SCLK, GPIO_OUT);
    gpio_put(PIN_PAW3205_SCLK, 1);

    gpio_init(PIN_PAW3205_SDIO);
    gpio_set_dir(PIN_PAW3205_SDIO, GPIO_OUT);
    gpio_put(PIN_PAW3205_SDIO, 0);

    gpio_init(PIN_PAW3205_MOTSWK);
    gpio_set_dir(PIN_PAW3205_MOTSWK, GPIO_IN);
    gpio_pull_up(PIN_PAW3205_MOTSWK);

    // 上电重同步时序 (Datasheet: tRESYNC = 1us, tSIWTT = 320ms)
    gpio_put(PIN_PAW3205_SCLK, 1);
    gpio_put(PIN_PAW3205_SCLK, 0);
    sleep_us(2);
    gpio_put(PIN_PAW3205_SCLK, 1);
    sleep_ms(320);
    gpio_put(PIN_PAW3205_SCLK, 0);
    sleep_ms(10);

    // 禁用休眠模式并配置 CPI 档位
    paw3205_write_register(REG_OP_MODE, PAW3205_NO_SLEEP);
    sleep_ms(10);
    paw3205_write_register(REG_CONF, 0x04);
    sleep_ms(20);
}

void paw3205_core1_task(void)
{
    paw3205_hw_init();

    while (1) {
        // MOTSWK 为低电平时表示传感器检测到运动
        if (!gpio_get(PIN_PAW3205_MOTSWK)) {
            uint8_t status = paw3205_read_register(REG_MOTION_STATUS);
            if (status & PAW3205_MOTION_MASK) {
                // 先在栈局部变量中完整读取 DEL_X 和 DEL_Y，杜绝跨核半途读取竞争
                int8_t dx = (int8_t)paw3205_read_register(REG_DEL_X);
                int8_t dy = (int8_t)paw3205_read_register(REG_DEL_Y);

                if (dx != 0 || dy != 0) {
                    critical_section_enter_blocking(&g_sensor_crit_sec);
                    g_sensor_accum_x += dx;
                    g_sensor_accum_y += dy;
                    critical_section_exit(&g_sensor_crit_sec);
                }
            }
        }
        sleep_us(PAW3205_POLL_INTERVAL_US);
    }
}

void paw3205_consume_delta(int16_t *out_dx, int16_t *out_dy)
{
    critical_section_enter_blocking(&g_sensor_crit_sec);
    *out_dx = g_sensor_accum_x;
    *out_dy = g_sensor_accum_y;
    g_sensor_accum_x = 0;
    g_sensor_accum_y = 0;
    critical_section_exit(&g_sensor_crit_sec);
}
