/**
 * @file    dev_lcd.c
 * @brief   外部并口 LCD 设备驱动实现
 *
 * @note    当前设备为 ILI9488。总线原语只在本文件内使用，
 *          对外提供控制器级语义接口。
 */

#include "./dev_lcd/dev_lcd.h"
#include "../../BSP/bsp_gpio/bsp_gpio.h"
#include "../../BSP/bsp_fsmc/bsp_fsmc.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * 刷屏时每写多少个像素让步一次 CPU。
 * 全屏 320x480 = 153600 点：取值过小会让 taskYIELD 本身成为开销，
 * 过大则网络任务的调度延迟变得可感知。512 点约合一行多一点，
 * 在 153600 点上只引入 300 次切换，开销可忽略。
 */
#define DEV_LCD_FILL_YIELD_PIXELS   (512U)

/*
 * LCD 总线地址只在 Device 内部使用。
 * 16 位 EXMC 下，命令和数据通过相邻半字地址区分。
 *
 * 原理（FSMC/EXMC 8080 时序）：LCD 的 RS 引脚（命令/数据选择）
 * 直接连到 EXMC 地址线的 A0（代码里是 bit1，见下）。
 * CPU 写 0x64000000 时 A0=0 -> RS=0 -> LCD 认为是“命令”；
 * CPU 写 0x64000002 时 A0=1 -> RS=1 -> LCD 认为是“数据/参数”。
 * 16 位总线下地址步进是 2 字节，所以命令/数据地址差 2 而不是 1。
 * 于是“写命令”退化成“向固定地址存一个数”，不需要任何 GPIO 翻转。 */
#define LCD_BUS_BASE_ADDR       ((uint32_t)0x64000000UL)
#define LCD_BUS_CMD_ADDR        (LCD_BUS_BASE_ADDR)
#define LCD_BUS_DATA_ADDR       (LCD_BUS_BASE_ADDR + 2UL)

/** ILI9488 读 ID 命令（仅 Device 内部使用） */
#define LCD_CMD_READ_ID         (0x00D3U)
/** ILI9488 期望控制器 ID（仅 Device 内部使用） */
#define LCD_EXPECTED_ID         (0x9488U)
/** ILI9488 列地址设置命令（仅 Device 内部使用） */
#define LCD_CMD_COLUMN_ADDRESS  (0x002AU)
/** ILI9488 页地址设置命令（仅 Device 内部使用） */
#define LCD_CMD_PAGE_ADDRESS    (0x002BU)
/** ILI9488 开始写 GRAM 命令（仅 Device 内部使用） */
#define LCD_CMD_MEMORY_WRITE    (0x002CU)

/** LCD 命令寄存器内存映射地址（仅 Device 内部使用）
 * @note volatile 是必须的：告诉编译器“每次都真去访存”，
 *       禁止把连续两次写合并、禁止把读优化掉。
 *       没有 volatile，while(读状态) 可能被优化成读一次就死循环。 */
static volatile uint16_t *const s_dev_lcd_command =
    (volatile uint16_t *)LCD_BUS_CMD_ADDR;
/** LCD 数据寄存器内存映射地址（仅 Device 内部使用） */
static volatile uint16_t *const s_dev_lcd_data =
    (volatile uint16_t *)LCD_BUS_DATA_ADDR;
/** 初始化标志（0=未初始化，1=已初始化） */
static uint8_t s_dev_lcd_inited = 0U;
/** ILI9488 控制器初始化标志（0=未完成，1=已完成） */
static uint8_t s_dev_lcd_controller_ready = 0U;

/*
 * 总线原语必须先在本文件声明，再在本文件内定义；禁止放入 dev_lcd.h。
 */
static ret_t lcd_bus_write_command(uint16_t command);
static ret_t lcd_bus_write_data(uint16_t data);
static ret_t lcd_bus_read_data(uint16_t *data);
static ret_t lcd_bus_write_reg(uint16_t reg,
                               uint16_t data);
static ret_t lcd_bus_read_reg(uint16_t reg,
                              uint16_t *data);

/** 8x16 ASCII 字模，来源：麒麟 GD32F303 ILI9488 参考例程。 */
static const uint8_t s_lcd_ascii_8x16[95][16] = {  
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x10,0x10,0x00,0x00},
{0x00,0x12,0x24,0x24,0x48,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x12,0x12,0x12,0x7E,0x24,0x24,0x24,0x7E,0x24,0x24,0x24,0x00,0x00},
{0x00,0x00,0x08,0x3C,0x4A,0x4A,0x48,0x38,0x0C,0x0A,0x0A,0x4A,0x4A,0x3C,0x08,0x08},
{0x00,0x00,0x00,0x44,0xA4,0xA8,0xA8,0xB0,0x54,0x1A,0x2A,0x2A,0x4A,0x44,0x00,0x00},
{0x00,0x00,0x00,0x30,0x48,0x48,0x48,0x50,0x6E,0xA4,0x94,0x98,0x89,0x76,0x00,0x00},
{0x00,0x60,0x20,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x02,0x04,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x10,0x08,0x08,0x04,0x02,0x00},
{0x00,0x40,0x20,0x10,0x10,0x08,0x08,0x08,0x08,0x08,0x08,0x10,0x10,0x20,0x40,0x00},
{0x00,0x00,0x00,0x00,0x10,0x10,0xD6,0x38,0x38,0xD6,0x10,0x10,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x7F,0x08,0x08,0x08,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x20,0x20,0x40},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x60,0x00,0x00},
{0x00,0x00,0x02,0x04,0x04,0x04,0x08,0x08,0x10,0x10,0x10,0x20,0x20,0x40,0x40,0x00},
{0x00,0x00,0x00,0x18,0x24,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18,0x00,0x00},
{0x00,0x00,0x00,0x08,0x38,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x3E,0x00,0x00},
{0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x02,0x04,0x08,0x10,0x20,0x42,0x7E,0x00,0x00},
{0x00,0x00,0x00,0x3C,0x42,0x42,0x02,0x04,0x18,0x04,0x02,0x42,0x42,0x3C,0x00,0x00},
{0x00,0x00,0x00,0x04,0x0C,0x0C,0x14,0x24,0x24,0x44,0x7F,0x04,0x04,0x1F,0x00,0x00},
{0x00,0x00,0x00,0x7E,0x40,0x40,0x40,0x78,0x44,0x02,0x02,0x42,0x44,0x38,0x00,0x00},
{0x00,0x00,0x00,0x18,0x24,0x40,0x40,0x5C,0x62,0x42,0x42,0x42,0x22,0x1C,0x00,0x00},
{0x00,0x00,0x00,0x7E,0x42,0x04,0x04,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x00,0x00},
{0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x24,0x18,0x24,0x42,0x42,0x42,0x3C,0x00,0x00},
{0x00,0x00,0x00,0x38,0x44,0x42,0x42,0x42,0x46,0x3A,0x02,0x02,0x24,0x18,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x10},
{0x00,0x00,0x00,0x02,0x04,0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x04,0x02,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x40,0x20,0x10,0x08,0x04,0x02,0x04,0x08,0x10,0x20,0x40,0x00,0x00},
{0x00,0x00,0x00,0x3C,0x42,0x42,0x62,0x04,0x08,0x08,0x08,0x00,0x18,0x18,0x00,0x00},
{0x00,0x00,0x00,0x38,0x44,0x5A,0xAA,0xAA,0xAA,0xAA,0xAA,0x5C,0x42,0x3C,0x00,0x00},
{0x00,0x00,0x00,0x10,0x10,0x18,0x28,0x28,0x24,0x3C,0x44,0x42,0x42,0xE7,0x00,0x00},
{0x00,0x00,0x00,0xF8,0x44,0x44,0x44,0x78,0x44,0x42,0x42,0x42,0x44,0xF8,0x00,0x00},
{0x00,0x00,0x00,0x3E,0x42,0x42,0x80,0x80,0x80,0x80,0x80,0x42,0x44,0x38,0x00,0x00},
{0x00,0x00,0x00,0xF8,0x44,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x44,0xF8,0x00,0x00},
{0x00,0x00,0x00,0xFC,0x42,0x48,0x48,0x78,0x48,0x48,0x40,0x42,0x42,0xFC,0x00,0x00},
{0x00,0x00,0x00,0xFC,0x42,0x48,0x48,0x78,0x48,0x48,0x40,0x40,0x40,0xE0,0x00,0x00},
{0x00,0x00,0x00,0x3C,0x44,0x44,0x80,0x80,0x80,0x8E,0x84,0x44,0x44,0x38,0x00,0x00},
{0x00,0x00,0x00,0xE7,0x42,0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x42,0xE7,0x00,0x00},
{0x00,0x00,0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00},
{0x00,0x00,0x00,0x3E,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x88,0xF0},
{0x00,0x00,0x00,0xEE,0x44,0x48,0x50,0x70,0x50,0x48,0x48,0x44,0x44,0xEE,0x00,0x00},
{0x00,0x00,0x00,0xE0,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x42,0xFE,0x00,0x00},
{0x00,0x00,0x00,0xEE,0x6C,0x6C,0x6C,0x6C,0x6C,0x54,0x54,0x54,0x54,0xD6,0x00,0x00},
{0x00,0x00,0x00,0xC7,0x62,0x62,0x52,0x52,0x4A,0x4A,0x4A,0x46,0x46,0xE2,0x00,0x00},
{0x00,0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00},
{0x00,0x00,0x00,0xFC,0x42,0x42,0x42,0x42,0x7C,0x40,0x40,0x40,0x40,0xE0,0x00,0x00},
{0x00,0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x82,0xB2,0x4C,0x38,0x06,0x00},
{0x00,0x00,0x00,0xFC,0x42,0x42,0x42,0x7C,0x48,0x48,0x44,0x44,0x42,0xE3,0x00,0x00},
{0x00,0x00,0x00,0x3E,0x42,0x42,0x40,0x20,0x18,0x04,0x02,0x42,0x42,0x7C,0x00,0x00},
{0x00,0x00,0x00,0xFE,0x92,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00},
{0x00,0x00,0x00,0xE7,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00},
{0x00,0x00,0x00,0xE7,0x42,0x42,0x44,0x24,0x24,0x28,0x28,0x18,0x10,0x10,0x00,0x00},
{0x00,0x00,0x00,0xD6,0x54,0x54,0x54,0x54,0x54,0x6C,0x28,0x28,0x28,0x28,0x00,0x00},
{0x00,0x00,0x00,0xE7,0x42,0x24,0x24,0x18,0x18,0x18,0x24,0x24,0x42,0xE7,0x00,0x00},
{0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00},
{0x00,0x00,0x00,0x7E,0x84,0x04,0x08,0x08,0x10,0x20,0x20,0x42,0x42,0xFC,0x00,0x00},
{0x00,0x1E,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x1E,0x00},
{0x00,0x00,0x40,0x20,0x20,0x20,0x10,0x10,0x10,0x08,0x08,0x04,0x04,0x04,0x02,0x02},
{0x00,0x78,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x78,0x00},
{0x00,0x18,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
{0x00,0x60,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x0C,0x34,0x44,0x4C,0x36,0x00,0x00},
{0x00,0x00,0x00,0x00,0xC0,0x40,0x40,0x58,0x64,0x42,0x42,0x42,0x64,0x58,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x22,0x40,0x40,0x40,0x22,0x1C,0x00,0x00},
{0x00,0x00,0x00,0x00,0x06,0x02,0x02,0x3E,0x42,0x42,0x42,0x42,0x46,0x3B,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x42,0x42,0x7E,0x40,0x42,0x3C,0x00,0x00},
{0x00,0x00,0x00,0x00,0x0C,0x12,0x10,0x7C,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x44,0x44,0x38,0x40,0x3C,0x42,0x42,0x3C},
{0x00,0x00,0x00,0x00,0xC0,0x40,0x40,0x5C,0x62,0x42,0x42,0x42,0x42,0xE7,0x00,0x00},
{0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x70,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00},
{0x00,0x00,0x00,0x0C,0x0C,0x00,0x00,0x1C,0x04,0x04,0x04,0x04,0x04,0x04,0x44,0x78},
{0x00,0x00,0x00,0x00,0xC0,0x40,0x40,0x4E,0x48,0x50,0x70,0x48,0x44,0xEE,0x00,0x00},
{0x00,0x00,0x00,0x10,0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x49,0x49,0x49,0x49,0x49,0xED,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDC,0x62,0x42,0x42,0x42,0x42,0xE7,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8,0x64,0x42,0x42,0x42,0x64,0x58,0x40,0xE0},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1A,0x26,0x42,0x42,0x42,0x26,0x1A,0x02,0x07},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x32,0x20,0x20,0x20,0x20,0xF8,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x42,0x40,0x3C,0x02,0x42,0x7C,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x7C,0x10,0x10,0x10,0x10,0x12,0x0C,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC6,0x42,0x42,0x42,0x42,0x46,0x3B,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x89,0x4A,0x5A,0x54,0x24,0x24,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x76,0x24,0x18,0x18,0x18,0x24,0x6E,0x00,0x00},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE7,0x42,0x24,0x24,0x18,0x18,0x10,0x10,0x60},
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x44,0x08,0x10,0x10,0x22,0x7E,0x00,0x00},
{0x00,0x03,0x04,0x04,0x04,0x04,0x04,0x04,0x08,0x04,0x04,0x04,0x04,0x04,0x03,0x00},
{0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08},
{0x00,0xC0,0x20,0x20,0x20,0x20,0x20,0x20,0x10,0x20,0x20,0x20,0x20,0x20,0xC0,0x00},
{0x20,0x5A,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};

 
typedef struct
{
    uint16_t command;              /**< 控制器命令 */
    const uint16_t *data;          /**< 命令参数；无参数时为 NULL */
    uint8_t data_count;            /**< 参数个数 */
} lcd_init_step_t;

/* 以下参数表严格取自麒麟 GD32F303 ILI9488 例程。 */
static const uint16_t s_ili9488_f7[] =
{
    0x00A9U, 0x0051U, 0x002CU, 0x0082U
};
static const uint16_t s_ili9488_c0[] =
{
    0x0011U, 0x0009U
};
static const uint16_t s_ili9488_c1[] =
{
    0x0041U
};
static const uint16_t s_ili9488_c5[] =
{
    0x0000U, 0x000AU, 0x0080U
};
static const uint16_t s_ili9488_b1[] =
{
    0x00B0U, 0x0011U
};
static const uint16_t s_ili9488_b4[] =
{
    0x0002U
};
static const uint16_t s_ili9488_b6[] =
{
    0x0002U, 0x0022U
};
static const uint16_t s_ili9488_b7[] =
{
    0x00C6U
};
static const uint16_t s_ili9488_be[] =
{
    0x0000U, 0x0004U
};
static const uint16_t s_ili9488_e9[] =
{
    0x0000U
};
static const uint16_t s_ili9488_36[] =
{
    0x0008U
};
static const uint16_t s_ili9488_3a[] =
{
    0x0055U
};
static const uint16_t s_ili9488_e0[] =
{
    0x0000U, 0x0007U, 0x0010U, 0x0009U,
    0x0017U, 0x000BU, 0x0041U, 0x0089U,
    0x004BU, 0x000AU, 0x000CU, 0x000EU,
    0x0018U, 0x001BU, 0x000FU
};
static const uint16_t s_ili9488_e1[] =
{
    0x0000U, 0x0017U, 0x001AU, 0x0004U,
    0x000EU, 0x0006U, 0x002FU, 0x0045U,
    0x0043U, 0x0002U, 0x000AU, 0x0009U,
    0x0032U, 0x0036U, 0x000FU
};

/** ILI9488 初始化命令表；0x11/0x29 为无参数命令。 */
static const lcd_init_step_t s_ili9488_init_sequence[] =
{
    {0x00F7U, s_ili9488_f7, (uint8_t)(sizeof(s_ili9488_f7) / sizeof(s_ili9488_f7[0]))},
    {0x00C0U, s_ili9488_c0, (uint8_t)(sizeof(s_ili9488_c0) / sizeof(s_ili9488_c0[0]))},
    {0x00C1U, s_ili9488_c1, (uint8_t)(sizeof(s_ili9488_c1) / sizeof(s_ili9488_c1[0]))},
    {0x00C5U, s_ili9488_c5, (uint8_t)(sizeof(s_ili9488_c5) / sizeof(s_ili9488_c5[0]))},
    {0x00B1U, s_ili9488_b1, (uint8_t)(sizeof(s_ili9488_b1) / sizeof(s_ili9488_b1[0]))},
    {0x00B4U, s_ili9488_b4, (uint8_t)(sizeof(s_ili9488_b4) / sizeof(s_ili9488_b4[0]))},
    {0x00B6U, s_ili9488_b6, (uint8_t)(sizeof(s_ili9488_b6) / sizeof(s_ili9488_b6[0]))},
    {0x00B7U, s_ili9488_b7, (uint8_t)(sizeof(s_ili9488_b7) / sizeof(s_ili9488_b7[0]))},
    {0x00BEU, s_ili9488_be, (uint8_t)(sizeof(s_ili9488_be) / sizeof(s_ili9488_be[0]))},
    {0x00E9U, s_ili9488_e9, (uint8_t)(sizeof(s_ili9488_e9) / sizeof(s_ili9488_e9[0]))},
    {0x0036U, s_ili9488_36, (uint8_t)(sizeof(s_ili9488_36) / sizeof(s_ili9488_36[0]))},
    {0x003AU, s_ili9488_3a, (uint8_t)(sizeof(s_ili9488_3a) / sizeof(s_ili9488_3a[0]))},
    {0x00E0U, s_ili9488_e0, (uint8_t)(sizeof(s_ili9488_e0) / sizeof(s_ili9488_e0[0]))},
    {0x00E1U, s_ili9488_e1, (uint8_t)(sizeof(s_ili9488_e1) / sizeof(s_ili9488_e1[0]))},
    {0x0011U, NULL, 0U},
    {0x0029U, NULL, 0U},
};

static ret_t lcd_apply_init_sequence(void);
static ret_t lcd_set_window(uint16_t x_start,
                            uint16_t y_start,
                            uint16_t x_end,
                            uint16_t y_end);
static ret_t lcd_draw_char_internal(uint16_t x,
                                    uint16_t y,
                                    uint8_t ch,
                                    uint16_t color,
                                    uint16_t bg_color);

/************************************************************
 * @brief   初始化 LCD 外部总线
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始化
 ************************************************************/
ret_t dev_lcd_init(void)
{
    ret_t ret;

    if (s_dev_lcd_inited != 0U)
    {
        return RET_ALREADY_INIT;
    }

    /* LCD 控制器尚未完成初始化，先通过 BSP 关闭背光。 */
    ret = bsp_gpio_write(BSP_GPIO_LCD_BL, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_fsmc_init(BSP_LCD);
    if (ret != RET_OK)
    {
        return ret;
    }

    s_dev_lcd_inited = 1U;

    DBG_log("dev_lcd bus init ok, base=0x%08lX\r\n",
            (unsigned long)LCD_BUS_BASE_ADDR);

    return RET_OK;
}

/************************************************************
 * @brief   读取 ILI9488 控制器 ID
 * @param   id  输出控制器 ID
 * @retval  RET_OK 成功; RET_NOT_INIT 总线未初始化; RET_NULL_PTR 空指针
 ************************************************************/
ret_t dev_lcd_read_id(uint16_t *id)
{
    ret_t ret;
    uint16_t dummy;
    uint16_t id_high;
    uint16_t id_low;

    if (s_dev_lcd_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (id == NULL)
    {
        return RET_NULL_PTR;
    }

    /* ILI9488 的 D3 读取顺序为：dummy、00、94、88。
     * 发完 0xD3 命令后，前两次读到的是无效的 dummy 数据（控制器流水线
     * 延迟），必须丢弃；第 3、4 次才是 ID 高/低字节。
     * 不丢 dummy 直接拼，会得到 0x0094 之类的错 ID。 */
    ret = lcd_bus_read_reg(LCD_CMD_READ_ID, &dummy);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_read_data(&dummy);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_read_data(&id_high);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_read_data(&id_low);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 拼 16 位 ID：16 位总线每次读回的高 8 位无意义，&0xFF 只取低字节，
     * 高字节左移 8 位后与低字节或起来，得到 0x9488。 */
    *id = (uint16_t)(((uint16_t)(id_high & 0x00FFU) << 8) |
                     (uint16_t)(id_low & 0x00FFU));

    return RET_OK;
}

/************************************************************
 * @brief   初始化 ILI9488 控制器并打开 LCD 背光
 * @retval  RET_OK 成功; RET_ALREADY_INIT 控制器已初始化;
 *          RET_NOT_INIT LCD 外部总线未初始化;
 *          RET_DEVICE_NOT_FOUND 控制器 ID 不匹配
 ************************************************************/
ret_t dev_lcd_init_controller(void)
{
    ret_t ret;
    uint16_t lcd_id = 0U;

    if (s_dev_lcd_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (s_dev_lcd_controller_ready != 0U)
    {
        return RET_ALREADY_INIT;
    }

    ret = dev_lcd_read_id(&lcd_id);
    if (ret != RET_OK)
    {
        return ret;
    }

    DBG_log("ILI9488 controller id=0x%04X\r\n", (unsigned int)lcd_id);

    if (lcd_id != LCD_EXPECTED_ID)
    {
        DBG_log("ILI9488 controller id mismatch, expected=0x%04X\r\n",
                (unsigned int)LCD_EXPECTED_ID);
        return RET_DEVICE_NOT_FOUND;
    }

    ret = lcd_apply_init_sequence();
    if (ret != RET_OK)
    {
        DBG_log("ILI9488 controller sequence failed, ret=%ld\r\n",
                (long)ret);
        return ret;
    }

    /* 只有控制器初始化序列全部成功后才打开背光。 */
    ret = bsp_gpio_write(BSP_GPIO_LCD_BL, BSP_GPIO_ON);
    if (ret != RET_OK)
    {
        return ret;
    }

    s_dev_lcd_controller_ready = 1U;

    DBG_log("dev_lcd controller init ok, id=0x%04X, backlight=on\r\n",
            (unsigned int)lcd_id);

    return RET_OK;
}

/************************************************************
 * @brief   写 LCD 命令（Device 内部总线原语）
 ************************************************************/
static ret_t lcd_bus_write_command(uint16_t command)
{
    if (s_dev_lcd_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    *s_dev_lcd_command = command;

    return RET_OK;
}

/************************************************************
 * @brief   写 LCD 数据（Device 内部总线原语）
 ************************************************************/
static ret_t lcd_bus_write_data(uint16_t data)
{
    if (s_dev_lcd_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    *s_dev_lcd_data = data;

    return RET_OK;
}

/************************************************************
 * @brief   读 LCD 数据（Device 内部总线原语）
 ************************************************************/
static ret_t lcd_bus_read_data(uint16_t *data)
{
    if (s_dev_lcd_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (data == NULL)
    {
        return RET_NULL_PTR;
    }

    *data = *s_dev_lcd_data;

    return RET_OK;
}

/************************************************************
 * @brief   写 LCD 寄存器（Device 内部兼容封装）
 ************************************************************/
static ret_t lcd_bus_write_reg(uint16_t reg, uint16_t data)
{
    ret_t ret;

    ret = lcd_bus_write_command(reg);
    if (ret != RET_OK)
    {
        return ret;
    }

    return lcd_bus_write_data(data);
}

/************************************************************
 * @brief   读 LCD 寄存器（Device 内部兼容封装）
 ************************************************************/
static ret_t lcd_bus_read_reg(uint16_t reg, uint16_t *data)
{
    ret_t ret;

    ret = lcd_bus_write_command(reg);
    if (ret != RET_OK)
    {
        return ret;
    }

    return lcd_bus_read_data(data);
}

/************************************************************
 * @brief   执行 ILI9488 初始化命令表（Device 内部）
 * @note    表驱动初始化：所有寄存器配置收敛在 s_ili9488_init_sequence
 *          数组里，函数只负责“解释执行”。换屏时只改表不动函数。
 *          每一步按参数个数分三种写法：
 *          0 个参数=纯命令（如 Sleep Out）；1 个=命令+1 数据；
 *          多个=先发命令再连续发 N 个数据（ILI9488 的寄存器多为多参数）。
 ************************************************************/
static ret_t lcd_apply_init_sequence(void)
{
    uint32_t i;
    uint32_t j;
    ret_t ret;
    const lcd_init_step_t *step;

    /* 表长度用 sizeof 相除自适应：增删表项不用改循环，
     * 这是 C 语言遍历静态表的标准惯用法。 */
    for (i = 0U;
         i < (sizeof(s_ili9488_init_sequence) /
              sizeof(s_ili9488_init_sequence[0]));
         i++)
    {
        step = &s_ili9488_init_sequence[i];

        if (step->data_count == 0U)
        {
            ret = lcd_bus_write_command(step->command);
        }
        else if (step->data_count == 1U)
        {
            ret = lcd_bus_write_reg(step->command, step->data[0]);
        }
        else
        {
            ret = lcd_bus_write_command(step->command);
            if (ret == RET_OK)
            {
                for (j = 0U; j < step->data_count; j++)
                {
                    ret = lcd_bus_write_data(step->data[j]);
                    if (ret != RET_OK)
                    {
                        break;
                    }
                }
            }
        }

        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return RET_OK;
}

/************************************************************
 * @brief   设置 ILI9488 GRAM 地址窗口（Device 内部）
 * @note    “开窗”是 TFT 画图的总前置步骤，分三步：
 *          1) 0x2A 命令 + 起/止列地址（各 2 字节，大端）；
 *          2) 0x2B 命令 + 起/止页地址（各 2 字节，大端）；
 *          3) 0x2C 命令：告诉控制器“后面来的数据全部按顺序填进窗内”，
 *             地址指针自动递增，填满一行自动换行。
 *          之后 fill_rect/draw_char 只需连续写像素，不用逐点设地址。 
 ************************************************************/
static ret_t lcd_set_window(uint16_t x_start,
                            uint16_t y_start,
                            uint16_t x_end,
                            uint16_t y_end)
{
    ret_t ret;

    /* 列地址：起始列高字节/低字节、结束列高字节/低字节，大端顺序 */
    ret = lcd_bus_write_command(LCD_CMD_COLUMN_ADDRESS);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_data((uint16_t)(x_start >> 8));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_data((uint16_t)(x_start & 0x00FFU));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_data((uint16_t)(x_end >> 8));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_data((uint16_t)(x_end & 0x00FFU));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_command(LCD_CMD_PAGE_ADDRESS);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_data((uint16_t)(y_start >> 8));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_data((uint16_t)(y_start & 0x00FFU));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_data((uint16_t)(y_end >> 8));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = lcd_bus_write_data((uint16_t)(y_end & 0x00FFU));
    if (ret != RET_OK)
    {
        return ret;
    }

    return lcd_bus_write_command(LCD_CMD_MEMORY_WRITE);
}

/************************************************************
 * @brief   填充一个闭区间矩形
 * @param   x_start  起始 X 坐标
 * @param   y_start  起始 Y 坐标
 * @param   x_end    结束 X 坐标（包含）
 * @param   y_end    结束 Y 坐标（包含）
 * @param   color    RGB565 颜色
 * @retval  RET_OK 成功; RET_NOT_READY 控制器未初始化;
 *          RET_INVALID_PARAM 起止坐标非法; RET_OUT_OF_RANGE 坐标越界
 ************************************************************/
ret_t dev_lcd_fill_rect(uint16_t x_start,
                        uint16_t y_start,
                        uint16_t x_end,
                        uint16_t y_end,
                        uint16_t color)
{
    ret_t ret;
    uint32_t point_count;
    uint32_t i;

    if (s_dev_lcd_controller_ready == 0U)
    {
        return RET_NOT_READY;
    }

    if ((x_start > x_end) || (y_start > y_end))
    {
        return RET_INVALID_PARAM;
    }

    if ((x_end >= DEV_LCD_WIDTH) || (y_end >= DEV_LCD_HEIGHT))
    {
        return RET_OUT_OF_RANGE;
    }

    ret = lcd_set_window(x_start, y_start, x_end, y_end);
    if (ret != RET_OK)
    {
        return ret;
    }

    point_count = ((uint32_t)x_end - (uint32_t)x_start + 1U) *
                  ((uint32_t)y_end - (uint32_t)y_start + 1U);

    /*
     * 逐像素写 GRAM。
     * 这里刻意"内联"写数据寄存器（不复用 lcd_bus_write_data），原因：
     *   1) 省掉 15 万次函数调用与状态检查，刷屏耗时直接砍掉一大截；
     *   2) 单次循环体极短，编译器可以把它排成紧凑的 store 序列。
     * s_dev_lcd_inited 已在函数入口由 lcd_set_window() 间接保证（它同样
     * 检查了该标志），此处无需重复判断。
     *
     * 为什么每 DEV_LCD_FILL_YIELD_PIXELS 个像素让一次 CPU：
     *   全屏 320x480 = 153600 次外部总线写。若一口气写完，LCD 任务会
     *   独占 CPU 数十毫秒，App 层负责 485 轮询与 MQTT 上报的 task
     *   会被彻底饿死（tick 虽在走，但没有任务切换点）。
     *   周期性 taskYIELD() 让同优先级任务有机会运行，代价可忽略。
     */
    for (i = 0U; i < point_count; i++)
    {
        *s_dev_lcd_data = color;

        if ((i % DEV_LCD_FILL_YIELD_PIXELS) ==
            (DEV_LCD_FILL_YIELD_PIXELS - 1U))
        {
            taskYIELD();
        }
    }

    return RET_OK;
}

/************************************************************
 * @brief   清空 LCD 整个显示区域
 * @param   color RGB565 颜色
 * @retval  RET_OK 成功; RET_NOT_READY 控制器未初始化
 ************************************************************/
ret_t dev_lcd_clear(uint16_t color)
{
    return dev_lcd_fill_rect(0U,
                             0U,
                             (uint16_t)(DEV_LCD_WIDTH - 1U),
                             (uint16_t)(DEV_LCD_HEIGHT - 1U),
                             color);
}

/************************************************************
 * @brief   绘制一个像素点
 * @param   x      X 坐标
 * @param   y      Y 坐标
 * @param   color  RGB565 颜色
 * @retval  RET_OK 成功; RET_NOT_READY 控制器未初始化;
 *          RET_OUT_OF_RANGE 坐标越界
 ************************************************************/
ret_t dev_lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    return dev_lcd_fill_rect(x, y, x, y, color);
}

/************************************************************
 * @brief   绘制一个 ASCII 字符（Device 内部实现）
 ************************************************************/
static ret_t lcd_draw_char_internal(uint16_t x,
                                    uint16_t y,
                                    uint8_t ch,
                                    uint16_t color,
                                    uint16_t bg_color)
{
    const uint8_t *glyph;
    uint8_t row;
    uint8_t column;
    uint8_t pixels;
    ret_t ret;

    if (s_dev_lcd_controller_ready == 0U)
    {
        return RET_NOT_READY;
    }

    if ((ch < (uint8_t)' ') || (ch > (uint8_t)'~'))
    {
        return RET_INVALID_PARAM;
    }

    if ((x > (uint16_t)(DEV_LCD_WIDTH - DEV_LCD_FONT_WIDTH)) ||
        (y > (uint16_t)(DEV_LCD_HEIGHT - DEV_LCD_FONT_HEIGHT)))
    {
        return RET_OUT_OF_RANGE;
    }

    ret = lcd_set_window(x,
                         y,
                         (uint16_t)(x + DEV_LCD_FONT_WIDTH - 1U),
                         (uint16_t)(y + DEV_LCD_FONT_HEIGHT - 1U));
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 字模下标：表里只存了从空格(' '=0x20)开始的 95 个可打印字符，
     * ch-' ' 把 ASCII 码换算成表内偏移。 */
    glyph = s_lcd_ascii_8x16[(uint8_t)(ch - (uint8_t)' ')];
    /* 逐行扫描字模：glyph[row] 是该行的 8 个像素，bit7=最左像素。
     * 掩码 0x80>>column 依次取出 b7..b0：为 1 画前景色，为 0 画背景色。
     * 写顺序与窗口自动递增方向一致（先行后列），16 行 × 8 列
     * 正好填满前面开的 8x16 窗口。 */
    for (row = 0U; row < DEV_LCD_FONT_HEIGHT; row++)
    {
        pixels = glyph[row];
        for (column = 0U; column < DEV_LCD_FONT_WIDTH; column++)
        {
            if ((pixels & (uint8_t)(0x80U >> column)) != 0U)
            {
                ret = lcd_bus_write_data(color);
            }
            else
            {
                ret = lcd_bus_write_data(bg_color);
            }

            if (ret != RET_OK)
            {
                return ret;
            }
        }
    }

    return RET_OK;
}

/************************************************************
 * @brief   绘制一个 ASCII 字符
 ************************************************************/
ret_t dev_lcd_draw_char(uint16_t x,
                        uint16_t y,
                        char ch,
                        uint16_t color,
                        uint16_t bg_color)
{
    return lcd_draw_char_internal(x,
                                  y,
                                  (uint8_t)ch,
                                  color,
                                  bg_color);
}

/************************************************************
 * @brief   绘制一行 ASCII 文本
 ************************************************************/
ret_t dev_lcd_draw_text(uint16_t x,
                        uint16_t y,
                        const char *text,
                        uint16_t color,
                        uint16_t bg_color)
{
    const char *cursor;
    uint32_t text_length;
    uint32_t index;
    uint8_t ch;
    ret_t ret;

    if (s_dev_lcd_controller_ready == 0U)
    {
        return RET_NOT_READY;
    }

    if (text == NULL)
    {
        return RET_NULL_PTR;
    }

    text_length = 0U;
    cursor = text;
    while (*cursor != '\0')
    {
        ch = (uint8_t)(*cursor);
        if ((ch < (uint8_t)' ') || (ch > (uint8_t)'~'))
        {
            return RET_INVALID_PARAM;
        }

        text_length++;
        cursor++;
    }

    if (text_length == 0U)
    {
        return RET_OK;
    }

    if ((x >= DEV_LCD_WIDTH) ||
        (y > (uint16_t)(DEV_LCD_HEIGHT - DEV_LCD_FONT_HEIGHT)) ||
        (text_length >
         ((uint32_t)DEV_LCD_WIDTH - (uint32_t)x) / DEV_LCD_FONT_WIDTH))
    {
        return RET_OUT_OF_RANGE;
    }

    for (index = 0U; index < text_length; index++)
    {
        ret = lcd_draw_char_internal(
            (uint16_t)(x + index * DEV_LCD_FONT_WIDTH),
            y,
            (uint8_t)text[index],
            color,
            bg_color);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return RET_OK;
}

/************************************************************
 * @brief   按整数倍缩放绘制一行 ASCII 文本
 ************************************************************/
ret_t dev_lcd_draw_text_scaled(uint16_t x,
                               uint16_t y,
                               const char *text,
                               uint8_t scale,
                               uint16_t color,
                               uint16_t bg_color)
{
    const uint8_t *glyph;
    const char *cursor;
    uint32_t text_length;
    uint32_t total_width;
    uint32_t total_height;
    uint32_t char_index;
    uint8_t row;
    uint8_t column;
    uint8_t repeat_x;
    uint8_t repeat_y;
    uint8_t pixels;
    uint16_t pixel_color;
    ret_t ret;

    if (s_dev_lcd_controller_ready == 0U)
    {
        return RET_NOT_READY;
    }

    if (text == NULL)
    {
        return RET_NULL_PTR;
    }

    if ((scale == 0U) || (scale > 4U))
    {
        return RET_INVALID_PARAM;
    }

    text_length = 0U;
    cursor = text;
    while (*cursor != '\0')
    {
        if (((uint8_t)*cursor < (uint8_t)' ') ||
            ((uint8_t)*cursor > (uint8_t)'~'))
        {
            return RET_INVALID_PARAM;
        }
        text_length++;
        cursor++;
    }

    if (text_length == 0U)
    {
        return RET_OK;
    }

    total_width = text_length * DEV_LCD_FONT_WIDTH * scale;
    total_height = DEV_LCD_FONT_HEIGHT * scale;
    if (((uint32_t)x + total_width > DEV_LCD_WIDTH) ||
        ((uint32_t)y + total_height > DEV_LCD_HEIGHT))
    {
        return RET_OUT_OF_RANGE;
    }

    for (char_index = 0U; char_index < text_length; char_index++)
    {
        ret = lcd_set_window(
            (uint16_t)((uint32_t)x +
                       char_index * DEV_LCD_FONT_WIDTH * scale),
            y,
            (uint16_t)((uint32_t)x +
                       (char_index + 1U) * DEV_LCD_FONT_WIDTH * scale - 1U),
            (uint16_t)((uint32_t)y + total_height - 1U));
        if (ret != RET_OK)
        {
            return ret;
        }

        glyph = s_lcd_ascii_8x16[
            (uint8_t)text[char_index] - (uint8_t)' '];
        for (row = 0U; row < DEV_LCD_FONT_HEIGHT; row++)
        {
            pixels = glyph[row];
            for (repeat_y = 0U; repeat_y < scale; repeat_y++)
            {
                for (column = 0U; column < DEV_LCD_FONT_WIDTH; column++)
                {
                    pixel_color =
                        ((pixels & (uint8_t)(0x80U >> column)) != 0U) ?
                        color : bg_color;
                    for (repeat_x = 0U; repeat_x < scale; repeat_x++)
                    {
                        ret = lcd_bus_write_data(pixel_color);
                        if (ret != RET_OK)
                        {
                            return ret;
                        }
                    }
                }
            }
        }
    }

    return RET_OK;
}
