/**
 * @file    bsp_gpio.c
 * @brief   GPIO BSP 驱动实现（结构体表管理）
 *
 * @note    通过 bsp_gpio_t 结构体表配置每个引脚的端口、时钟、引脚号、模式和速度，
 *          init() 时遍历整表批量初始化。
 */

#include "./bsp_gpio/bsp_gpio.h"
#include "gd32f30x.h"

/**
 * @brief   GPIO 硬件信息结构体
 */
typedef struct
{
    uint32_t gpio_periph;       /**< GPIO 端口基址 */
    rcu_periph_enum gpio_clk;   /**< GPIO 时钟 */
    uint32_t gpio_pin;          /**< 引脚号 */
    uint32_t gpio_mode;         /**< 引脚模式 */
    uint32_t gpio_speed;        /**< 引脚速度 */
} bsp_gpio_t;

/** GPIO 硬件信息表
 * @note 表驱动设计：新增一个 LED/控制脚只需加一行，
 *       init 循环自动覆盖。id 就是数组下标（见 bsp_gpio.h 枚举），
 *       增删行必须同步改枚举顺序，否则 id 错位。 */
static bsp_gpio_t bsp_gpio_tables[] =
{
    /* LED1:      PA8  推挽输出 2MHz   */
    {GPIOA, RCU_GPIOA, GPIO_PIN_8,  GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ},
    /* LED2:      PE6  推挽输出 2MHz   */
    {GPIOE, RCU_GPIOE, GPIO_PIN_6,  GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ},
    /* LED3:      PF6  推挽输出 2MHz   */
    {GPIOF, RCU_GPIOF, GPIO_PIN_6,  GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ},
    /* WiFi_CTRL: PG7  开漏输出 10MHz  */
    {GPIOG, RCU_GPIOG, GPIO_PIN_7,  GPIO_MODE_OUT_OD, GPIO_OSPEED_10MHZ},
    /* SPI Flash CS: PE2 推挽输出 50MHz */
    {GPIOE, RCU_GPIOE, GPIO_PIN_2,  GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ},
    /* LCD 背光：PA1 推挽输出 2MHz，初始化后保持关闭 */
    {GPIOA, RCU_GPIOA, GPIO_PIN_1,  GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ},
    /* GT911 复位：PB9 推挽输出 10MHz */
    {GPIOB, RCU_GPIOB, GPIO_PIN_9,  GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ},
    /* GT911 中断/地址选择：PB12 推挽输出 10MHz，地址锁存后切换输入 */
    {GPIOB, RCU_GPIOB, GPIO_PIN_12, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ},
};

/** FSMC/EXMC 外部 SRAM 复用 GPIO 配置表
 * @note 全部 AF_PP（复用推挽）：这些引脚不归 GPIO 管，
 *       归 EXMC 控制器驱动（地址/数据/片选时序由硬件产生）。
 *       GPIO 模式下 EXMC 信号出不来，这是外扩 SRAM/LCD 无反应的常见原因。
 *       const 修饰：表只读，进 Flash 不占 RAM。 */
static const bsp_gpio_t bsp_fsmc_gpio_tables[] =
{
    /* Address bus: A0-A5 */
    {GPIOF, RCU_GPIOF, GPIO_PIN_0,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOF, RCU_GPIOF, GPIO_PIN_1,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOF, RCU_GPIOF, GPIO_PIN_2,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOF, RCU_GPIOF, GPIO_PIN_3,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOF, RCU_GPIOF, GPIO_PIN_4,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOF, RCU_GPIOF, GPIO_PIN_5,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},

    /* Address bus: A6-A9 */
    {GPIOF, RCU_GPIOF, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOF, RCU_GPIOF, GPIO_PIN_13, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOF, RCU_GPIOF, GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOF, RCU_GPIOF, GPIO_PIN_15, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},

    /* Address bus: A10-A15 */
    {GPIOG, RCU_GPIOG, GPIO_PIN_0,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOG, RCU_GPIOG, GPIO_PIN_1,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOG, RCU_GPIOG, GPIO_PIN_2,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOG, RCU_GPIOG, GPIO_PIN_3,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOG, RCU_GPIOG, GPIO_PIN_4,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOG, RCU_GPIOG, GPIO_PIN_5,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},

    /* Address bus: A16-A18 */
    {GPIOD, RCU_GPIOD, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOD, RCU_GPIOD, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOD, RCU_GPIOD, GPIO_PIN_13, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},

    /* Data bus: D0-D3 */
    {GPIOD, RCU_GPIOD, GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOD, RCU_GPIOD, GPIO_PIN_15, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOD, RCU_GPIOD, GPIO_PIN_0,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOD, RCU_GPIOD, GPIO_PIN_1,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},

    /* Data bus: D4-D12 */
    {GPIOE, RCU_GPIOE, GPIO_PIN_7,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOE, RCU_GPIOE, GPIO_PIN_8,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOE, RCU_GPIOE, GPIO_PIN_9,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOE, RCU_GPIOE, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOE, RCU_GPIOE, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOE, RCU_GPIOE, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOE, RCU_GPIOE, GPIO_PIN_13, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOE, RCU_GPIOE, GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOE, RCU_GPIOE, GPIO_PIN_15, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},

    /* Data bus: D13-D15 */
    {GPIOD, RCU_GPIOD, GPIO_PIN_8,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOD, RCU_GPIOD, GPIO_PIN_9,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},
    {GPIOD, RCU_GPIOD, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX},

    /* LCD：EXMC 区域1控制信号：片选、读使能、写使能 */
    {GPIOG, RCU_GPIOG, GPIO_PIN_9,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX}, /* PG9：LCD片选（EXMC_NE1） */
    {GPIOD, RCU_GPIOD, GPIO_PIN_4,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX}, /* PD4：LCD读使能（EXMC_NOE） */
    {GPIOD, RCU_GPIOD, GPIO_PIN_5,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX}, /* PD5：LCD写使能（EXMC_NWE） */

    /* SRAM：EXMC 区域3控制信号：片选、低字节使能、高字节使能 */
    {GPIOG, RCU_GPIOG, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_OSPEED_MAX}, /* PG12：SRAM片选（EXMC_NE3） */
    {GPIOE, RCU_GPIOE, GPIO_PIN_0,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX}, /* PE0：SRAM低字节使能（EXMC_NBL0） */
    {GPIOE, RCU_GPIOE, GPIO_PIN_1,  GPIO_MODE_AF_PP, GPIO_OSPEED_MAX}, /* PE1：SRAM高字节使能（EXMC_NBL1） */
};

#define BSP_GPIO_NUM    (sizeof(bsp_gpio_tables) / sizeof(bsp_gpio_tables[0]))
#define BSP_FSMC_GPIO_NUM    (sizeof(bsp_fsmc_gpio_tables) / sizeof(bsp_fsmc_gpio_tables[0]))

/************************************************************
 * @brief   初始化所有 GPIO（批量配置表中引脚）
 * @retval  RET_OK 成功
 ************************************************************/
ret_t bsp_gpio_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_GPIO_NUM; i++)
    {
        const bsp_gpio_t *gpio_info = &bsp_gpio_tables[i];

        rcu_periph_clock_enable(gpio_info->gpio_clk);
        gpio_init(gpio_info->gpio_periph,
                  gpio_info->gpio_mode,
                  gpio_info->gpio_speed,
                   gpio_info->gpio_pin);
    }

    /* LCD 控制器完成初始化前，背光保持关闭，避免上电时显示异常画面。 */
    return bsp_gpio_write(BSP_GPIO_LCD_BL, BSP_GPIO_OFF);
}

/************************************************************
 * @brief   初始化 EXMC 地址/数据/控制复用引脚
 * @note    由 bsp_fsmc_init() 调用，顺序必须在 EXMC 控制器配置之前：
 *          先把引脚交给 EXMC（AF_PP），再配控制器时序。
 * @retval  RET_OK 成功
 ************************************************************/
ret_t bsp_gpio_fsmc_init(void)
{
    uint32_t i;

    for (i = 0U; i < BSP_FSMC_GPIO_NUM; i++)
    {
        const bsp_gpio_t *gpio_info = &bsp_fsmc_gpio_tables[i];

        rcu_periph_clock_enable(gpio_info->gpio_clk);
        gpio_init(gpio_info->gpio_periph,
                  gpio_info->gpio_mode,
                  gpio_info->gpio_speed,
                  gpio_info->gpio_pin);
    }

    return RET_OK;
}

/************************************************************
 * @brief   读取 GPIO 输入电平
 * @param   id     GPIO 设备 ID
 * @param   level  输出电平（0 或 1）
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t bsp_gpio_read(bsp_gpio_id_t id, uint8_t *level)
{
    const bsp_gpio_t *gpio_info;

    if (level == 0)
    {
        return RET_NULL_PTR;
    }

    if (id >= BSP_GPIO_NUM)
    {
        return RET_INVALID_PARAM;
    }

    gpio_info = &bsp_gpio_tables[id];

    *level = (gpio_input_bit_get(gpio_info->gpio_periph,
                                 gpio_info->gpio_pin) == SET) ? 1U : 0U;

    return RET_OK;
}

/************************************************************
 * @brief   按 GPIO 表配置指定引脚为默认输出模式
 ************************************************************/
ret_t bsp_gpio_config_output(bsp_gpio_id_t id)
{
    const bsp_gpio_t *gpio_info;

    if (id >= BSP_GPIO_NUM)
    {
        return RET_INVALID_PARAM;
    }

    gpio_info = &bsp_gpio_tables[id];
    rcu_periph_clock_enable(gpio_info->gpio_clk);
    gpio_init(gpio_info->gpio_periph,
              gpio_info->gpio_mode,
              gpio_info->gpio_speed,
              gpio_info->gpio_pin);

    return RET_OK;
}

/************************************************************
 * @brief   将指定 GPIO 切换为浮空输入
 ************************************************************/
ret_t bsp_gpio_config_input(bsp_gpio_id_t id)
{
    const bsp_gpio_t *gpio_info;

    if (id >= BSP_GPIO_NUM)
    {
        return RET_INVALID_PARAM;
    }

    gpio_info = &bsp_gpio_tables[id];
    rcu_periph_clock_enable(gpio_info->gpio_clk);
    gpio_init(gpio_info->gpio_periph,
              GPIO_MODE_IN_FLOATING,
              GPIO_OSPEED_50MHZ,
              gpio_info->gpio_pin);

    return RET_OK;
}

/************************************************************
 * @brief   设置 GPIO 输出电平
 * @param   id     GPIO 设备 ID
 * @param   level  输出电平（BSP_GPIO_ON / BSP_GPIO_OFF）
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t bsp_gpio_write(bsp_gpio_id_t id, uint8_t level)
{
    const bsp_gpio_t *gpio_info;

    if (id >= BSP_GPIO_NUM)
    {
        return RET_INVALID_PARAM;
    }

    gpio_info = &bsp_gpio_tables[id];

    if (level != 0U)
    {
        gpio_bit_set(gpio_info->gpio_periph, gpio_info->gpio_pin);
    }
    else
    {
        gpio_bit_reset(gpio_info->gpio_periph, gpio_info->gpio_pin);
    }

    return RET_OK;
}
