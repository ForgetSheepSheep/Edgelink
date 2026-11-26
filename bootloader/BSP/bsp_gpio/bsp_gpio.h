/**
 * @file    bsp_gpio.h
 * @brief   GPIO BSP 驱动接口
 *
 * @note    本模块通过固定硬件配置表统一管理 GPIO 引脚，
 *          上层通过 bsp_gpio_id_t 枚举索引选择对应引脚；不使用回调。
 *          init() 时批量初始化表中所有 GPIO，并将 LCD 背光置为关闭。
 */
#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "config.h"

/** GPIO 电平定义 */
#define BSP_GPIO_OFF    0U
#define BSP_GPIO_ON     1U

/**
 * @brief   GPIO 设备 ID 枚举
 */
typedef enum
{
    BSP_GPIO_LED1 = 0U,         /**< LED1     (PA8)  */
    BSP_GPIO_LED2,              /**< LED2     (PE6)  */
    BSP_GPIO_LED3,              /**< LED3     (PF6)  */
    BSP_GPIO_WIFI_CTRL,         /**< WiFi控制  (PG7)  */
    BSP_GPIO_SPI_FLASH_CS,      /**< SPI Flash CS (PE2) */
    BSP_GPIO_LCD_BL,            /**< LCD背光   (PA1)  */
    BSP_GPIO_GT911_RST,         /**< GT911复位 (PB9)  */
    BSP_GPIO_GT911_INT,         /**< GT911中断/地址选择 (PB12) */
    BSP_GPIO_MAX
} bsp_gpio_id_t;

/**
 * @brief   初始化所有 GPIO（批量配置表中引脚）
 * @retval  RET_OK 成功
 */
ret_t bsp_gpio_init(void);

/**
 * @brief   初始化 FSMC/EXMC 复用 GPIO
 */
ret_t bsp_gpio_fsmc_init(void);

/**
 * @brief   读取 GPIO 输入电平
 * @param   id     GPIO 设备 ID
 * @param   level  输出电平（0 或 1）
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法
 */
ret_t bsp_gpio_read(bsp_gpio_id_t id, uint8_t *level);

/************************************************************
 * @brief   按 GPIO 表配置指定引脚为默认输出模式
 * @param   id  GPIO 设备 ID
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t bsp_gpio_config_output(bsp_gpio_id_t id);

/************************************************************
 * @brief   将指定 GPIO 切换为浮空输入
 * @param   id  GPIO 设备 ID
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t bsp_gpio_config_input(bsp_gpio_id_t id);

/**
 * @brief   设置 GPIO 输出电平
 * @param   id     GPIO 设备 ID
 * @param   level  输出电平（BSP_GPIO_ON / BSP_GPIO_OFF）
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法
 */
ret_t bsp_gpio_write(bsp_gpio_id_t id, uint8_t level);

#endif /* __BSP_GPIO_H__ */
