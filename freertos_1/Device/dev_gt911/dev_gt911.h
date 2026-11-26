/**
 * @file    dev_gt911.h
 * @brief   GT911 触摸控制器设备驱动接口
 *
 * @note    本 Device 层依赖通用 GPIO/I²C BSP，负责 GT911 寄存器语义和设备状态；
 *          不创建专用的 bsp_gt911 模块。
 *          当前实现复位、地址选择、初始化、产品 ID、状态/坐标读取、
 *          状态清零和中断到任务通知；坐标变换和手势仍由上层决定。
 */
#ifndef __DEV_GT911_H__
#define __DEV_GT911_H__

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

/** GT911 产品 ID 读取 4 字节，另留 1 字节字符串结束符。 */
#define DEV_GT911_PRODUCT_ID_LENGTH       (4U)
#define DEV_GT911_PRODUCT_ID_TEXT_SIZE    (DEV_GT911_PRODUCT_ID_LENGTH + 1U)

/** GT911 最多同时报告的触摸点数量。 */
#define DEV_GT911_TOUCH_POINT_MAX         (5U)

/** GT911 触摸状态。 */
typedef enum
{
    DEV_GT911_TOUCH_UP = 0U,
    DEV_GT911_TOUCH_DOWN = 1U
} dev_gt911_touch_state_t;

/** 单个 GT911 触摸点（坐标和接触面积均为小端 16 位值）。 */
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t size;
} dev_gt911_touch_point_t;

/** 一次 GT911 触摸扫描结果。 */
typedef struct
{
    dev_gt911_touch_state_t state;
    uint8_t point_count;
    dev_gt911_touch_point_t points[DEV_GT911_TOUCH_POINT_MAX];
} dev_gt911_touch_info_t;

/************************************************************
 * @brief   初始化 GT911 并读取/校验产品 ID
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始化;
 *          RET_DEVICE_NOT_FOUND 产品 ID 无效; 其他为 BSP/I²C 错误
 ************************************************************/
ret_t dev_gt911_init(void);

/************************************************************
 * @brief   获取已经校验并规范化为 ASCII 文本的 GT911 产品 ID
 * @param   buffer      输出缓冲区，至少 DEV_GT911_PRODUCT_ID_TEXT_SIZE 字节
 * @param   buffer_size 输出缓冲区大小
 * @retval  RET_OK 成功; RET_NOT_INIT 设备未初始化; RET_NULL_PTR 空指针;
 *          RET_LENGTH_ERROR 缓冲区长度不足
 ************************************************************/
ret_t dev_gt911_product_id_get(char *buffer, uint16_t buffer_size);

/************************************************************
 * @brief   读取一次 GT911 触摸状态、坐标并清除状态
 * @param   info 输出的触摸快照；无触摸时 state=DEV_GT911_TOUCH_UP
 * @retval  RET_OK 成功（包含无触摸）; RET_NOT_INIT 未初始化;
 *          RET_NULL_PTR 空指针; RET_DATA_FORMAT_ERROR 点数非法;
 *          其他为 I²C 读写错误
 ************************************************************/
ret_t dev_gt911_touch_read(dev_gt911_touch_info_t *info);

/************************************************************
 * @brief   配置 PB12 为 GT911 INT 上升沿 EXTI12，并使能中断
 * @retval  RET_OK 成功; RET_NOT_INIT 设备未初始化;
 *          RET_ALREADY_INIT 已配置; 其他为硬件配置错误
 ************************************************************/
ret_t dev_gt911_touch_irq_init(void);

/************************************************************
 * @brief   注册由 GT911 INT 唤醒的任务
 * @param   task 任务句柄
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_NOT_INIT 未初始化;
 *          RET_ALREADY_INIT 已注册
 ************************************************************/
ret_t dev_gt911_register_notify_task(TaskHandle_t task);

/************************************************************
 * @brief   GT911 EXTI10_15 中断入口调用的 Device 钩子
 * @note    只清除 EXTI12 并发送任务通知，不执行 I²C、日志或 LCD 操作。
 ************************************************************/
void dev_gt911_irq_handler(void);

#endif /* __DEV_GT911_H__ */
