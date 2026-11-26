/**
 * @file    bsp_uart.h
 * @brief   UART BSP 驱动接口（结构体表管理）
 *
 * @note    通过 uart_info_t 结构体表管理 4 路串口，
 *          接收支持 DMA+IDLE 中断（环形缓冲）或逐字节 RBNE 中断两种方式，
 *          任务可注册为接收通知任务，收到数据后由 ISR 唤醒。
 */
#ifndef __BSP_UART_H__
#define __BSP_UART_H__
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"

/**
 * @brief   UART 设备 ID 枚举
 */
typedef enum
{
    BSP_UART_HOST = 0U,         /**< 主机调试串口 USART0（PA9/PA10） */
    BSP_UART_NET,               /**< 网络模块串口 USART2（PB10/PB11） */
    BSP_UART_DISPLAY,           /**< 显示模块串口 UART4（PC12/PD2） */
    BSP_UART_LORA,              /**< LoRa 模块串口 UART3（PC10/PC11） */
    BSP_UART_MAX                /**< 串口总数（同时作为数组长度与越界判断边界） */
} bsp_uart_id_t;

/************************************************************
 * @brief   初始化指定 UART（环形缓冲 + 中断/DMA 接收）
 * @param   uart_id  UART 设备 ID
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法; RET_NO_MEMORY 内存不足
 ************************************************************/
ret_t bsp_uart_init(bsp_uart_id_t uart_id);

/************************************************************
 * @brief   发送数据（互斥保护）
 * @param   uart_id   UART 设备 ID
 * @param   string    待发送数据
 * @param   length    数据长度
 * @param   timeout_ms 等待互斥锁超时（ms）
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_TIMEOUT 超时; RET_NOT_INIT 未初始化
 ************************************************************/
ret_t bsp_uart_transmit(bsp_uart_id_t uart_id, uint8_t *string, uint16_t length, uint16_t timeout_ms);

/************************************************************
 * @brief   从接收环形缓冲区取一个字节（非阻塞）
 * @param   uart_id  UART 设备 ID
 * @param   byte     读出的字节
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_EMPTY 缓冲区为空
 ************************************************************/
ret_t bsp_uart_receive(bsp_uart_id_t uart_id, uint8_t *byte);

/************************************************************
 * @brief   注册接收通知任务（收到数据时由 ISR 唤醒）
 * @param   uart_id  UART 设备 ID
 * @param   task     任务句柄
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_INIT 未初始化
 ************************************************************/
ret_t bsp_uart_register_notify_task(bsp_uart_id_t uart_id, TaskHandle_t task);

#endif /* __BSP_UART_H__ */
