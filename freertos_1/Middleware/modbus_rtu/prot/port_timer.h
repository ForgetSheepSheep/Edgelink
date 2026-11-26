/**
 * @file    port_timer.h
 * @brief   Modbus RTU 定时器移植层接口
 *
 * 定时器以 50us 为一个基本计数单位，适合实现 RTU 的 3.5T 帧间隔。
 * 到期回调运行在硬件 ISR 中，回调内只能做快速通知，不能阻塞。
 */
#ifndef __MODBUS_RTU_PORT_TIMER_H__
#define __MODBUS_RTU_PORT_TIMER_H__

#include <stddef.h>
#include <stdint.h>

typedef void (*modbus_port_timer_callback_t)(void);

/**
 * @brief   初始化 Modbus RTU 硬件定时器
 * @param   timeout_50us  超时时间，单位 50us
 * @return  1=成功，0=参数非法或硬件配置无法表示
 * @note    初始化后定时器保持停止，必须调用 start 才开始计时。
 */
uint8_t modbus_port_timer_init(uint16_t timeout_50us);

/**
 * @brief   设置定时器到期回调
 * @param   callback  到期时在 TIMER ISR 中调用的函数；传 NULL 表示不回调
 */
void modbus_port_timer_set_callback(modbus_port_timer_callback_t callback);

/**
 * @brief   启动一次超时计时
 * @note    每次 start 都从 0 开始；到期后自动停止，不是周期定时器。
 */
void modbus_port_timer_start(void);

/**
 * @brief   停止并清零当前计时
 */
void modbus_port_timer_stop(void);

#endif /* __MODBUS_RTU_PORT_TIMER_H__ */
