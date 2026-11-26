/**
 * @file    bsp_systick.h
 * @brief   SysTick 定时器 BSP 驱动接口（已弃用）
 *
 * @note    SysTick 是 Cortex-M 内核自带的节拍定时器。
 *          本模块已弃用：当前工程统一使用 FreeRTOS 的
 *          xTaskGetTickCount() 获取系统 tick。
 */
#ifndef __BSP_SYSTICK_H__
#define __BSP_SYSTICK_H__
#include "config.h"

/************************************************************
 * @brief   注册 SysTick 周期回调（已弃用）
 * @param   pfunc  周期回调函数指针
 ************************************************************/
void bsp_systick_callback(void (*pfunc)(void));

/************************************************************
 * @brief   初始化 SysTick 定时器（已弃用）
 ************************************************************/
void bsp_systick_init(void);

/************************************************************
 * @brief   获取当前系统毫秒 tick（已弃用）
 * @retval  自启动以来经过的毫秒数
 ************************************************************/
uint32_t bsp_systick_get(void);

#endif /* __BSP_SYSTICK_H__ */
