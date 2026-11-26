/**
 * @file    bsp_delay.h
 * @brief   微秒/毫秒延时 BSP 驱动接口（DWT 计数器实现）
 *
 * @note    基于 Cortex-M 内核 DWT->CYCCNT 周期计数器实现高精度忙等延时，
 *          延时前会自动完成初始化，不依赖 SysTick。
 */
#ifndef __BSP_DELAY_H__
#define __BSP_DELAY_H__

#include "config.h"

/************************************************************
 * @brief   初始化 DWT 周期计数器
 * @retval  RET_OK 成功
 ************************************************************/
ret_t bsp_delay_init(void);

/************************************************************
 * @brief   忙等延时（微秒）
 * @param   us  延时微秒数
 * @retval  RET_OK 成功
 ************************************************************/
ret_t bsp_delay_us(uint32_t us);

/************************************************************
 * @brief   忙等延时（毫秒）
 * @param   ms  延时毫秒数
 * @retval  RET_OK 成功
 ************************************************************/
ret_t bsp_delay_ms(uint32_t ms);

/**
 * @brief   获取当前 DWT 周期计数
 * @retval  当前周期计数值
 * @note    用于外设轮询的超时起点，不执行固定延时。
 */
uint32_t bsp_delay_cycle_get(void);

/**
 * @brief   判断从指定周期起点开始是否已超时
 * @param   start_cycle 周期计数起点
 * @param   timeout_us  超时时间（微秒）
 * @retval  1 已超时；0 未超时
 */
uint8_t bsp_delay_timeout_expired(uint32_t start_cycle,
                                  uint32_t timeout_us);

#endif /* __BSP_DELAY_H__ */
