/**
 * @file    bsp_wwdgt.h
 * @brief   窗口看门狗（WWDGT）BSP 驱动接口
 *
 * @note    窗口看门狗基于 PCLK1，喂狗必须落在"窗口"内，
 *          过早（窗口外）或过晚（超时）都会触发复位。
 */
#ifndef __BSP_WWDGT_H__
#define __BSP_WWDGT_H__

#include "config.h"

/************************************************************
 * @brief   初始化窗口看门狗
 * @retval  RET_OK 成功
 * @note    计数器 0x7F、窗口 0x5F、8 分频
 ************************************************************/
ret_t bsp_wwdgt_init(void);

/************************************************************
 * @brief   喂狗（更新计数器到 0x7F）
 * @retval  RET_OK 成功
 ************************************************************/
ret_t bsp_wwdgt_feed(void);

#endif /* __BSP_WWDGT_H__ */
