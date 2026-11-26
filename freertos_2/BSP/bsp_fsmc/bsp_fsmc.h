/**
 * @file    bsp_fsmc.h
 * @brief   外部存储器控制器（EXMC/FSMC）BSP 驱动接口
 *
 * @note    本模块管理 EXMC 上的外设（外部 SRAM、LCD 等），
 *          上层通过 bsp_fsmc_id_t 枚举索引选择目标外设。
 */
#ifndef  __BSP_FSMC_H__
#define  __BSP_FSMC_H__
#include "config.h"

/**
 * @brief   EXMC 外设 ID 枚举
 */
typedef enum
{
    BSP_SRAM = 0U,          /**< 外部 SRAM */
    BSP_LCD,                /**< 外部 LCD */
    BSP_FSMC_MAX
} bsp_fsmc_id_t;

/************************************************************
 * @brief   初始化指定 EXMC 外设
 * @param   id  外设 ID（BSP_SRAM / BSP_LCD）
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_SUPPORT 该外设未使能
 ************************************************************/
ret_t bsp_fsmc_init(bsp_fsmc_id_t id);

#endif /* __BSP_FSMC_H__ */
