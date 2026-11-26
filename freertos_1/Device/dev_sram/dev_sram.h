/**
 * @file    dev_sram.h
 * @brief   外部 SRAM 设备驱动接口
 *
 * @note    外部 SRAM 通过 EXMC/FSMC 接口访问（BSP_SRAM 区，Region3），
 *          地址参数 addr 为相对外部 SRAM 基址的偏移量。
 */
#ifndef __DEV_SRAM_H__
#define __DEV_SRAM_H__

#include "config.h"

/** 外部 SRAM 基地址 */
#define DEV_SRAM_BASE_ADDR    ((uint32_t)0x6C000000UL)
/** 外部 SRAM 容量（1MB） */
#define DEV_SRAM_SIZE         (1UL * 1024UL * 1024UL)

/************************************************************
 * @brief   初始化外部 SRAM（依赖 bsp_fsmc_init(BSP_SRAM)）
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始化; RET_NOT_SUPPORT 未使能
 ************************************************************/
ret_t dev_sram_init(void);

/************************************************************
 * @brief   从外部 SRAM 读取数据
 * @param   addr  读取偏移地址（相对 SRAM 基址）
 * @param   buf   数据缓冲区
 * @param   len   数据长度（字节）
 * @retval  RET_OK 成功; RET_NOT_INIT 未初始化; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t dev_sram_read(uint32_t addr, uint8_t *buf, uint32_t len);

/************************************************************
 * @brief   向外部 SRAM 写入数据
 * @param   addr  写入偏移地址（相对 SRAM 基址）
 * @param   buf   数据缓冲区
 * @param   len   数据长度（字节）
 * @retval  RET_OK 成功; RET_NOT_INIT 未初始化; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t dev_sram_write(uint32_t addr, const uint8_t *buf, uint32_t len);

/************************************************************
 * @brief   外部 SRAM 读写自测（写 256 字节回读比对）
 * @retval  RET_OK 成功; RET_DATA_ERROR 数据校验失败
 ************************************************************/
ret_t dev_sram_test(void);

#endif /* __DEV_SRAM_H__ */
