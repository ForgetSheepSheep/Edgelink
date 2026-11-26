/**
 * @file    dev_flash.h
 * @brief   内部 Flash 设备驱动接口（封装 bsp_flash）
 *
 * @note    本模块对 GD32F30x 内部 Flash 进行设备层封装，提供统一的
 *          初始化、擦除、写入、读取接口，供 App 层使用。
 *          底层依赖 bsp_flash（FMC 操作）。
 */
#ifndef __DEV_FLASH_H__
#define __DEV_FLASH_H__

#include "config.h"

/* 内部 Flash 页大小（GD32F30x = 2KB） */
#define DEV_FLASH_PAGE_SIZE     2048UL

/**
 * @brief   初始化内部 Flash
 * @retval  RET_OK 成功; 其他见 ret_error.h
 */
ret_t dev_flash_init(void);

/**
 * @brief   擦除内部 Flash 指定区域
 * @param   addr  起始地址（建议页对齐）
 * @param   size  擦除大小（字节）
 * @retval  RET_OK 成功; 其他见 ret_error.h
 */
ret_t dev_flash_erase(uint32_t addr, uint32_t size);

/**
 * @brief   向内部 Flash 写入数据
 * @param   addr  写入地址（建议 4 字节对齐）
 * @param   buf   数据缓冲区
 * @param   len   数据长度（字节）
 * @retval  RET_OK 成功; 其他见 ret_error.h
 */
ret_t dev_flash_write(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * @brief   从内部 Flash 读取数据（直接内存读取）
 * @param   addr  读取地址
 * @param   buf   接收缓冲区
 * @param   len   读取长度（字节）
 * @retval  RET_OK 成功; 其他见 ret_error.h
 */
ret_t dev_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);

#endif /* __DEV_FLASH_H__ */


