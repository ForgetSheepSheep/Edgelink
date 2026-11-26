/**
 * @file    bsp_flash.h
 * @brief   内部 Flash BSP 驱动接口（GD32F30x FMC 操作）
 *
 * @note    本模块直接操作 GD32F30x 内部 Flash（FMC），提供擦除和写入接口。
 *          内部 Flash 页大小为 2KB，写入以 4 字节（word）为单位。
 */
#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

#include "config.h"

/** 内部 Flash 页大小（GD32F30x = 2KB） */
#define BSP_FLASH_PAGE_SIZE    2048UL
/** GD32F303ZE 内部 Flash 起始地址 */
#define BSP_FLASH_BASE_ADDR    0x08000000UL
/** GD32F303ZE 内部 Flash 结束地址（开区间，容量 512 KiB） */
#define BSP_FLASH_END_ADDR     0x08080000UL

/**
 * @brief   擦除内部 Flash 指定区域
 * @param   addr  起始地址（必须页对齐且位于内部 Flash 范围）
 * @param   size  擦除大小（字节，向上按页对齐）
 * @retval  RET_OK 成功; RET_INVALID_PARAM 地址/范围非法; RET_ERROR 擦除失败
 */
ret_t bsp_flash_erase(uint32_t addr, uint32_t size);

/**
 * @brief   向内部 Flash 写入数据
 * @param   addr  写入地址（必须 4 字节对齐且位于内部 Flash 范围）
 * @param   buf   数据缓冲区
 * @param   len   数据长度（字节）
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 地址/范围非法;
 *         RET_ERROR 写入/校验失败
 */
ret_t bsp_flash_write(uint32_t addr, uint8_t *buf, uint32_t len);

#endif /* __BSP_FLASH_H__ */
