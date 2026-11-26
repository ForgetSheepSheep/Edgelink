/**
 * @file    bsp_flash.c
 * @brief   内部 Flash BSP 驱动实现（GD32F30x FMC 操作）
 *
 * @note    通过 FMC（Flash Memory Controller）实现内部 Flash 的擦除与写入。
 *          擦除以 2KB 页为单位，写入以 4 字节 word 为单位。
 *          尾部不足 4 字节时自动补 0xFF 填充。
 */

#include "./bsp_flash/bsp_flash.h"
#include "gd32f30x.h"

/************************************************************
 * @brief   擦除内部 Flash 指定区域
 * @param   addr  起始地址（必须页对齐且位于内部 Flash 范围）
 * @param   size  擦除大小（字节，向上按页对齐）
 * @retval  RET_OK 成功; RET_INVALID_PARAM 地址/范围非法; RET_ERROR 擦除失败
 ************************************************************/
ret_t bsp_flash_erase(uint32_t addr, uint32_t size)
{
    uint32_t erase_addr;
    uint32_t end_addr;
    fmc_state_enum fmc_state;

    if (size == 0U)
    {
        return RET_OK;
    }

    /* 擦除必须从页边界开始，并且不能越过 GD32F303ZE 的 512 KiB Flash。 */
    if ((addr < BSP_FLASH_BASE_ADDR) ||
        (addr >= BSP_FLASH_END_ADDR) ||
        ((addr & (BSP_FLASH_PAGE_SIZE - 1U)) != 0U) ||
        (size > (BSP_FLASH_END_ADDR - addr)))
    {
        return RET_INVALID_PARAM;
    }

    /* 向上按页对齐：擦除最小单位是页，size 不是页整数倍时多擦尾页。
     * 公式 (x+N-1)&~(N-1) 是 2 的幂对齐的标准写法（N=2KB 页）。 */
    end_addr = addr + size;
    if ((end_addr & (BSP_FLASH_PAGE_SIZE - 1U)) != 0U)
    {
        end_addr += BSP_FLASH_PAGE_SIZE -
                    (end_addr & (BSP_FLASH_PAGE_SIZE - 1U));
    }
    if (end_addr > BSP_FLASH_END_ADDR)
    {
        return RET_INVALID_PARAM;
    }

    /* 解锁 FMC：上电默认锁定，防跑飞代码误擦；操作完必须重新锁定。
     * 先清三类历史标志（END/WPERR/PGERR），否则旧标志会污染本次判断。 */
    fmc_unlock();

    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* 逐页擦除：任一页失败立即锁定返回——已擦的页不会恢复，
     * 调用者需按“擦除非原子”来设计上层逻辑（如先擦后写、掉电续擦）。 */
    for (erase_addr = addr; erase_addr < end_addr; erase_addr += BSP_FLASH_PAGE_SIZE)
    {
        fmc_state = fmc_page_erase(erase_addr);
        if (fmc_state != FMC_READY)
        {
            fmc_lock();
            return RET_ERROR;
        }
    }

    fmc_lock();
    return RET_OK;
}

/************************************************************
 * @brief   向内部 Flash 写入数据
 * @param   addr  写入地址（必须 4 字节对齐且位于内部 Flash 范围）
 * @param   buf   数据缓冲区
 * @param   len   数据长度（字节）
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 地址/范围非法;
 *         RET_ERROR 写入/校验失败
 * @note    尾部不足 4 字节时自动补 0xFF 填充为一个完整 word
 ************************************************************/
ret_t bsp_flash_write(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint32_t write_data;
    uint32_t remain;
    fmc_state_enum fmc_state;

    if (buf == NULL)
    {
        return RET_NULL_PTR;
    }

    if (len == 0U)
    {
        return RET_OK;
    }

    /* 写入以 word 为单位，拒绝未对齐地址和越过 Flash 末端的请求。 */
    if ((addr < BSP_FLASH_BASE_ADDR) ||
        (addr >= BSP_FLASH_END_ADDR) ||
        ((addr & 3U) != 0U) ||
        (len > (BSP_FLASH_END_ADDR - addr)))
    {
        return RET_INVALID_PARAM;
    }

    fmc_unlock();

    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    i = 0U;
    while (i < len)
    {
        remain = len - i;
        /* 预置全 1：Flash 物理特性只能把 1 写成 0，擦除后全为 0xFF，
         * 所以缺的字节补 0xFF = “保持擦除态”，不影响已有内容。 */
        write_data = 0xFFFFFFFFUL;

        if (remain >= 4U)
        {
            /* 组装完整 4 字节 */
            write_data = ((uint32_t)buf[i]) |
                         ((uint32_t)buf[i + 1U] << 8) |
                         ((uint32_t)buf[i + 2U] << 16) |
                         ((uint32_t)buf[i + 3U] << 24);
        }
        else
        {
            /* 尾部不足 4 字节，补 0xFF */
            if (remain >= 1U)
            {
                write_data &= 0xFFFFFF00UL;
                write_data |= (uint32_t)buf[i];
            }
            if (remain >= 2U)
            {
                write_data &= 0xFFFF00FFUL;
                write_data |= (uint32_t)buf[i + 1U] << 8;
            }
            if (remain >= 3U)
            {
                write_data &= 0xFF00FFFFUL;
                write_data |= (uint32_t)buf[i + 2U] << 16;
            }
        }

        /* word 编程：库函数内部等待 BUSY 并检查标志 */
        fmc_state = fmc_word_program(addr, write_data);
        if (fmc_state != FMC_READY)
        {
            fmc_lock();
            return RET_ERROR;
        }

        /* 回读校验：编程“成功”只代表时序走完，掉电/干扰可能导致
         * 实际写入值不对；读回比对是最后一道防线，OTA 等场景必备。 */
        if (*(volatile uint32_t *)addr != write_data)
        {
            fmc_lock();
            return RET_ERROR;
        }

        addr += 4U;
        i += (remain >= 4U) ? 4U : remain;
    }

    fmc_lock();
    return RET_OK;
}
