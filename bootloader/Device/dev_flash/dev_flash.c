/**
 * @file    dev_flash.c
 * @brief   内部 Flash 设备驱动（封装 bsp_flash）
 *
 * @note    本模块对 GD32F30x 内部 Flash 进行设备层封装，
 *          提供初始化检查、擦除、写入、读取接口，供 App 层使用。
 *          底层依赖 bsp_flash（FMC 操作）。
 */

#include "dev_flash.h"
#include "../../BSP/bsp_flash/bsp_flash.h"

static volatile uint8_t s_dev_flash_inited = 0U;

/************************************************************
 * @brief   初始化内部 Flash
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始化
 ************************************************************/
ret_t dev_flash_init(void)
{
    if (s_dev_flash_inited != 0U)
    {
        return RET_ALREADY_INIT;
    }

    s_dev_flash_inited = 1U;
    DBG_log("dev_flash init ok");
    return RET_OK;
}

/************************************************************
 * @brief   擦除内部 Flash 指定区域
 * @param   addr  起始地址（建议页对齐）
 * @param   size  擦除大小（字节）
 * @retval  RET_OK 成功; RET_NOT_INIT 未初始化; 其他见 ret_error.h
 ************************************************************/
ret_t dev_flash_erase(uint32_t addr, uint32_t size)
{
    ret_t ret;

    if (s_dev_flash_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (size == 0U)
    {
        return RET_OK;
    }

    ret = bsp_flash_erase(addr, size);
    if (ret != RET_OK)
    {
        DBG_Error("dev_flash erase failed, addr=0x%08lX size=%lu ret=%ld",
                  (unsigned long)addr, (unsigned long)size, (long)ret);
        return ret;
    }

    return RET_OK;
}

/************************************************************
 * @brief   向内部 Flash 写入数据
 * @param   addr  写入地址（建议 4 字节对齐）
 * @param   buf   数据缓冲区
 * @param   len   数据长度（字节）
 * @retval  RET_OK 成功; RET_NOT_INIT 未初始化; 其他见 ret_error.h
 ************************************************************/
ret_t dev_flash_write(uint32_t addr, uint8_t *buf, uint32_t len)
{
    ret_t ret;

    if (s_dev_flash_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (buf == NULL)
    {
        return RET_NULL_PTR;
    }

    if (len == 0U)
    {
        return RET_OK;
    }

    ret = bsp_flash_write(addr, buf, len);
    if (ret != RET_OK)
    {
        DBG_Error("dev_flash write failed, addr=0x%08lX len=%lu ret=%ld",
                  (unsigned long)addr, (unsigned long)len, (long)ret);
        return ret;
    }

    return RET_OK;
}

/************************************************************
 * @brief   从内部 Flash 读取数据（直接内存读取）
 * @param   addr  读取地址
 * @param   buf   接收缓冲区
 * @param   len   读取长度（字节）
 * @retval  RET_OK 成功; RET_NOT_INIT 未初始化; 其他见 ret_error.h
 * @note    内部 Flash 读取无需特殊操作，直接 memcpy 即可
 ************************************************************/
ret_t dev_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (s_dev_flash_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (buf == NULL)
    {
        return RET_NULL_PTR;
    }

    if (len == 0U)
    {
        return RET_OK;
    }

    /* 内部 Flash 读取 = 直接内存访问 */
    (void)memcpy(buf, (const void *)addr, len);

    return RET_OK;
}




