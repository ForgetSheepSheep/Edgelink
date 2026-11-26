/**
 * @file    dev_sram.c
 * @brief   外部 SRAM 设备驱动实现
 *
 * @note    初始化时通过 bsp_fsmc_init(BSP_SRAM) 配置 EXMC 控制器，
 *          之后直接以指针方式访问外部 SRAM 地址空间。
 */

#include "./dev_sram/dev_sram.h"
#include "../../BSP/bsp_fsmc/bsp_fsmc.h"

/** 自测数据长度（字节） */
#define DEV_SRAM_TEST_SIZE    256U

/** 初始化标志（0=未初始化，1=已初始化） */
static uint8_t s_dev_sram_inited = 0U;
/** 自测写数据缓冲区 */
static uint8_t s_dev_sram_test_write[DEV_SRAM_TEST_SIZE];
/** 自测读数据缓冲区 */
static uint8_t s_dev_sram_test_read[DEV_SRAM_TEST_SIZE];
/** 外部 SRAM 内存映射指针
 * @note SRAM 没有命令/时序协议：EXMC 配好后，对这个地址读写，
 *       硬件自动产生片选/读写/地址线时序，软件侧就是普通指针访问。
 *       volatile 同 LCD：防止编译器合并或删除访存。 */
static volatile uint8_t *const s_dev_sram_memory =
    (volatile uint8_t *)DEV_SRAM_BASE_ADDR;

/************************************************************
 * @brief   初始化外部 SRAM（依赖 bsp_fsmc_init(BSP_SRAM)）
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始化
 ************************************************************/
ret_t dev_sram_init(void)
{
    ret_t ret;

    if (s_dev_sram_inited != 0U)
    {
        return RET_ALREADY_INIT;
    }

    ret = bsp_fsmc_init(BSP_SRAM);
    if (ret != RET_OK)
    {
        return ret;
    }

    s_dev_sram_inited = 1U;

    DBG_log("dev_sram init ok, base=0x%08lX size=%lu\r\n",
            (unsigned long)DEV_SRAM_BASE_ADDR,
            (unsigned long)DEV_SRAM_SIZE);

    return RET_OK;
}

/************************************************************
 * @brief   从外部 SRAM 读取数据
 * @param   addr  读取偏移地址（相对 SRAM 基址）
 * @param   buf   数据缓冲区
 * @param   len   数据长度（字节）
 * @retval  RET_OK 成功; RET_NOT_INIT 未初始化; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t dev_sram_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (s_dev_sram_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    /* 越界检查写成 len > SIZE-addr 而不是 addr+len > SIZE：
     * addr+len 可能 32 位溢出回绕（如 addr=0xFFFFFFF0,len=0x20），
     * 溢出后比较结果为假、放行非法访问；减法形式永远不会溢出。 */
    if ((buf == NULL) || (addr >= DEV_SRAM_SIZE) ||
        (len > (DEV_SRAM_SIZE - addr)))
    {
        return RET_INVALID_PARAM;
    }

    /* len=0 视为空操作成功：调用者按“写 0 字节也算成功”处理，
     * 避免上层为长度 0 单独分支。 */
    if (len == 0U)
    {
        return RET_OK;
    }

    /* 逐字节搬运：SRAM 无需 WREN/等待，每次赋值就是一次 EXMC 总线周期 */
    for (uint32_t i = 0U; i < len; i++)
    {
        buf[i] = s_dev_sram_memory[addr + i];
    }

    return RET_OK;
}

/************************************************************
 * @brief   向外部 SRAM 写入数据
 * @param   addr  写入偏移地址（相对 SRAM 基址）
 * @param   buf   数据缓冲区
 * @param   len   数据长度（字节）
 * @retval  RET_OK 成功; RET_NOT_INIT 未初始化; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t dev_sram_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (s_dev_sram_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if ((buf == NULL) || (addr >= DEV_SRAM_SIZE) ||
        (len > (DEV_SRAM_SIZE - addr)))
    {
        return RET_INVALID_PARAM;
    }

    if (len == 0U)
    {
        return RET_OK;
    }

    for (uint32_t i = 0U; i < len; i++)
    {
        s_dev_sram_memory[addr + i] = buf[i];
    }

    return RET_OK;
}

/************************************************************
 * @brief   外部 SRAM 读写自测（写 256 字节回读比对）
 * @retval  RET_OK 成功; RET_NOT_INIT 未初始化; RET_DATA_ERROR 数据校验失败
 ************************************************************/
ret_t dev_sram_test(void)
{
    ret_t ret;
    uint32_t i;

    if (s_dev_sram_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    /* 测试图案 0x5A^i：0x5A=01011010b 是 01 交替，
     * 异或递增下标后每个字节都不同。交替码能检出地址线/数据线短路
     * （全 0/全 0xFF 测不出短路），每字节不同能检出地址错位。 */
    for (i = 0U; i < DEV_SRAM_TEST_SIZE; i++)
    {
        s_dev_sram_test_write[i] = (uint8_t)(0x5AU ^ (uint8_t)i);
    }

    ret = dev_sram_write(0U,
                         s_dev_sram_test_write,
                         DEV_SRAM_TEST_SIZE);
    if (ret != RET_OK)
    {
        DBG_log("dev_sram write test failed, ret=%ld\r\n", (long)ret);
        return ret;
    }

    ret = dev_sram_read(0U,
                        s_dev_sram_test_read,
                        DEV_SRAM_TEST_SIZE);
    if (ret != RET_OK)
    {
        DBG_log("dev_sram read test failed, ret=%ld\r\n", (long)ret);
        return ret;
    }

    for (i = 0U; i < DEV_SRAM_TEST_SIZE; i++)
    {
        if (s_dev_sram_test_read[i] != s_dev_sram_test_write[i])
        {
            DBG_log("dev_sram verify failed: offset=%lu expected=0x%02X actual=0x%02X\r\n",
                    (unsigned long)i,
                    (unsigned int)s_dev_sram_test_write[i],
                    (unsigned int)s_dev_sram_test_read[i]);
            return RET_DATA_ERROR;
        }
    }

    DBG_log("dev_sram test passed: %u bytes\r\n",
            (unsigned int)DEV_SRAM_TEST_SIZE);
    return RET_OK;
}
