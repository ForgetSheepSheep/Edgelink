/**
 * @file    bsp_delay.c
 * @brief   微秒/毫秒延时 BSP 驱动实现（DWT 计数器实现）
 *
 * @note    通过 DWT->CYCCNT 读取 CPU 周期数实现延时，
 *          首次调用 bsp_delay_us() 时会自动初始化。
 */

#include "bsp_delay/bsp_delay.h"
#include "gd32f30x.h"

/** DWT 延时初始化标志（0=未初始化，1=已初始化） */
static volatile uint8_t g_dwt_delay_init = 0U;

/************************************************************
 * @brief   初始化 DWT 周期计数器
 * @retval  RET_OK 成功
 * @note    使能 DWT->CYCCNT，供 bsp_delay_us() 计周期用
 ************************************************************/
ret_t bsp_delay_init(void)
{
    /* 三步缺一不可：
     * 1) TRCENA：打开 DWT 跟踪总开关，不开则 CYCCNT 永远为 0；
     * 2) CYCCNT 清零：从 0 开始计，方便换算；
     * 3) CYCCNTENA：启动周期计数器，之后每个 CPU 时钟自动 +1。
     * 注意：此计数器与系统节拍无关，在中断和主循环中都能使用。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    g_dwt_delay_init = 1U;

    return RET_OK;
}

/************************************************************
 * @brief   忙等延时（微秒）
 * @param   us  延时微秒数
 * @retval  RET_OK 成功
 * @note    使用无符号回绕减法计算经过周期，兼容 CYCCNT 溢出
 ************************************************************/
ret_t bsp_delay_us(uint32_t us)
{
    uint32_t start_tick;
    uint32_t delay_tick;

    if (g_dwt_delay_init == 0U)
    {
        (void)bsp_delay_init();
    }

    start_tick = DWT->CYCCNT;
    /* 需要等待的周期数 = 每微秒周期数 × 微秒数。
     * 隐含限制：us 太大时乘法溢出（如 120MHz 下 us>35 秒），
     * 大延时由 bsp_delay_ms 分片调用，避免单次乘法溢出。 */
    delay_tick = (SystemCoreClock / 1000000U) * us;

    /* 无符号减法比较：CYCCNT 是 32 位，120MHz 下约 35 秒回绕一次，
     * (当前-起点) 用无符号运算，回绕后差值依然正确。 */
    while ((uint32_t)(DWT->CYCCNT - start_tick) < delay_tick)
    {
    }

    return RET_OK;
}

/************************************************************
 * @brief   忙等延时（毫秒）
 * @param   ms  延时毫秒数
 * @retval  RET_OK 成功
 * @note    循环调用 bsp_delay_us(1000) 实现
 ************************************************************/
ret_t bsp_delay_ms(uint32_t ms)
{
    while (ms--)
    {
        (void)bsp_delay_us(1000U);
    }

    return RET_OK;
}

/************************************************************
 * @brief   获取当前 DWT 周期计数
 * @retval  当前周期计数值
 ************************************************************/
uint32_t bsp_delay_cycle_get(void)
{
    if (g_dwt_delay_init == 0U)
    {
        (void)bsp_delay_init();
    }

    return DWT->CYCCNT;
}

/************************************************************
 * @brief   判断 DWT 周期计数是否达到超时
 * @retval  1 已超时；0 未超时
 ************************************************************/
uint8_t bsp_delay_timeout_expired(uint32_t start_cycle,
                                  uint32_t timeout_us)
{
    uint32_t cycles_per_us;
    uint32_t timeout_cycles;

    if (g_dwt_delay_init == 0U)
    {
        (void)bsp_delay_init();
    }

    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U)
    {
        cycles_per_us = 1U;
    }

    /* 防乘法溢出：先算“不溢出的最大 us”，超了就钳到最大值，
     * 而不是让它回绕成一个很小的超时（那样会立刻误报超时）。 */
    if (timeout_us > (0xFFFFFFFFUL / cycles_per_us))
    {
        timeout_cycles = 0xFFFFFFFFUL;
    }
    else
    {
        timeout_cycles = cycles_per_us * timeout_us;
    }

    if (timeout_cycles == 0U)
    {
        timeout_cycles = 1U;
    }

    return ((uint32_t)(DWT->CYCCNT - start_cycle) >= timeout_cycles) ? 1U : 0U;
}
