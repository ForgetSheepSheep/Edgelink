/**
 * @file    bsp_fwdgt.c
 * @brief   独立看门狗（FWDGT）BSP 驱动实现
 *
 * @note    独立看门狗一旦使能只能由系统复位或 FWDGT 复位关闭。
 */

#include "bsp_fwdgt.h"
#include "gd32f30x_fwdgt.h"
#include "gd32f30x_dbg.h"

/************************************************************
 * @brief   初始化独立看门狗
 * @retval  RET_OK 成功
 * @note    重载值 3124 + 128 分频，约 10s 超时
 ************************************************************/
ret_t bsp_fwdgt_init(void)
{
    /* 调试器 halt 内核时冻结看门狗计数：否则 J-Link 读取内存/寄存器
     * 造成的停机超过超时时间会触发真实 IWDG 复位，干扰故障定位。 */
    dbg_periph_enable(DBG_FWDGT_HOLD);

    /* 四步固定顺序，错一步看门狗行为即错：
     * 1) 写使能：FWDGT 寄存器上电写保护，先开锁；
     * 2) 配重载值+分频：超时 = 重载值 × 分频 / 看门狗时钟
     *    （时钟源为片内独立 IRC40K，不依赖主时钟，主时钟挂掉它照样复位）；
     * 3) 先喂一次：计数器初值装入，避免使能瞬间已是过期值；
     * 4) 使能：一旦使能，软件再也关不掉，只能等复位。 */
    fwdgt_write_enable();

    fwdgt_config(3124U, FWDGT_PSC_DIV128);

    fwdgt_counter_reload();

    fwdgt_enable();

    return RET_OK;
}

/************************************************************
 * @brief   喂狗（重载计数器）
 * @retval  RET_OK 成功
 * @note    需在超时周期内周期调用，否则触发系统复位
 ************************************************************/
ret_t bsp_fwdgt_feed(void)
{
    fwdgt_counter_reload();

    return RET_OK;
}
