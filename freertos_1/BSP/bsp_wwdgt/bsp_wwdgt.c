/**
 * @file    bsp_wwdgt.c
 * @brief   窗口看门狗（WWDGT）BSP 驱动实现
 *
 * @note    窗口看门狗喂狗时机必须位于窗口期（计数器小于 0x5F 时）,
 *          否则提前复位；超时溢出同样复位。
 */

#include "bsp_wwdgt.h"
#include "gd32f30x.h"
#include "gd32f30x_wwdgt.h"

/************************************************************
 * @brief   初始化窗口看门狗
 * @retval  RET_OK 成功
 * @note    计数器 0x7F、窗口 0x5F、8 分频
 ************************************************************/
ret_t bsp_wwdgt_init(void)
{
    /* 窗口看门狗与独立看门狗的区别：喂狗有“最早期限”。
     * 计数器从 0x7F 递减：0x7F->0x5F 是禁区（此时喂狗=提前喂=复位），
     * 0x5F 以下才是合法窗口；减到 0x3F 还没喂=超时复位。
     * 作用是“程序跑飞提前喂狗也救不了”——必须按时、不能提前。 */
    rcu_periph_clock_enable(RCU_WWDGT);

    wwdgt_config(0x7FU, 0x5FU, WWDGT_CFG_PSC_DIV8);

    wwdgt_enable();

    return RET_OK;
}

/************************************************************
 * @brief   喂狗（更新计数器到 0x7F）
 * @retval  RET_OK 成功
 * @note    必须在窗口期内调用，提前喂狗会立即复位
 ************************************************************/
ret_t bsp_wwdgt_feed(void)
{
    wwdgt_counter_update(0x7FU);

    return RET_OK;
}
