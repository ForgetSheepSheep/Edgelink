/**
 * @file    bsp_systick.c
 * @brief   One-millisecond SysTick timebase with bounded busy-wait delay.
 */
#include "bsp_systick.h"
#include "gd32f30x.h"

static volatile uint32_t s_systick_ms;
static volatile uint8_t s_systick_enabled;

ret_t bsp_systick_init(void)
{
    uint32_t reload = SystemCoreClock / BSP_SYSTICK_FREQUENCY_HZ;

    if ((reload == 0U) || (reload > SysTick_LOAD_RELOAD_Msk + 1UL))
    {
        return RET_INVALID_PARAM;
    }

    s_systick_ms = 0U;
    SysTick->CTRL = 0U;
    SysTick->LOAD = reload - 1UL;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
    s_systick_enabled = 1U;
    return RET_OK;
}

void bsp_systick_deinit(void)
{
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    s_systick_enabled = 0U;
}

uint32_t bsp_systick_get(void)
{
    return s_systick_ms;
}

ret_t bsp_systick_delay_ms(uint32_t milliseconds)
{
    uint32_t start = bsp_systick_get();

    if (s_systick_enabled == 0U)
    {
        return RET_NOT_INIT;
    }

    while ((uint32_t)(bsp_systick_get() - start) < milliseconds)
    {
        __NOP();
    }
    return RET_OK;
}

void SysTick_Handler(void)
{
    if (s_systick_enabled != 0U)
    {
        s_systick_ms++;
    }
}
