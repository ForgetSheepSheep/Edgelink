/**
 * @file    port_timer.c
 * @brief   Modbus RTU 硬件定时器移植实现
 *
 * 参考原来的 TIMER3/50us 例程，但不依赖 FreeModbus 的 BOOL、ret_t 或
 * 协议栈回调符号；协议层通过函数指针注册自己的到期处理函数。
 */
#include <stddef.h>
#include "gd32f30x.h"
#include "port_timer.h"

#ifndef MODBUS_PORT_TIMER
#define MODBUS_PORT_TIMER                    TIMER3
#endif
#ifndef MODBUS_PORT_TIMER_RCU
#define MODBUS_PORT_TIMER_RCU                RCU_TIMER3
#endif
#ifndef MODBUS_PORT_TIMER_IRQ
#define MODBUS_PORT_TIMER_IRQ                TIMER3_IRQn
#endif

/* 当前工程 TIMER3 的计时输入为 120MHz；改时钟时可在编译选项覆盖。 */
#ifndef MODBUS_PORT_TIMER_CLOCK_HZ
#define MODBUS_PORT_TIMER_CLOCK_HZ            (120000000UL)
#endif
#ifndef MODBUS_PORT_TIMER_TICK_HZ
#define MODBUS_PORT_TIMER_TICK_HZ             (20000UL) /* 50us */
#endif
#ifndef MODBUS_PORT_TIMER_IRQ_PRE_PRIORITY
#define MODBUS_PORT_TIMER_IRQ_PRE_PRIORITY    (6U)
#endif
#ifndef MODBUS_PORT_TIMER_IRQ_SUB_PRIORITY
#define MODBUS_PORT_TIMER_IRQ_SUB_PRIORITY    (0U)
#endif

static uint8_t s_timer_initialized;
static volatile uint8_t s_timer_running;
static modbus_port_timer_callback_t s_timer_callback;

static void modbus_port_timer_expired_irq(void);

static void modbus_port_timer_expired_irq(void)
{
    timer_disable(MODBUS_PORT_TIMER);
    timer_interrupt_disable(MODBUS_PORT_TIMER, TIMER_INT_UP);
    timer_counter_value_config(MODBUS_PORT_TIMER, 0U);
    s_timer_running = 0U;

    if (s_timer_callback != NULL)
    {
        /* 这里仍是硬件 ISR 上下文，回调只能发通知/置标志。 */
        s_timer_callback();
    }
}

uint8_t modbus_port_timer_init(uint16_t timeout_50us)
{
    timer_parameter_struct timer_initpara;
    uint32_t timer_divider;
    uint32_t prescaler;

    if (timeout_50us == 0U)
    {
        return 0U;
    }

    timer_divider = MODBUS_PORT_TIMER_CLOCK_HZ / MODBUS_PORT_TIMER_TICK_HZ;
    if (timer_divider == 0U)
    {
        return 0U;
    }

    prescaler = timer_divider - 1U;
    if (prescaler > 0xFFFFUL)
    {
        return 0U;
    }

    rcu_periph_clock_enable(MODBUS_PORT_TIMER_RCU);
    timer_disable(MODBUS_PORT_TIMER);
    timer_deinit(MODBUS_PORT_TIMER);
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler = (uint16_t)prescaler;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.period = (uint32_t)timeout_50us - 1U;
    timer_initpara.repetitioncounter = 0U;
    timer_init(MODBUS_PORT_TIMER, &timer_initpara);

    timer_counter_value_config(MODBUS_PORT_TIMER, 0U);
    timer_interrupt_flag_clear(MODBUS_PORT_TIMER, TIMER_INT_UP);
    timer_interrupt_disable(MODBUS_PORT_TIMER, TIMER_INT_UP);
    nvic_irq_enable(MODBUS_PORT_TIMER_IRQ,
                    MODBUS_PORT_TIMER_IRQ_PRE_PRIORITY,
                    MODBUS_PORT_TIMER_IRQ_SUB_PRIORITY);

    s_timer_initialized = 1U;
    s_timer_running = 0U;
    return 1U;
}

void modbus_port_timer_set_callback(modbus_port_timer_callback_t callback)
{
    s_timer_callback = callback;
}

void modbus_port_timer_start(void)
{
    if (s_timer_initialized == 0U)
    {
        return;
    }

    timer_disable(MODBUS_PORT_TIMER);
    timer_counter_value_config(MODBUS_PORT_TIMER, 0U);
    timer_interrupt_flag_clear(MODBUS_PORT_TIMER, TIMER_INT_UP);
    timer_interrupt_enable(MODBUS_PORT_TIMER, TIMER_INT_UP);
    s_timer_running = 1U;
    timer_enable(MODBUS_PORT_TIMER);
}

void modbus_port_timer_stop(void)
{
    if (s_timer_initialized == 0U)
    {
        return;
    }

    timer_disable(MODBUS_PORT_TIMER);
    timer_counter_value_config(MODBUS_PORT_TIMER, 0U);
    timer_interrupt_flag_clear(MODBUS_PORT_TIMER, TIMER_INT_UP);
    timer_interrupt_disable(MODBUS_PORT_TIMER, TIMER_INT_UP);
    s_timer_running = 0U;
}

void TIMER3_IRQHandler(void)
{
    if (timer_interrupt_flag_get(MODBUS_PORT_TIMER, TIMER_INT_FLAG_UP) != RESET)
    {
        timer_interrupt_flag_clear(MODBUS_PORT_TIMER, TIMER_INT_FLAG_UP);
        modbus_port_timer_expired_irq();
    }
}
