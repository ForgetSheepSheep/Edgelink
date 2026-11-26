/**
 * @file    port_serial.c
 * @brief   Modbus RTU/RS485 两接口移植实现
 *
 * 发送：USART1 TX 使用 DMA0_CH6，DMA 完成后由 USART TC 中断收尾。
 * 接收：第一版按 RTU 字节状态机采用 RXNE 中断逐字节写入 ring buffer；
 *       每个字节都会重启 t3.5 一次性定时器，定时器到期才结束当前帧。
 * 硬件映射集中在本文件顶部，协议层不接触 USART、GPIO 和 DE。
 */
#include <stddef.h>
#include <stdint.h>
#include "gd32f30x.h"
#include "port_serial.h"
#include "port_timer.h"

#ifndef MODBUS_PORT_USART
#define MODBUS_PORT_USART                    USART1
#endif
#ifndef MODBUS_PORT_RCU_USART
#define MODBUS_PORT_RCU_USART                RCU_USART1
#endif
#ifndef MODBUS_PORT_GPIO
#define MODBUS_PORT_GPIO                     GPIOA
#endif
#ifndef MODBUS_PORT_RCU_GPIO
#define MODBUS_PORT_RCU_GPIO                 RCU_GPIOA
#endif
#ifndef MODBUS_PORT_TX_PIN
#define MODBUS_PORT_TX_PIN                   GPIO_PIN_2
#endif
#ifndef MODBUS_PORT_RX_PIN
#define MODBUS_PORT_RX_PIN                   GPIO_PIN_3
#endif
#ifndef MODBUS_PORT_DE_GPIO
#define MODBUS_PORT_DE_GPIO                  GPIOC
#endif
#ifndef MODBUS_PORT_RCU_DE_GPIO
#define MODBUS_PORT_RCU_DE_GPIO              RCU_GPIOC
#endif
#ifndef MODBUS_PORT_DE_PIN
#define MODBUS_PORT_DE_PIN                   GPIO_PIN_5
#endif

/* USART1 默认 TX DMA 映射：DMA0_CH6。RX 第一版使用 RXNE，不占 RX DMA。 */
#ifndef MODBUS_PORT_DMA
#define MODBUS_PORT_DMA                      DMA0
#endif
#ifndef MODBUS_PORT_RCU_DMA
#define MODBUS_PORT_RCU_DMA                  RCU_DMA0
#endif
#ifndef MODBUS_PORT_TX_DMA_CH
#define MODBUS_PORT_TX_DMA_CH                DMA_CH6
#endif
#ifndef MODBUS_PORT_TX_DMA_IRQ
#define MODBUS_PORT_TX_DMA_IRQ               DMA0_Channel6_IRQn
#endif
#ifndef MODBUS_PORT_USART_IRQ
#define MODBUS_PORT_USART_IRQ                USART1_IRQn
#endif

/* 仅用于本层默认配置，后续按实际从站参数修改。 */
#ifndef MODBUS_PORT_BAUDRATE
#define MODBUS_PORT_BAUDRATE                 (9600U)
#endif
#ifndef MODBUS_PORT_DATA_BITS
#define MODBUS_PORT_DATA_BITS                (8U)
#endif
#ifndef MODBUS_PORT_PARITY
#define MODBUS_PORT_PARITY                   (0U) /* 0=none, 1=odd, 2=even */
#endif

/* 0 表示按波特率自动计算；非 0 时直接使用指定的 50us 单位数。 */
#ifndef MODBUS_PORT_T35_50US
#define MODBUS_PORT_T35_50US                 (0U)
#endif

/* 软件 ring 的容量；实际可用容量为 SIZE-1。 */
#ifndef MODBUS_PORT_RX_RING_SIZE
#define MODBUS_PORT_RX_RING_SIZE             (512U)
#endif

/* 阻塞接口的超时保护；超时后返回已经完成的字节数或 0。 */
#ifndef MODBUS_PORT_WAIT_LIMIT
#define MODBUS_PORT_WAIT_LIMIT               (1000000UL)
#endif

/* 本层 ISR 不调用 FreeRTOS API。 */
#ifndef MODBUS_PORT_IRQ_PRE_PRIORITY
#define MODBUS_PORT_IRQ_PRE_PRIORITY         (6U)
#endif
#ifndef MODBUS_PORT_IRQ_SUB_PRIORITY
#define MODBUS_PORT_IRQ_SUB_PRIORITY         (0U)
#endif

typedef enum
{
    MODBUS_PORT_DIRECTION_RX = 0U,
    MODBUS_PORT_DIRECTION_TX = 1U
} modbus_port_direction_t;

static uint8_t s_port_ready;
static volatile uint8_t s_port_direction = MODBUS_PORT_DIRECTION_TX;
static volatile uint8_t s_tx_busy;
static volatile uint8_t s_tx_done;
static volatile uint8_t s_tx_error;

/* 软件 ring：RXNE ISR 只推进 write，接收接口只推进 read。 */
static volatile uint8_t s_rx_ring_buffer[MODBUS_PORT_RX_RING_SIZE];
static volatile uint16_t s_rx_ring_read;
static volatile uint16_t s_rx_ring_write;
static volatile uint8_t s_rx_ring_overflow;

/* t3.5 到期时冻结的 ring 写位置，用来确定当前帧的边界。 */
static volatile uint8_t s_rx_frame_ready;
static volatile uint16_t s_rx_frame_end;

static uint8_t modbus_port_serial_init(void);
static uint16_t modbus_port_get_t35_ticks(void);
static void modbus_port_enter_tx(void);
static void modbus_port_enter_rx(void);
static void modbus_port_ring_push_from_isr(uint8_t value);
static uint8_t modbus_port_ring_pop(uint8_t *value);
static uint8_t modbus_port_wait_tx_done(void);
static void modbus_port_config_tx_dma(const uint8_t *data, uint16_t length);
static void modbus_port_rxne_irq_handler(void);
static void modbus_port_timer_expired(void);

const modbus_port_ops_t g_modbus_port_serial_ops =
{
    modbus_port_init,
    modbus_port_send,
    modbus_port_receive
};

static uint16_t modbus_port_get_t35_ticks(void)
{
    uint32_t ticks;

    if (MODBUS_PORT_T35_50US != 0U)
    {
        if (MODBUS_PORT_T35_50US > 0xFFFFUL)
        {
            return 0U;
        }
        return (uint16_t)MODBUS_PORT_T35_50US;
    }

    if (MODBUS_PORT_BAUDRATE == 0U)
    {
        return 0U;
    }

    /* 8N1/8E1 按 11 bit 字符时间计算，结果向上取整到 50us。 */
    if (MODBUS_PORT_BAUDRATE > 19200U)
    {
        ticks = 35U; /* Modbus 规范的高速固定值约 1.75ms。 */
    }
    else
    {
        ticks = (770000UL + MODBUS_PORT_BAUDRATE - 1UL) /
                MODBUS_PORT_BAUDRATE;
    }

    if (ticks == 0U)
    {
        ticks = 1U;
    }
    if (ticks > 0xFFFFUL)
    {
        return 0U;
    }
    return (uint16_t)ticks;
}

static void modbus_port_ring_push_from_isr(uint8_t value)
{
    uint16_t write_pos;
    uint16_t next_pos;

    write_pos = s_rx_ring_write;
    next_pos = (uint16_t)(write_pos + 1U);
    if (next_pos >= MODBUS_PORT_RX_RING_SIZE)
    {
        next_pos = 0U;
    }

    if (next_pos == s_rx_ring_read)
    {
        /* 不覆盖旧数据；置位溢出标志，避免静默丢帧。 */
        s_rx_ring_overflow = 1U;
        return;
    }

    s_rx_ring_buffer[write_pos] = value;
    s_rx_ring_write = next_pos;
}

static uint8_t modbus_port_ring_pop(uint8_t *value)
{
    uint16_t read_pos;

    if (value == NULL)
    {
        return 0U;
    }

    read_pos = s_rx_ring_read;
    if (read_pos == s_rx_ring_write)
    {
        return 0U;
    }

    *value = s_rx_ring_buffer[read_pos];
    read_pos++;
    if (read_pos >= MODBUS_PORT_RX_RING_SIZE)
    {
        read_pos = 0U;
    }
    s_rx_ring_read = read_pos;
    return 1U;
}

static void modbus_port_timer_expired(void)
{
    /* TIMER ISR 中只做两个原子变量更新，不在 ISR 里拷贝整帧。 */
    s_rx_frame_end = s_rx_ring_write;
    s_rx_frame_ready = 1U;
}

static void modbus_port_enter_tx(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_port_direction == MODBUS_PORT_DIRECTION_RX)
    {
        usart_interrupt_disable(MODBUS_PORT_USART, USART_INT_RBNE);
        modbus_port_timer_stop();
    }

    gpio_bit_set(MODBUS_PORT_DE_GPIO, MODBUS_PORT_DE_PIN);
    s_port_direction = MODBUS_PORT_DIRECTION_TX;

    if ((primask & 0x01U) == 0U)
    {
        __enable_irq();
    }
}

static void modbus_port_enter_rx(void)
{
    usart_interrupt_disable(MODBUS_PORT_USART, USART_INT_TC);
    usart_dma_transmit_config(MODBUS_PORT_USART, USART_TRANSMIT_DMA_DISABLE);
    dma_channel_disable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
    dma_interrupt_flag_clear(MODBUS_PORT_DMA,
                             MODBUS_PORT_TX_DMA_CH,
                             DMA_INT_FLAG_FTF | DMA_INT_FLAG_ERR);

    gpio_bit_reset(MODBUS_PORT_DE_GPIO, MODBUS_PORT_DE_PIN);
    s_port_direction = MODBUS_PORT_DIRECTION_RX;
    usart_interrupt_enable(MODBUS_PORT_USART, USART_INT_RBNE);
}

static uint8_t modbus_port_serial_init(void)
{
    dma_parameter_struct dma_struct;
    uint16_t t35_ticks;

    if (s_port_ready != 0U)
    {
        return 1U;
    }

    t35_ticks = modbus_port_get_t35_ticks();
    if (t35_ticks == 0U)
    {
        return 0U;
    }

    rcu_periph_clock_enable(MODBUS_PORT_RCU_GPIO);
    rcu_periph_clock_enable(MODBUS_PORT_RCU_DE_GPIO);
    rcu_periph_clock_enable(MODBUS_PORT_RCU_USART);
    rcu_periph_clock_enable(MODBUS_PORT_RCU_DMA);

    gpio_init(MODBUS_PORT_GPIO,
              GPIO_MODE_AF_PP,
              GPIO_OSPEED_50MHZ,
              MODBUS_PORT_TX_PIN);
    gpio_init(MODBUS_PORT_GPIO,
              GPIO_MODE_IN_FLOATING,
              GPIO_OSPEED_50MHZ,
              MODBUS_PORT_RX_PIN);
    gpio_init(MODBUS_PORT_DE_GPIO,
              GPIO_MODE_OUT_PP,
              GPIO_OSPEED_50MHZ,
              MODBUS_PORT_DE_PIN);
    gpio_bit_reset(MODBUS_PORT_DE_GPIO, MODBUS_PORT_DE_PIN);

    usart_deinit(MODBUS_PORT_USART);
    usart_baudrate_set(MODBUS_PORT_USART, MODBUS_PORT_BAUDRATE);
    usart_stop_bit_set(MODBUS_PORT_USART, USART_STB_1BIT);
    usart_word_length_set(MODBUS_PORT_USART,
                          (MODBUS_PORT_PARITY == 0U) &&
                          (MODBUS_PORT_DATA_BITS == 8U)
                              ? USART_WL_8BIT
                              : USART_WL_9BIT);

    if (MODBUS_PORT_PARITY == 1U)
    {
        usart_parity_config(MODBUS_PORT_USART, USART_PM_ODD);
    }
    else if (MODBUS_PORT_PARITY == 2U)
    {
        usart_parity_config(MODBUS_PORT_USART, USART_PM_EVEN);
    }
    else
    {
        usart_parity_config(MODBUS_PORT_USART, USART_PM_NONE);
    }

    usart_transmit_config(MODBUS_PORT_USART, USART_TRANSMIT_ENABLE);
    usart_receive_config(MODBUS_PORT_USART, USART_RECEIVE_ENABLE);
    usart_dma_receive_config(MODBUS_PORT_USART, USART_RECEIVE_DMA_DISABLE);
    usart_dma_transmit_config(MODBUS_PORT_USART, USART_TRANSMIT_DMA_DISABLE);
    usart_enable(MODBUS_PORT_USART);

    /* TX DMA：调用 send 时重配 memory_addr/number，DMA 完成后交给 USART TC。 */
    dma_deinit(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
    dma_struct_para_init(&dma_struct);
    dma_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_struct.periph_addr = (uint32_t)(MODBUS_PORT_USART + 0x04U);
    dma_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_struct.memory_addr = 0U;
    dma_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_struct.number = 1U;
    dma_struct.priority = DMA_PRIORITY_HIGH;
    dma_init(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH, &dma_struct);
    dma_circulation_disable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
    dma_interrupt_enable(MODBUS_PORT_DMA,
                         MODBUS_PORT_TX_DMA_CH,
                         DMA_INT_FLAG_FTF | DMA_INT_FLAG_ERR);
    dma_interrupt_flag_clear(MODBUS_PORT_DMA,
                             MODBUS_PORT_TX_DMA_CH,
                             DMA_INT_FLAG_FTF | DMA_INT_FLAG_ERR);

    s_rx_ring_read = 0U;
    s_rx_ring_write = 0U;
    s_rx_ring_overflow = 0U;
    s_rx_frame_ready = 0U;
    s_rx_frame_end = 0U;
    s_tx_busy = 0U;
    s_tx_done = 0U;
    s_tx_error = 0U;
    s_port_direction = MODBUS_PORT_DIRECTION_TX;

    if (modbus_port_timer_init(t35_ticks) == 0U)
    {
        return 0U;
    }
    modbus_port_timer_set_callback(modbus_port_timer_expired);

    s_port_ready = 1U;
    nvic_irq_enable(MODBUS_PORT_TX_DMA_IRQ,
                    MODBUS_PORT_IRQ_PRE_PRIORITY,
                    MODBUS_PORT_IRQ_SUB_PRIORITY);
    nvic_irq_enable(MODBUS_PORT_USART_IRQ,
                    MODBUS_PORT_IRQ_PRE_PRIORITY,
                    MODBUS_PORT_IRQ_SUB_PRIORITY);

    modbus_port_enter_rx();
    return 1U;
}

uint8_t modbus_port_init(void)
{
    return modbus_port_serial_init();
}

static void modbus_port_config_tx_dma(const uint8_t *data, uint16_t length)
{
    dma_channel_disable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
    dma_interrupt_flag_clear(MODBUS_PORT_DMA,
                             MODBUS_PORT_TX_DMA_CH,
                             DMA_INT_FLAG_FTF | DMA_INT_FLAG_ERR);
    dma_memory_address_config(MODBUS_PORT_DMA,
                              MODBUS_PORT_TX_DMA_CH,
                              (uint32_t)data);
    dma_transfer_number_config(MODBUS_PORT_DMA,
                               MODBUS_PORT_TX_DMA_CH,
                               length);
    dma_transfer_direction_config(MODBUS_PORT_DMA,
                                  MODBUS_PORT_TX_DMA_CH,
                                  DMA_MEMORY_TO_PERIPHERAL);
    dma_memory_increase_enable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
    dma_periph_increase_disable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
}

static uint8_t modbus_port_wait_tx_done(void)
{
    uint32_t wait_count;

    wait_count = MODBUS_PORT_WAIT_LIMIT;
    while (s_tx_done == 0U)
    {
        if (wait_count == 0U)
        {
            return 0U;
        }
        wait_count--;
    }

    return (s_tx_error == 0U) ? 1U : 0U;
}

uint16_t modbus_port_send(const uint8_t *data, uint16_t length)
{
    uint8_t completed;

    if ((data == NULL) || (length == 0U))
    {
        return 0U;
    }

    if (modbus_port_init() == 0U)
    {
        return 0U;
    }
    if (s_tx_busy != 0U)
    {
        return 0U;
    }

    s_tx_busy = 1U;
    s_tx_done = 0U;
    s_tx_error = 0U;

    modbus_port_enter_tx();
    usart_interrupt_disable(MODBUS_PORT_USART, USART_INT_TC);
    usart_flag_clear(MODBUS_PORT_USART, USART_FLAG_TC);
    modbus_port_config_tx_dma(data, length);
    usart_dma_transmit_config(MODBUS_PORT_USART, USART_TRANSMIT_DMA_ENABLE);
    dma_channel_enable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);

    completed = modbus_port_wait_tx_done();
    if (completed == 0U)
    {
        dma_channel_disable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
        usart_dma_transmit_config(MODBUS_PORT_USART, USART_TRANSMIT_DMA_DISABLE);
        usart_interrupt_disable(MODBUS_PORT_USART, USART_INT_TC);
        modbus_port_enter_rx();
        s_tx_busy = 0U;
        return 0U;
    }

    s_tx_busy = 0U;
    return length;
}

uint16_t modbus_port_receive(uint8_t *data, uint16_t capacity)
{
    uint16_t index;
    uint16_t frame_end;
    uint32_t wait_count;
    uint8_t value;
    uint32_t primask;

    if ((data == NULL) || (capacity == 0U))
    {
        return 0U;
    }

    if (modbus_port_init() == 0U)
    {
        return 0U;
    }
    modbus_port_enter_rx();

    wait_count = MODBUS_PORT_WAIT_LIMIT;
    while (s_rx_frame_ready == 0U)
    {
        if (wait_count == 0U)
        {
            return 0U;
        }
        wait_count--;
    }

    /* 快照当前帧结束位置；后续新帧字节不会混入本次复制。 */
    frame_end = s_rx_frame_end;
    index = 0U;
    while ((s_rx_ring_read != frame_end) && (index < capacity))
    {
        if (modbus_port_ring_pop(&value) == 0U)
        {
            break;
        }
        data[index] = value;
        index++;
    }

    /* 缓冲区不够，保留剩余字节和 frame_ready，调用者可用更大缓冲重取。 */
    if (s_rx_ring_read != frame_end)
    {
        return 0U;
    }

    if (s_rx_ring_overflow != 0U)
    {
        s_rx_ring_overflow = 0U;
        s_rx_frame_ready = 0U;
        return 0U;
    }

    /* 防止清掉“复制期间已经产生的新帧”通知。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if ((s_rx_frame_ready != 0U) &&
        (s_rx_frame_end == frame_end) &&
        (s_rx_ring_read == frame_end))
    {
        s_rx_frame_ready = 0U;
    }
    if ((primask & 0x01U) == 0U)
    {
        __enable_irq();
    }

    return index;
}

static void modbus_port_rxne_irq_handler(void)
{
    uint8_t value;

    value = (uint8_t)usart_data_receive(MODBUS_PORT_USART);
    modbus_port_ring_push_from_isr(value);

    /* 新字节到来，重新开始等待 t3.5 静默间隔。 */
    modbus_port_timer_start();
}

void DMA0_Channel6_IRQHandler(void)
{
    if (dma_interrupt_flag_get(MODBUS_PORT_DMA,
                               MODBUS_PORT_TX_DMA_CH,
                               DMA_INT_FLAG_ERR) != RESET)
    {
        dma_interrupt_flag_clear(MODBUS_PORT_DMA,
                                 MODBUS_PORT_TX_DMA_CH,
                                 DMA_INT_FLAG_ERR);
        dma_channel_disable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
        usart_dma_transmit_config(MODBUS_PORT_USART, USART_TRANSMIT_DMA_DISABLE);
        usart_interrupt_disable(MODBUS_PORT_USART, USART_INT_TC);
        s_tx_error = 1U;
        s_tx_done = 1U;
        modbus_port_enter_rx();
        return;
    }

    if (dma_interrupt_flag_get(MODBUS_PORT_DMA,
                               MODBUS_PORT_TX_DMA_CH,
                               DMA_INT_FLAG_FTF) != RESET)
    {
        dma_interrupt_flag_clear(MODBUS_PORT_DMA,
                                 MODBUS_PORT_TX_DMA_CH,
                                 DMA_INT_FLAG_FTF);
        dma_channel_disable(MODBUS_PORT_DMA, MODBUS_PORT_TX_DMA_CH);
        usart_dma_transmit_config(MODBUS_PORT_USART, USART_TRANSMIT_DMA_DISABLE);
        /* DMA 已经写入最后一个字节，TC 才代表移位寄存器也发完。 */
        usart_interrupt_enable(MODBUS_PORT_USART, USART_INT_TC);
    }
}

void USART1_IRQHandler(void)
{
    if ((s_port_direction == MODBUS_PORT_DIRECTION_RX) &&
        (usart_interrupt_flag_get(MODBUS_PORT_USART, USART_INT_FLAG_RBNE) != RESET))
    {
        modbus_port_rxne_irq_handler();
    }

    if ((s_port_direction == MODBUS_PORT_DIRECTION_TX) &&
        (s_tx_busy != 0U) &&
        (usart_interrupt_flag_get(MODBUS_PORT_USART, USART_INT_FLAG_TC) != RESET))
    {
        usart_interrupt_disable(MODBUS_PORT_USART, USART_INT_TC);
        usart_flag_clear(MODBUS_PORT_USART, USART_FLAG_TC);
        modbus_port_enter_rx();
        s_tx_done = 1U;
    }
}
