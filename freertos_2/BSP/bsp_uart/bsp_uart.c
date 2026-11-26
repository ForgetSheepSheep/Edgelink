/**
 * @file    bsp_uart.c
 * @brief   UART BSP 驱动实现（结构体表管理 + 环形缓冲 + 中断/DMA 接收）
 *
 * @note    每路串口的硬件参数集中在 uart_info_t 配置表中：
 *          - use_dma=1：DMA 循环接收 + IDLE 空闲中断，一帧结束唤醒通知任务
 *          - use_dma=0：RBNE 逐字节中断，数据写入环形缓冲
 *          发送使用互斥锁保护，支持多任务并发发送。
 */

#include "./bsp_uart/bsp_uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "ring_buffer/ring_buffer.h"
#include "gd32f30x.h"                   // Device header

#define BSP_UART_NUM    BSP_UART_MAX                     /**< 串口数量 */
#define BSP_UART_BAUDRATE    115200U                     /**< 波特率 */
#define BSP_UART_RING_BUFFER_SIZE    1024U               /**< 接收环形缓冲大小 */
#define BSP_UART_NET_RING_BUFFER_SIZE 2048U              /**< 网络/OTA 接收环形缓冲大小 */

#ifndef BSP_UART_IRQ_PRE_PRIORITY
#define BSP_UART_IRQ_PRE_PRIORITY    5U                  /**< 串口中断抢占优先级 */
#endif

#ifndef BSP_UART_IRQ_SUB_PRIORITY
#define BSP_UART_IRQ_SUB_PRIORITY    0U                  /**< 串口中断子优先级 */
#endif

/** 各串口接收环形缓冲控制块 */
static ring_buffer_t g_uart_cb_table[BSP_UART_NUM];
/** 各串口按需分配的接收数据区指针 */
static uint8_t *s_uart_rx_buffer[BSP_UART_NUM] = {NULL};

/**
 * @brief   UART 硬件信息结构体
 */
typedef struct
{
    rcu_periph_enum rcu_uart;       /**< UART 外设时钟 */
    uint32_t uart_prot;             /**< UART 外设基址 */
    rcu_periph_enum rcu_tx_gpio;    /**< TX GPIO 时钟 */
    uint32_t tx_gpio_prot;          /**< TX GPIO 端口 */
    uint32_t tx_pin;                /**< TX 引脚 */
    rcu_periph_enum rcu_rx_gpio;    /**< RX GPIO 时钟 */
    uint32_t rx_gpio_prot;          /**< RX GPIO 端口 */
    uint32_t rx_pin;                /**< RX 引脚 */
    IRQn_Type irq;                  /**< 串口中断号 */
    uint32_t gpio_remap;            /**< GPIO 重映射配置（0=不重映射） */
    uint32_t dma_no;                /**< DMA 外设编号 */
    rcu_periph_enum rcu_dma;        /**< DMA 时钟 */
    dma_channel_enum dma_ch;        /**< DMA 通道 */
    uint8_t use_dma;                /**< 1=DMA+IDLE 接收，0=RBNE 逐字节接收 */
    ring_buffer_t *rx_ring;         /**< 接收环形缓冲控制块指针 */
    uint16_t rx_buffer_size;        /**< 接收缓冲大小 */
} uart_info_t;

/** UART 硬件信息表（按 bsp_uart_id_t 索引） */
static const uart_info_t g_uart_info_table[BSP_UART_MAX] =
{
    /* 主机串口：USART0（PA9=TX, PA10=RX），DMA0_CH4 接收 */
    [BSP_UART_HOST] = {
        .rcu_uart    = RCU_USART0,
        .uart_prot   = USART0,
        .rcu_tx_gpio = RCU_GPIOA,    .tx_gpio_prot = GPIOA,  .tx_pin = GPIO_PIN_9,
        .rcu_rx_gpio = RCU_GPIOA,    .rx_gpio_prot = GPIOA,  .rx_pin = GPIO_PIN_10,
        .irq         = USART0_IRQn,
        .gpio_remap  = 0U,
        .dma_no      = DMA0,         .rcu_dma = RCU_DMA0,    .dma_ch = DMA_CH4,
        .use_dma     = 1U,
        .rx_ring     = &g_uart_cb_table[BSP_UART_HOST],
        .rx_buffer_size = BSP_UART_RING_BUFFER_SIZE,
    },
    /* 网络串口：USART2（PB10=TX, PB11=RX），DMA0_CH2 接收 */
    [BSP_UART_NET] = {
        .rcu_uart    = RCU_USART2,
        .uart_prot   = USART2,
        .rcu_tx_gpio = RCU_GPIOB,    .tx_gpio_prot = GPIOB,  .tx_pin = GPIO_PIN_10,
        .rcu_rx_gpio = RCU_GPIOB,    .rx_gpio_prot = GPIOB,  .rx_pin = GPIO_PIN_11,
        .irq         = USART2_IRQn,
        .gpio_remap  = 0U,
        .dma_no      = DMA0,         .rcu_dma = RCU_DMA0,    .dma_ch = DMA_CH2,
        .use_dma     = 1U,
        .rx_ring     = &g_uart_cb_table[BSP_UART_NET],
        .rx_buffer_size = BSP_UART_NET_RING_BUFFER_SIZE,
    },
    /* 显示串口：UART4（PC12=TX, PD2=RX），逐字节 RBNE 中断接收 */
    [BSP_UART_DISPLAY] = {
        .rcu_uart    = RCU_UART4,
        .uart_prot   = UART4,
        .rcu_tx_gpio = RCU_GPIOC,    .tx_gpio_prot = GPIOC,  .tx_pin = GPIO_PIN_12,
        .rcu_rx_gpio = RCU_GPIOD,    .rx_gpio_prot = GPIOD,  .rx_pin = GPIO_PIN_2,
        .irq         = UART4_IRQn,
        .gpio_remap  = 0U,
        .dma_no      = 0U,           .rcu_dma = RCU_DMA0,    .dma_ch = DMA_CH0,
        .use_dma     = 0U,
        .rx_ring     = &g_uart_cb_table[BSP_UART_DISPLAY],
        .rx_buffer_size = BSP_UART_RING_BUFFER_SIZE,
    },
    /* LoRa 串口：UART3（PC10=TX, PC11=RX），DMA1_CH0 接收 */
    [BSP_UART_LORA] = {
        .rcu_uart    = RCU_UART3,
        .uart_prot   = UART3,
        .rcu_tx_gpio = RCU_GPIOC,    .tx_gpio_prot = GPIOC,  .tx_pin = GPIO_PIN_10,
        .rcu_rx_gpio = RCU_GPIOC,    .rx_gpio_prot = GPIOC,  .rx_pin = GPIO_PIN_11,
        .irq         = UART3_IRQn,
        .gpio_remap  = 0U,
        .dma_no      = DMA1,         .rcu_dma = RCU_DMA1,    .dma_ch = DMA_CH0,
        .use_dma     = 1U,
        .rx_ring     = &g_uart_cb_table[BSP_UART_LORA],
        .rx_buffer_size = BSP_UART_RING_BUFFER_SIZE,
    },
};

/** 各串口发送互斥锁（防止多任务并发发送冲突） */
static SemaphoreHandle_t s_uart_tx_mutex[BSP_UART_NUM] = {NULL};
/** 各串口接收通知任务句柄（由 ISR 唤醒） */
static TaskHandle_t s_uart_rx_task[BSP_UART_NUM] = {NULL};
static ret_t uart_lock(bsp_uart_id_t uart_id, uint16_t timeout);
static ret_t uart_unlock(bsp_uart_id_t uart_id);
static ret_t bsp_uart_byte(bsp_uart_id_t uart_id, uint8_t byte);
static ret_t bsp_uart_flush(bsp_uart_id_t uart_id);
static void uart_idle_irq_handler(bsp_uart_id_t uart_id);
static void uart_rbne_irq_handler(bsp_uart_id_t uart_id);

/************************************************************
 * @brief   初始化指定 UART（环形缓冲 + 中断/DMA 接收）
 * @param   uart_id  UART 设备 ID
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法; RET_NO_MEMORY 内存不足
 * @note    use_dma=1 时配置 DMA 循环接收 + IDLE 中断；
 *          use_dma=0 时使能 RBNE 逐字节中断
 ************************************************************/
ret_t bsp_uart_init(bsp_uart_id_t uart_id)
{
    const uart_info_t *uart_info;
    ret_t ret;

    /* 【步骤0】参数合法性检查：uart_id 是数组下标，越界直接拒绝。
     * 注意 bsp_uart_id_t 的最后一个枚举值 BSP_UART_MAX 表示数量本身，
     * 因此用 >= 而不是 > 做边界判断。 */
    if (uart_id >= BSP_UART_NUM)
    {
        return RET_INVALID_PARAM;
    }

    /* 【步骤1】拿到本路串口的硬件配置行。
     * 用 const 指针只读访问配置表，防止初始化过程误改全局配置。 */
    uart_info = &g_uart_info_table[uart_id];

    /* 接收区按实际初始化的串口分配，未使用的配置项不占静态 RAM。 */
    if (s_uart_rx_buffer[uart_id] == NULL)
    {
        s_uart_rx_buffer[uart_id] = (uint8_t *)pvPortMalloc(uart_info->rx_buffer_size);
        if (s_uart_rx_buffer[uart_id] == NULL)
        {
            return RET_NO_MEMORY;
        }
    }

    /* 【步骤2】初始化接收环形缓冲。
     * 条件三者缺一不可：控制块存在、数据区存在、大小至少 2 字节
     * （环形缓冲用“读写指针相差1判满”，大小<2 无法存任何数据）。
     * 失败直接返回：连缓冲都没有，后续开中断没有意义。 */
    if ((uart_info->rx_ring != NULL) && (uart_info->rx_buffer_size >= 2U))
    {
        ret = ring_buffer_init(uart_info->rx_ring, s_uart_rx_buffer[uart_id], uart_info->rx_buffer_size);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    /* 【步骤3】打开三路时钟：UART 外设本身 + TX 引脚所在 GPIO + RX 引脚所在 GPIO。
     * GD32 的外设上电默认时钟关闭，不开时钟时任何寄存器读写都无效，
     * 这是新手最常见的“配置不生效”原因。 */
    rcu_periph_clock_enable(uart_info->rcu_uart);
    rcu_periph_clock_enable(uart_info->rcu_tx_gpio);
    rcu_periph_clock_enable(uart_info->rcu_rx_gpio);

    /* 【步骤4】可选的重映射：只有配置表里 gpio_remap 非 0 的串口才走这里。
     * 重映射要先开 AF（复用功能）时钟，否则映射寄存器写不进去。
     * 本工程四路串口都没用重映射，此分支目前走不到，保留为扩展点。 */
    if (uart_info->gpio_remap != 0U)
    {
        rcu_periph_clock_enable(RCU_AF);
        gpio_pin_remap_config(uart_info->gpio_remap, ENABLE);
    }

    /* 【步骤5】配置引脚模式：
     * TX 必须是复用推挽输出（AF_PP），由 USART 外设驱动电平；
     * RX 必须是浮空输入（IN_FLOATING），把电平采样权交给 USART。
     * 若 RX 配成推挽输出，会和对方 TX 对打，烧引脚/收不到数据。 */
    gpio_init(uart_info->tx_gpio_prot, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, uart_info->tx_pin);
    gpio_init(uart_info->rx_gpio_prot, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, uart_info->rx_pin);

    /* 【步骤6】USART 通信格式配置：先复位外设，再逐项设置。
     * usart_deinit 把之前残留的配置清零，避免重复 init 时旧值干扰。
     * 格式固定为 115200-8-N-1（8 数据位 / 1 停止位 / 无校验），
     * 最后同时打开发送器和接收器，再使能 USART。 */
    usart_deinit(uart_info->uart_prot);
    usart_baudrate_set(uart_info->uart_prot, BSP_UART_BAUDRATE);
    usart_word_length_set(uart_info->uart_prot, USART_WL_8BIT);
    usart_stop_bit_set(uart_info->uart_prot, USART_STB_1BIT);
    usart_parity_config(uart_info->uart_prot, USART_PM_NONE);
    usart_transmit_config(uart_info->uart_prot, USART_TRANSMIT_ENABLE);
    usart_receive_config(uart_info->uart_prot, USART_RECEIVE_ENABLE);
    usart_enable(uart_info->uart_prot);

    /* 【步骤7】接收路径二选一：DMA+IDLE 模式 或 RBNE 逐字节中断模式。
     * 前者 CPU 零开销，适合 HOST/NET/LORA 三路高速数据；
     * 后者每个字节都进一次中断，适合 DISPLAY 路低速屏指令。 */
    if (uart_info->use_dma != 0U)
    {
        dma_parameter_struct dma_struct;

        /* 7a. 开 IDLE（总线空闲）中断 + NVIC 使能。
         * IDLE 的含义：RX 线上连续 1 个字节时间没有跳变，
         * 说明对方“一帧发完了”——这是不定长帧的天然分帧信号。
         * 抢占优先级 5：在 FreeRTOSConfig 里 MAX_SYSCALL=5，
         * 数值 5 表示“可以使用 FromISR API 的最低优先级”，
         * 恰好满足 ISR 里调用 vTaskNotifyGiveFromISR 的要求。 */
        usart_interrupt_enable(uart_info->uart_prot, USART_INT_IDLE);
        nvic_irq_enable(uart_info->irq, BSP_UART_IRQ_PRE_PRIORITY, BSP_UART_IRQ_SUB_PRIORITY);

        /* 7b. DMA 通道复位 + 参数结构体填默认值，
         * 避免结构体局部变量里的随机值污染配置。 */
        rcu_periph_clock_enable(uart_info->rcu_dma);
        dma_deinit(uart_info->dma_no, uart_info->dma_ch);
        dma_struct_para_init(&dma_struct);

        /* 7c. DMA 搬运方向：外设 -> 内存（接收方向）。
         * periph_addr 固定为 USART 基址 + 0x04：0x04 是数据寄存器
         * USART_DATA 的偏移，DMA 每次从这个地址“取”一个收到的字节。
         * 外设地址不自增（periph_inc=DISABLE），因为数据源永远是
         * 同一个数据寄存器；内存地址自增，把字节依次铺进缓冲。 */
        dma_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
        dma_struct.periph_addr = uart_info->uart_prot + 0x04U;
        dma_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        dma_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
        dma_struct.memory_addr = (uint32_t)s_uart_rx_buffer[uart_id];
        dma_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        dma_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
        /* 一次搬运总数 = 整个缓冲大小；配合下面的循环模式，
         * DMA 写满缓冲后会自动卷绕回开头，不需要 CPU 干预。 */
        dma_struct.number = uart_info->rx_buffer_size;
        dma_struct.priority = DMA_PRIORITY_HIGH;

        dma_init(uart_info->dma_no, uart_info->dma_ch, &dma_struct);
        /* 循环模式（circulation）：DMA 计数器归零后自动重装，
         * 形成“永远写”的环形 DMA，这是 DMA+IDLE 方案的核心。 */
        dma_circulation_enable(uart_info->dma_no, uart_info->dma_ch);

        /* 7d. 打通 USART -> DMA 的请求线，再启动 DMA 通道。
         * 顺序不能反：先让 USART 知道“收到字节就通知 DMA”，
         * 再打开 DMA 开关，从此接收全程不需要 CPU。 */
        usart_dma_receive_config(uart_info->uart_prot, USART_RECEIVE_DMA_ENABLE);
        dma_channel_enable(uart_info->dma_no, uart_info->dma_ch);
    }
    else
    {
        /* 逐字节模式：每收到 1 个字节进 1 次中断（RBNE=读数据寄存器非空），
         * 中断里由软件把字节搬进环形缓冲。开销大但逻辑简单。 */
        usart_interrupt_enable(uart_info->uart_prot, USART_INT_RBNE);
        nvic_irq_enable(uart_info->irq, BSP_UART_IRQ_PRE_PRIORITY, BSP_UART_IRQ_SUB_PRIORITY);
    }

    /* 【步骤8】创建本路串口的发送互斥锁。
     * 多个任务可能同时调用 bsp_uart_transmit，没有互斥锁的话
     * 两个任务的字节流会交错（A 的半包 + B 的半包），对端无法解析。
     * 只创建一次：重复 init 不会重复建，避免泄漏旧句柄。 */
    if (s_uart_tx_mutex[uart_id] == NULL)
    {
        s_uart_tx_mutex[uart_id] = xSemaphoreCreateMutex();
        if (s_uart_tx_mutex[uart_id] == NULL)
        {
            return RET_NO_MEMORY;
        }
    }

    return RET_OK;
}
/************************************************************
 * @brief   注册接收通知任务（收到数据时由 ISR 唤醒）
 * @param   uart_id  UART 设备 ID
 * @param   task     任务句柄
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_INIT 未初始化
 ************************************************************/
ret_t bsp_uart_register_notify_task(
    bsp_uart_id_t uart_id,
    TaskHandle_t task)
{
    /* 1. 检查uart_id是否越界 */
    if(uart_id >= BSP_UART_MAX)
    {
        return RET_INVALID_PARAM;
    }
    /* 2. 检查任务句柄是否为NULL */
    if(task == NULL)
    {
        return RET_NULL_PTR;
    }
    /* 3. 检查对应UART是否已经初始化 */
    if(s_uart_tx_mutex[uart_id] == NULL)
    {
        return RET_NOT_INIT;
    }
    /* 4. 把任务句柄保存到对应数组位置 */
    s_uart_rx_task[uart_id] = task;
    return RET_OK;
}
/************************************************************
 * @brief   发送数据（互斥保护）
 * @param   uart_id   UART 设备 ID
 * @param   string    待发送数据
 * @param   length    数据长度
 * @param   timeout_ms 等待互斥锁超时（ms）
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_TIMEOUT 超时; RET_NOT_INIT 未初始化
 ************************************************************/
ret_t bsp_uart_transmit(bsp_uart_id_t uart_id, uint8_t *string, uint16_t length, uint16_t timeout_ms)
{
    ret_t ret;
    uint16_t i;

    /* 参数检查：串口号越界 或 长度为 0 都拒绝。
     * length<1 就发送等于什么都没干，直接报错比静默返回更利于定位。 */
    if ((uart_id >= BSP_UART_NUM) || (length < 1U))
    {
        return RET_INVALID_PARAM;
    }

    if (string == NULL)
    {
        return RET_NULL_PTR;
    }

    /* 加锁：拿到互斥锁后，本次发送独占串口，其他任务阻塞等待。
     * timeout_ms 是“等锁”的最大毫秒数，等不到就返回超时，
     * 避免一个任务死等把系统拖住。注意：锁没拿到就直接返回，
     * 这里不需要解锁（因为根本没加上）。 */
    ret = uart_lock(uart_id, timeout_ms);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 逐字节发送：每个字节写进 USART 数据寄存器。
     * 中途任一字节失败（理论上轮询模式不会失败，防御性写法），
     * 必须先解锁再返回——否则互斥锁永远被占，其他任务全部饿死。 */
    for (i = 0U; i < length; i++)
    {
        ret = bsp_uart_byte(uart_id, string[i]);
        if (ret != RET_OK)
        {
            (void)uart_unlock(uart_id);
            return ret;
        }
    }

    /* 等待发送完成：最后一个字节只是“写进了移位寄存器”，
     * 还在一位一位往外发；flush 等 TC 标志确保对端收全了整包，
     * 调用者返回后立即复用发送缓冲才安全。 */
    ret = bsp_uart_flush(uart_id);
    if (ret != RET_OK)
    {
        (void)uart_unlock(uart_id);
        return ret;
    }

    /* 解锁并把解锁结果作为函数返回值透传。 */
    return uart_unlock(uart_id);
}

/************************************************************
 * @brief   从接收环形缓冲区取一个字节（非阻塞）
 * @param   uart_id  UART 设备 ID
 * @param   byte     读出的字节
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_EMPTY 缓冲区为空
 ************************************************************/
ret_t bsp_uart_receive(bsp_uart_id_t uart_id, uint8_t *byte)
{
    const uart_info_t *uart_info;

    if (uart_id >= BSP_UART_NUM)
    {
        return RET_INVALID_PARAM;
    }

    if (byte == NULL)
    {
        return RET_NULL_PTR;
    }

    uart_info = &g_uart_info_table[uart_id];

    /* 从环形缓冲取 1 字节：纯内存操作，非阻塞。
     * 缓冲为空时返回 RET_EMPTY，调用者（uart_rx 任务）据此决定
     * 是继续取还是阻塞等下一次 ISR 通知。 */
    return ring_buffer_read(uart_info->rx_ring, byte);
}

/**
 * @brief   获取串口发送互斥锁（内部）
 * @retval  RET_OK 成功; RET_TIMEOUT 超时; RET_NOT_INIT / RET_INVALID_PARAM
 */
static ret_t uart_lock(bsp_uart_id_t uart_id, uint16_t timeout)
{
    if (uart_id >= BSP_UART_NUM)
    {
        return RET_INVALID_PARAM;
    }

    if (s_uart_tx_mutex[uart_id] == NULL)
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_uart_tx_mutex[uart_id], pdMS_TO_TICKS(timeout)) == pdTRUE)
    {
        return RET_OK;
    }

    return RET_TIMEOUT;
}

/**
 * @brief   释放串口发送互斥锁（内部）
 * @retval  RET_OK 成功; RET_OS_ERROR 释放失败; RET_NOT_INIT / RET_INVALID_PARAM
 */
static ret_t uart_unlock(bsp_uart_id_t uart_id)
{
    if (uart_id >= BSP_UART_NUM)
    {
        return RET_INVALID_PARAM;
    }

    if (s_uart_tx_mutex[uart_id] == NULL)
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreGive(s_uart_tx_mutex[uart_id]) == pdTRUE)
    {
        return RET_OK;
    }

    return RET_OS_ERROR;
}

/************************************************************
 * @brief   发送单个字节：等发送缓冲空后写入数据寄存器（内部）
 * @param   uart_id  UART 设备 ID
 * @param   byte     待发送字节
 * @note    TBE（发送缓冲空）= 数据寄存器已把上一个字节搬进移位寄存器，
 *          可以接受下一个字节。轮询等待是阻塞式的，但发送路径已由
 *          互斥锁串行化，不会和别的任务抢，时序确定。
 ************************************************************/
static ret_t bsp_uart_byte(bsp_uart_id_t uart_id, uint8_t byte)
{
    if (uart_id >= BSP_UART_NUM)
    {
        return RET_INVALID_PARAM;
    }

    /* 等 TBE：上一个字节还没被硬件取走就不能写，否则会覆盖丢失 */
    while (usart_flag_get(g_uart_info_table[uart_id].uart_prot, USART_FLAG_TBE) == RESET)
    {
    }

    /* 写数据寄存器：硬件自动启动发送（起始位+8数据位+停止位） */
    usart_data_transmit(g_uart_info_table[uart_id].uart_prot, byte);

    return RET_OK;
}

/************************************************************
 * @brief   等待整包发送完成：TC 标志置位（内部）
 * @param   uart_id  UART 设备 ID
 * @note    TC（发送完成）= 移位寄存器也空了，最后一位停止位已发出。
 *          它和 TBE 的区别：TBE 只保证“可以写下一个”，TC 保证
 *          “最后一个字节真正离开了芯片”。485 方向切换、复用发送缓冲
 *          都依赖 TC，不能用 TBE 代替。
 ************************************************************/
static ret_t bsp_uart_flush(bsp_uart_id_t uart_id)
{
    if (uart_id >= BSP_UART_NUM)
    {
        return RET_INVALID_PARAM;
    }

    while (usart_flag_get(g_uart_info_table[uart_id].uart_prot, USART_FLAG_TC) == RESET)
    {
    }

    return RET_OK;
}

/**
 * @brief   IDLE 空闲中断处理（DMA 接收模式，内部）
 * @note    一帧数据接收完成（总线空闲）时触发：
 *          更新环形缓冲写指针，并通过 vTaskNotifyGiveFromISR 唤醒通知任务
 */
static void uart_idle_irq_handler(bsp_uart_id_t uart_id)
{
    const uart_info_t *uart_info;
    uint16_t current_pos;

    if (uart_id >= BSP_UART_NUM)
    {
        return;
    }

    uart_info = &g_uart_info_table[uart_id];
    if (usart_interrupt_flag_get(uart_info->uart_prot, USART_INT_FLAG_IDLE) != RESET)
    {
        /* 清 IDLE 标志的硬件固定套路：先读状态寄存器（库函数内部已读），
         * 再读一次数据寄存器。只调 clear 不读 DATA 的话标志清不掉，
         * 中断会一直重进。这里读出的字节是废数据，直接丢弃。 */
        usart_interrupt_flag_clear(uart_info->uart_prot, USART_INT_FLAG_IDLE);
        (void)usart_data_receive(uart_info->uart_prot);

        /* 计算 DMA 当前写到了缓冲的哪里：
         * dma_transfer_number_get 读到的是 DMA 计数器“还剩多少没搬”，
         * 用“总数 - 剩余”就得到“已经搬了多少”，即 DMA 写指针位置。
         * 循环模式下该值在 0~size 之间循环，% size 是防御性取模，
         * 保证算出的下标永远落在缓冲范围内。 */
        current_pos = (uint16_t)((uart_info->rx_buffer_size - dma_transfer_number_get(uart_info->dma_no, uart_info->dma_ch))
                                 % uart_info->rx_buffer_size);

        /* 把 DMA 的真实写位置同步给环形缓冲的 write 指针。
         * 环形缓冲的读指针由任务端的 ring_buffer_read 推进，
         * 写指针由 DMA 硬件推进——这里是软硬件指针的“对表”时刻：
         * 从旧 write 到 current_pos 之间的字节，就是本帧新到的数据。 */
        uart_info->rx_ring->write = current_pos;
        if (s_uart_rx_task[uart_id] != NULL)
        {
            BaseType_t higher_priority_task_woken = pdFALSE;

            /* 用 FromISR 版接口给接收任务发通知：通知计数 +1。
             * 中断里绝不能调普通版 xTaskNotifyGive（会操作调度器）。
             * higher_priority_task_woken 是输出参数：如果这次唤醒的任务
             * 优先级比当前被中断的任务高，置 pdTRUE。 */
            vTaskNotifyGiveFromISR(s_uart_rx_task[uart_id] ,&higher_priority_task_woken);

            /* 中断退出前按需切换：如果唤醒了更高优先级任务，
             * 中断返回时直接切过去，而不是回到被中断的低优先级任务
             * 再等一次 tick——这是 ISR 唤醒实时性的关键。 */
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }

    }
}

/**
 * @brief   RBNE 接收中断处理（逐字节模式，内部）
 * @note    每收到一个字节读取并写入环形缓冲
 */
static void uart_rbne_irq_handler(bsp_uart_id_t uart_id)
{
    const uart_info_t *uart_info;
    uint8_t byte;

    if (uart_id >= BSP_UART_NUM)
    {
        return;
    }

    uart_info = &g_uart_info_table[uart_id];
    if (usart_interrupt_flag_get(uart_info->uart_prot, USART_INT_FLAG_RBNE) != RESET)
    {
        /* RBNE=接收数据寄存器非空：读 DATA 既拿到字节，
         * 又自动清除 RBNE 标志（一举两得，不需要手动清）。
         * 随手写入环形缓冲；缓冲写满时 ring_buffer_write 内部丢弃
         * 最老数据，保证 ISR 永远不阻塞、不等待。 */
        byte = (uint8_t)usart_data_receive(uart_info->uart_prot);
        (void)ring_buffer_write(uart_info->rx_ring, &byte);
    }
}

/* 四个 ISR 都是同一个分发逻辑：查配置表的 use_dma，
 * DMA 路走 IDLE 分帧处理，逐字节路走 RBNE 搬运。
 * 这样新增串口时只需在配置表加一行，ISR 照抄即可。 */

/** USART0 中断入口（主机串口，DMA+IDLE 模式） */
void USART0_IRQHandler(void)
{
    if (g_uart_info_table[BSP_UART_HOST].use_dma != 0U)
    {
        uart_idle_irq_handler(BSP_UART_HOST);
    }
    else
    {
        uart_rbne_irq_handler(BSP_UART_HOST);
    }
}

/** USART2 中断入口（网络串口，DMA+IDLE 模式） */
void USART2_IRQHandler(void)
{
    if (g_uart_info_table[BSP_UART_NET].use_dma != 0U)
    {
        uart_idle_irq_handler(BSP_UART_NET);
    }
    else
    {
        uart_rbne_irq_handler(BSP_UART_NET);
    }
}

/** UART3 中断入口（LoRa 串口，DMA+IDLE 模式） */
void UART3_IRQHandler(void)
{
    if (g_uart_info_table[BSP_UART_LORA].use_dma != 0U)
    {
        uart_idle_irq_handler(BSP_UART_LORA);
    }
    else
    {
        uart_rbne_irq_handler(BSP_UART_LORA);
    }
}

/** UART4 中断入口（显示串口，RBNE 逐字节模式） */
void UART4_IRQHandler(void)
{
    if (g_uart_info_table[BSP_UART_DISPLAY].use_dma != 0U)
    {
        uart_idle_irq_handler(BSP_UART_DISPLAY);
    }
    else
    {
        uart_rbne_irq_handler(BSP_UART_DISPLAY);
    }
}
