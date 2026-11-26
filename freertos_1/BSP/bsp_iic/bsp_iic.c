/**
 * @file    bsp_iic.c
 * @brief   I2C BSP 驱动实现（软件模拟 / 硬件 I2C0 双模式）
 *
 * @note    总线配置由内部结构体数组维护：
 *          - AT24C02：PB6=SCL、PB7=SDA，模式由 BSP_IIC_MODE 决定；
 *          - GT911：PF10=SCL、PF9=SDA，固定使用软件 I2C。
 *          公共接口先选择总线实例，再由实例 mode 分发到软/硬件实现。
 *          读写均带超时保护，避免总线挂死导致任务阻塞。
 */

#include "bsp_iic.h"
#include "bsp_delay/bsp_delay.h"
#include "gd32f30x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/** IIC 模块日志开关：1=输出 DBG_log，0=屏蔽本模块全部日志。 */
#define BSP_IIC_LOG_ENABLE   (0U)

#if BSP_IIC_LOG_ENABLE
#define IIC_LOG(...)         DBG_log(__VA_ARGS__)
#else
#define IIC_LOG(...)
#endif

/** 总线恢复时引脚驱动延时（us）。 */
#define BSP_I2C_RECOVER_DELAY_US   5U
/** 等待I2C全局事务锁的超时：事务毫秒级，100ms足够宽裕。 */
#define BSP_I2C_LOCK_TIMEOUT_MS    (100U)
/** 硬件 I2C 地址阶段失败后的重试次数（清错误标志 + 总线恢复后再发一次）。 */
#define BSP_I2C_ADDR_RETRY_COUNT   (3U)

/**
 * @brief   I2C 全局事务互斥量
 * @note    必须存在的原因：s_bsp_i2c_active 是全局"当前总线"指针，
 *          每次事务入口经 select_bus 改写；若无锁，两个任务在不同
 *          总线上的事务会互相翻指针——对方等标志的超时值（timeout_us
 *          各总线不同）和引脚上下文全被偷换（板测实证：AT24C02硬I2C
 *          多步事务中途被GT911软I2C轮询插入，SBSEND以1ms超时误判）。
 *          全局串行化保证任一时刻只有一个I2C事务在总线上。
 */
static SemaphoreHandle_t s_bsp_iic_txn_mutex = NULL;

/************************************************************
 * @brief   I2C BSP 全局初始化：创建全局事务互斥量
 * @note    参照 dev_gd25q128 的模式——锁在模块 init 里创建，
 *          只创建一次，且发生在调度器启动前（单线程上下文）。
 *          不能用"首次事务时懒创建"：两个任务第一次同时进锁函数
 *          会在判空和写回句柄之间互相穿插，创建出两把锁，
 *          互斥失效且泄漏一把（并发下的初始化竞态）。
 * @retval  RET_OK 成功; RET_ALREADY_INIT 已初始化; RET_NO_MEMORY 堆不足
 ************************************************************/
ret_t bsp_iic_init(void)
{
    if (s_bsp_iic_txn_mutex != NULL)
    {
        return RET_ALREADY_INIT;
    }

    s_bsp_iic_txn_mutex = xSemaphoreCreateMutex();
    if (s_bsp_iic_txn_mutex == NULL)
    {
        return RET_NO_MEMORY;
    }

    return RET_OK;
}

/**
 * @brief   获取全局事务锁（锁必须已由 bsp_iic_init() 创建）
 */
static ret_t bsp_iic_txn_lock(void)
{
    if (s_bsp_iic_txn_mutex == NULL)
    {
        /* 未全局初始化就发起事务属于调用顺序错误，显式报错而不是静默创建。 */
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_bsp_iic_txn_mutex,
                       pdMS_TO_TICKS(BSP_I2C_LOCK_TIMEOUT_MS)) != pdTRUE)
    {
        return RET_TIMEOUT;
    }

    return RET_OK;
}

static void bsp_iic_txn_unlock(void)
{
    (void)xSemaphoreGive(s_bsp_iic_txn_mutex);
}

/**
 * @brief   I2C 总线实例配置
 * @note    结构体数组只在本 BSP 内部使用；上层通过 bus_id 选择实例。
 */
typedef struct
{
    rcu_periph_enum gpio_rcu;   /**< GPIO 时钟 */
    rcu_periph_enum i2c_rcu;    /**< 硬件 I2C 外设时钟 */
    uint32_t periph;            /**< 硬件 I2C 外设基址 */
    uint32_t gpio_port;         /**< GPIO 端口基址 */
    uint32_t scl_pin;           /**< SCL 引脚 */
    uint32_t sda_pin;           /**< SDA 引脚 */
    uint8_t mode;               /**< BSP_IIC_MODE_SOFT / BSP_IIC_MODE_HARD */
    uint32_t speed_hz;          /**< 硬件 I2C 速率 */
    uint32_t bit_delay_us;      /**< 软件 I2C 半周期延时 */
    uint32_t timeout_us;        /**< 软件/硬件等待超时 */
    volatile uint8_t initialized;
} bsp_i2c_bus_t;

/* 内部函数声明：所有 static 实现统一在文件前部声明。 */
static void hard_i2c_gpio_to_od_output(void);
static void hard_i2c_gpio_to_af_od(void);
static ret_t hard_i2c_recover_bus_lines(void);
static void hard_i2c_periph_init(void);

static void soft_i2c_delay(void);
static ret_t soft_i2c_clock_high(void);
static ret_t soft_i2c_start(void);
static ret_t soft_i2c_stop(void);
static ret_t soft_i2c_send_byte(uint8_t byte);
static ret_t soft_i2c_read_byte(uint8_t *byte);
static ret_t soft_i2c_wait_ack(void);
static ret_t soft_i2c_send_ack(void);
static ret_t soft_i2c_send_nack(void);
static ret_t soft_i2c_send_addr(uint16_t slave_addr, uint8_t is_read);
static ret_t soft_i2c_send_mem_addr(uint16_t mem_addr, uint8_t mem_addr_size);
static ret_t soft_i2c_mem_write_bytes(uint16_t slave_addr,
                                      uint16_t mem_addr,
                                      uint8_t mem_addr_size,
                                      const uint8_t *buffer,
                                      uint16_t len);
static ret_t soft_i2c_mem_read_bytes(uint16_t slave_addr,
                                     uint16_t mem_addr,
                                     uint8_t mem_addr_size,
                                     uint8_t *buffer,
                                     uint16_t len);

static ret_t hard_i2c_wait_flag(uint32_t i2c_periph,
                                i2c_flag_enum flag,
                                FlagStatus expected);
static void hard_i2c_reset_bus(void);
static void hard_i2c_dump(void);
static void hard_i2c_clear_error_flags(void);
static ret_t hard_i2c_wait_stop(void);
static ret_t hard_i2c_finish(ret_t ret);
static ret_t hard_i2c_try_addr_once(uint16_t slave_addr, uint32_t direction);
static ret_t hard_i2c_start_and_send_addr(uint16_t slave_addr,
                                          uint32_t direction);
static ret_t hard_i2c_write_one(uint8_t data);
static ret_t hard_i2c_receive_current(uint8_t *buffer, uint16_t len);
static ret_t hard_i2c_send_mem_addr(uint16_t mem_addr, uint8_t mem_addr_size);
static ret_t hard_i2c_mem_write_bytes(uint16_t slave_addr,
                                      uint16_t mem_addr,
                                      uint8_t mem_addr_size,
                                      const uint8_t *buffer,
                                      uint16_t len);
static ret_t hard_i2c_mem_read_bytes(uint16_t slave_addr,
                                     uint16_t mem_addr,
                                     uint8_t mem_addr_size,
                                     uint8_t *buffer,
                                     uint16_t len);

static ret_t bsp_i2c_init_impl(void);
static ret_t bsp_i2c_gpio_init_impl(void);
static ret_t bsp_i2c_mem_write_impl(uint16_t slave_addr,
                                    uint16_t mem_addr,
                                    uint8_t mem_addr_size,
                                    const uint8_t *buffer,
                                    uint16_t len);
static ret_t bsp_i2c_mem_read_impl(uint16_t slave_addr,
                                   uint16_t mem_addr,
                                   uint8_t mem_addr_size,
                                   uint8_t *buffer,
                                   uint16_t len);
static ret_t bsp_iic_select_bus(bsp_iic_bus_id_t bus_id);
static ret_t bsp_iic_bus_prepare(bsp_iic_bus_id_t bus_id);

/**
 * @brief   I2C 总线实例表
 * @note    AT24C02 与 GT911 使用不同引脚，不能共用同一组 GPIO 宏。
 */
static bsp_i2c_bus_t s_bsp_i2c_bus_table[BSP_IIC_BUS_COUNT] =
{
    /* AT24C02：PB6=SCL、PB7=SDA，模式由 BSP_IIC_MODE 决定。 */
    [BSP_IIC_BUS_AT24C02] = {
        .gpio_rcu = RCU_GPIOB,
        .i2c_rcu = RCU_I2C0,
        .periph = I2C0,
        .gpio_port = GPIOB,
        .scl_pin = GPIO_PIN_6,
        .sda_pin = GPIO_PIN_7,
        .mode = (uint8_t)BSP_IIC_MODE,
        .speed_hz = 100000U,
        .bit_delay_us = 4U,
        .timeout_us = 100000U,
        .initialized = 0U
    },
    /* GT911：PF10=SCL、PF9=SDA，固定软件 I2C，16 位寄存器地址。 */
    [BSP_IIC_BUS_GT911] = {
        .gpio_rcu = RCU_GPIOF,
        .i2c_rcu = RCU_I2C0,
        .periph = I2C0,
        .gpio_port = GPIOF,
        .scl_pin = GPIO_PIN_10,
        .sda_pin = GPIO_PIN_9,
        .mode = BSP_IIC_MODE_SOFT,
        .speed_hz = 100000U,
        .bit_delay_us = 4U,
        .timeout_us = 1000U,
        .initialized = 0U
    }
};

/** 当前事务使用的总线实例；公共入口选择后，内部原语只访问该实例。 */
static bsp_i2c_bus_t *s_bsp_i2c_active = 0;

/* 软 I2C 引脚操作宏：开漏输出置高表示释放，由外部上拉提供高电平。 */
#define SOFT_I2C_SCL_HIGH() \
    do { gpio_bit_set(s_bsp_i2c_active->gpio_port, s_bsp_i2c_active->scl_pin); } while (0)

#define SOFT_I2C_SCL_LOW() \
    do { gpio_bit_reset(s_bsp_i2c_active->gpio_port, s_bsp_i2c_active->scl_pin); } while (0)

#define SOFT_I2C_SDA_HIGH() \
    do { gpio_bit_set(s_bsp_i2c_active->gpio_port, s_bsp_i2c_active->sda_pin); } while (0)

#define SOFT_I2C_SDA_LOW() \
    do { gpio_bit_reset(s_bsp_i2c_active->gpio_port, s_bsp_i2c_active->sda_pin); } while (0)

#define SOFT_I2C_SCL_READ() \
    gpio_input_bit_get(s_bsp_i2c_active->gpio_port, s_bsp_i2c_active->scl_pin)

#define SOFT_I2C_SDA_READ() \
    gpio_input_bit_get(s_bsp_i2c_active->gpio_port, s_bsp_i2c_active->sda_pin)

/** 恢复期间也必须开漏；禁止与从机低电平形成推挽争用。 */
static void hard_i2c_gpio_to_od_output(void)
{
    SOFT_I2C_SCL_HIGH();
    SOFT_I2C_SDA_HIGH();
    gpio_init(s_bsp_i2c_active->gpio_port, GPIO_MODE_OUT_OD,
              GPIO_OSPEED_50MHZ,
              s_bsp_i2c_active->scl_pin | s_bsp_i2c_active->sda_pin);
}

static void hard_i2c_gpio_to_af_od(void)
{
    gpio_init(s_bsp_i2c_active->gpio_port, GPIO_MODE_AF_OD,
              GPIO_OSPEED_50MHZ,
              s_bsp_i2c_active->scl_pin | s_bsp_i2c_active->sda_pin);
}

/** 有界恢复：释放线路，必要时补时钟，最后生成 STOP。 */
static ret_t hard_i2c_recover_bus_lines(void)
{
    uint8_t pulse;
    ret_t ret;
    i2c_deinit(s_bsp_i2c_active->periph);
    hard_i2c_gpio_to_od_output();
    ret = soft_i2c_clock_high();
    if (ret != RET_OK)
    {
        SOFT_I2C_SCL_HIGH();
        return ret;
    }

    for (pulse = 0U; (pulse < 9U) && (SOFT_I2C_SDA_READ() == RESET); pulse++)
    {
        SOFT_I2C_SCL_LOW();
        (void)bsp_delay_us(BSP_I2C_RECOVER_DELAY_US);
        ret = soft_i2c_clock_high();
        if (ret != RET_OK)
        {
            SOFT_I2C_SCL_HIGH();
            return ret;
        }
    }

    /* 先在 SCL 低时建立 SDA 低，随后释放 SCL、SDA 形成 STOP。 */
    SOFT_I2C_SCL_LOW();
    SOFT_I2C_SDA_LOW();
    (void)bsp_delay_us(BSP_I2C_RECOVER_DELAY_US);
    ret = soft_i2c_clock_high();
    SOFT_I2C_SDA_HIGH();
    SOFT_I2C_SCL_HIGH();
    (void)bsp_delay_us(BSP_I2C_RECOVER_DELAY_US);
    if (ret != RET_OK)
    {
        return ret;
    }
    return ((SOFT_I2C_SCL_READ() != RESET) &&
            (SOFT_I2C_SDA_READ() != RESET)) ? RET_OK : RET_BUS_ERROR;
}

/** 每次配置前复位外设，避免旧 CTL0/CKCFG 位混入新配置。 */
static void hard_i2c_periph_init(void)
{
    i2c_deinit(s_bsp_i2c_active->periph);
    if (s_bsp_i2c_active->periph == I2C0)
    {
        rcu_periph_clock_enable(RCU_AF);
        gpio_pin_remap_config(GPIO_I2C0_REMAP, DISABLE); /* PB6/PB7 */
        /* PB7 与 EXMC_NADV 复用，且 AFIO_PCF1.EXMC_NADV 复位默认值为 0，
         * 含义是“NADV 连接到 PB7”。LCD 走 EXMC 后，PB7 被 NADV 驱动，
         * I2C0_SDA 被夺走：地址阶段永远收不到 ACK（stat0=AERR、SCL 被
         * 拉低、SDA 无跳变）。置 1 断开 NADV，PB7 才能交还给 I2C0_SDA。
         * 这正是“boot 探测(EXMC 未使能)通过、任务探测(EXMC 已使能)失败”
         * 的根因——与 AERR 残留无关。 */
        AFIO_PCF1 |= AFIO_PCF1_EXMC_NADV;
    }
    hard_i2c_gpio_to_af_od();
    i2c_clock_config(s_bsp_i2c_active->periph, s_bsp_i2c_active->speed_hz, I2C_DTCY_2);
    i2c_mode_addr_config(s_bsp_i2c_active->periph, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0U);
    i2c_ackpos_config(s_bsp_i2c_active->periph, I2C_ACKPOS_CURRENT);
    i2c_enable(s_bsp_i2c_active->periph);
    i2c_ack_config(s_bsp_i2c_active->periph, I2C_ACK_ENABLE);
}
/* 软件 I2C 原语始终编译，运行时由总线实例的 mode 选择。 */
/** 软 I2C 位延时（半周期） */
static void soft_i2c_delay(void)
{
    (void)bsp_delay_us(s_bsp_i2c_active->bit_delay_us);
}

/**
 * @brief   释放 SCL 并等待总线真正变高
 * @retval  RET_OK 成功; RET_TIMEOUT 从机或硬件一直拉低 SCL
 */
static ret_t soft_i2c_clock_high(void)
{
    uint32_t cycle_start;

    SOFT_I2C_SCL_HIGH();
    cycle_start = bsp_delay_cycle_get();
    while (SOFT_I2C_SCL_READ() == RESET)
    {
        if (bsp_delay_timeout_expired(cycle_start, s_bsp_i2c_active->timeout_us) != 0U)
        {
            SOFT_I2C_SCL_LOW();
            return RET_TIMEOUT;
        }
    }

    soft_i2c_delay();
    return RET_OK;
}

/** 软 I2C 起始条件：SCL 高电平期间 SDA 由高拉低 */
static ret_t soft_i2c_start(void)
{
    ret_t ret;

    /* 先释放 SDA 并把 SCL 推高：START 的前提是总线空闲（双线都高），
     * clock_high 内部还会等 SCL 真正变高，防从机时钟拉伸卡死。 */
    SOFT_I2C_SDA_HIGH();
    ret = soft_i2c_clock_high();
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 总线仲裁检查：SCL 已高但 SDA 还低，说明有别的设备占着总线
     * （或 SDA 被卡死），此时发 START 会和对方冲突，直接报总线错。 */
    if (SOFT_I2C_SDA_READ() == RESET)
    {
        SOFT_I2C_SCL_LOW();
        return RET_BUS_ERROR;
    }

    /* 产生 START 边沿：SCL 保持高，SDA 高->低跳变。
     * 跳变后立刻拉低 SCL，把总线锁进“主机发送”状态，
     * 从机看到 START 后开始等待地址字节。 */
    SOFT_I2C_SDA_LOW();
    soft_i2c_delay();
    SOFT_I2C_SCL_LOW();
    soft_i2c_delay();

    return RET_OK;
}

/** 软 I2C 停止条件：SCL 高电平期间 SDA 由低拉高 */
static ret_t soft_i2c_stop(void)
{
    ret_t ret;

    SOFT_I2C_SDA_LOW();
    soft_i2c_delay();
    ret = soft_i2c_clock_high();
    if (ret != RET_OK)
    {
        return ret;
    }

    SOFT_I2C_SDA_HIGH();
    soft_i2c_delay();

    if ((SOFT_I2C_SDA_READ() == RESET) || (SOFT_I2C_SCL_READ() == RESET))
    {
        return RET_BUS_ERROR;
    }

    return RET_OK;
}

/** 软 I2C 发送一个字节（MSB 先行） */
static ret_t soft_i2c_send_byte(uint8_t byte)
{
    uint8_t bit_index;
    ret_t ret;

    /* 循环 8 次，每次发 1 位：I2C 规定高位先行（MSB first）。
     * 时序铁律：SDA 必须在 SCL 为低时搭建好，SCL 高期间 SDA 不许动
     * （高期间动 SDA 会被误认成 START/STOP）。 */
    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        /* 取最高位决定 SDA 电平：1=释放（上拉变高），0=拉低 */
        if ((byte & 0x80U) != 0U)
        {
            SOFT_I2C_SDA_HIGH();
        }
        else
        {
            SOFT_I2C_SDA_LOW();
        }

        /* 左移一位，让下一位升到最高位的位置 */
        byte <<= 1U;
        soft_i2c_delay();
        /* SCL 推高：从机在 SCL 上升沿采样 SDA。
         * clock_high 自带超时，若从机拉伸 SCL 不放则报错退出，
         * 退出前先释放 SDA，避免把总线按死。 */
        ret = soft_i2c_clock_high();
        if (ret != RET_OK)
        {
            SOFT_I2C_SDA_HIGH();
            return ret;
        }

        /* 一位结束：拉低 SCL，准备下一位 */
        SOFT_I2C_SCL_LOW();
        soft_i2c_delay();
    }

    return RET_OK;
}

/** 软 I2C 读取一个字节（MSB 先行） */
static ret_t soft_i2c_read_byte(uint8_t *byte)
{
    uint8_t value;
    uint8_t bit_index;
    ret_t ret;

    if (byte == NULL)
    {
        return RET_NULL_PTR;
    }

    value = 0U;
    /* 读字节前必须释放 SDA：读的时候 SDA 由从机驱动，
     * 主机若还按着 SDA，从机的数据会被短路。 */
    SOFT_I2C_SDA_HIGH();

    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        /* SCL 推高，从机在此时把数据位放到 SDA 上 */
        ret = soft_i2c_clock_high();
        if (ret != RET_OK)
        {
            SOFT_I2C_SCL_LOW();
            return ret;
        }

        /* 先左移腾出最低位，再把采样到的位拼进去，
         * 8 轮下来 value 就是 MSB 先行的完整字节 */
        value <<= 1U;

        if (SOFT_I2C_SDA_READ() != RESET)
        {
            value |= 0x01U;
        }

        SOFT_I2C_SCL_LOW();
        soft_i2c_delay();
    }

    *byte = value;
    return RET_OK;
}

/**
 * @brief   等待从机 ACK（内部）
 * @retval  RET_OK 收到 ACK; RET_NACK_ERROR 无应答; RET_TIMEOUT 超时
 */
static ret_t soft_i2c_wait_ack(void)
{
    ret_t ret;

    SOFT_I2C_SDA_HIGH();
    soft_i2c_delay();
    ret = soft_i2c_clock_high();
    if (ret != RET_OK)
    {
        DBG_Warning("soft iic wait ack clock timeout, slave=unknown");
        SOFT_I2C_SCL_LOW();
        return ret;
    }

    if (SOFT_I2C_SDA_READ() != RESET)
    {
        DBG_Warning("soft iic wait ack nack, slave=unknown");
        SOFT_I2C_SCL_LOW();
        return RET_NACK_ERROR;
    }

    SOFT_I2C_SCL_LOW();
    soft_i2c_delay();

    return RET_OK;
}

/** 软 I2C 发送 ACK */
static ret_t soft_i2c_send_ack(void)
{
    SOFT_I2C_SDA_LOW();
    soft_i2c_delay();
    if (soft_i2c_clock_high() != RET_OK)
    {
        SOFT_I2C_SCL_LOW();
        return RET_TIMEOUT;
    }

    SOFT_I2C_SCL_LOW();
    soft_i2c_delay();
    SOFT_I2C_SDA_HIGH();

    return RET_OK;
}

/** 软 I2C 发送 NACK */
static ret_t soft_i2c_send_nack(void)
{
    SOFT_I2C_SDA_HIGH();
    soft_i2c_delay();
    if (soft_i2c_clock_high() != RET_OK)
    {
        SOFT_I2C_SCL_LOW();
        return RET_TIMEOUT;
    }

    SOFT_I2C_SCL_LOW();
    soft_i2c_delay();

    return RET_OK;
}

/**
 * @brief   发送从机地址字节（内部）
 * @param   slave_addr  从机地址（8 位格式）
 * @param   is_read     1=读方向，0=写方向
 */
static ret_t soft_i2c_send_addr(uint16_t slave_addr, uint8_t is_read)
{
    uint8_t addr_byte;
    ret_t ret;

    addr_byte = (uint8_t)(slave_addr | (is_read & 0x01U));
    ret = soft_i2c_send_byte(addr_byte);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = soft_i2c_wait_ack();
    if (ret != RET_OK)
    {
        return ret;
    }

    return RET_OK;
}

/**
 * @brief   发送从机寄存器地址（内部）
 * @param   mem_addr       寄存器地址
 * @param   mem_addr_size  地址宽度（BSP_IIC_MEM_ADDR_8BIT / _16BIT）
 */
static ret_t soft_i2c_send_mem_addr(uint16_t mem_addr, uint8_t mem_addr_size)
{
    ret_t ret;

    if (mem_addr_size == BSP_IIC_MEM_ADDR_16BIT)
    {
        ret = soft_i2c_send_byte((uint8_t)((mem_addr >> 8U) & 0xFFU));
        if (ret != RET_OK)
        {
            return ret;
        }

        ret = soft_i2c_wait_ack();
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    ret = soft_i2c_send_byte((uint8_t)(mem_addr & 0xFFU));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = soft_i2c_wait_ack();
    if (ret != RET_OK)
    {
        return ret;
    }

    return RET_OK;
}

/**
 * @brief   软 I2C 向从机寄存器地址连续写数据（内部）
 * @retval  RET_OK 成功; RET_NACK_ERROR 无应答; RET_INVALID_PARAM 参数非法
 */
static ret_t soft_i2c_mem_write_bytes(uint16_t slave_addr,
                                      uint16_t mem_addr,
                                      uint8_t mem_addr_size,
                                      const uint8_t *buffer,
                                      uint16_t len)
{
    uint16_t index;
    ret_t ret;

    if ((buffer == NULL) || (len == 0U))
    {
        return RET_INVALID_PARAM;
    }

    if ((mem_addr_size != BSP_IIC_MEM_ADDR_8BIT) && (mem_addr_size != BSP_IIC_MEM_ADDR_16BIT))
    {
        return RET_INVALID_PARAM;
    }

    ret = soft_i2c_start();
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = soft_i2c_send_addr(slave_addr, 0U);
    if (ret != RET_OK)
    {
        (void)soft_i2c_stop();
        return ret;
    }

    ret = soft_i2c_send_mem_addr(mem_addr, mem_addr_size);
    if (ret != RET_OK)
    {
        (void)soft_i2c_stop();
        return ret;
    }

    for (index = 0U; index < len; index++)
    {
        ret = soft_i2c_send_byte(buffer[index]);
        if (ret != RET_OK)
        {
            (void)soft_i2c_stop();
            return ret;
        }

        ret = soft_i2c_wait_ack();
        if (ret != RET_OK)
        {
            (void)soft_i2c_stop();
            return ret;
        }
    }

    return soft_i2c_stop();
}

/**
 * @brief   软 I2C 从从机寄存器地址连续读数据（内部）
 * @retval  RET_OK 成功; RET_NACK_ERROR 无应答; RET_INVALID_PARAM 参数非法
 */
static ret_t soft_i2c_mem_read_bytes(uint16_t slave_addr,
                                     uint16_t mem_addr,
                                     uint8_t mem_addr_size,
                                     uint8_t *buffer,
                                     uint16_t len)
{
    uint16_t index;
    ret_t ret;

    if ((buffer == NULL) || (len == 0U))
    {
        return RET_INVALID_PARAM;
    }

    if ((mem_addr_size != BSP_IIC_MEM_ADDR_8BIT) && (mem_addr_size != BSP_IIC_MEM_ADDR_16BIT))
    {
        return RET_INVALID_PARAM;
    }

    ret = soft_i2c_start();
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = soft_i2c_send_addr(slave_addr, 0U);
    if (ret != RET_OK)
    {
        (void)soft_i2c_stop();
        return ret;
    }

    /* 先以“写方向”把要读的寄存器地址告诉从机：
     * 这一步只传地址不传数据，从机收到后把内部地址指针拨到该寄存器。 */
    ret = soft_i2c_send_mem_addr(mem_addr, mem_addr_size);
    if (ret != RET_OK)
    {
        (void)soft_i2c_stop();
        return ret;
    }

    /* 重复起始（Repeated START）：不发 STOP，直接再发一次 START。
     * 作用是“总线不释放、直接掉头转读”——若中间插 STOP，
     * 多主机环境下总线会被别人抢走，从机的地址指针也可能复位。 */
    ret = soft_i2c_start();
    if (ret != RET_OK)
    {
        (void)soft_i2c_stop();
        return ret;
    }

    /* 第二次寻址：方向位=1（读）。从机应答后开始吐数据。 */
    ret = soft_i2c_send_addr(slave_addr, 1U);
    if (ret != RET_OK)
    {
        (void)soft_i2c_stop();
        return ret;
    }

    for (index = 0U; index < len; index++)
    {
        ret = soft_i2c_read_byte(&buffer[index]);
        if (ret != RET_OK)
        {
            (void)soft_i2c_stop();
            return ret;
        }

        /* 读完每个字节主机必须表态：
         * 非最后一个回 ACK=0（“还有，继续发”）；
         * 最后一个回 NACK=1（“够了，停”），从机收到 NACK 后释放 SDA，
         * 主机随后发 STOP 才不会和从机打架。 */
        if (index < (uint16_t)(len - 1U))
        {
            ret = soft_i2c_send_ack();
        }
        else
        {
            ret = soft_i2c_send_nack();
        }

        if (ret != RET_OK)
        {
            (void)soft_i2c_stop();
            return ret;
        }
    }

    return soft_i2c_stop();
}
/* 硬件事务阶段只在全局事务锁内修改；诊断不能在正常地址阶段读 STAT1。 */
static const char *s_hard_i2c_phase = "idle";
static uint8_t s_hard_i2c_address_byte;

static void hard_i2c_dump(void)
{
    uint32_t periph = s_bsp_i2c_active->periph;
    uint32_t ctl0 = I2C_CTL0(periph);
    uint32_t stat0 = I2C_STAT0(periph);
    uint32_t stat1 = I2C_STAT1(periph);
    DBG_log("hard iic pins ctl=0x%08lX octl=0x%04lX remap=0x%08lX address=0x%02X",
            (unsigned long)GPIO_CTL0(s_bsp_i2c_active->gpio_port),
            (unsigned long)GPIO_OCTL(s_bsp_i2c_active->gpio_port),
            (unsigned long)AFIO_PCF0, (unsigned int)s_hard_i2c_address_byte);
    DBG_Warning("hard iic phase=%s ctl0=0x%04X ctl1=0x%04X stat0=0x%04X stat1=0x%04X scl=%u sda=%u",
                s_hard_i2c_phase, (unsigned int)ctl0, (unsigned int)I2C_CTL1(periph),
                (unsigned int)stat0, (unsigned int)stat1,
                (unsigned int)SOFT_I2C_SCL_READ(), (unsigned int)SOFT_I2C_SDA_READ());
}

/**
 * @brief   清除硬件 I2C 粘滞错误标志（AERR / BERR / LOSTARB）
 *
 * @note    GD32F30x 勘误：从机对地址字节回 NACK 时 STAT0.AERR 置位；
 *          该位是粘滞的，若不显式清除，硬件会持续把 SCL 拉低不放
 *          （实测 stat0=0x0400 stat1=0x0003 scl=0 sda=1），
 *          下一次 START 无法发出，整条总线表现为“卡死”。
 *          手册规定的清除序列为“先读 STAT0 再读 STAT1”，这里再用库函数
 *          写 0 清除各标志位，两种机制互为兜底，确保三者全部归零。
 *          只在事务入口或地址阶段失败后调用：此刻 ADDSEND 尚未使用，
 *          读 STAT1 不会误清正常流程需要的事件标志。
 */
static void hard_i2c_clear_error_flags(void)
{
    uint32_t periph = s_bsp_i2c_active->periph;
    /* 手册序列：读 STAT0 后读 STAT1，清除 AERR/BERR/LOSTARB。 */
    (void)I2C_STAT0(periph);
    (void)I2C_STAT1(periph);
    /* 库函数按位写 0 清除；三者在同一 STAT0 寄存器，互为兜底。 */
    i2c_flag_clear(periph, I2C_FLAG_AERR);
    i2c_flag_clear(periph, I2C_FLAG_BERR);
    i2c_flag_clear(periph, I2C_FLAG_LOSTARB);
}

/** 每一个硬件等待都有超时；NACK 不再伪装成超时。 */
static ret_t hard_i2c_wait_flag(uint32_t periph, i2c_flag_enum flag, FlagStatus expected)
{
    uint32_t start = bsp_delay_cycle_get();
    uint32_t status;
    for (;;)
    {
        status = I2C_STAT0(periph);
        if ((status & (I2C_STAT0_BERR | I2C_STAT0_LOSTARB)) != 0U)
        {
            hard_i2c_dump();
            return RET_BUS_ERROR;
        }
        if ((status & I2C_STAT0_AERR) != 0U)
        {
            hard_i2c_dump();
            return RET_NACK_ERROR;
        }
        if (i2c_flag_get(periph, flag) == expected)
        {
            return RET_OK;
        }
        if (bsp_delay_timeout_expired(start, s_bsp_i2c_active->timeout_us) != 0U)
        {
            hard_i2c_dump();
            return RET_TIMEOUT;
        }
    }
}

/** STOP 自清零且总线空闲后，才能恢复 ACK / 释放事务锁。 */
static ret_t hard_i2c_wait_stop(void)
{
    uint32_t periph = s_bsp_i2c_active->periph;
    uint32_t start = bsp_delay_cycle_get();
    s_hard_i2c_phase = "stop";
    while ((I2C_CTL0(periph) & I2C_CTL0_STOP) != 0U)
    {
        if (bsp_delay_timeout_expired(start, s_bsp_i2c_active->timeout_us) != 0U)
        {
            hard_i2c_dump();
            return RET_TIMEOUT;
        }
    }
    return hard_i2c_wait_flag(periph, I2C_FLAG_I2CBSY, RESET);
}

static void hard_i2c_reset_bus(void)
{
    ret_t ret;
    DBG_Warning("hard iic reset bus");
    ret = hard_i2c_recover_bus_lines();
    if (ret != RET_OK)
    {
        s_bsp_i2c_active->initialized = 0U;
        DBG_Warning("hard iic recovery FAIL ret=%ld scl=%u sda=%u", (long)ret,
                    (unsigned int)SOFT_I2C_SCL_READ(), (unsigned int)SOFT_I2C_SDA_READ());
        return; /* 保持外设关闭、GPIO 开漏释放，不能假报恢复成功。 */
    }
    hard_i2c_periph_init();
}

/** 全部读写出口共用收尾；失败不自动重发可能已部分完成的写入。 */
static ret_t hard_i2c_finish(ret_t ret)
{
    uint32_t periph = s_bsp_i2c_active->periph;
    if (ret == RET_OK)
    {
        ret = hard_i2c_wait_stop();
    }
    if (ret != RET_OK)
    {
        if ((I2C_STAT0(periph) & I2C_STAT0_LOSTARB) != 0U)
        {
            /* 仲裁丢失不发 STOP / 恢复脉冲，避免干扰胜出的主机。 */
            hard_i2c_periph_init();
        }
        else
        {
            hard_i2c_reset_bus();
        }
        return ret;
    }
    i2c_ackpos_config(periph, I2C_ACKPOS_CURRENT);
    i2c_ack_config(periph, I2C_ACK_ENABLE);
    s_hard_i2c_phase = "idle";
    return RET_OK;
}

/** 地址阶段单次尝试：START + 从机地址 + 等 ADDSEND；不含重试。 */
static ret_t hard_i2c_try_addr_once(uint16_t slave_addr, uint32_t direction)
{
    uint32_t periph = s_bsp_i2c_active->periph;
    ret_t ret;

    s_hard_i2c_phase = "idle-before-start";
    ret = hard_i2c_wait_flag(periph, I2C_FLAG_I2CBSY, RESET);
    if (ret != RET_OK)
    {
        return ret;
    }
    s_hard_i2c_phase = "first-start";
    i2c_start_on_bus(periph);
    ret = hard_i2c_wait_flag(periph, I2C_FLAG_SBSEND, SET);
    if (ret != RET_OK)
    {
        return ret;
    }
    s_hard_i2c_phase = "write-address";
    i2c_master_addressing(periph, (uint32_t)slave_addr, direction);
    ret = hard_i2c_wait_flag(periph, I2C_FLAG_ADDSEND, SET);
    if (ret == RET_OK)
    {
        i2c_flag_clear(periph, I2C_FLAG_ADDSEND);
    }
    return ret;
}

/**
 * @brief   发送 START 并把从机地址发出去，失败自动重试
 * @note    地址阶段一旦 NACK，GD32 硬件会把 AERR 置位并持续拉低 SCL；
 *          若只返回错误而不清标志，上层即便再次调用也发不出 START。
 *          因此每次失败先清粘滞错误标志、再有界恢复总线后重试，
 *          最多 BSP_I2C_ADDR_RETRY_COUNT 次。
 */
static ret_t hard_i2c_start_and_send_addr(uint16_t slave_addr, uint32_t direction)
{
    ret_t ret = RET_INVALID_STATE;
    uint8_t attempt;

    s_hard_i2c_address_byte = (uint8_t)((slave_addr & 0xFEU) |
                                      ((direction == I2C_RECEIVER) ? 1U : 0U));

    for (attempt = 0U; attempt < BSP_I2C_ADDR_RETRY_COUNT; attempt++)
    {
        ret = hard_i2c_try_addr_once(slave_addr, direction);
        if (ret == RET_OK)
        {
            return RET_OK;
        }
        /* 失败先清 AERR/BERR/LOSTARB：不清则 SCL 被硬件一直拉低。 */
        hard_i2c_clear_error_flags();
        if ((uint8_t)(attempt + 1U) >= BSP_I2C_ADDR_RETRY_COUNT)
        {
            break; /* 最后一次不再恢复，交由调用方统一收尾。 */
        }
        hard_i2c_reset_bus();
        if (s_bsp_i2c_active->initialized == 0U)
        {
            /* 线路无法释放，重试没有意义，直接报总线错误。 */
            return RET_BUS_ERROR;
        }
    }
    return ret;
}

/** BTC 确保最后一字节完成后，才发送重复 START 或 STOP。 */
static ret_t hard_i2c_write_one(uint8_t data)
{
    uint32_t periph = s_bsp_i2c_active->periph;
    ret_t ret = hard_i2c_wait_flag(periph, I2C_FLAG_TBE, SET);
    if (ret != RET_OK)
    {
        return ret;
    }
    i2c_data_transmit(periph, data);
    return hard_i2c_wait_flag(periph, I2C_FLAG_BTC, SET);
}

/** GD32F30x 手册 solution B：入口 ADDSEND 尚未清除。 */
static ret_t hard_i2c_receive_current(uint8_t *buffer, uint16_t len)
{
    uint32_t periph = s_bsp_i2c_active->periph;
    uint16_t remaining = len;
    ret_t ret;
    s_hard_i2c_phase = "receive";
    if (len == 1U)
    {
        /* 只保护短寄存器序列，禁止在临界区内等待硬件。 */
        taskENTER_CRITICAL();
        i2c_ack_config(periph, I2C_ACK_DISABLE);
        i2c_flag_clear(periph, I2C_FLAG_ADDSEND);
        i2c_stop_on_bus(periph);
        taskEXIT_CRITICAL();
        ret = hard_i2c_wait_flag(periph, I2C_FLAG_RBNE, SET);
        if (ret != RET_OK)
        {
            return ret;
        }
        *buffer = i2c_data_receive(periph);
        return RET_OK;
    }
    if (len == 2U)
    {
        /* POAP 已在重复 START 前设为 NEXT。 */
        i2c_ack_config(periph, I2C_ACK_DISABLE);
        i2c_flag_clear(periph, I2C_FLAG_ADDSEND);
        ret = hard_i2c_wait_flag(periph, I2C_FLAG_BTC, SET);
        if (ret != RET_OK)
        {
            return ret;
        }
        taskENTER_CRITICAL();
        i2c_stop_on_bus(periph);
        buffer[0] = i2c_data_receive(periph);
        buffer[1] = i2c_data_receive(periph);
        taskEXIT_CRITICAL();
        return RET_OK;
    }

    i2c_flag_clear(periph, I2C_FLAG_ADDSEND);
    while (remaining > 3U)
    {
        ret = hard_i2c_wait_flag(periph, I2C_FLAG_RBNE, SET);
        if (ret != RET_OK)
        {
            return ret;
        }
        *buffer++ = i2c_data_receive(periph);
        remaining--;
    }
    /* 最后三字节用 BTC 拉伸时钟，避免任务抢占使末字节被多 ACK。 */
    ret = hard_i2c_wait_flag(periph, I2C_FLAG_BTC, SET);
    if (ret != RET_OK)
    {
        return ret;
    }
    i2c_ack_config(periph, I2C_ACK_DISABLE);
    *buffer++ = i2c_data_receive(periph);
    ret = hard_i2c_wait_flag(periph, I2C_FLAG_BTC, SET);
    if (ret != RET_OK)
    {
        return ret;
    }
    taskENTER_CRITICAL();
    i2c_stop_on_bus(periph);
    *buffer++ = i2c_data_receive(periph);
    *buffer = i2c_data_receive(periph);
    taskEXIT_CRITICAL();
    return RET_OK;
}

static ret_t hard_i2c_send_mem_addr(uint16_t mem_addr, uint8_t mem_addr_size)
{
    ret_t ret;
    s_hard_i2c_phase = "memory-address";
    if (mem_addr_size == BSP_IIC_MEM_ADDR_16BIT)
    {
        ret = hard_i2c_write_one((uint8_t)(mem_addr >> 8U));
        if (ret != RET_OK)
        {
            return ret;
        }
    }
    return hard_i2c_write_one((uint8_t)mem_addr);
}

static ret_t hard_i2c_mem_write_bytes(uint16_t slave_addr, uint16_t mem_addr,
                                    uint8_t mem_addr_size, const uint8_t *buffer, uint16_t len)
{
    uint16_t index;
    ret_t ret = hard_i2c_start_and_send_addr(slave_addr, I2C_TRANSMITTER);
    if (ret == RET_OK)
    {
        ret = hard_i2c_send_mem_addr(mem_addr, mem_addr_size);
    }
    s_hard_i2c_phase = "write-data";
    for (index = 0U; (index < len) && (ret == RET_OK); index++)
    {
        ret = hard_i2c_write_one(buffer[index]);
    }
    if (ret == RET_OK)
    {
        i2c_stop_on_bus(s_bsp_i2c_active->periph);
    }
    return hard_i2c_finish(ret);
}

static ret_t hard_i2c_mem_read_bytes(uint16_t slave_addr, uint16_t mem_addr,
                                   uint8_t mem_addr_size, uint8_t *buffer, uint16_t len)
{
    uint32_t periph = s_bsp_i2c_active->periph;
    ret_t ret = hard_i2c_start_and_send_addr(slave_addr, I2C_TRANSMITTER);
    if (ret != RET_OK)
    {
        return hard_i2c_finish(ret);
    }
    ret = hard_i2c_send_mem_addr(mem_addr, mem_addr_size);
    if (ret != RET_OK)
    {
        return hard_i2c_finish(ret);
    }
    i2c_ackpos_config(periph, (len == 2U) ? I2C_ACKPOS_NEXT : I2C_ACKPOS_CURRENT);
    s_hard_i2c_phase = "repeated-start";
    i2c_start_on_bus(periph);
    ret = hard_i2c_wait_flag(periph, I2C_FLAG_SBSEND, SET);
    if (ret != RET_OK)
    {
        return hard_i2c_finish(ret);
    }
    s_hard_i2c_phase = "read-address";
    s_hard_i2c_address_byte = (uint8_t)(slave_addr | 1U);
    i2c_master_addressing(periph, (uint32_t)slave_addr, I2C_RECEIVER);
    ret = hard_i2c_wait_flag(periph, I2C_FLAG_ADDSEND, SET);
    if (ret == RET_OK)
    {
        ret = hard_i2c_receive_current(buffer, len);
    }
    return hard_i2c_finish(ret);
}
/**
 * @brief   I2C GPIO 初始化实现（内部）：只配置引脚，不检查总线状态
 * @retval  RET_OK 成功
 */
ret_t bsp_iic_boot_address_probe(void)
{
    bsp_i2c_bus_t *previous = s_bsp_i2c_active;
    ret_t ret;
    /* 不取 RTOS 锁、不调用接收临界区；调用者保证启动前单线程。 */
    s_bsp_i2c_active = &s_bsp_i2c_bus_table[BSP_IIC_BUS_AT24C02];
    if (s_bsp_i2c_active->initialized != 0U)
    {
        s_bsp_i2c_active = previous;
        return RET_INVALID_STATE;
    }
    ret = bsp_delay_init();
    if (ret != RET_OK)
    {
        s_bsp_i2c_active = previous;
        return ret;
    }
    rcu_periph_clock_enable(s_bsp_i2c_active->gpio_rcu);
    rcu_periph_clock_enable(s_bsp_i2c_active->i2c_rcu);
    ret = hard_i2c_recover_bus_lines();
    if (ret == RET_OK)
    {
        hard_i2c_periph_init();
        s_hard_i2c_phase = "boot-before-start";
        hard_i2c_dump();
        /* 8 位总线地址 0xA0，对应 AT24C02 的 7 位地址 0x50。 */
        ret = hard_i2c_start_and_send_addr(0xA0U, I2C_TRANSMITTER);
        if (ret == RET_OK)
        {
            i2c_stop_on_bus(s_bsp_i2c_active->periph);
        }
        ret = hard_i2c_finish(ret);
    }
    i2c_deinit(s_bsp_i2c_active->periph);
    hard_i2c_gpio_to_od_output();
    s_bsp_i2c_active = previous;
    return ret;
}

ret_t bsp_iic_task_address_probe(void)
{
    ret_t ret = bsp_iic_txn_lock();
    if (ret != RET_OK)
    {
        return ret;
    }
    /* 与启动前共用完全相同的地址探测；锁保护活动总线指针。 */
    ret = bsp_iic_boot_address_probe();
    bsp_iic_txn_unlock();
    return ret;
}

ret_t bsp_iic_task_address_probe_noswitch(void)
{
    ret_t ret = bsp_iic_txn_lock();
    if (ret != RET_OK)
    {
        return ret;
    }
    /* 先取锁再挂起调度；内部不允许阻塞式 RTOS 调用。
     * 硬件等待有界，中断保持开启，退出必须恢复调度与锁。 */
    vTaskSuspendAll();
    ret = bsp_iic_boot_address_probe();
    (void)xTaskResumeAll();
    bsp_iic_txn_unlock();
    return ret;
}

static ret_t bsp_i2c_gpio_init_impl(void)
{
    if (s_bsp_i2c_active == 0)
    {
        return RET_INVALID_STATE;
    }

    rcu_periph_clock_enable(s_bsp_i2c_active->gpio_rcu);

    if (s_bsp_i2c_active->mode == BSP_IIC_MODE_SOFT)
    {
        gpio_init(s_bsp_i2c_active->gpio_port,
                  GPIO_MODE_OUT_OD,
                  GPIO_OSPEED_10MHZ,
                  s_bsp_i2c_active->scl_pin | s_bsp_i2c_active->sda_pin);
        SOFT_I2C_SCL_HIGH();
        SOFT_I2C_SDA_HIGH();
        return RET_OK;
    }

    if (s_bsp_i2c_active->mode == BSP_IIC_MODE_HARD)
    {
        hard_i2c_gpio_to_af_od();
        return RET_OK;
    }

    return RET_INVALID_PARAM;
}

/**
 * @brief   I2C 初始化实现（内部）：时钟、GPIO、模式和总线状态检查
 * @retval  RET_OK 成功
 */
static ret_t bsp_i2c_init_impl(void)
{
    ret_t ret;
    uint8_t scl_level;
    uint8_t sda_level;

    if (s_bsp_i2c_active == 0)
    {
        return RET_INVALID_STATE;
    }

    ret = bsp_delay_init();
    if (ret != RET_OK)
    {
        return ret;
    }

    IIC_LOG("bsp_iic init, mode=%s, periph=0x%08lx, port=0x%08lx, scl=%lu, sda=%lu",
            (s_bsp_i2c_active->mode == BSP_IIC_MODE_SOFT) ? "soft" : "hard",
            s_bsp_i2c_active->periph,
            s_bsp_i2c_active->gpio_port,
            s_bsp_i2c_active->scl_pin,
            s_bsp_i2c_active->sda_pin);

    ret = bsp_i2c_gpio_init_impl();
    if (ret != RET_OK)
    {
        return ret;
    }

    if (s_bsp_i2c_active->mode == BSP_IIC_MODE_SOFT)
    {
        /* 开漏释放后由外部上拉建立高电平，先等待一个半周期再采样。 */
        (void)bsp_delay_us(s_bsp_i2c_active->bit_delay_us);

        scl_level = (SOFT_I2C_SCL_READ() == SET) ? 1U : 0U;
        sda_level = (SOFT_I2C_SDA_READ() == SET) ? 1U : 0U;
        IIC_LOG("bsp_iic idle level, scl=%u, sda=%u\r\n",
                (unsigned int)scl_level,
                (unsigned int)sda_level);

        if ((scl_level == 0U) || (sda_level == 0U))
        {
            return RET_BUS_ERROR;
        }
    }
    else if (s_bsp_i2c_active->mode == BSP_IIC_MODE_HARD)
    {
        rcu_periph_clock_enable(s_bsp_i2c_active->i2c_rcu);
        ret = hard_i2c_recover_bus_lines();
        if (ret != RET_OK)
        {
            DBG_Warning("hard iic init line recovery FAIL ret=%ld", (long)ret);
            return ret;
        }
        hard_i2c_periph_init();
        DBG_log("hard iic fix-v3-pincheck init pclk1=%lu ckcfg=0x%04X rt=%u scl=%u sda=%u",
                (unsigned long)rcu_clock_freq_get(CK_APB1),
                (unsigned int)I2C_CKCFG(s_bsp_i2c_active->periph),
                (unsigned int)I2C_RT(s_bsp_i2c_active->periph),
                (unsigned int)SOFT_I2C_SCL_READ(), (unsigned int)SOFT_I2C_SDA_READ());
        s_hard_i2c_phase = "initialized-before-start";
        hard_i2c_dump();
    }
    else
    {
        return RET_INVALID_PARAM;
    }

    return RET_OK;
}

/** 写实现分发（内部）：按当前总线实例 mode 选择软/硬 I2C */
static ret_t bsp_i2c_mem_write_impl(uint16_t slave_addr,
                                    uint16_t mem_addr,
                                    uint8_t mem_addr_size,
                                    const uint8_t *buffer,
                                    uint16_t len)
{
    if (s_bsp_i2c_active == 0)
    {
        return RET_INVALID_STATE;
    }

    if (s_bsp_i2c_active->mode == BSP_IIC_MODE_SOFT)
    {
        return soft_i2c_mem_write_bytes(slave_addr, mem_addr, mem_addr_size, buffer, len);
    }

    if (s_bsp_i2c_active->mode == BSP_IIC_MODE_HARD)
    {
        return hard_i2c_mem_write_bytes(slave_addr, mem_addr, mem_addr_size, buffer, len);
    }

    return RET_INVALID_PARAM;
}

/** 读实现分发（内部）：按当前总线实例 mode 选择软/硬 I2C */
static ret_t bsp_i2c_mem_read_impl(uint16_t slave_addr,
                                   uint16_t mem_addr,
                                   uint8_t mem_addr_size,
                                   uint8_t *buffer,
                                   uint16_t len)
{
    if (s_bsp_i2c_active == 0)
    {
        return RET_INVALID_STATE;
    }

    if (s_bsp_i2c_active->mode == BSP_IIC_MODE_SOFT)
    {
        return soft_i2c_mem_read_bytes(slave_addr, mem_addr, mem_addr_size, buffer, len);
    }

    if (s_bsp_i2c_active->mode == BSP_IIC_MODE_HARD)
    {
        return hard_i2c_mem_read_bytes(slave_addr, mem_addr, mem_addr_size, buffer, len);
    }

    return RET_INVALID_PARAM;
}

/** 选择总线实例；原语调用前必须先选择有效实例。
 * @note 全文件共用一个 s_bsp_i2c_active 全局指针做“当前总线”，
 *       好处是软/硬原语不用层层传参；代价是非可重入——
 *       两个任务若同时操作不同总线，后选中的会覆盖先选中的。
 *       因此多任务并发用 I2C 时，上层必须加互斥锁串行化。 */
static ret_t bsp_iic_select_bus(bsp_iic_bus_id_t bus_id)
{
    if ((uint32_t)bus_id >= (uint32_t)BSP_IIC_BUS_COUNT)
    {
        return RET_INVALID_PARAM;
    }

    s_bsp_i2c_active = &s_bsp_i2c_bus_table[bus_id];
    return RET_OK;
}

/** 选择并检查总线初始化状态。
 * @note 每次公开读写都先走这里：选中总线 + 确认已初始化，
 *       未初始化直接拒掉，避免引脚没配就发波形。 */
static ret_t bsp_iic_bus_prepare(bsp_iic_bus_id_t bus_id)
{
    ret_t ret;

    ret = bsp_iic_select_bus(bus_id);
    if (ret != RET_OK)
    {
        return ret;
    }

    if (s_bsp_i2c_active->initialized == 0U)
    {
        return RET_NOT_INIT;
    }

    return RET_OK;
}

/************************************************************
 * @brief   仅初始化指定软件 I²C 总线的 GPIO 引脚
 ************************************************************/
ret_t bsp_iic_bus_gpio_init(bsp_iic_bus_id_t bus_id)
{
    ret_t ret;

    /* select_bus同样改写全局指针，与其他事务互斥。 */
    ret = bsp_iic_txn_lock();
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_iic_select_bus(bus_id);
    if (ret == RET_OK)
    {
        if (s_bsp_i2c_active->initialized != 0U)
        {
            ret = RET_ALREADY_INIT;
        }
        else
        {
            ret = bsp_delay_init();
            if (ret == RET_OK)
            {
                ret = bsp_i2c_gpio_init_impl();
            }
        }
    }

    bsp_iic_txn_unlock();
    return ret;
}

/************************************************************
 * @brief   初始化指定 I2C 总线（含延时模块初始化）
 * @retval  RET_OK 成功; RET_ALREADY_INIT 已初始化
 ************************************************************/
ret_t bsp_iic_bus_init(bsp_iic_bus_id_t bus_id)
{
    ret_t ret;

    /* 初始化也会改写全局活动总线指针，同样必须持锁。 */
    ret = bsp_iic_txn_lock();
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_iic_select_bus(bus_id);
    if (ret == RET_OK)
    {
        if (s_bsp_i2c_active->initialized != 0U)
        {
            ret = RET_ALREADY_INIT;
        }
        else
        {
            ret = bsp_i2c_init_impl();
            if (ret == RET_OK)
            {
                s_bsp_i2c_active->initialized = 1U;
            }
        }
    }

    bsp_iic_txn_unlock();
    return ret;
}

/************************************************************
 * @brief   使用指定总线向从机"寄存器地址"连续写入数据
 ************************************************************/
ret_t bsp_iic_bus_write(bsp_iic_bus_id_t bus_id,
                        uint16_t slave_addr,
                        uint16_t mem_addr,
                        uint8_t mem_addr_size,
                        const uint8_t *buffer,
                        uint16_t len)
{
    ret_t ret;

    /* 纯参数检查不碰硬件，放在锁外，避免无效参数占用事务锁。 */
    if ((buffer == NULL) || (len == 0U))
    {
        return RET_INVALID_PARAM;
    }

    if ((mem_addr_size != BSP_IIC_MEM_ADDR_8BIT) &&
        (mem_addr_size != BSP_IIC_MEM_ADDR_16BIT))
    {
        return RET_INVALID_PARAM;
    }

    IIC_LOG("bsp_iic_bus_write bus=%u slave=0x%02x mem=0x%04x addr_size=%u len=%u",
            (unsigned int)bus_id,
            (unsigned int)slave_addr,
            (unsigned int)mem_addr,
            (unsigned int)mem_addr_size,
            (unsigned int)len);

    /* 从select_bus翻指针到事务完成必须全程持锁：
     * 中途被其他总线的任务插入会偷换全局上下文（板测实证）。 */
    ret = bsp_iic_txn_lock();
    if (ret == RET_OK)
    {
        ret = bsp_iic_bus_prepare(bus_id);
        if (ret == RET_OK)
        {
            ret = bsp_i2c_mem_write_impl(slave_addr, mem_addr, mem_addr_size,
                                         buffer, len);
        }

        bsp_iic_txn_unlock();
    }

    return ret;
}

/************************************************************
 * @brief   使用指定总线从从机"寄存器地址"连续读取数据
 ************************************************************/
ret_t bsp_iic_bus_read(bsp_iic_bus_id_t bus_id,
                       uint16_t slave_addr,
                       uint16_t mem_addr,
                       uint8_t mem_addr_size,
                       uint8_t *buffer,
                       uint16_t len)
{
    ret_t ret;

    /* 纯参数检查不碰硬件，放在锁外，避免无效参数占用事务锁。 */
    if ((buffer == NULL) || (len == 0U))
    {
        return RET_INVALID_PARAM;
    }

    if ((mem_addr_size != BSP_IIC_MEM_ADDR_8BIT) &&
        (mem_addr_size != BSP_IIC_MEM_ADDR_16BIT))
    {
        return RET_INVALID_PARAM;
    }

    IIC_LOG("bsp_iic_bus_read bus=%u slave=0x%02x mem=0x%04x addr_size=%u len=%u",
            (unsigned int)bus_id,
            (unsigned int)slave_addr,
            (unsigned int)mem_addr,
            (unsigned int)mem_addr_size,
            (unsigned int)len);

    /* 与bus_write同一把全局事务锁：读事务同样不允许被其他总线的
     * 任务中途插入（select_bus翻指针+超时值偷换，板测实证）。 */
    ret = bsp_iic_txn_lock();
    if (ret == RET_OK)
    {
        ret = bsp_iic_bus_prepare(bus_id);
        if (ret == RET_OK)
        {
            ret = bsp_i2c_mem_read_impl(slave_addr, mem_addr, mem_addr_size,
                                        buffer, len);
        }

        bsp_iic_txn_unlock();
    }

    return ret;
}
