/**
 * @file    bsp_spi.c
 * @brief   SPI BSP 驱动实现（结构体表管理）
 *
 * @note    通过 spi_info_t 结构体表配置每路 SPI 的硬件参数，
 *          统一在 bsp_spi_init() 中完成时钟使能、GPIO 配置、SPI 参数设置。
 */

#include "./bsp_spi/bsp_spi.h"
#include "gd32f30x.h"

#define BSP_SPI_NUM    BSP_SPI_MAX

/**
 * @brief   SPI 硬件信息结构体
 */
typedef struct
{
    rcu_periph_enum rcu_spi;    /**< SPI 外设时钟 */
    uint32_t spi_port;          /**< SPI 端口基址 */
    rcu_periph_enum rcu_gpio;   /**< GPIO 时钟 */
    uint32_t gpio_port;         /**< GPIO 端口基址 */
    uint32_t sck_pin;           /**< SCK 引脚 */
    uint32_t miso_pin;          /**< MISO 引脚 */
    uint32_t mosi_pin;          /**< MOSI 引脚 */
    uint32_t prescale;          /**< 波特率预分频 */
    uint32_t cpol_cpha;         /**< 时钟极性/相位 */
} spi_info_t;

/** SPI 硬件信息表 */
static const spi_info_t g_spi_info_table[BSP_SPI_MAX] =
{
    [BSP_SPI_FLASH] = {
        .rcu_spi   = RCU_SPI1,
        .spi_port  = SPI1,
        .rcu_gpio  = RCU_GPIOB,
        .gpio_port = GPIOB,
        .sck_pin   = GPIO_PIN_13,
        .miso_pin  = GPIO_PIN_14,
        .mosi_pin  = GPIO_PIN_15,
        .prescale  = SPI_PSC_8,
        .cpol_cpha = SPI_CK_PL_LOW_PH_1EDGE,
    },
};

/************************************************************
 * @brief   初始化指定 SPI 外设
 * @param   spi_id  SPI 设备 ID
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t bsp_spi_init(bsp_spi_id_t spi_id)
{
    const spi_info_t *spi_info;
    spi_parameter_struct spi_init_struct;

    if (spi_id >= BSP_SPI_NUM)
    {
        return RET_INVALID_PARAM;
    }

    spi_info = &g_spi_info_table[spi_id];

    /* 1. 使能 GPIO 时钟 */
    rcu_periph_clock_enable(spi_info->rcu_gpio);

    /* 2. 使能 SPI 时钟 */
    rcu_periph_clock_enable(spi_info->rcu_spi);

    /* 3. 配置 SCK、MOSI 为复用推挽输出 */
    gpio_init(spi_info->gpio_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              spi_info->sck_pin | spi_info->mosi_pin);

    /* 4. 配置 MISO 为浮空输入 */
    gpio_init(spi_info->gpio_port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,
              spi_info->miso_pin);

    /* 5. 复位 SPI */
    spi_i2s_deinit(spi_info->spi_port);

    /* 6. 配置 SPI 参数：
     * 全双工主机、8 位帧、MSB 先行；
     * CPOL=0/CPHA=0（模式0）：空闲 SCK 为低，第一个边沿采样——
     * GD25Q128 上电默认支持模式0/3，这里选模式0；
     * NSS_SOFT：片选不用硬件 NSS 脚，改由 GPIO 手动控 CS
     * （一主多从时硬件 NSS 不够用，软件 CS 更灵活）。 */
    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = spi_info->cpol_cpha;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = spi_info->prescale;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;

    spi_init(spi_info->spi_port, &spi_init_struct);

    /* 7. 拉高内部 NSS：软件 NSS 模式下，若内部 NSS 为低，
     * 外设会误判“自己被选中做从机”并报模式错误（MODF），
     * 拉高即声明“我是主机”。 */
    spi_nss_internal_high(spi_info->spi_port);

    /* 8. 使能 SPI */
    spi_enable(spi_info->spi_port);

    return RET_OK;
}

 /************************************************************
 * @brief   SPI 全双工读写一个字节，并返回明确的错误码
 ************************************************************/
ret_t bsp_spi_read_write_byte_ex(bsp_spi_id_t spi_id,
                                 uint8_t tx_data,
                                 uint8_t *rx_data)
{
    const spi_info_t *spi_info;
    uint32_t timeout = 0xFFFFU;

    if (rx_data == NULL)
    {
        return RET_NULL_PTR;
    }

    if (spi_id >= BSP_SPI_NUM)
    {
        return RET_INVALID_PARAM;
    }

    spi_info = &g_spi_info_table[spi_id];

    /* 等发送缓冲空（TBE）：上一字节已进入移位寄存器，可以写新字节。
     * timeout 递减到 0 报超时——纯软件计数器式超时，精度随主频变，
     * 只防“SPI 没开时钟”之类的死等，不做精确计时。 */
    while (RESET == spi_i2s_flag_get(spi_info->spi_port, SPI_FLAG_TBE))
    {
        if (timeout-- == 0U)
        {
            return RET_TIMEOUT;
        }
    }

    /* 写数据寄存器即启动传输：8 个 SCK 内 MOSI 发出 tx_data，
     * 同时 MISO 移入 1 字节，全双工一次完成。 */
    spi_i2s_data_transmit(spi_info->spi_port, tx_data);

    timeout = 0xFFFFU;

    /* 等接收缓冲非空（RBNE）：8 个 SCK 走完，对方字节已锁存。
     * 两次等待都用独立计数器，第二个 timeout 必须重装。 */
    while (RESET == spi_i2s_flag_get(spi_info->spi_port, SPI_FLAG_RBNE))
    {
        if (timeout-- == 0U)
        {
            return RET_TIMEOUT;
        }
    }

    /* 读数据寄存器：拿到 MISO 字节，同时清掉 RBNE 标志 */
    *rx_data = (uint8_t)spi_i2s_data_receive(spi_info->spi_port);
    return RET_OK;
}

/************************************************************
 * @brief   SPI 全双工读写一个字节（兼容接口）
 * @param   spi_id   SPI 设备 ID
 * @param   tx_data  要发送的字节
 * @retval  接收到的字节（错误或超时返回 0xFF）
 ************************************************************/
uint8_t bsp_spi_read_write_byte(bsp_spi_id_t spi_id, uint8_t tx_data)
{
    uint8_t rx_data = 0xFFU;

    if (bsp_spi_read_write_byte_ex(spi_id, tx_data, &rx_data) != RET_OK)
    {
        return 0xFFU;
    }

    return rx_data;
}
