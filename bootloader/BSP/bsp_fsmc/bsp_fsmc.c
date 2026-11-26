/**
 * @file    bsp_fsmc.c
 * @brief   外部存储器控制器（EXMC/FSMC）BSP 驱动实现
 *
 * @note    通过 bsp_fsmc_t 结构体表保存每个外设的 EXMC 配置，
 *          init() 时先初始化复用 GPIO 与 EXMC 时钟，再按表配置。
 */

#include "bsp_fsmc/bsp_fsmc.h"
#include "bsp_gpio/bsp_gpio.h"
#include "gd32f30x.h"

/**
 * @brief   EXMC 外设配置结构体
 */
typedef struct
{
    uint8_t enabled;                            /**< 外设是否使能（1=使能） */
    exmc_norsram_parameter_struct fsmc_init;    /**< EXMC 初始化参数 */
    exmc_norsram_timing_parameter_struct fsmc_timing;   /**< EXMC 时序参数 */
    exmc_norsram_timing_parameter_struct fsmc_write_timing;
} bsp_fsmc_t;

/** EXMC 外设配置表（按 bsp_fsmc_id_t 索引） */
static bsp_fsmc_t s_bsp_fsmc[BSP_FSMC_MAX] =
{
    [BSP_SRAM] =
    {
        .enabled = 1U,
        .fsmc_init =
        {
            .norsram_region    = EXMC_BANK0_NORSRAM_REGION3,
            .address_data_mux  = DISABLE,
            .memory_type       = EXMC_MEMORY_TYPE_SRAM,
            .databus_width     = EXMC_NOR_DATABUS_WIDTH_16B,
            .burst_mode        = DISABLE,
            .nwait_config      = EXMC_NWAIT_CONFIG_BEFORE,
            .nwait_polarity    = EXMC_NWAIT_POLARITY_LOW,
            .wrap_burst_mode   = DISABLE,
            .asyn_wait         = DISABLE,
            .extended_mode     = DISABLE,
            .memory_write      = ENABLE,
            .nwait_signal      = DISABLE,
            .write_mode        = EXMC_ASYN_WRITE,
            .read_write_timing = NULL,
            .write_timing      = NULL,
        },
        .fsmc_timing =
        {
            .asyn_access_mode       = EXMC_ACCESS_MODE_A,
            .asyn_address_setuptime = 1U,   /* -1 → 0（传 0 会下溢成最大值，见 BSP_LCD 注释） */
            .asyn_address_holdtime  = 1U,   /* -1 → 0 */
            .asyn_data_setuptime    = 1U,   /* -1 → 0；SRAM 若时序不足可调大 */
            .bus_latency            = 1U,   /* -1 → 0 */
            .syn_clk_division       = 0U,
            .syn_data_latency       = 0U,
        },
    },

    /* External LCD：区域1（基址 0x64000000，与 dev_lcd 的总线地址对应）、
     * 16 位总线、读写分离时序。
     * extended_mode 的意义：读时序 data_setuptime=15（读慢，ILI9488
     * 读数据需要长建立时间），写时序 data_setuptime=2（写快）。
     * 若读写共用慢时序，刷屏帧率直接掉一半。 */
    [BSP_LCD] =
    {
        .enabled = 1U,
        .fsmc_init =
        {
            .norsram_region    = EXMC_BANK0_NORSRAM_REGION1,
            .address_data_mux  = DISABLE,
            .memory_type       = EXMC_MEMORY_TYPE_SRAM,
            .databus_width     = EXMC_NOR_DATABUS_WIDTH_16B,
            .burst_mode        = DISABLE,
            .nwait_config      = EXMC_NWAIT_CONFIG_BEFORE,
            .nwait_polarity    = EXMC_NWAIT_POLARITY_LOW,
            .wrap_burst_mode   = DISABLE,
            .asyn_wait         = DISABLE,
            .extended_mode     = ENABLE,
            .memory_write      = ENABLE,
            .nwait_signal      = DISABLE,
            .write_mode        = EXMC_ASYN_WRITE,
            .read_write_timing = NULL,
            .write_timing      = NULL,
        },
        /*
         * ⚠️ 时序参数最小值是 1，不是 0！
         * GD32 库 gd32f30x_exmc.c 的 exmc_norsram_init() 对每个时序值
         * 都做 (值 - 1) 再写寄存器：
         *     sntcfg = ((asyn_address_setuptime - 1U) & EXMC_SNTCFG_ASET) | ...
         * 传 0 会下溢成 0xFFFFFFFF，与掩码 & 之后得到"掩码全 1"，
         * 把该字段直接顶到最大值（ASET/AHLD 变成 15 个 HCLK，是意图的 16 倍）。
         * 后果：单次外部总线访问耗时被放大，连续刷屏（15 万次写）时
         * 触发总线错误 IMPRECISERR → HardFault。
         * 传 1 才是"寄存器写 0 = 最短时序"。
         */
        .fsmc_timing =
        {
            .asyn_access_mode       = EXMC_ACCESS_MODE_A,
            .asyn_address_setuptime = 1U,   /* -1 → 0，最短地址建立 */
            .asyn_address_holdtime  = 1U,   /* -1 → 0，最短地址保持 */
            .asyn_data_setuptime    = 15U,  /* -1 → 14，读数据慢建立 */
            .bus_latency            = 1U,   /* 异步模式不参与，避免下溢成 0xF */
            .syn_clk_division       = 0U,   /* 同步模式未用 */
            .syn_data_latency       = 0U,
        },
        .fsmc_write_timing =
        {
            .asyn_access_mode       = EXMC_ACCESS_MODE_A,
            .asyn_address_setuptime = 1U,   /* -1 → 0 */
            .asyn_address_holdtime  = 1U,   /* -1 → 0 */
            .asyn_data_setuptime    = 3U,   /* -1 → 2，写数据快 */
            .bus_latency            = 1U,
            .syn_clk_division       = 0U,
            .syn_data_latency       = 0U,
        },
    },
};

/************************************************************
 * @brief   按配置表初始化指定 EXMC 外设（内部）
 * @param   id  外设 ID
 * @retval  RET_OK 成功; RET_INVALID_PARAM / RET_NOT_SUPPORT
 ************************************************************/
static ret_t fsmc_init(bsp_fsmc_id_t id)
{
    bsp_fsmc_t *fsmc_info;

    if (id >= BSP_FSMC_MAX)
    {
        return RET_INVALID_PARAM;
    }

    if (s_bsp_fsmc[id].enabled == 0U)
    {
        return RET_NOT_SUPPORT;
    }

    fsmc_info = &s_bsp_fsmc[id];
    /* 时序指针回填：静态表里不能写“指向自己成员”的指针（编译期地址未定），
     * 所以运行时把 read_write_timing 指向本行时序；扩展模式再挂写时序。 */
    fsmc_info->fsmc_init.read_write_timing = &fsmc_info->fsmc_timing;

    if (fsmc_info->fsmc_init.extended_mode == ENABLE)
    {
        fsmc_info->fsmc_init.write_timing = &fsmc_info->fsmc_write_timing;
    }

    exmc_norsram_init(&fsmc_info->fsmc_init);
    exmc_norsram_enable(fsmc_info->fsmc_init.norsram_region);

    return RET_OK;
}

/************************************************************
 * @brief   初始化指定 EXMC 外设
 * @param   id  外设 ID（BSP_SRAM / BSP_LCD）
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_SUPPORT 该外设未使能
 * @note    依次完成：复用 GPIO 初始化 -> EXMC 时钟使能 -> 外设配置
 ************************************************************/
ret_t bsp_fsmc_init(bsp_fsmc_id_t id)
{
    ret_t ret;

    if (id >= BSP_FSMC_MAX)
    {
        return RET_INVALID_PARAM;
    }

    if (s_bsp_fsmc[id].enabled == 0U)
    {
        return RET_NOT_SUPPORT;
    }

    ret = bsp_gpio_fsmc_init();
    if (ret != RET_OK)
    {
        return ret;
    }

    rcu_periph_clock_enable(RCU_EXMC);

    return fsmc_init(id);
}
