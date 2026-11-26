/**
 * @file    dev_at24c02.c
 * @brief   AT24C02 EEPROM Device 层实现（256字节，8字节页，磨损平衡参数区）
 *
 * @note    分层与不变量（与 dev_gd25q128 同一套路）：
 *          一个设备一把内部互斥量；公共 API 各自一次 Take/Give；
 *          BSP 原语和内部辅助不碰锁；锁句柄不出头文件。
 *          写路径：跨页自动拆分，每页写后固定延时6ms等内部写周期
 *          （不用ACK轮询，原因见 at24c02_write_page_core 注释）。
 *
 * 参数区磨损平衡设计（0x00~0x3F，共64字节，8槽×8字节=每槽一页）：
 *   槽布局：[0..3]=seq(u32小端) [4..5]=data(u16) [6..7]=CRC16(seq+data)
 *   写：轮换到"当前最大seq槽位的下一槽"整页写入，旧槽全部保留；
 *   读：扫全部槽，取CRC有效且seq最大者；全部无效=未格式化；
 *   掉电：单页写被中断最多损坏本槽CRC，上一槽完好，读端自动回退；
 *   寿命：每单元100万次，写压力按槽轮换均摊8个页，
 *         参数区总寿命约 8×100万=800万次参数写。
 */

#include "./dev_at24c02/dev_at24c02.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "../../BSP/bsp_iic/bsp_iic.h"

/** 设备7位地址0x50（A2A1A0=000）对应的8位写地址。 */
#define AT24C02_I2C_ADDR              (0xA0U)
/** 页写入内部写周期最大时间（AT24C02手册 tWC=5ms max）。 */
#define AT24C02_WRITE_CYCLE_MS        (5U)
/** 页写后固定保护延时：tWC最大5ms，10ms留一倍裕量。 */
#define AT24C02_WRITE_CYCLE_GUARD_MS  (10U)
/** 参数区起始地址（区域0x00~0x3F，其余0x40~0xFF留作普通存储）。 */
#define AT24C02_PARAM_BASE            (0x00U)
/** 参数槽大小=页大小，保证每槽一次整页事务。 */
#define AT24C02_PARAM_SLOT_SIZE       (DEV_AT24C02_PAGE_SIZE)
/** CRC16-CCITT 有效初值。 */
#define AT24C02_CRC16_INIT            (0xFFFFU)

/** Device 初始化状态：0=未初始化，1=已初始化。 */
static uint8_t s_dev_at24c02_inited = 0U;
/** Device 内部互斥量：句柄不出头文件，App 不直接操作。 */
static SemaphoreHandle_t s_dev_at24c02_mutex = NULL;
/** 等待设备互斥量的超时时间：有限等待，超时报告而不是无声挂死。 */
#define AT24C02_MUTEX_TIMEOUT_MS      (500U)

/* 内部函数声明：先声明，后定义。 */
static ret_t at24c02_write_page_core(uint16_t address,
                                     const uint8_t *buffer,
                                     uint8_t len);
static ret_t at24c02_write_core(uint16_t address,
                                const uint8_t *buffer,
                                uint16_t len);
static uint16_t at24c02_crc16(const uint8_t *data, uint16_t len);

/************************************************************
 * @brief   单页写核心（调用者必须已持锁，len不得超过页大小且不跨页）
 * @note    写后用固定延时等内部写周期，不用ACK轮询：
 *          tWC手册最大5ms，延时6ms留1ms裕量。
 *          原ACK轮询方案板测踩坑：轮询在芯片忙期发起的读会收到
 *          NACK/无数据，而当前BSP硬I2C错误路径没有完整的
 *          "STOP+等BUSY清零+清标志"收尾，残留的半截事务会让
 *          下一个事务START发不出去（SBSEND超时），跨页写必现。
 *          固定延时不产生任何NACK事务，从根上绕开该BSP缺陷。
 ************************************************************/
static ret_t at24c02_write_page_core(uint16_t address,
                                     const uint8_t *buffer,
                                     uint8_t len)
{
    ret_t ret;

    ret = bsp_iic_bus_write(BSP_IIC_BUS_AT24C02,
                            AT24C02_I2C_ADDR,
                            address,
                            BSP_IIC_MEM_ADDR_8BIT,
                            buffer,
                            len);
    if (ret != RET_OK)
    {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(AT24C02_WRITE_CYCLE_GUARD_MS));
    return RET_OK;
}

/************************************************************
 * @brief   任意长度写核心（调用者必须已持锁）：跨页自动拆分
 * @note    页写跨过页末会卷绕回本页开头覆盖旧数据，
 *          所以每次只写"本页剩余"部分，逐页推进。
 ************************************************************/
static ret_t at24c02_write_core(uint16_t address,
                                const uint8_t *buffer,
                                uint16_t len)
{
    ret_t ret;
    uint16_t offset;
    uint16_t page_offset;
    uint16_t page_remaining;
    uint8_t chunk_len;

    offset = 0U;
    while (offset < len)
    {
        page_offset = (uint16_t)((address + offset) % DEV_AT24C02_PAGE_SIZE);
        page_remaining = (uint16_t)(DEV_AT24C02_PAGE_SIZE - page_offset);
        chunk_len = (uint8_t)(len - offset);
        if (chunk_len > page_remaining)
        {
            chunk_len = (uint8_t)page_remaining;
        }

        ret = at24c02_write_page_core((uint16_t)(address + offset),
                                      buffer + offset,
                                      chunk_len);
        if (ret != RET_OK)
        {
            return ret;
        }

        offset += chunk_len;
    }

    return RET_OK;
}

/************************************************************
 * @brief   CRC16-CCITT（多项式0x1021，初值0xFFFF），查表省内存版
 ************************************************************/
static uint16_t at24c02_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc;
    uint16_t index;
    uint8_t bit;
    uint8_t byte;

    crc = AT24C02_CRC16_INIT;
    for (index = 0U; index < len; index++)
    {
        byte = data[index];
        crc ^= (uint16_t)((uint16_t)byte << 8U);
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}

/************************************************************
 * @brief   初始化：总线 + 器件探测 + 互斥量
 * @note    器件探测=读1字节：无ACK即报设备未找到，
 *          把缺片/接线问题挡在初始化阶段。
 *          此处不加锁：init 只在调度器启动前被调用（单线程上下文），
 *          且 RET_ALREADY_INIT 保证探测全生命周期仅此一次；
 *          若未来改为任务中动态 init，必须加锁。
 ************************************************************/
ret_t dev_at24c02_init(void)
{
    ret_t ret;
    uint8_t probe;

    if (s_dev_at24c02_inited != 0U)
    {
        return RET_ALREADY_INIT;
    }

    ret = bsp_iic_bus_init(BSP_IIC_BUS_AT24C02);
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        return ret;
    }

    /* 器件探测：读1字节验证地址帧有ACK。 */
    ret = bsp_iic_bus_read(BSP_IIC_BUS_AT24C02,
                           AT24C02_I2C_ADDR,
                           0U,
                           BSP_IIC_MEM_ADDR_8BIT,
                           &probe,
                           1U);
    if (ret != RET_OK)
    {
        DBG_log("dev_at24c02 init probe FAIL, ret=%ld\r\n", (long)ret);
        return ret; /* 保留 NACK / 超时 / 总线故障，不能统一伪装成缺片。 */
    }

    s_dev_at24c02_mutex = xSemaphoreCreateMutex();
    if (s_dev_at24c02_mutex == NULL)
    {
        return RET_NO_MEMORY;
    }

    s_dev_at24c02_inited = 1U;
    DBG_log("dev_at24c02 init ok, probe ACK, addr=0x%02X\r\n",
            (unsigned int)AT24C02_I2C_ADDR);
    return RET_OK;
}

/************************************************************
 * @brief   连续读：一次事务即可跨页（读无页边界限制）
 ************************************************************/
ret_t dev_at24c02_read(uint16_t address,
                       uint8_t *buffer,
                       uint16_t len)
{
    ret_t ret;

    if (s_dev_at24c02_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (buffer == NULL)
    {
        return RET_NULL_PTR;
    }

    if ((len == 0U) ||
        ((uint32_t)address >= DEV_AT24C02_TOTAL_SIZE) ||
        ((uint32_t)len > (DEV_AT24C02_TOTAL_SIZE - address)))
    {
        return RET_INVALID_PARAM;
    }

    if (xSemaphoreTake(s_dev_at24c02_mutex,
                       pdMS_TO_TICKS(AT24C02_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    ret = bsp_iic_bus_read(BSP_IIC_BUS_AT24C02,
                           AT24C02_I2C_ADDR,
                           address,
                           BSP_IIC_MEM_ADDR_8BIT,
                           buffer,
                           len);

    (void)xSemaphoreGive(s_dev_at24c02_mutex);
    return ret;
}

/************************************************************
 * @brief   任意长度写：跨页自动拆分 + 每页ACK轮询
 ************************************************************/
ret_t dev_at24c02_write(uint16_t address,
                        const uint8_t *buffer,
                        uint16_t len)
{
    ret_t ret;

    if (s_dev_at24c02_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (buffer == NULL)
    {
        return RET_NULL_PTR;
    }

    if ((len == 0U) ||
        ((uint32_t)address >= DEV_AT24C02_TOTAL_SIZE) ||
        ((uint32_t)len > (DEV_AT24C02_TOTAL_SIZE - address)))
    {
        return RET_INVALID_PARAM;
    }

    if (xSemaphoreTake(s_dev_at24c02_mutex,
                       pdMS_TO_TICKS(AT24C02_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    ret = at24c02_write_core(address, buffer, len);

    (void)xSemaphoreGive(s_dev_at24c02_mutex);
    return ret;
}

/************************************************************
 * @brief   参数区：把整个参数区格式化为初始状态
 * @note    不需要物理擦成0xFF：旧槽靠"seq最大者胜出"规则自然失效，
 *          只写0号槽一个基线seq=0即可。
 ************************************************************/
ret_t dev_at24c02_param_format(void)
{
    ret_t ret;
    uint8_t slot[AT24C02_PARAM_SLOT_SIZE];

    if (s_dev_at24c02_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_dev_at24c02_mutex,
                       pdMS_TO_TICKS(AT24C02_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        return RET_TIMEOUT;
    }

    slot[0] = 0U;
    slot[1] = 0U;
    slot[2] = 0U;
    slot[3] = 0U;
    slot[4] = 0U;
    slot[5] = 0U;
    slot[6] = (uint8_t)(at24c02_crc16(slot, 6U) >> 8U);
    slot[7] = (uint8_t)(at24c02_crc16(slot, 6U) & 0xFFU);

    ret = at24c02_write_page_core(AT24C02_PARAM_BASE,
                                  slot,
                                  AT24C02_PARAM_SLOT_SIZE);

    (void)xSemaphoreGive(s_dev_at24c02_mutex);
    return ret;
}

/************************************************************
 * @brief   参数区：写入一个参数值（磨损平衡：轮换到下一槽）
 ************************************************************/
ret_t dev_at24c02_param_write(uint16_t value)
{
    ret_t ret;
    uint8_t slot[AT24C02_PARAM_SLOT_SIZE];
    uint8_t scan[AT24C02_PARAM_SLOT_SIZE];
    uint32_t max_seq;
    uint8_t max_index;
    uint8_t have_valid;
    uint8_t index;
    uint32_t seq;
    uint16_t crc;
    uint8_t target;

    if (s_dev_at24c02_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_dev_at24c02_mutex,
                       pdMS_TO_TICKS(AT24C02_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        return RET_TIMEOUT;
    }

    /* 扫描：找CRC有效且seq最大的槽，确定轮换目标和下一序号。 */
    max_seq = 0U;
    max_index = 0U;
    have_valid = 0U;
    for (index = 0U; index < DEV_AT24C02_PARAM_SLOTS; index++)
    {
        ret = bsp_iic_bus_read(BSP_IIC_BUS_AT24C02,
                               AT24C02_I2C_ADDR,
                               (uint16_t)(AT24C02_PARAM_BASE +
                                          (uint16_t)(index * AT24C02_PARAM_SLOT_SIZE)),
                               BSP_IIC_MEM_ADDR_8BIT,
                               scan,
                               AT24C02_PARAM_SLOT_SIZE);
        if (ret != RET_OK)
        {
            (void)xSemaphoreGive(s_dev_at24c02_mutex);
            return ret;
        }

        crc = at24c02_crc16(scan, 6U);
        if ((crc != (uint16_t)((uint16_t)scan[6] << 8U | scan[7])) ||
            ((scan[0] == 0xFFU) && (scan[1] == 0xFFU) &&
             (scan[2] == 0xFFU) && (scan[3] == 0xFFU)))
        {
            /* CRC无效或全FF的空白槽：跳过。 */
            continue;
        }

        seq = (uint32_t)scan[0] | ((uint32_t)scan[1] << 8U) |
              ((uint32_t)scan[2] << 16U) | ((uint32_t)scan[3] << 24U);
        if ((have_valid == 0U) || (seq > max_seq))
        {
            max_seq = seq;
            max_index = index;
            have_valid = 1U;
        }
    }

    /* 区从未格式化或全部损坏：从0号槽写基线seq=0。 */
    if (have_valid == 0U)
    {
        max_seq = 0U;
        max_index = DEV_AT24C02_PARAM_SLOTS - 1U;
    }

    target = (uint8_t)((max_index + 1U) % DEV_AT24C02_PARAM_SLOTS);

    slot[0] = (uint8_t)((max_seq + 1U) & 0xFFU);
    slot[1] = (uint8_t)(((max_seq + 1U) >> 8U) & 0xFFU);
    slot[2] = (uint8_t)(((max_seq + 1U) >> 16U) & 0xFFU);
    slot[3] = (uint8_t)(((max_seq + 1U) >> 24U) & 0xFFU);
    slot[4] = (uint8_t)(value & 0xFFU);
    slot[5] = (uint8_t)((value >> 8U) & 0xFFU);
    crc = at24c02_crc16(slot, 6U);
    slot[6] = (uint8_t)(crc >> 8U);
    slot[7] = (uint8_t)(crc & 0xFFU);

    ret = at24c02_write_page_core(
        (uint16_t)(AT24C02_PARAM_BASE + (uint16_t)(target * AT24C02_PARAM_SLOT_SIZE)),
        slot,
        AT24C02_PARAM_SLOT_SIZE);

    (void)xSemaphoreGive(s_dev_at24c02_mutex);
    return ret;
}

/************************************************************
 * @brief   参数区：读取最新参数值（CRC有效且seq最大的槽）
 * @retval  RET_OK 读到; RET_EMPTY 无有效槽（未格式化/全损坏）
 ************************************************************/
ret_t dev_at24c02_param_read(uint16_t *value)
{
    ret_t ret;
    uint8_t scan[AT24C02_PARAM_SLOT_SIZE];
    uint32_t max_seq;
    uint16_t best_value;
    uint8_t have_valid;
    uint8_t index;
    uint32_t seq;
    uint16_t crc;

    if (s_dev_at24c02_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (value == NULL)
    {
        return RET_NULL_PTR;
    }

    if (xSemaphoreTake(s_dev_at24c02_mutex,
                       pdMS_TO_TICKS(AT24C02_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        return RET_TIMEOUT;
    }

    max_seq = 0U;
    best_value = 0U;
    have_valid = 0U;
    for (index = 0U; index < DEV_AT24C02_PARAM_SLOTS; index++)
    {
        ret = bsp_iic_bus_read(BSP_IIC_BUS_AT24C02,
                               AT24C02_I2C_ADDR,
                               (uint16_t)(AT24C02_PARAM_BASE +
                                          (uint16_t)(index * AT24C02_PARAM_SLOT_SIZE)),
                               BSP_IIC_MEM_ADDR_8BIT,
                               scan,
                               AT24C02_PARAM_SLOT_SIZE);
        if (ret != RET_OK)
        {
            (void)xSemaphoreGive(s_dev_at24c02_mutex);
            return ret;
        }

        crc = at24c02_crc16(scan, 6U);
        if ((crc != (uint16_t)((uint16_t)scan[6] << 8U | scan[7])) ||
            ((scan[0] == 0xFFU) && (scan[1] == 0xFFU) &&
             (scan[2] == 0xFFU) && (scan[3] == 0xFFU)))
        {
            continue;
        }

        seq = (uint32_t)scan[0] | ((uint32_t)scan[1] << 8U) |
              ((uint32_t)scan[2] << 16U) | ((uint32_t)scan[3] << 24U);
        if ((have_valid == 0U) || (seq > max_seq))
        {
            max_seq = seq;
            best_value = (uint16_t)((uint16_t)scan[4] | ((uint16_t)scan[5] << 8U));
            have_valid = 1U;
        }
    }

    (void)xSemaphoreGive(s_dev_at24c02_mutex);

    if (have_valid == 0U)
    {
        return RET_EMPTY;
    }

    *value = best_value;
    return RET_OK;
}
