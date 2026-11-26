/**
 * @file    dev_gd25q128.c
 * @brief   GD25Q128 外部 SPI NOR Flash Device 层接口
 *
 * @note    当前完成 SPI/CS 初始化、JEDEC ID、状态读取、写使能验证、连续
 *          读取、单页/跨页写入和扇区/块/整片擦除；一个设备一把互斥量，
 *          锁由 Device 层公共 API 统一 Take/Give，内部辅助与 BSP 不碰锁。
 */

#include "./dev_gd25q128/dev_gd25q128.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "../../BSP/bsp_gpio/bsp_gpio.h"
#include "../../BSP/bsp_spi/bsp_spi.h"

/** JEDEC ID 读取命令。 */
#define GD25Q128_CMD_JEDEC_ID       (0x9FU)
/** 状态寄存器读取命令。 */
#define GD25Q128_CMD_READ_STATUS    (0x05U)
/** 写使能命令。 */
#define GD25Q128_CMD_WRITE_ENABLE   (0x06U)
/** Page Program 命令。 */
#define GD25Q128_CMD_PAGE_PROGRAM    (0x02U)
/** 读取数据命令。 */
#define GD25Q128_CMD_READ_DATA       (0x03U)
/** 4KB 扇区擦除命令。 */
#define GD25Q128_CMD_SECTOR_ERASE    (0x20U)
/** 64KB 块擦除命令。 */
#define GD25Q128_CMD_BLOCK_ERASE     (0xD8U)
/** 整片擦除命令。 */
#define GD25Q128_CMD_CHIP_ERASE      (0xC7U)
/** 状态寄存器 BUSY/WIP 位。 */
#define GD25Q128_STATUS_BUSY_MASK    (0x01U)
/** 状态寄存器 WEL 位。 */
#define GD25Q128_STATUS_WEL_MASK     (0x02U)
/** SPI 读操作使用的空字节；发送它才能产生 SCK。 */
#define GD25Q128_SPI_DUMMY_BYTE     (0xFFU)
/** 普通读写操作等待 BUSY 清零的任务级超时时间。 */
#define GD25Q128_PAGE_PROGRAM_TIMEOUT_MS   (100U)
/** 4KB 扇区擦除的任务级超时时间。 */
#define GD25Q128_SECTOR_ERASE_TIMEOUT_MS   (2000U)
/** 64KB 块擦除的任务级超时时间。 */
#define GD25Q128_BLOCK_ERASE_TIMEOUT_MS    (10000U)
/** 整片擦除的任务级超时时间。 */
#define GD25Q128_CHIP_ERASE_TIMEOUT_MS     (120000U)
/** BUSY 轮询之间让出 CPU 的时间。 */
#define GD25Q128_STATUS_POLL_DELAY_MS      (1U)
/** 等待设备互斥量的超时时间：有限等待，超时报告而不是无声挂死。 */
#define GD25Q128_MUTEX_TIMEOUT_MS          (500U)

/* 内部函数声明：先声明，后定义。 */
static ret_t gd25q128_jedec_id_read(dev_gd25q128_jedec_id_t *id);
static ret_t gd25q128_wait_ready(uint32_t timeout_ms);
static ret_t gd25q128_spi_recover(void);
static ret_t gd25q128_send_command(uint8_t command);
static ret_t gd25q128_send_command_address(uint8_t command,
                                           uint32_t address);
static ret_t gd25q128_prepare_write(uint32_t timeout_ms);
static ret_t gd25q128_erase_with_address(uint8_t command,
                                         uint32_t address,
                                         uint32_t timeout_ms,
                                         uint32_t alignment);
static ret_t gd25q128_erase_without_address(uint8_t command,
                                            uint32_t timeout_ms);
static ret_t gd25q128_page_program_core(uint32_t address,
                                        const uint8_t *buffer,
                                        uint16_t len);

/** Device 初始化状态：0=未初始化，1=已初始化。 */
static uint8_t s_dev_gd25q128_inited = 0U;
/** Device 创建一个互斥锁。 */
static SemaphoreHandle_t s_dev_gd25q128_mutex = NULL;
/** JEDEC ID 期望值：厂商 0xC8(GigaDevice) 类型 0x40(SPI NOR) 容量 0x18(2^24=16MB)。 */
#define GD25Q128_JEDEC_MANUFACTURER   (0xC8U)
#define GD25Q128_JEDEC_MEMORY_TYPE    (0x40U)
#define GD25Q128_JEDEC_CAPACITY       (0x18U)
/************************************************************
 * @brief   初始化 GD25Q128 Device 框架
 ************************************************************/
ret_t dev_gd25q128_init(void)
{
    ret_t ret;
    dev_gd25q128_jedec_id_t id;

    if (s_dev_gd25q128_inited != 0U)
    {
        return RET_ALREADY_INIT;
    }

    /* CS 是低有效：先配置为输出并拉高，确保 SPI 初始化期间处于空闲。 */
    ret = bsp_gpio_config_output(BSP_GPIO_SPI_FLASH_CS);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_spi_init(BSP_SPI_FLASH);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* JEDEC ID 校验：开机即确认"芯片在、是这块芯片"，把缺片/接线/
     * 错型号问题挡在初始化阶段，而不是推迟到首次读写才暴露。
     * 此处不加锁：init 只在调度器启动前被调用（单线程上下文），
     * 且 RET_ALREADY_INIT 保证 ID 校验全生命周期仅此一次；
     * 若未来改为任务中动态 init，必须改走加锁的公共 API。 */
    ret = gd25q128_jedec_id_read(&id);
    if (ret != RET_OK)
    {
        DBG_log("dev_gd25q128 init JEDEC read FAIL, ret=%ld\r\n", (long)ret);
        return ret;
    }

    if ((id.manufacturer != GD25Q128_JEDEC_MANUFACTURER) ||
        (id.memory_type != GD25Q128_JEDEC_MEMORY_TYPE) ||
        (id.capacity != GD25Q128_JEDEC_CAPACITY))
    {
        DBG_log("dev_gd25q128 init JEDEC check FAIL, "
                "expect=%02X %02X %02X actual=%02X %02X %02X\r\n",
                (unsigned int)GD25Q128_JEDEC_MANUFACTURER,
                (unsigned int)GD25Q128_JEDEC_MEMORY_TYPE,
                (unsigned int)GD25Q128_JEDEC_CAPACITY,
                (unsigned int)id.manufacturer,
                (unsigned int)id.memory_type,
                (unsigned int)id.capacity);
        return RET_DEVICE_NOT_FOUND;
    }

    s_dev_gd25q128_mutex = xSemaphoreCreateMutex();
    if (s_dev_gd25q128_mutex == NULL)
    {
        /* 堆不足创建失败：不允许标记初始化成功，
         * 否则后续公共 API 会对 NULL 句柄 Take。 */
        return RET_NO_MEMORY;
    }

    s_dev_gd25q128_inited = 1U;
    DBG_log("dev_gd25q128 init ok, JEDEC ID=%02X %02X %02X, CS idle=high\r\n",
            (unsigned int)id.manufacturer,
            (unsigned int)id.memory_type,
            (unsigned int)id.capacity);
    return RET_OK;
}

/************************************************************
 * @brief   获取 JEDEC ID 框架接口
 ************************************************************/
ret_t dev_gd25q128_jedec_id_get(dev_gd25q128_jedec_id_t *id)
{
    ret_t ret;

    if (id == NULL)
    {
        return RET_NULL_PTR;
    }

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    /* 读 ID 也是一次完整的 CS 事务，与其他公共 API 竞争同一把设备锁。 */
    if (xSemaphoreTake(s_dev_gd25q128_mutex,
                       pdMS_TO_TICKS(GD25Q128_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    ret = gd25q128_jedec_id_read(id);

    (void)xSemaphoreGive(s_dev_gd25q128_mutex);
    return ret;
}

/************************************************************
 * @brief   读取三字节 JEDEC ID
 * @note    一次事务：CS低 -> 0x9F -> 3个0xFF -> CS高。
 ************************************************************/
static ret_t gd25q128_jedec_id_read(dev_gd25q128_jedec_id_t *id)
{
    ret_t ret;
    ret_t cs_ret;
    uint8_t rx_data;

    if (id == NULL)
    {
        return RET_NULL_PTR;
    }

    /* 拉低 CS 选中 Flash：SPI 总线可挂多个从机，
     * CS 低的那一路才响应总线，一次事务中 CS 必须全程保持低。 */
    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 发命令 0x9F：返回值 rx_data 是命令字节同时移回的废数据，直接丢弃。
     * SPI 是全双工：每发 1 字节必收 1 字节，不存在“只发不收”。 */
    ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                     GD25Q128_CMD_JEDEC_ID,
                                     &rx_data);
    /* 发空字节 0xFF 产生 8 个 SCK：Flash 借这 8 个时钟把
     * 第 1 个 ID 字节（厂商）送到 MISO 线上，主机采样存入 manufacturer。
     * 后两字节同理：一次事务共 1 命令 + 3 空字节 = 4 字节。 */
    if (ret == RET_OK)
    {
        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         GD25Q128_SPI_DUMMY_BYTE,
                                         &id->manufacturer);
    }
    if (ret == RET_OK)
    {
        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         GD25Q128_SPI_DUMMY_BYTE,
                                         &id->memory_type);
    }
    if (ret == RET_OK)
    {
        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         GD25Q128_SPI_DUMMY_BYTE,
                                         &id->capacity);
    }

    /* 无论读回什么内容，事务结束都必须释放 CS。
     * 错误优先级逻辑：数据错优先报数据错（ret 已非 OK 就不再覆盖）；
     * 数据 OK 但 CS 拉高失败（几乎不可能，属 GPIO 故障），才报 CS 的错。
     * CS 不释放的后果：Flash 一直被选中，下一次事务的第 1 个字节
     * 会被它当成新命令，整条总线错乱。 */
    cs_ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if ((ret == RET_OK) && (cs_ret != RET_OK))
    {
        ret = cs_ret;
    }
    if (ret != RET_OK)
    {
        return ret;
    }

    DBG_log("GD25Q128 JEDEC ID=%02X %02X %02X\r\n",
            (unsigned int)id->manufacturer,
            (unsigned int)id->memory_type,
            (unsigned int)id->capacity);

    return RET_OK;
}

/************************************************************
 * @brief   内部读取 GD25Q128 状态寄存器
 * @note    状态寄存器只服务于 Device 内部的 BUSY/WEL 状态机。
 ************************************************************/
static ret_t gd25q128_status_get(uint8_t *status)
{
    ret_t ret;
    ret_t cs_ret;
    uint8_t rx_data;

    if (status == NULL)
    {
        return RET_NULL_PTR;
    }

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    *status = GD25Q128_SPI_DUMMY_BYTE;

    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                     GD25Q128_CMD_READ_STATUS,
                                     &rx_data);
    if (ret == RET_OK)
    {
        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         GD25Q128_SPI_DUMMY_BYTE,
                                         status);
    }

    cs_ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if ((ret == RET_OK) && (cs_ret != RET_OK))
    {
        ret = cs_ret;
    }

    return ret;
}

/************************************************************
 * @brief   内部发送写使能命令并确认 WEL
 * @note    WREN 只允许由 Device 层的写/擦事务调用，不对 App 层暴露。
 ************************************************************/
static ret_t gd25q128_write_enable(void)
{
    ret_t ret;
    ret_t cs_ret;
    uint8_t rx_data;
    uint8_t status;

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                     GD25Q128_CMD_WRITE_ENABLE,
                                     &rx_data);

    /* 命令帧无论 SPI 是否成功，都必须尝试释放 CS。 */
    cs_ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if ((ret == RET_OK) && (cs_ret != RET_OK))
    {
        ret = cs_ret;
    }
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 0x06 的返回字节没有业务含义，必须另起事务读取 WEL。 */
    ret = gd25q128_status_get(&status);
    if (ret != RET_OK)
    {
        return ret;
    }

    return ((status & GD25Q128_STATUS_WEL_MASK) != 0U) ? RET_OK : RET_FLASH_WRITE_ERROR;
}

/************************************************************
 * @brief   等待 GD25Q128 内部操作完成
 * @note    使用 FreeRTOS tick 作为总截止时间，每次轮询后让出 CPU。
 ************************************************************/
static ret_t gd25q128_wait_ready(uint32_t timeout_ms)
{
    ret_t ret;
    uint8_t status;
    TickType_t start_tick;
    TickType_t timeout_ticks;

    if (timeout_ms == 0U)
    {
        return RET_INVALID_PARAM;
    }

    /* 用 FreeRTOS tick 做总截止：start 记起点，timeout 换算成 tick。
     * timeout=0 时换算结果为 0，强制给 1 个 tick，
     * 避免“超时时间为 0”导致一次轮询都不做就直接超时。 */
    start_tick = xTaskGetTickCount();
    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks == 0U)
    {
        timeout_ticks = 1U;
    }

    for (;;)
    {
        ret = gd25q128_status_get(&status);
        if (ret != RET_OK)
        {
            return ret;
        }

        /* BUSY(WIP)=0 表示 Flash 内部擦写完成，可以发下一条命令 */
        if ((status & GD25Q128_STATUS_BUSY_MASK) == 0U)
        {
            return RET_OK;
        }

        /* tick 差值比较：无符号减法天然处理计数器回绕，
         * 即使 xTaskGetTickCount 在等待期间溢出，差值依然正确。 */
        if ((xTaskGetTickCount() - start_tick) >= timeout_ticks)
        {
            return RET_TIMEOUT;
        }

        /* 每次轮询让出 1ms CPU：Flash 编程要几百 us 到几 ms，
         * 忙等会饿死同优先级任务；delay 让调度器去跑别的任务。 */
        vTaskDelay(pdMS_TO_TICKS(GD25Q128_STATUS_POLL_DELAY_MS));
    }
}

/************************************************************
 * @brief   SPI 超时后的最小恢复动作
 * @note    不假设上一次 Page Program 是否已完成；这里只恢复通信入口，
 *          后续事务仍必须重新读取 BUSY 和执行必要的回读校验。
 ************************************************************/
static ret_t gd25q128_spi_recover(void)
{
    ret_t ret;

    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if (ret != RET_OK)
    {
        return ret;
    }

    return bsp_spi_init(BSP_SPI_FLASH);
}

/************************************************************
 * @brief   发送一个无地址 Flash 命令帧
 ************************************************************/
static ret_t gd25q128_send_command(uint8_t command)
{
    ret_t ret;
    ret_t cs_ret;
    uint8_t rx_data;

    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                     command,
                                     &rx_data);

    cs_ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if ((ret == RET_OK) && (cs_ret != RET_OK))
    {
        ret = cs_ret;
    }

    return ret;
}

/************************************************************
 * @brief   发送一个带 24 位地址的 Flash 命令帧
 ************************************************************/
static ret_t gd25q128_send_command_address(uint8_t command,
                                            uint32_t address)
{
    ret_t ret;
    ret_t cs_ret;
    uint8_t rx_data;

    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* do{...}while(0) + break 结构：把“顺序执行的 4 字节”
     * 写成可中途跳出的块。任一步失败 break 出来，
     * 统一走到底部的 CS 释放，保证失败时 CS 也一定被拉高。
     * 比 4 层 if 嵌套扁平，比 goto 更符合编码规范。 */
    do
    {
        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         command,
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        /* 24 位地址大端发送：先高字节 A23-A16，再中字节，
         * 最后低字节。Flash 协议固定大端，顺序反了会写错地址。 */
        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)(address >> 16U),
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)(address >> 8U),
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)address,
                                         &rx_data);
    } while (0);

    cs_ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if ((ret == RET_OK) && (cs_ret != RET_OK))
    {
        ret = cs_ret;
    }

    return ret;
}

/************************************************************
 * @brief   准备一次写/擦事务并确认 BUSY=0、WEL=1
 ************************************************************/
static ret_t gd25q128_prepare_write(uint32_t timeout_ms)
{
    ret_t ret;
    uint8_t status;

    ret = gd25q128_wait_ready(timeout_ms);
    if (ret != RET_OK)
    {
        if (ret == RET_TIMEOUT)
        {
            (void)gd25q128_spi_recover();
        }
        return ret;
    }

    ret = gd25q128_write_enable();
    if (ret != RET_OK)
    {
        if (ret == RET_TIMEOUT)
        {
            (void)gd25q128_spi_recover();
        }
        return ret;
    }

    /* WREN 与后续写/擦命令必须是两个独立的 CS 事务。 */
    ret = gd25q128_status_get(&status);
    if (ret != RET_OK)
    {
        if (ret == RET_TIMEOUT)
        {
            (void)gd25q128_spi_recover();
        }
        return ret;
    }

    if ((status & GD25Q128_STATUS_BUSY_MASK) != 0U)
    {
        return RET_BUSY;
    }

    if ((status & GD25Q128_STATUS_WEL_MASK) == 0U)
    {
        return RET_FLASH_WRITE_ERROR;
    }

    return RET_OK;
}

/************************************************************
 * @brief   在当前页内执行一次 Page Program
 ************************************************************/
ret_t dev_gd25q128_page_program(uint32_t address,
                                 const uint8_t *buffer,
                                 uint16_t len)
{
    ret_t ret;
    uint32_t page_offset;
    uint32_t page_remaining;

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (buffer == NULL)
    {
        return RET_NULL_PTR;
    }

    if (len == 0U)
    {
        return RET_INVALID_PARAM;
    }

    if (address >= DEV_GD25Q128_TOTAL_SIZE)
    {
        return RET_INVALID_PARAM;
    }

    /* 页边界检查：Page Program 硬件特性——写跨过页末不会进下一页，
     * 而是卷绕回本页开头覆盖旧数据。address & 255 取页内偏移
     * （页大小 256 是 2 的幂，等价于 address % 256，更快），
     * 剩余空间不够就拒绝，让上层自己拆成多次页写。 */
    page_offset = address & (DEV_GD25Q128_PAGE_SIZE - 1UL);
    page_remaining = DEV_GD25Q128_PAGE_SIZE - page_offset;
    if ((uint32_t)len > page_remaining)
    {
        return RET_INVALID_PARAM;
    }

    /* 公共入口统一拿锁：参数检查通过后才 Take，不占着锁做纯软件检查。 */
    if (xSemaphoreTake(s_dev_gd25q128_mutex,
                       pdMS_TO_TICKS(GD25Q128_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    ret = gd25q128_page_program_core(address, buffer, len);

    /* 从 Take 成功这一刻起，本函数所有离开路径都必须 Give 一次；
     * core 内部已保证错误路径做完 CS/recover 收尾，这里只负责放锁。 */
    (void)xSemaphoreGive(s_dev_gd25q128_mutex);
    return ret;
}

/************************************************************
 * @brief   在当前页内执行一次 Page Program（调用者必须已持锁）
 * @note    不加锁的事务核心：公共入口 page_program 和跨页 write
 *          各自拿一次锁后调用这里，避免同一任务重复 Take 死锁。
 ************************************************************/
static ret_t gd25q128_page_program_core(uint32_t address,
                                        const uint8_t *buffer,
                                        uint16_t len)
{
    ret_t ret;
    ret_t cs_ret;
    uint8_t rx_data;
    uint16_t index;

    /* 先确保上一项内部操作完成，再申请下一次编程权限。 */
    ret = gd25q128_prepare_write(GD25Q128_PAGE_PROGRAM_TIMEOUT_MS);
    if (ret != RET_OK)
    {
        if (ret == RET_TIMEOUT)
        {
            (void)gd25q128_spi_recover();
        }
        return ret;
    }

    /* CS 拉低后，无论中间哪一个字节失败，都必须执行 CS 拉高收尾。 */
    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    do
    {
        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         GD25Q128_CMD_PAGE_PROGRAM,
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)(address >> 16U),
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)(address >> 8U),
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)address,
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        for (index = 0U; index < len; index++)
        {
            ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                             buffer[index],
                                             &rx_data);
            if (ret != RET_OK)
            {
                break;
            }
        }
    } while (0);

    cs_ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if ((ret == RET_OK) && (cs_ret != RET_OK))
    {
        ret = cs_ret;
    }
    if (ret != RET_OK)
    {
        if (ret == RET_TIMEOUT)
        {
            (void)gd25q128_spi_recover();
        }
        return ret;
    }

    /* CS 高之后才启动内部编程；此时不能立刻开始下一页。 */
    ret = gd25q128_wait_ready(GD25Q128_PAGE_PROGRAM_TIMEOUT_MS);
    if (ret == RET_TIMEOUT)
    {
        (void)gd25q128_spi_recover();
    }

    return ret;
}

/************************************************************
 * @brief   连续读取 GD25Q128 数据
 * @note    发送一次命令和地址后，保持 CS 有效并连续发送空字节取数。
 ************************************************************/
ret_t dev_gd25q128_read(uint32_t address,
                         uint8_t *buffer,
                         uint16_t len)
{
    ret_t ret;
    ret_t cs_ret;
    uint8_t rx_data;
    uint16_t index;

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (buffer == NULL)
    {
        return RET_NULL_PTR;
    }

    if (len == 0U)
    {
        return RET_INVALID_PARAM;
    }

    if (address >= DEV_GD25Q128_TOTAL_SIZE)
    {
        return RET_INVALID_PARAM;
    }

    if ((uint32_t)len > (DEV_GD25Q128_TOTAL_SIZE - address))
    {
        return RET_INVALID_PARAM;
    }

    if (xSemaphoreTake(s_dev_gd25q128_mutex,
                       pdMS_TO_TICKS(GD25Q128_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    ret = gd25q128_wait_ready(GD25Q128_PAGE_PROGRAM_TIMEOUT_MS);
    if (ret != RET_OK)
    {
        if (ret == RET_TIMEOUT)
        {
            (void)gd25q128_spi_recover();
        }
        (void)xSemaphoreGive(s_dev_gd25q128_mutex);
        return ret;
    }

    ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        (void)xSemaphoreGive(s_dev_gd25q128_mutex);
        return ret;
    }

    do
    {
        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         GD25Q128_CMD_READ_DATA,
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)(address >> 16U),
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)(address >> 8U),
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                         (uint8_t)address,
                                         &rx_data);
        if (ret != RET_OK)
        {
            break;
        }

        for (index = 0U; index < len; index++)
        {
            ret = bsp_spi_read_write_byte_ex(BSP_SPI_FLASH,
                                             GD25Q128_SPI_DUMMY_BYTE,
                                             &buffer[index]);
            if (ret != RET_OK)
            {
                break;
            }
        }
    } while (0);

    cs_ret = bsp_gpio_write(BSP_GPIO_SPI_FLASH_CS, BSP_GPIO_ON);
    if ((ret == RET_OK) && (cs_ret != RET_OK))
    {
        ret = cs_ret;
    }
    if (ret == RET_TIMEOUT)
    {
        (void)gd25q128_spi_recover();
    }

    (void)xSemaphoreGive(s_dev_gd25q128_mutex);
    return ret;
}

/************************************************************
 * @brief   写入任意长度数据，并自动拆分为多个页事务
 * @note    每一页都由 dev_gd25q128_page_program() 独立完成，
 *          因而不会让 Page Program 跨页回卷。
 ************************************************************/
ret_t dev_gd25q128_write(uint32_t address,
                          const uint8_t *buffer,
                          uint32_t len)
{
    ret_t ret;
    uint32_t offset;
    uint32_t page_offset;
    uint32_t page_remaining;
    uint32_t chunk_len;

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (buffer == NULL)
    {
        return RET_NULL_PTR;
    }

    if (len == 0U)
    {
        return RET_INVALID_PARAM;
    }

    if (address >= DEV_GD25Q128_TOTAL_SIZE)
    {
        return RET_INVALID_PARAM;
    }

    if (len > (DEV_GD25Q128_TOTAL_SIZE - address))
    {
        return RET_INVALID_PARAM;
    }

    /* 连续写是一个公共业务请求：整个跨页过程外层只拿一次锁，
     * 内部页事务调用不加锁的 core，避免同一任务重复 Take 自我死锁。 */
    if (xSemaphoreTake(s_dev_gd25q128_mutex,
                       pdMS_TO_TICKS(GD25Q128_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    /* 跨页自动拆分：每次只写“本页剩余”和“还剩多少”中的较小值。
     * 举例：从地址 0x100F0 写 32 字节——页内偏移 0xF0，本页剩 16，
     * 第一 chunk 写 16（填满本页），第二 chunk 从下一页开头写 16。
     * 每 chunk 都是一次独立的 Page Program 事务（含 WREN+等待），
     * 中间任何一页失败都直接返回，offset 之前的页已写好、之后的不写。 */
    offset = 0U;
    while (offset < len)
    {
        page_offset = (address + offset) & (DEV_GD25Q128_PAGE_SIZE - 1UL);
        page_remaining = DEV_GD25Q128_PAGE_SIZE - page_offset;
        chunk_len = len - offset;
        if (chunk_len > page_remaining)
        {
            chunk_len = page_remaining;
        }

        ret = gd25q128_page_program_core(address + offset,
                                         buffer + offset,
                                         (uint16_t)chunk_len);
        if (ret != RET_OK)
        {
            (void)xSemaphoreGive(s_dev_gd25q128_mutex);
            return ret;
        }

        offset += chunk_len;
    }

    (void)xSemaphoreGive(s_dev_gd25q128_mutex);
    return RET_OK;
}

/************************************************************
 * @brief   执行带地址的擦除事务
 * @note    调用者已经完成初始化检查；这里统一处理 BUSY/WEL、CS 和超时。
 ************************************************************/
static ret_t gd25q128_erase_with_address(uint8_t command,
                                          uint32_t address,
                                          uint32_t timeout_ms,
                                          uint32_t alignment)
{
    ret_t ret;

    /* 对齐检查：扇区擦除地址必须是 4KB 整数倍，块擦除 64KB 整数倍。
     * 硬件会忽略地址的低位——传 0x1001 擦扇区实际擦的是 0x1000，
     * 静默错擦比报错更危险，所以这里强制要求调用者对齐。 */
    if ((alignment == 0U) ||
        (address >= DEV_GD25Q128_TOTAL_SIZE) ||
        ((address % alignment) != 0U))
    {
        return RET_INVALID_PARAM;
    }

    ret = gd25q128_prepare_write(timeout_ms);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = gd25q128_send_command_address(command, address);
    if (ret != RET_OK)
    {
        if (ret == RET_TIMEOUT)
        {
            (void)gd25q128_spi_recover();
        }
        return ret;
    }

    ret = gd25q128_wait_ready(timeout_ms);
    if (ret == RET_TIMEOUT)
    {
        (void)gd25q128_spi_recover();
    }

    return ret;
}

/************************************************************
 * @brief   执行无地址的擦除事务
 ************************************************************/
static ret_t gd25q128_erase_without_address(uint8_t command,
                                             uint32_t timeout_ms)
{
    ret_t ret;

    ret = gd25q128_prepare_write(timeout_ms);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = gd25q128_send_command(command);
    if (ret != RET_OK)
    {
        if (ret == RET_TIMEOUT)
        {
            (void)gd25q128_spi_recover();
        }
        return ret;
    }

    ret = gd25q128_wait_ready(timeout_ms);
    if (ret == RET_TIMEOUT)
    {
        (void)gd25q128_spi_recover();
    }

    return ret;
}

/************************************************************
 * @brief   擦除一个 4KB 扇区
 ************************************************************/
ret_t dev_gd25q128_sector_erase(uint32_t address)
{
    ret_t ret;

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_dev_gd25q128_mutex,
                       pdMS_TO_TICKS(GD25Q128_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    ret = gd25q128_erase_with_address(GD25Q128_CMD_SECTOR_ERASE,
                                       address,
                                       GD25Q128_SECTOR_ERASE_TIMEOUT_MS,
                                       DEV_GD25Q128_SECTOR_SIZE);

    (void)xSemaphoreGive(s_dev_gd25q128_mutex);
    return ret;
}

/************************************************************
 * @brief   擦除一个 64KB 块
 ************************************************************/
ret_t dev_gd25q128_block_erase(uint32_t address)
{
    ret_t ret;

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_dev_gd25q128_mutex,
                       pdMS_TO_TICKS(GD25Q128_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    ret = gd25q128_erase_with_address(GD25Q128_CMD_BLOCK_ERASE,
                                       address,
                                       GD25Q128_BLOCK_ERASE_TIMEOUT_MS,
                                       DEV_GD25Q128_BLOCK_SIZE);

    (void)xSemaphoreGive(s_dev_gd25q128_mutex);
    return ret;
}

/************************************************************
 * @brief   擦除整片 GD25Q128
 ************************************************************/
ret_t dev_gd25q128_chip_erase(void)
{
    ret_t ret;

    if (s_dev_gd25q128_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_dev_gd25q128_mutex,
                       pdMS_TO_TICKS(GD25Q128_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        /* 没拿到锁：没碰任何硬件，不恢复、不 Give，直接报等锁超时。 */
        return RET_TIMEOUT;
    }

    ret = gd25q128_erase_without_address(GD25Q128_CMD_CHIP_ERASE,
                                          GD25Q128_CHIP_ERASE_TIMEOUT_MS);

    (void)xSemaphoreGive(s_dev_gd25q128_mutex);
    return ret;
}
