/**
 * @file    dev_gt911.c
 * @brief   GT911 触摸控制器 Device 层实现
 *
 * @note    当前实现 GT911 复位/地址选择、通用 I²C 初始化、产品 ID、
 *          状态/坐标读取、状态清零和 INT 到 Touch Task 的通知。
 */

#include "./dev_gt911/dev_gt911.h"
#include "../../BSP/bsp_gpio/bsp_gpio.h"
#include "../../BSP/bsp_delay/bsp_delay.h"
#include "../../BSP/bsp_iic/bsp_iic.h"
#include "gd32f30x.h"

/** GT911 模块日志开关：1=输出 DBG_log，0=屏蔽本模块全部日志。 */
#define DEV_GT911_LOG_ENABLE (0U)

#if DEV_GT911_LOG_ENABLE
#define GT911_LOG(...)       DBG_log(__VA_ARGS__)
#else
#define GT911_LOG(...)
#endif

/** GT911 复位时序选定的 8 位写地址，读方向由 I²C 原语自动置位。 */
#define GT911_I2C_ADDR_WRITE       (0x28U)
/** GT911 产品 ID 寄存器。 */
#define GT911_PID_REG              (0x8140U)
/** GT911 触摸状态寄存器。 */
#define GT911_STATUS_REG           (0x814EU)
/** GT911 第一个触摸点寄存器。 */
#define GT911_FIRST_POINT_REG      (0x8150U)
/** GT911 相邻触摸点记录之间的寄存器跨度。 */
#define GT911_POINT_STRIDE         (8U)
/** 每个触摸点实际使用的坐标/面积字节数。 */
#define GT911_POINT_DATA_LENGTH    (6U)
/** 状态寄存器的数据准备标志。 */
#define GT911_STATUS_READY_MASK    (0x80U)
/** 状态寄存器的触摸点数量掩码。 */
#define GT911_STATUS_COUNT_MASK    (0x0FU)
/** GT911 INT 使用 PB12 -> EXTI12，上升沿触发。 */
#define GT911_EXTI_LINE            (EXTI_12)
/** 需要调用 FreeRTOS FromISR API 的中断优先级。 */
#define GT911_IRQ_PRE_PRIORITY     (5U)
#define GT911_IRQ_SUB_PRIORITY     (0U)

static volatile uint8_t s_dev_gt911_inited = 0U;
static volatile uint8_t s_dev_gt911_irq_inited = 0U;
static volatile TaskHandle_t s_dev_gt911_notify_task = NULL;
static char s_dev_gt911_product_id[DEV_GT911_PRODUCT_ID_TEXT_SIZE] = {0};

/* 内部函数声明：先声明，后定义。 */
static ret_t gt911_addr_select(void);
static ret_t gt911_id_get(uint8_t *id);
static ret_t gt911_product_id_validate(const uint8_t *id);
static ret_t gt911_touch_status_get(uint8_t *status);
static ret_t gt911_touch_points_get(uint8_t point_count,
                                    dev_gt911_touch_info_t *info);
static ret_t gt911_touch_status_clear(void);

/************************************************************
 * @brief   按 GT911 时序选择 0x28/0x29 I²C 地址
 *
 * @note    GPIO BSP 只提供引脚控制；复位和地址选择时序属于
 *          GT911 Device 层协议，因此延时也在本函数中完成。
 ************************************************************/
static ret_t gt911_addr_select(void)
{
    ret_t ret;

    /* 明确执行 GPIO 初始化，确保本函数不依赖调用者的初始化细节。 */
    ret = bsp_gpio_config_output(BSP_GPIO_GT911_RST);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_gpio_config_output(BSP_GPIO_GT911_INT);
    if (ret != RET_OK)
    {
        return ret;
    }

    GT911_LOG("dev_gt911 gpio output init pass\r\n");

    /* GT911 上电时根据 INT 引脚电平锁存自己的 I2C 地址，
     * 因此复位时序必须由软件精确编排（参考 GT911 datasheet 复位章节）：
     * 1) RST 拉低：让芯片进入复位，地址锁存器清零；
     * 2) INT 先拉低再拉高：把地址选择电平准备好（高=0x28 系列地址）；
     * 3) RST 拉高：芯片起振，在 RST 上升沿采样 INT 电平并锁存地址；
     * 4) 等 10ms 让芯片完成上电初始化，之后 INT 才能改回输入。 */
    ret = bsp_gpio_write(BSP_GPIO_GT911_RST, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_gpio_write(BSP_GPIO_GT911_INT, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_delay_ms(1U);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_gpio_write(BSP_GPIO_GT911_INT, BSP_GPIO_ON);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_delay_ms(1U);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_gpio_write(BSP_GPIO_GT911_RST, BSP_GPIO_ON);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = bsp_delay_ms(10U);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 地址锁存完成后，INT 释放为输入，供后续 EXTI 使用。
     * 若一直保持输出高，GT911 的 INT 中断信号会被本机输出短路。 */
    ret = bsp_gpio_config_input(BSP_GPIO_GT911_INT);
    if (ret != RET_OK)
    {
        return ret;
    }

    GT911_LOG("dev_gt911 addr select pass\r\n");
    return RET_OK;
}

/************************************************************
 * @brief   读取 GT911 4 字节产品 ID
 * @param   id  4 字节输出缓冲区
 ************************************************************/
static ret_t gt911_id_get(uint8_t *id)
{
    if (id == NULL)
    {
        return RET_NULL_PTR;
    }

    return bsp_iic_bus_read(BSP_IIC_BUS_GT911,
                            GT911_I2C_ADDR_WRITE,
                            GT911_PID_REG,
                            BSP_IIC_MEM_ADDR_16BIT,
                            id,
                            DEV_GT911_PRODUCT_ID_LENGTH);
}

/************************************************************
 * @brief   按板级例程校验 GT911 产品 ID
 *
 * @note    例程只判断首字节为字符 '9'。第 4 个寄存器字节可能是
 *          0x00 或其他器件保留值，不能强制要求 4 个字节全部可打印。
 ************************************************************/
static ret_t gt911_product_id_validate(const uint8_t *id)
{
    if (id == NULL)
    {
        return RET_NULL_PTR;
    }

    if (id[0] != (uint8_t)'9')
    {
        return RET_DATA_FORMAT_ERROR;
    }

    return RET_OK;
}

/************************************************************
 * @brief   初始化 GT911，并读取产品 ID
 ************************************************************/
ret_t dev_gt911_init(void)
{
    uint8_t id[DEV_GT911_PRODUCT_ID_LENGTH];
    uint16_t index;
    uint16_t text_index;
    ret_t ret;

    if (s_dev_gt911_inited != 0U)
    {
        return RET_ALREADY_INIT;
    }

    /* 对齐模板顺序：先初始化 PF9/PF10，再执行 PB9/PB12 地址选择。 */
    ret = bsp_iic_bus_gpio_init(BSP_IIC_BUS_GT911);
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        GT911_LOG("dev_gt911 iic gpio init failed, ret=%ld\r\n", (long)ret);
        return ret;
    }

    GT911_LOG("dev_gt911 iic gpio init pass\r\n");

    ret = gt911_addr_select();
    if (ret != RET_OK)
    {
        GT911_LOG("dev_gt911 gpio init failed, ret=%ld\r\n", (long)ret);
        return ret;
    }

    ret = bsp_iic_bus_init(BSP_IIC_BUS_GT911);
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        GT911_LOG("dev_gt911 iic init failed, ret=%ld\r\n", (long)ret);
        return ret;
    }

    ret = gt911_id_get(id);
    if (ret != RET_OK)
    {
        GT911_LOG("dev_gt911 product id read failed, ret=%ld\r\n", (long)ret);
        return ret;
    }

    GT911_LOG("dev_gt911 product id raw=%02X %02X %02X %02X\r\n",
            (unsigned int)id[0],
            (unsigned int)id[1],
            (unsigned int)id[2],
            (unsigned int)id[3]);

    ret = gt911_product_id_validate(id);
    if (ret != RET_OK)
    {
        GT911_LOG("dev_gt911 product id invalid, ret=%ld\r\n", (long)ret);
        return RET_DEVICE_NOT_FOUND;
    }

    /* Device 层保留完整原始读取结果，同时只把可打印前缀交给文本接口。
     * 例如 39 30 FF FF 保存为字符串 "90"，不会把 0xFF 送进 LCD ASCII 字库。 */
    text_index = 0U;
    for (index = 0U;
         index < DEV_GT911_PRODUCT_ID_LENGTH;
         index++)
    {
        if ((id[index] < 0x20U) || (id[index] > 0x7EU))
        {
            break;
        }

        s_dev_gt911_product_id[text_index] = (char)id[index];
        text_index++;
    }

    while (text_index < DEV_GT911_PRODUCT_ID_LENGTH)
    {
        s_dev_gt911_product_id[text_index] = '\0';
        text_index++;
    }
    s_dev_gt911_product_id[DEV_GT911_PRODUCT_ID_LENGTH] = '\0';
    s_dev_gt911_inited = 1U;

    GT911_LOG("dev_gt911 init ok, product_id=%s\r\n",
            s_dev_gt911_product_id);

    return RET_OK;
}

/************************************************************
 * @brief   获取已经校验过的 GT911 产品 ID
 ************************************************************/
ret_t dev_gt911_product_id_get(char *buffer, uint16_t buffer_size)
{
    uint16_t index;

    if (s_dev_gt911_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (buffer == NULL)
    {
        return RET_NULL_PTR;
    }

    if (buffer_size < DEV_GT911_PRODUCT_ID_TEXT_SIZE)
    {
        return RET_LENGTH_ERROR;
    }

    for (index = 0U; index < DEV_GT911_PRODUCT_ID_TEXT_SIZE; index++)
    {
        buffer[index] = s_dev_gt911_product_id[index];
    }

    return RET_OK;
}

/************************************************************
 * @brief   读取 GT911 状态寄存器
 ************************************************************/
static ret_t gt911_touch_status_get(uint8_t *status)
{
    if (status == NULL)
    {
        return RET_NULL_PTR;
    }

    return bsp_iic_bus_read(BSP_IIC_BUS_GT911,
                            GT911_I2C_ADDR_WRITE,
                            GT911_STATUS_REG,
                            BSP_IIC_MEM_ADDR_16BIT,
                            status,
                            1U);
}

/************************************************************
 * @brief   按例程的 8 字节步长读取触摸点
 * @note    每个点的前 6 字节为 X/Y/面积，后 2 字节为保留字段。
 ************************************************************/
static ret_t gt911_touch_points_get(uint8_t point_count,
                                    dev_gt911_touch_info_t *info)
{
    uint8_t raw[DEV_GT911_TOUCH_POINT_MAX * GT911_POINT_STRIDE];
    uint16_t raw_length;
    uint16_t offset;
    uint8_t index;
    ret_t ret;

    if (info == NULL)
    {
        return RET_NULL_PTR;
    }

    if ((point_count == 0U) ||
        (point_count > DEV_GT911_TOUCH_POINT_MAX))
    {
        return RET_DATA_FORMAT_ERROR;
    }

    raw_length = (uint16_t)point_count * GT911_POINT_STRIDE;
    ret = bsp_iic_bus_read(BSP_IIC_BUS_GT911,
                           GT911_I2C_ADDR_WRITE,
                           GT911_FIRST_POINT_REG,
                           BSP_IIC_MEM_ADDR_16BIT,
                           raw,
                           raw_length);
    if (ret != RET_OK)
    {
        return ret;
    }

    for (index = 0U; index < point_count; index++)
    {
        offset = (uint16_t)index * GT911_POINT_STRIDE;
        info->points[index].x = (uint16_t)raw[offset] |
                                ((uint16_t)raw[offset + 1U] << 8U);
        info->points[index].y = (uint16_t)raw[offset + 2U] |
                                ((uint16_t)raw[offset + 3U] << 8U);
        info->points[index].size = (uint16_t)raw[offset + 4U] |
                                   ((uint16_t)raw[offset + 5U] << 8U);
    }

    return RET_OK;
}

/************************************************************
 * @brief   向 GT911 状态寄存器写 0，清除本次触摸数据标志
 ************************************************************/
static ret_t gt911_touch_status_clear(void)
{
    uint8_t clear_value;

    clear_value = 0U;

    return bsp_iic_bus_write(BSP_IIC_BUS_GT911,
                             GT911_I2C_ADDR_WRITE,
                             GT911_STATUS_REG,
                             BSP_IIC_MEM_ADDR_16BIT,
                             &clear_value,
                             1U);
}

/************************************************************
 * @brief   读取一次 GT911 触摸状态、坐标并清除状态
 ************************************************************/
ret_t dev_gt911_touch_read(dev_gt911_touch_info_t *info)
{
    uint8_t status;
    uint8_t point_count;
    uint8_t index;
    ret_t ret;

    if (s_dev_gt911_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (info == NULL)
    {
        return RET_NULL_PTR;
    }

    /* 先把输出结构清零并预置为“抬起”：调用者拿到的一定是确定状态，
     * 不会残留上一次触摸的旧坐标。READY=0（无新数据）时直接返回此状态。 */
    info->state = DEV_GT911_TOUCH_UP;
    info->point_count = 0U;
    for (index = 0U; index < DEV_GT911_TOUCH_POINT_MAX; index++)
    {
        info->points[index].x = 0U;
        info->points[index].y = 0U;
        info->points[index].size = 0U;
    }

    ret = gt911_touch_status_get(&status);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* READY 位（bit7）=0：芯片没有新触摸数据，保持 UP 状态返回。
     * 这是轮询/通知唤醒后的高频路径，大部分调用走这里。 */
    if ((status & GT911_STATUS_READY_MASK) == 0U)
    {
        return RET_OK;
    }

    /* 低 4 位是触摸点个数。点数为 0：GT911 在手指释放等时机会上报
     * “缓冲有效但 0 个点”的空帧，属正常无触摸——清掉 READY 后按 UP
     * 返回，不算错误，否则每次抬指都会误报一次格式错误。 */
    point_count = (uint8_t)(status & GT911_STATUS_COUNT_MASK);
    if (point_count == 0U)
    {
        ret = gt911_touch_status_clear();
        if (ret != RET_OK)
        {
            return ret;
        }

        return RET_OK;
    }

    /* 点数超范围才是真异常（I²C 误读或线干扰）：清 READY 防中断风暴，
     * 报格式错误。若板测再现此分支，临时打开 DEV_GT911_LOG_ENABLE
     * 查看 GT911_LOG 输出的原始 status 字节定位干扰源。 */
    if (point_count > DEV_GT911_TOUCH_POINT_MAX)
    {
        GT911_LOG("dev_gt911 invalid point count, raw status=0x%02X\r\n",
                  (unsigned int)status);

        ret = gt911_touch_status_clear();
        if (ret != RET_OK)
        {
            return ret;
        }

        return RET_DATA_FORMAT_ERROR;
    }

    ret = gt911_touch_points_get(point_count, info);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 坐标读取完成后再清状态，读失败时保留现场供下一次任务重试。 */
    ret = gt911_touch_status_clear();
    if (ret != RET_OK)
    {
        return ret;
    }

    info->state = DEV_GT911_TOUCH_DOWN;
    info->point_count = point_count;
    return RET_OK;
}

/************************************************************
 * @brief   配置 PB12 -> EXTI12 上升沿
 ************************************************************/
ret_t dev_gt911_touch_irq_init(void)
{
    if (s_dev_gt911_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (s_dev_gt911_irq_inited != 0U)
    {
        return RET_ALREADY_INIT;
    }

    /* EXTI 配置四步：开 AF 时钟（EXTI 挂在 AF 上）->
     * 把 PB12 映射到 EXTI12 -> 上升沿中断模式 ->
     * 先清一次 pending（防初始化前毛刺误触发）-> NVIC 使能。
     * GT911 有触摸时把 INT 拉高一个脉冲，上升沿正好捕获。 */
    rcu_periph_clock_enable(RCU_AF);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB,
                            GPIO_PIN_SOURCE_12);
    exti_init(GT911_EXTI_LINE,
              EXTI_INTERRUPT,
              EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(GT911_EXTI_LINE);
    nvic_irq_enable(EXTI10_15_IRQn,
                    GT911_IRQ_PRE_PRIORITY,
                    GT911_IRQ_SUB_PRIORITY);

    s_dev_gt911_irq_inited = 1U;
    GT911_LOG("dev_gt911 touch irq init pass, line=12\r\n");
    return RET_OK;
}

/************************************************************
 * @brief   注册 GT911 INT 对应的 Touch Task
 ************************************************************/
ret_t dev_gt911_register_notify_task(TaskHandle_t task)
{
    if (s_dev_gt911_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    if (task == NULL)
    {
        return RET_NULL_PTR;
    }

    if (s_dev_gt911_notify_task != NULL)
    {
        return RET_ALREADY_INIT;
    }

    s_dev_gt911_notify_task = task;
    return RET_OK;
}

/************************************************************
 * @brief   GT911 EXTI10_15 中断钩子
 * @note    ISR 只清 pending、累加通知；I²C 和日志全部在任务中执行。
 ************************************************************/
void dev_gt911_irq_handler(void)
{
    BaseType_t higher_priority_task_woken;
    TaskHandle_t notify_task;

    if (exti_interrupt_flag_get(GT911_EXTI_LINE) == RESET)
    {
        return;
    }

    exti_interrupt_flag_clear(GT911_EXTI_LINE);
    notify_task = (TaskHandle_t)s_dev_gt911_notify_task;
    higher_priority_task_woken = pdFALSE;

    if (notify_task != NULL)
    {
        vTaskNotifyGiveFromISR(notify_task,
                               &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}
