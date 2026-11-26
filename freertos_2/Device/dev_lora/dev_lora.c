/**
 * @file    dev_lora.c
 * @brief   DX-LR22-433T22D LoRa Device层实现骨架
 *
 * @note    建议在纯协议层通过固定报文验证后，再按TODO-D01～TODO-D06
 *          接入UART和FreeRTOS。当前文件不处理读取、控制等App业务。
 */

#include "dev_lora/dev_lora.h"
#include "bsp_uart/bsp_uart.h"
#include "FreeRTOS.h"
#include "task.h"

/* ======================== 资源配置 ======================== */

#define DEV_LORA_RX_TASK_STACK_SIZE             384U
#define DEV_LORA_RX_TASK_PRIORITY                  3U

/* ======================== 私有状态 ======================== */

typedef struct
{
    uint16_t local_addr;
    uint8_t channel;
    uint8_t initialized;
    TaskHandle_t rx_task;
    dev_lora_protocol_parser_t parser;
    dev_lora_recv_cb_t recv_cb;
} dev_lora_handle_t;

static dev_lora_handle_t s_lora_handle;

/* ======================== 内部接口声明 ======================== */

static ret_t dev_lora_send_raw(uint8_t *data,
                               uint16_t len,
                               uint16_t timeout_ms);
static ret_t dev_lora_receive_byte(uint8_t *byte);
static void dev_lora_rx_task(void *param);

/* ======================== UART适配 ======================== */

/**
 * @brief  通过LoRa UART发送连续字节
 * @note   TODO-D01：完成参数检查后调用
 *         bsp_uart_transmit(BSP_UART_LORA, ...)，原样传播错误码。
 */
static ret_t dev_lora_send_raw(uint8_t *data,
                               uint16_t len,
                               uint16_t timeout_ms)
{
    (void)data;
    (void)len;
    (void)timeout_ms;
    return RET_NOT_SUPPORT;
}

/**
 * @brief  从LoRa UART环形缓冲区取一个字节
 * @note   TODO-D02：参数检查后调用
 *         bsp_uart_receive(BSP_UART_LORA, byte)。无数据时返回RET_EMPTY。
 */
static ret_t dev_lora_receive_byte(uint8_t *byte)
{
    (void)byte;
    return RET_NOT_SUPPORT;
}

/* ======================== 接收任务 ======================== */

/**
 * @brief  LoRa UART接收与协议解析任务
 *
 * @note   TODO-D03：
 *         1. 永久阻塞等待UART ISR任务通知；
 *         2. while读取，直到dev_lora_receive_byte()返回RET_EMPTY；
 *         3. 每个字节交给dev_lora_protocol_parse_byte()；
 *         4. frame_ready为1时检查DST等于local_addr；
 *         5. 通过recv_cb把完整帧交给App；
 *         6. CRC或格式错误只丢当前候选帧，继续排空UART。
 */
static void dev_lora_rx_task(void *param)
{
    (void)param;

    /* TODO-D03：实现接收循环后删除下面两行。 */
    (void)dev_lora_receive_byte;
    vTaskDelete(NULL);
}

/* ======================== 公共接口 ======================== */

ret_t dev_lora_init(uint16_t local_addr, uint8_t channel)
{
    if ((local_addr == 0x0000U) || (local_addr == 0xFFFFU))
    {
        return RET_ADDR_ERROR;
    }

    /*
     * TODO-D04：初始化顺序：
     * 1. 防止重复初始化；
     * 2. bsp_uart_init(BSP_UART_LORA)；
     * 3. 保存local_addr和channel；
     * 4. 复位protocol parser；
     * 5. 创建dev_lora_rx_task；
     * 6. bsp_uart_register_notify_task(BSP_UART_LORA, rx_task)；
     * 7. 所有步骤成功后才设置initialized=1。
     *
     * AUX引脚尚未加入bsp_gpio枚举。本阶段先完成UART与协议闭环，
     * 接入AUX时再补“低电平空闲、高电平忙”的发送状态检查。
     */
    (void)channel;
    (void)dev_lora_rx_task;
    return RET_NOT_SUPPORT;
}

ret_t dev_lora_send(const dev_lora_protocol_frame_t *frame,
                    uint16_t timeout_ms)
{
    uint8_t tx_buffer[DEV_LORA_TX_MAX_SIZE];

    if (frame == NULL)
    {
        return RET_NULL_PTR;
    }

    if (s_lora_handle.initialized == 0U)
    {
        return RET_NOT_INIT;
    }

    if ((frame->src != s_lora_handle.local_addr)
        || (frame->dst == 0x0000U)
        || (frame->dst == 0xFFFFU))
    {
        return RET_ADDR_ERROR;
    }

    if ((frame->len > DEV_LORA_PROTOCOL_DATA_MAX_SIZE)
        || (timeout_ms == 0U))
    {
        return RET_INVALID_PARAM;
    }

    /*
     * TODO-D05：
     * 1. tx_buffer[0..2]写入目标地址H、目标地址L、目标信道；
     * 2. 从tx_buffer[3]开始调用dev_lora_protocol_encode()；
     * 3. 总长度为3+encoded_size；
     * 4. 调用dev_lora_send_raw()连续发送完整报文。
     */
    (void)tx_buffer;
    (void)dev_lora_send_raw;
    return RET_NOT_SUPPORT;
}

void dev_lora_register_recv_cb(dev_lora_recv_cb_t cb)
{
    /* TODO-D06：思考回调修改与接收任务读取之间是否需要临界区。 */
    s_lora_handle.recv_cb = cb;
}
