/**
 * @file    dev_lora_protocol.c
 * @brief   EL-LORA V1.0 协议组帧与逐字节解析骨架
 *
 * @note    建议严格按 TODO-P00～TODO-P08 顺序实现。先完成纯协议层，
 *          用固定字节数组验证后，再接入 dev_lora 的 UART 接收任务。
 */

#include "dev_lora/dev_lora_protocol.h"

/* 线上帧中，SRC到LEN固定为8字节。 */
#define LORA_PROTOCOL_FIXED_FIELD_SIZE          8U

/* ======================== 内部辅助接口 ======================== */

/**
 * @brief  从大端字节流读取一个16位整数
 * @note   TODO-P00：实现后供SRC、DST、SEQ解码复用。
 */
static uint16_t dev_lora_protocol_read_u16_be(const uint8_t *data)
{
    (void)data;
    return 0U;
}

/**
 * @brief  按大端顺序写入一个16位整数
 * @note   TODO-P00：实现后供SRC、DST、SEQ编码复用。
 */
static void dev_lora_protocol_write_u16_be(uint8_t *data, uint16_t value)
{
    (void)data;
    (void)value;
}

/**
 * @brief  把fixed_fields中的8字节解码到frame
 * @note   TODO-P04：依次解析SRC、DST、SEQ、FUNC、LEN。
 */
static ret_t dev_lora_protocol_decode_fixed(dev_lora_protocol_parser_t *parser)
{
    (void)parser;
    return RET_NOT_SUPPORT;
}

/* ======================== 公共接口 ======================== */

void dev_lora_protocol_parser_reset(dev_lora_protocol_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    (void)memset(parser, 0, sizeof(*parser));
    parser->state = DEV_LORA_PARSE_WAIT_SOF_H;
}

uint16_t dev_lora_protocol_crc16(const uint8_t *data, uint16_t len)
{
    /*
     * TODO-P01：
     * 1. data为NULL或len为0时如何处理，要先写清接口约定；
     * 2. crc从0xFFFF开始；
     * 3. 每个输入字节先与crc低8位异或；
     * 4. 每个字节执行8次移位，多项式为0xA001；
     * 5. 用协议文档中的请求帧验证结果应为0x3B1A。
     */
    (void)data;
    (void)len;
    return 0U;
}

ret_t dev_lora_protocol_encode(const dev_lora_protocol_frame_t *frame,
                               uint8_t *output,
                               uint16_t output_size,
                               uint16_t *encoded_size)
{
    if ((frame == NULL) || (output == NULL) || (encoded_size == NULL))
    {
        return RET_NULL_PTR;
    }

    *encoded_size = 0U;

    if (frame->len > DEV_LORA_PROTOCOL_DATA_MAX_SIZE)
    {
        return RET_DATA_LENGTH_ERROR;
    }

    if (output_size < (uint16_t)(DEV_LORA_PROTOCOL_FRAME_MIN_SIZE
                                 + frame->len))
    {
        return RET_LENGTH_ERROR;
    }

    /*
     * TODO-P02：
     * 1. 写入A5 5A；
     * 2. 以大端写入SRC、DST、SEQ；
     * 3. 写入FUNC、LEN和DATA；
     * 4. 对SRC～DATA计算CRC16；
     * 5. 依次写入CRC_L、CRC_H；
     * 6. 最后再设置encoded_size。
     */
    return RET_NOT_SUPPORT;
}

ret_t dev_lora_protocol_parse_byte(dev_lora_protocol_parser_t *parser,
                                   uint8_t byte,
                                   dev_lora_protocol_frame_t *output,
                                   uint8_t *frame_ready)
{
    if ((parser == NULL) || (output == NULL) || (frame_ready == NULL))
    {
        return RET_NULL_PTR;
    }

    *frame_ready = 0U;

    /*
     * TODO-P03：WAIT_SOF_H
     * 只接受0xA5；其他字节保持在本状态。
     *
     * TODO-P04：WAIT_SOF_L
     * 收到0x5A进入固定字段；再次收到0xA5时仍可能是新帧头首字节；
     * 其他字节退回WAIT_SOF_H。
     *
     * TODO-P05：FIXED_FIELDS
     * 累计8字节后调用decode_fixed()；LEN>64时复位并返回长度错误；
     * LEN为0直接进入CRC_L，否则进入DATA。
     *
     * TODO-P06：DATA
     * 严格接收LEN个字节，不能在DATA内部寻找A5 5A。
     *
     * TODO-P07：CRC_L/CRC_H
     * 收齐线上CRC后，对SRC～DATA重新计算CRC16并比较。
     *
     * TODO-P08：完整帧输出
     * CRC正确才复制到output并令frame_ready=1；随后复位解析器。
     * CRC错误也必须复位，但返回RET_CRC_ERROR。
     */
    (void)byte;
    (void)dev_lora_protocol_read_u16_be;
    (void)dev_lora_protocol_write_u16_be;
    (void)dev_lora_protocol_decode_fixed;
    (void)LORA_PROTOCOL_FIXED_FIELD_SIZE;
    return RET_NOT_SUPPORT;
}
