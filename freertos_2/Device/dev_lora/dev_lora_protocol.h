/**
 * @file    dev_lora_protocol.h
 * @brief   EL-LORA V1.0 协议组帧与逐字节解析接口
 *
 * @note    本文件只描述二进制应用帧，不访问 UART、GPIO 或 FreeRTOS。
 *          协议正文见 EL-LORA_V1.0通信协议.md。
 */

#ifndef __DEV_LORA_PROTOCOL_H__
#define __DEV_LORA_PROTOCOL_H__

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 帧格式常量 ======================== */

#define DEV_LORA_PROTOCOL_SOF_H                 0xA5U
#define DEV_LORA_PROTOCOL_SOF_L                 0x5AU

#define DEV_LORA_PROTOCOL_DATA_MAX_SIZE         64U
#define DEV_LORA_PROTOCOL_FIXED_SIZE            10U
#define DEV_LORA_PROTOCOL_CRC_SIZE               2U
#define DEV_LORA_PROTOCOL_FRAME_MIN_SIZE        12U
#define DEV_LORA_PROTOCOL_FRAME_MAX_SIZE        76U

/* LoRa 定点发送前缀：[目标地址H][目标地址L][目标信道]。 */
#define DEV_LORA_ROUTE_PREFIX_SIZE               3U
#define DEV_LORA_TX_MAX_SIZE                    79U

/* ======================== 功能码 ======================== */

typedef enum
{
    DEV_LORA_FUNC_READ_REQUEST     = 0x01U,
    DEV_LORA_FUNC_CONTROL_REQUEST  = 0x02U,
    DEV_LORA_FUNC_READ_SUCCESS     = 0x41U,
    DEV_LORA_FUNC_CONTROL_SUCCESS  = 0x42U,
    DEV_LORA_FUNC_READ_ERROR       = 0x81U,
    DEV_LORA_FUNC_CONTROL_ERROR    = 0x82U
} dev_lora_protocol_func_t;

/* ======================== 初始对象编号 ======================== */

typedef enum
{
    DEV_LORA_OBJECT_DEVICE_STATUS  = 0x01U,
    DEV_LORA_OBJECT_TEMPERATURE    = 0x02U,
    DEV_LORA_OBJECT_HUMIDITY       = 0x03U,
    DEV_LORA_OBJECT_OUTPUT_1       = 0x10U,
    DEV_LORA_OBJECT_OUTPUT_2       = 0x11U,
    DEV_LORA_OBJECT_UNKNOWN        = 0xFFU
} dev_lora_protocol_object_t;

/* ======================== 协议错误码 ======================== */

typedef enum
{
    DEV_LORA_ERROR_UNSUPPORTED_FUNC   = 0x01U,
    DEV_LORA_ERROR_UNSUPPORTED_OBJECT = 0x02U,
    DEV_LORA_ERROR_DATA_LENGTH        = 0x03U,
    DEV_LORA_ERROR_INVALID_PARAM      = 0x04U,
    DEV_LORA_ERROR_DEVICE_BUSY        = 0x05U,
    DEV_LORA_ERROR_EXECUTE_FAILED     = 0x06U,
    DEV_LORA_ERROR_DEVICE_FAULT       = 0x07U
} dev_lora_protocol_error_t;

/* ======================== 解码结果 ======================== */

/**
 * @brief 一帧解码后的字段
 * @note  不允许直接发送该结构体；必须调用 dev_lora_protocol_encode()，
 *        避免结构体填充和 CPU 字节序影响线上格式。
 */
typedef struct
{
    uint16_t src;
    uint16_t dst;
    uint16_t seq;
    uint8_t func;
    uint8_t len;
    uint8_t data[DEV_LORA_PROTOCOL_DATA_MAX_SIZE];
} dev_lora_protocol_frame_t;

/* ======================== 逐字节解析状态 ======================== */

typedef enum
{
    DEV_LORA_PARSE_WAIT_SOF_H = 0,
    DEV_LORA_PARSE_WAIT_SOF_L,
    DEV_LORA_PARSE_FIXED_FIELDS,
    DEV_LORA_PARSE_DATA,
    DEV_LORA_PARSE_CRC_L,
    DEV_LORA_PARSE_CRC_H
} dev_lora_protocol_parse_state_t;

/**
 * @brief 逐字节解析上下文
 *
 * @note  fixed_fields 保存 SRC～LEN 共8字节。frame_ready 之前，
 *        frame 中的数据只属于未完成候选帧，调用者不得使用。
 */
typedef struct
{
    dev_lora_protocol_parse_state_t state;
    uint8_t fixed_fields[8U];
    uint8_t fixed_index;
    uint8_t data_index;
    uint8_t received_crc_l;
    dev_lora_protocol_frame_t frame;
} dev_lora_protocol_parser_t;

/**
 * @brief  初始化或复位解析器
 * @param  parser 解析上下文
 */
void dev_lora_protocol_parser_reset(dev_lora_protocol_parser_t *parser);

/**
 * @brief  向解析器送入一个UART字节
 * @param  parser      解析上下文
 * @param  byte        当前收到的字节
 * @param  output      完整且校验通过时输出帧
 * @param  frame_ready 输出标志：0表示尚未完成，1表示output有效
 * @return RET_OK 字节处理完成；RET_CRC_ERROR CRC错误；
 *         RET_DATA_LENGTH_ERROR 长度非法；其他值见ret_error.h
 *
 * @note   TODO-P03～TODO-P08 将在本函数中完成状态机。协议错误发生后，
 *         解析器必须恢复到寻找帧头状态，下一字节仍可继续解析。
 */
ret_t dev_lora_protocol_parse_byte(dev_lora_protocol_parser_t *parser,
                                   uint8_t byte,
                                   dev_lora_protocol_frame_t *output,
                                   uint8_t *frame_ready);

/**
 * @brief  将字段编码成EL-LORA应用帧
 * @param  frame         待编码字段
 * @param  output        输出缓冲区
 * @param  output_size   输出缓冲区容量
 * @param  encoded_size  实际生成的字节数
 * @return RET_OK 成功；其他值见ret_error.h
 *
 * @note   TODO-P02：按线上字节序逐字段写入，不能memcpy整个结构体。
 */
ret_t dev_lora_protocol_encode(const dev_lora_protocol_frame_t *frame,
                               uint8_t *output,
                               uint16_t output_size,
                               uint16_t *encoded_size);

/**
 * @brief  计算CRC16/MODBUS
 * @param  data 待校验数据
 * @param  len  数据长度
 * @return CRC16计算结果
 *
 * @note   TODO-P01：初值0xFFFF，多项式0xA001，逐字节低位先处理。
 */
uint16_t dev_lora_protocol_crc16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_LORA_PROTOCOL_H__ */
