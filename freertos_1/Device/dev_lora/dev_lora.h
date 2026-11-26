/**
 * @file    dev_lora.h
 * @brief   DX-LR22-433T22D LoRa Device层公共接口
 *
 * @note    Device层负责UART收发、LoRa定点发送前缀和接收任务；
 *          二进制应用帧由dev_lora_protocol模块负责编解码。
 */

#ifndef __DEV_LORA_H__
#define __DEV_LORA_H__

#include "config.h"
#include "dev_lora/dev_lora_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  有效协议帧接收回调
 * @param  frame 已通过长度、CRC和目标地址检查的帧
 * @note   frame只在回调执行期间有效；回调内只能快速复制或投递数据。
 */
typedef void (*dev_lora_recv_cb_t)(const dev_lora_protocol_frame_t *frame);

/**
 * @brief  初始化LoRa Device层
 * @param  local_addr 本机16位节点地址
 * @param  channel    本机LoRa信道
 * @return RET_OK 成功；其他值见ret_error.h
 *
 * @note   必须在FreeRTOS调度器启动后的任务上下文调用。
 */
ret_t dev_lora_init(uint16_t local_addr, uint8_t channel);

/**
 * @brief  使用模块定点模式发送一帧EL-LORA数据
 * @param  frame      待发送协议帧，SRC必须是本机地址
 * @param  timeout_ms UART发送和互斥等待超时
 * @return RET_OK 成功；其他值见ret_error.h
 */
ret_t dev_lora_send(const dev_lora_protocol_frame_t *frame,
                    uint16_t timeout_ms);

/**
 * @brief  注册有效协议帧回调
 * @param  cb 回调；NULL表示取消注册
 */
void dev_lora_register_recv_cb(dev_lora_recv_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_LORA_H__ */
