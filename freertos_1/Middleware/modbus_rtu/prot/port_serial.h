/**
 * @file    port_serial.h
 * @brief   Modbus RTU 串口收发接口
 */
#ifndef __MODBUS_RTU_PORT_SERIAL_H__
#define __MODBUS_RTU_PORT_SERIAL_H__

#include <stdint.h>
#include "../modbus_port.h"

/* USART/RS485 默认移植实现，注册给 Modbus 主站后使用。 */
extern const modbus_port_ops_t g_modbus_port_serial_ops;

/**
 * @brief   初始化 Modbus RS485 端口
 * @note    配置 USART、DMA、DE 方向脚和接收 ring buffer；重复调用安全。
 */
uint8_t modbus_port_init(void);

/**
 * @brief   发送一帧 Modbus 数据
 * @param   data    待发送数据；函数返回前必须保持有效
 * @param   length  数据长度
 * @return  实际完成发送的字节数；DMA/TC 超时或错误返回 0
 * @note    数据由 USART DMA 搬运，DMA 完成后继续等待 TC，最后才切回接收方向。
 */
uint16_t modbus_port_send(const uint8_t *data, uint16_t length);

/**
 * @brief   按 Modbus RTU 的 t3.5 帧间隔接收一帧数据
 * @param   data      接收缓冲区
 * @param   capacity  缓冲区容量，不是预期帧长度
 * @return  实际接收的帧长度；超时、缓冲区不足或 ring 溢出返回 0
 * @note    RXNE 中断逐字节写入 ring；每收到一个字节都会重启 t3.5
 *          一次性定时器。t3.5 内没有新字节时，才认定当前帧结束。
 */
uint16_t modbus_port_receive(uint8_t *data, uint16_t capacity);

#endif /* __MODBUS_RTU_PORT_SERIAL_H__ */
