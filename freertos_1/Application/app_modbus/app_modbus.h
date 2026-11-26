/**
 * @file    app_modbus.h
 * @brief   Modbus RTU 应用服务接口
 *
 * @note    该模块拥有 Modbus 串口主站初始化和事务互斥；业务 App 只能
 *          经本文件提供的语义化读写接口发起请求，不直接操作串口、帧或
 *          主站对象。
 */
#ifndef __APP_MODBUS_H__
#define __APP_MODBUS_H__

#include "ret_error.h"

/**
 * @brief 初始化 Modbus 主站服务
 * @note 创建主站事务互斥锁并启动一次性初始化任务。底层串口初始化
 *       可能重试，因此调用者不应在此函数中等待“已就绪”。
 */
ret_t app_modbus_init(void);

/** @brief 查询主站串口是否已经初始化完成。 */
uint8_t app_modbus_is_ready(void);

/**
 * @brief   读取保持寄存器或输入寄存器
 * @param   function_code 0x03（保持寄存器）或 0x04（输入寄存器）
 * @param   register_values 输出数组，至少容纳 quantity 个寄存器
 * @retval  RET_OK 成功；RET_TIMEOUT 超时；RET_CRC_ERROR CRC 错误；
 *          其他值表示参数、串口或 Modbus 响应错误
 * @note    本接口是线程安全的，一次调用对应一笔完整的 RS485 事务。
 */
ret_t app_modbus_read_registers(uint8_t slave_address,
                                uint8_t function_code,
                                uint16_t start_address,
                                uint16_t quantity,
                                uint16_t *register_values);

/**
 * @brief   读取线圈或离散输入
 * @note    bit_values 至少需要 (quantity + 7) / 8 字节。
 *          位 0 对应起始地址，接口内部负责解包响应字节。
 * @retval  RET_OK 成功；其他值表示底层事务或响应格式错误
 */
ret_t app_modbus_read_bits(uint8_t slave_address,
                           uint8_t function_code,
                           uint16_t start_address,
                           uint16_t quantity,
                           uint8_t *bit_values);

/** @brief 写入一个保持寄存器，并校验从站回显。 */
ret_t app_modbus_write_register(uint8_t slave_address,
                                uint16_t register_address,
                                uint16_t register_value);

/** @brief 写入一个线圈；is_on 非 0 写入 ON，0 写入 OFF。 */
ret_t app_modbus_write_coil(uint8_t slave_address,
                            uint16_t coil_address,
                            uint8_t is_on);

#endif /* __APP_MODBUS_H__ */
