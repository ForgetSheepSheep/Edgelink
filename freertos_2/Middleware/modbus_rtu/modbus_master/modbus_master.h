/**
 * @file    modbus_master.h
 * @brief   Modbus RTU master public interface
 */

#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include "../modbus_frame.h"
#include "../modbus_port.h"

/**
 * @brief   注册主站使用的底层移植接口
 * @param   ops  由具体平台提供的初始化、发送和接收接口表
 * @retval  MODBUS_STATUS_OK 成功; MODBUS_STATUS_INVALID_PARAM 接口不完整;
 *          MODBUS_STATUS_BUSY 主站正在执行事务
 * @note    ops 及其函数在主站使用期间必须持续有效。
 */
modbus_status_t modbus_master_port_register(const modbus_port_ops_t *ops);

/**
 * @brief   初始化 Modbus RTU 主站和底层端口
 * @retval  MODBUS_STATUS_OK 成功; MODBUS_STATUS_PORT_ERROR 底层端口初始化失败;
 *          MODBUS_STATUS_INVALID_PARAM 尚未注册底层移植接口
 * @note    调用前必须先注册底层移植接口；应用启动时调用一次。
 */
modbus_status_t modbus_master_init(void);

/**
 * @brief   根据请求帧中的功能码查表并执行对应处理函数
 * @param   frame  输入时为不含 CRC 的请求帧；成功返回时为完整响应帧
 * @retval  Modbus 主站执行状态
 * @note    功能码由 modbus_master_config.h 独立开关；响应业务数据由 App 解析。
 */
modbus_status_t modbus_master_execute(modbus_frame_t *frame);

#endif
