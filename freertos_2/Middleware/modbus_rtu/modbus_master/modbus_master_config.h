/**
 * @file    modbus_master_config.h
 * @brief   Modbus RTU master function-code switches
 */

#ifndef MODBUS_MASTER_CONFIG_H
#define MODBUS_MASTER_CONFIG_H

#define MODBUS_MASTER_FUNC_01_ENABLE    (1U) /* Read Coils */
#define MODBUS_MASTER_FUNC_02_ENABLE    (1U) /* Read Discrete Inputs */
#define MODBUS_MASTER_FUNC_03_ENABLE    (1U) /* Read Holding Registers */
#define MODBUS_MASTER_FUNC_04_ENABLE    (1U) /* Read Input Registers */
#define MODBUS_MASTER_FUNC_05_ENABLE    (1U) /* Write Single Coil */
#define MODBUS_MASTER_FUNC_06_ENABLE    (1U) /* Write Single Register */
#define MODBUS_MASTER_FUNC_08_ENABLE    (1U) /* Diagnostics */
#define MODBUS_MASTER_FUNC_0B_ENABLE    (1U) /* Get Comm Event Counter */
#define MODBUS_MASTER_FUNC_0F_ENABLE    (1U) /* Write Multiple Coils */
#define MODBUS_MASTER_FUNC_10_ENABLE    (1U) /* Write Multiple Registers */
#define MODBUS_MASTER_FUNC_11_ENABLE    (1U) /* Report Server ID */
#define MODBUS_MASTER_FUNC_16_ENABLE    (1U) /* Mask Write Register */
#define MODBUS_MASTER_FUNC_17_ENABLE    (1U) /* Read/Write Multiple Registers */
#define MODBUS_MASTER_FUNC_2B_ENABLE    (1U) /* Read Device Identification */

#if ((MODBUS_MASTER_FUNC_01_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_02_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_03_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_04_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_05_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_06_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_08_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_0B_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_0F_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_10_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_11_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_16_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_17_ENABLE == 0U) && \
     (MODBUS_MASTER_FUNC_2B_ENABLE == 0U))
#error "At least one Modbus master function must be enabled"
#endif

#endif
