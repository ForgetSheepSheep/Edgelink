/**
 * @file    modbus_frame.h
 * @brief   Modbus RTU protocol frame definition
 */

#ifndef MODBUS_FRAME_H
#define MODBUS_FRAME_H

#include <stdint.h>

/* RTU ADU maximum: address + function/PDU data + CRC. */
#define MODBUS_RTU_ADU_MAX_LENGTH       (256U)

typedef enum
{
    MODBUS_STATUS_OK = 0,
    MODBUS_STATUS_INVALID_PARAM,
    MODBUS_STATUS_BUSY,
    MODBUS_STATUS_SEND_FAILED,
    MODBUS_STATUS_TIMEOUT,
    MODBUS_STATUS_CRC_ERROR,
    MODBUS_STATUS_ADDRESS_ERROR,
    MODBUS_STATUS_FUNCTION_ERROR,
    MODBUS_STATUS_LENGTH_ERROR,
    MODBUS_STATUS_EXCEPTION,
    MODBUS_STATUS_PORT_ERROR
} modbus_status_t;

typedef struct
{
    uint8_t buffer[MODBUS_RTU_ADU_MAX_LENGTH];
    uint16_t length;
} modbus_frame_t;

#endif
