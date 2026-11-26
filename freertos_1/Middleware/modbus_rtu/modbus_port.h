/**
 * @file    modbus_port.h
 * @brief   Modbus RTU platform port contract
 */

#ifndef MODBUS_PORT_H
#define MODBUS_PORT_H

#include <stdint.h>

typedef struct
{
    uint8_t (*init)(void);
    uint16_t (*send)(const uint8_t *data, uint16_t length);
    uint16_t (*receive)(uint8_t *data, uint16_t capacity);
} modbus_port_ops_t;

#endif
