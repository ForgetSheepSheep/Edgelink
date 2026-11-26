/**
 * @file    modbus_master.c
 * @brief   Modbus RTU master function-code dispatch and transaction checks
 */

#include <stddef.h>
#include <stdint.h>
#include "modbus_master.h"
#include "modbus_master_config.h"
#include "../modbus_crc.h"

#define MODBUS_FUNCTION_READ_COILS                  (0x01U)
#define MODBUS_FUNCTION_READ_DISCRETE_INPUTS        (0x02U)
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS      (0x03U)
#define MODBUS_FUNCTION_READ_INPUT_REGISTERS        (0x04U)
#define MODBUS_FUNCTION_WRITE_SINGLE_COIL           (0x05U)
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER       (0x06U)
#define MODBUS_FUNCTION_DIAGNOSTICS                  (0x08U)
#define MODBUS_FUNCTION_GET_COMM_EVENT_COUNTER       (0x0BU)
#define MODBUS_FUNCTION_WRITE_MULTIPLE_COILS        (0x0FU)
#define MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTERS    (0x10U)
#define MODBUS_FUNCTION_REPORT_SERVER_ID             (0x11U)
#define MODBUS_FUNCTION_MASK_WRITE_REGISTER          (0x16U)
#define MODBUS_FUNCTION_READ_WRITE_REGISTERS         (0x17U)
#define MODBUS_FUNCTION_ENCAPSULATED_INTERFACE       (0x2BU)
#define MODBUS_MEI_READ_DEVICE_IDENTIFICATION        (0x0EU)
#define MODBUS_FUNCTION_EXCEPTION_MASK              (0x80U)

#define MODBUS_SLAVE_ADDRESS_MIN                    (1U)
#define MODBUS_SLAVE_ADDRESS_MAX                    (247U)
#define MODBUS_REQUEST_FIXED_LENGTH                 (6U)
#define MODBUS_RESPONSE_WRITE_LENGTH                (8U)
#define MODBUS_EXCEPTION_RESPONSE_LENGTH            (5U)
#define MODBUS_CRC_LENGTH                           (2U)
#define MODBUS_READ_REGISTER_MAX_QUANTITY           (125U)
#define MODBUS_READ_BIT_MAX_QUANTITY                (2000U)
#define MODBUS_WRITE_COIL_MAX_QUANTITY              (1968U)
#define MODBUS_WRITE_REGISTER_MAX_QUANTITY          (123U)
#define MODBUS_READ_WRITE_READ_MAX_QUANTITY          (125U)
#define MODBUS_READ_WRITE_WRITE_MAX_QUANTITY         (121U)

typedef modbus_status_t (*modbus_function_handler_t)(modbus_frame_t *frame);

typedef struct
{
    uint8_t function_code;
    modbus_function_handler_t handler;
} modbus_function_ops_t;

#if MODBUS_MASTER_FUNC_01_ENABLE
static modbus_status_t modbus_master_handle_01(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_02_ENABLE
static modbus_status_t modbus_master_handle_02(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_03_ENABLE
static modbus_status_t modbus_master_handle_03(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_04_ENABLE
static modbus_status_t modbus_master_handle_04(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_05_ENABLE
static modbus_status_t modbus_master_handle_05(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_06_ENABLE
static modbus_status_t modbus_master_handle_06(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_08_ENABLE
static modbus_status_t modbus_master_handle_08(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_0B_ENABLE
static modbus_status_t modbus_master_handle_0b(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_0F_ENABLE
static modbus_status_t modbus_master_handle_0f(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_10_ENABLE
static modbus_status_t modbus_master_handle_10(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_11_ENABLE
static modbus_status_t modbus_master_handle_11(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_16_ENABLE
static modbus_status_t modbus_master_handle_16(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_17_ENABLE
static modbus_status_t modbus_master_handle_17(modbus_frame_t *frame);
#endif
#if MODBUS_MASTER_FUNC_2B_ENABLE
static modbus_status_t modbus_master_handle_2b(modbus_frame_t *frame);
#endif
static modbus_status_t modbus_master_handle_read(modbus_frame_t *frame,
                                                  uint8_t function_code,
                                                  uint16_t max_quantity,
                                                  uint8_t bit_data);
static modbus_status_t modbus_master_handle_write_single(modbus_frame_t *frame,
                                                          uint8_t function_code,
                                                          uint8_t coil_data);
static modbus_status_t modbus_master_handle_write_multiple(modbus_frame_t *frame,
                                                            uint8_t function_code,
                                                            uint16_t max_quantity,
                                                            uint8_t bit_data);
static modbus_status_t modbus_master_exchange(modbus_frame_t *frame,
                                               uint8_t slave_address,
                                               uint8_t function_code);
static uint16_t modbus_master_u16_get(const uint8_t *data);
static uint8_t modbus_master_slave_address_valid(uint8_t slave_address);
static uint8_t modbus_master_write_address_valid(uint8_t slave_address);

static const modbus_function_ops_t s_modbus_function_table[] =
{
#if MODBUS_MASTER_FUNC_01_ENABLE
    {MODBUS_FUNCTION_READ_COILS,                 modbus_master_handle_01},
#endif
#if MODBUS_MASTER_FUNC_02_ENABLE
    {MODBUS_FUNCTION_READ_DISCRETE_INPUTS,       modbus_master_handle_02},
#endif
#if MODBUS_MASTER_FUNC_03_ENABLE
    {MODBUS_FUNCTION_READ_HOLDING_REGISTERS,   modbus_master_handle_03},
#endif
#if MODBUS_MASTER_FUNC_04_ENABLE
    {MODBUS_FUNCTION_READ_INPUT_REGISTERS,       modbus_master_handle_04},
#endif
#if MODBUS_MASTER_FUNC_05_ENABLE
    {MODBUS_FUNCTION_WRITE_SINGLE_COIL,          modbus_master_handle_05},
#endif
#if MODBUS_MASTER_FUNC_06_ENABLE
    {MODBUS_FUNCTION_WRITE_SINGLE_REGISTER,    modbus_master_handle_06},
#endif
#if MODBUS_MASTER_FUNC_08_ENABLE
    {MODBUS_FUNCTION_DIAGNOSTICS,                modbus_master_handle_08},
#endif
#if MODBUS_MASTER_FUNC_0B_ENABLE
    {MODBUS_FUNCTION_GET_COMM_EVENT_COUNTER,     modbus_master_handle_0b},
#endif
#if MODBUS_MASTER_FUNC_0F_ENABLE
    {MODBUS_FUNCTION_WRITE_MULTIPLE_COILS,       modbus_master_handle_0f},
#endif
#if MODBUS_MASTER_FUNC_10_ENABLE
    {MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTERS,   modbus_master_handle_10},
#endif
#if MODBUS_MASTER_FUNC_11_ENABLE
    {MODBUS_FUNCTION_REPORT_SERVER_ID,           modbus_master_handle_11},
#endif
#if MODBUS_MASTER_FUNC_16_ENABLE
    {MODBUS_FUNCTION_MASK_WRITE_REGISTER,        modbus_master_handle_16},
#endif
#if MODBUS_MASTER_FUNC_17_ENABLE
    {MODBUS_FUNCTION_READ_WRITE_REGISTERS,       modbus_master_handle_17},
#endif
#if MODBUS_MASTER_FUNC_2B_ENABLE
    {MODBUS_FUNCTION_ENCAPSULATED_INTERFACE,     modbus_master_handle_2b},
#endif
};

static uint8_t s_modbus_master_busy;
static const modbus_port_ops_t *s_modbus_port_ops;

static uint16_t modbus_master_u16_get(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static uint8_t modbus_master_slave_address_valid(uint8_t slave_address)
{
    return ((slave_address >= MODBUS_SLAVE_ADDRESS_MIN) &&
            (slave_address <= MODBUS_SLAVE_ADDRESS_MAX)) ? 1U : 0U;
}

static uint8_t modbus_master_write_address_valid(uint8_t slave_address)
{
    return (slave_address <= MODBUS_SLAVE_ADDRESS_MAX) ? 1U : 0U;
}

static modbus_status_t modbus_master_exchange(modbus_frame_t *frame,
                                               uint8_t slave_address,
                                               uint8_t function_code)
{
    uint16_t crc;
    uint16_t request_length;
    uint16_t response_length;
    uint8_t response_function;

    request_length = frame->length;
    if (request_length > (MODBUS_RTU_ADU_MAX_LENGTH - MODBUS_CRC_LENGTH))
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }

    crc = modbus_crc16(frame->buffer, request_length);
    frame->buffer[request_length] = (uint8_t)(crc & 0x00FFU);
    frame->buffer[request_length + 1U] = (uint8_t)(crc >> 8U);
    request_length = (uint16_t)(request_length + MODBUS_CRC_LENGTH);

    if (s_modbus_port_ops->send(frame->buffer, request_length) != request_length)
    {
        return MODBUS_STATUS_SEND_FAILED;
    }

    /* 地址 0 是广播：写命令发送完成即成功，从站不得应答。 */
    if (slave_address == 0U)
    {
        frame->length = request_length;
        return MODBUS_STATUS_OK;
    }

    response_length = s_modbus_port_ops->receive(frame->buffer,
                                                 MODBUS_RTU_ADU_MAX_LENGTH);
    frame->length = response_length;
    if (response_length == 0U)
    {
        return MODBUS_STATUS_TIMEOUT;
    }
    if (response_length < MODBUS_EXCEPTION_RESPONSE_LENGTH)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    if (modbus_crc16(frame->buffer, response_length) != 0U)
    {
        return MODBUS_STATUS_CRC_ERROR;
    }
    if (frame->buffer[0] != slave_address)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }

    response_function = frame->buffer[1];
    if (response_function == (uint8_t)(function_code |
                                       MODBUS_FUNCTION_EXCEPTION_MASK))
    {
        return (response_length == MODBUS_EXCEPTION_RESPONSE_LENGTH)
                   ? MODBUS_STATUS_EXCEPTION
                   : MODBUS_STATUS_LENGTH_ERROR;
    }
    if (response_function != function_code)
    {
        return MODBUS_STATUS_FUNCTION_ERROR;
    }

    return MODBUS_STATUS_OK;
}

static modbus_status_t modbus_master_handle_read(modbus_frame_t *frame,
                                                  uint8_t function_code,
                                                  uint16_t max_quantity,
                                                  uint8_t bit_data)
{
    modbus_status_t status;
    uint16_t quantity;
    uint16_t expected_data_length;
    uint16_t expected_response_length;
    uint8_t slave_address;

    if (frame->length != MODBUS_REQUEST_FIXED_LENGTH)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }

    slave_address = frame->buffer[0];
    if (modbus_master_slave_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }

    quantity = modbus_master_u16_get(&frame->buffer[4]);
    if ((quantity == 0U) || (quantity > max_quantity))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }

    status = modbus_master_exchange(frame, slave_address, function_code);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }

    expected_data_length = (bit_data != 0U)
                               ? (uint16_t)((quantity + 7U) / 8U)
                               : (uint16_t)(quantity * 2U);
    expected_response_length = (uint16_t)(expected_data_length + 5U);
    if ((frame->length != expected_response_length) ||
        (frame->buffer[2] != (uint8_t)expected_data_length))
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }

    return MODBUS_STATUS_OK;
}

static modbus_status_t modbus_master_handle_write_single(modbus_frame_t *frame,
                                                          uint8_t function_code,
                                                          uint8_t coil_data)
{
    modbus_status_t status;
    uint8_t slave_address;
    uint8_t request_echo[4];
    uint8_t index;

    if (frame->length != MODBUS_REQUEST_FIXED_LENGTH)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }

    slave_address = frame->buffer[0];
    if (modbus_master_write_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }
    if ((coil_data != 0U) &&
        !(((frame->buffer[4] == 0xFFU) && (frame->buffer[5] == 0x00U)) ||
          ((frame->buffer[4] == 0x00U) && (frame->buffer[5] == 0x00U))))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }

    for (index = 0U; index < 4U; index++)
    {
        request_echo[index] = frame->buffer[index + 2U];
    }

    status = modbus_master_exchange(frame, slave_address, function_code);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }
    if (slave_address == 0U)
    {
        return MODBUS_STATUS_OK;
    }
    if (frame->length != MODBUS_RESPONSE_WRITE_LENGTH)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    for (index = 0U; index < 4U; index++)
    {
        if (frame->buffer[index + 2U] != request_echo[index])
        {
            return MODBUS_STATUS_INVALID_PARAM;
        }
    }

    return MODBUS_STATUS_OK;
}

static modbus_status_t modbus_master_handle_write_multiple(modbus_frame_t *frame,
                                                            uint8_t function_code,
                                                            uint16_t max_quantity,
                                                            uint8_t bit_data)
{
    modbus_status_t status;
    uint16_t quantity;
    uint16_t expected_request_length;
    uint16_t expected_byte_count;
    uint8_t byte_count;
    uint8_t slave_address;
    uint8_t request_echo[4];
    uint8_t index;

    if (frame->length < 8U)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }

    slave_address = frame->buffer[0];
    if (modbus_master_write_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }

    quantity = modbus_master_u16_get(&frame->buffer[4]);
    byte_count = frame->buffer[6];
    if ((quantity == 0U) || (quantity > max_quantity))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }
    expected_byte_count = (bit_data != 0U)
                              ? (uint16_t)((quantity + 7U) / 8U)
                              : (uint16_t)(quantity * 2U);
    if (byte_count != (uint8_t)expected_byte_count)
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }

    expected_request_length = (uint16_t)(7U + byte_count);
    if (frame->length != expected_request_length)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }

    for (index = 0U; index < 4U; index++)
    {
        request_echo[index] = frame->buffer[index + 2U];
    }

    status = modbus_master_exchange(frame, slave_address, function_code);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }
    if (slave_address == 0U)
    {
        return MODBUS_STATUS_OK;
    }
    if (frame->length != MODBUS_RESPONSE_WRITE_LENGTH)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    for (index = 0U; index < 4U; index++)
    {
        if (frame->buffer[index + 2U] != request_echo[index])
        {
            return MODBUS_STATUS_INVALID_PARAM;
        }
    }

    return MODBUS_STATUS_OK;
}

#if MODBUS_MASTER_FUNC_08_ENABLE
static modbus_status_t modbus_master_handle_08(modbus_frame_t *frame)
{
    modbus_status_t status;
    uint8_t slave_address;
    uint8_t request_echo[4];
    uint8_t index;

    if (frame->length != MODBUS_REQUEST_FIXED_LENGTH)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    slave_address = frame->buffer[0];
    if (modbus_master_slave_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }
    for (index = 0U; index < 4U; index++)
    {
        request_echo[index] = frame->buffer[index + 2U];
    }

    status = modbus_master_exchange(frame,
                                    slave_address,
                                    MODBUS_FUNCTION_DIAGNOSTICS);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }
    if (frame->length != MODBUS_RESPONSE_WRITE_LENGTH)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    for (index = 0U; index < 4U; index++)
    {
        if (frame->buffer[index + 2U] != request_echo[index])
        {
            return MODBUS_STATUS_INVALID_PARAM;
        }
    }
    return MODBUS_STATUS_OK;
}
#endif

#if MODBUS_MASTER_FUNC_0B_ENABLE
static modbus_status_t modbus_master_handle_0b(modbus_frame_t *frame)
{
    modbus_status_t status;
    uint8_t slave_address;

    if (frame->length != 2U)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    slave_address = frame->buffer[0];
    if (modbus_master_slave_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }

    status = modbus_master_exchange(frame,
                                    slave_address,
                                    MODBUS_FUNCTION_GET_COMM_EVENT_COUNTER);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }
    return (frame->length == 8U)
               ? MODBUS_STATUS_OK
               : MODBUS_STATUS_LENGTH_ERROR;
}
#endif

#if MODBUS_MASTER_FUNC_11_ENABLE
static modbus_status_t modbus_master_handle_11(modbus_frame_t *frame)
{
    modbus_status_t status;
    uint16_t expected_length;
    uint8_t slave_address;

    if (frame->length != 2U)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    slave_address = frame->buffer[0];
    if (modbus_master_slave_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }

    status = modbus_master_exchange(frame,
                                    slave_address,
                                    MODBUS_FUNCTION_REPORT_SERVER_ID);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }
    expected_length = (uint16_t)(frame->buffer[2] + 5U);
    return (frame->length == expected_length)
               ? MODBUS_STATUS_OK
               : MODBUS_STATUS_LENGTH_ERROR;
}
#endif

#if MODBUS_MASTER_FUNC_16_ENABLE
static modbus_status_t modbus_master_handle_16(modbus_frame_t *frame)
{
    modbus_status_t status;
    uint8_t slave_address;
    uint8_t request_echo[6];
    uint8_t index;

    if (frame->length != 8U)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    slave_address = frame->buffer[0];
    if (modbus_master_write_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }
    for (index = 0U; index < 6U; index++)
    {
        request_echo[index] = frame->buffer[index + 2U];
    }

    status = modbus_master_exchange(frame,
                                    slave_address,
                                    MODBUS_FUNCTION_MASK_WRITE_REGISTER);
    if ((status != MODBUS_STATUS_OK) || (slave_address == 0U))
    {
        return status;
    }
    if (frame->length != 10U)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    for (index = 0U; index < 6U; index++)
    {
        if (frame->buffer[index + 2U] != request_echo[index])
        {
            return MODBUS_STATUS_INVALID_PARAM;
        }
    }
    return MODBUS_STATUS_OK;
}
#endif

#if MODBUS_MASTER_FUNC_17_ENABLE
static modbus_status_t modbus_master_handle_17(modbus_frame_t *frame)
{
    modbus_status_t status;
    uint16_t read_quantity;
    uint16_t write_quantity;
    uint16_t expected_length;
    uint16_t expected_byte_count;
    uint8_t slave_address;
    uint8_t write_byte_count;

    if (frame->length < 13U)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    slave_address = frame->buffer[0];
    if (modbus_master_slave_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }

    read_quantity = modbus_master_u16_get(&frame->buffer[4]);
    write_quantity = modbus_master_u16_get(&frame->buffer[8]);
    write_byte_count = frame->buffer[10];
    if ((read_quantity == 0U) ||
        (read_quantity > MODBUS_READ_WRITE_READ_MAX_QUANTITY) ||
        (write_quantity == 0U) ||
        (write_quantity > MODBUS_READ_WRITE_WRITE_MAX_QUANTITY) ||
        (write_byte_count != (uint8_t)(write_quantity * 2U)) ||
        (frame->length != (uint16_t)(11U + write_byte_count)))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }

    status = modbus_master_exchange(frame,
                                    slave_address,
                                    MODBUS_FUNCTION_READ_WRITE_REGISTERS);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }
    expected_byte_count = (uint16_t)(read_quantity * 2U);
    expected_length = (uint16_t)(expected_byte_count + 5U);
    if ((frame->length != expected_length) ||
        (frame->buffer[2] != (uint8_t)expected_byte_count))
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    return MODBUS_STATUS_OK;
}
#endif

#if MODBUS_MASTER_FUNC_2B_ENABLE
static modbus_status_t modbus_master_handle_2b(modbus_frame_t *frame)
{
    modbus_status_t status;
    uint16_t cursor;
    uint16_t data_end;
    uint8_t slave_address;
    uint8_t read_code;
    uint8_t object_count;
    uint8_t object_index;
    uint8_t object_length;

    if (frame->length != 5U)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    slave_address = frame->buffer[0];
    read_code = frame->buffer[3];
    if (modbus_master_slave_address_valid(slave_address) == 0U)
    {
        return MODBUS_STATUS_ADDRESS_ERROR;
    }
    if ((frame->buffer[2] != MODBUS_MEI_READ_DEVICE_IDENTIFICATION) ||
        (read_code < 1U) || (read_code > 4U))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }

    status = modbus_master_exchange(frame,
                                    slave_address,
                                    MODBUS_FUNCTION_ENCAPSULATED_INTERFACE);
    if (status != MODBUS_STATUS_OK)
    {
        return status;
    }
    if (frame->length < 10U)
    {
        return MODBUS_STATUS_LENGTH_ERROR;
    }
    if ((frame->buffer[2] != MODBUS_MEI_READ_DEVICE_IDENTIFICATION) ||
        (frame->buffer[3] != read_code))
    {
        return MODBUS_STATUS_FUNCTION_ERROR;
    }

    data_end = (uint16_t)(frame->length - MODBUS_CRC_LENGTH);
    object_count = frame->buffer[7];
    cursor = 8U;
    for (object_index = 0U; object_index < object_count; object_index++)
    {
        if ((uint16_t)(cursor + 2U) > data_end)
        {
            return MODBUS_STATUS_LENGTH_ERROR;
        }
        object_length = frame->buffer[cursor + 1U];
        cursor = (uint16_t)(cursor + 2U);
        if ((uint16_t)(cursor + object_length) > data_end)
        {
            return MODBUS_STATUS_LENGTH_ERROR;
        }
        cursor = (uint16_t)(cursor + object_length);
    }

    return (cursor == data_end)
               ? MODBUS_STATUS_OK
               : MODBUS_STATUS_LENGTH_ERROR;
}
#endif

#if MODBUS_MASTER_FUNC_01_ENABLE
static modbus_status_t modbus_master_handle_01(modbus_frame_t *frame)
{
    return modbus_master_handle_read(frame,
                                     MODBUS_FUNCTION_READ_COILS,
                                     MODBUS_READ_BIT_MAX_QUANTITY,
                                     1U);
}
#endif

#if MODBUS_MASTER_FUNC_02_ENABLE
static modbus_status_t modbus_master_handle_02(modbus_frame_t *frame)
{
    return modbus_master_handle_read(frame,
                                     MODBUS_FUNCTION_READ_DISCRETE_INPUTS,
                                     MODBUS_READ_BIT_MAX_QUANTITY,
                                     1U);
}
#endif

#if MODBUS_MASTER_FUNC_03_ENABLE
static modbus_status_t modbus_master_handle_03(modbus_frame_t *frame)
{
    return modbus_master_handle_read(frame,
                                     MODBUS_FUNCTION_READ_HOLDING_REGISTERS,
                                     MODBUS_READ_REGISTER_MAX_QUANTITY,
                                     0U);
}
#endif

#if MODBUS_MASTER_FUNC_04_ENABLE
static modbus_status_t modbus_master_handle_04(modbus_frame_t *frame)
{
    return modbus_master_handle_read(frame,
                                     MODBUS_FUNCTION_READ_INPUT_REGISTERS,
                                     MODBUS_READ_REGISTER_MAX_QUANTITY,
                                     0U);
}
#endif

#if MODBUS_MASTER_FUNC_05_ENABLE
static modbus_status_t modbus_master_handle_05(modbus_frame_t *frame)
{
    return modbus_master_handle_write_single(frame,
                                             MODBUS_FUNCTION_WRITE_SINGLE_COIL,
                                             1U);
}
#endif

#if MODBUS_MASTER_FUNC_06_ENABLE
static modbus_status_t modbus_master_handle_06(modbus_frame_t *frame)
{
    return modbus_master_handle_write_single(frame,
                                             MODBUS_FUNCTION_WRITE_SINGLE_REGISTER,
                                             0U);
}
#endif

#if MODBUS_MASTER_FUNC_0F_ENABLE
static modbus_status_t modbus_master_handle_0f(modbus_frame_t *frame)
{
    return modbus_master_handle_write_multiple(frame,
                                               MODBUS_FUNCTION_WRITE_MULTIPLE_COILS,
                                               MODBUS_WRITE_COIL_MAX_QUANTITY,
                                               1U);
}
#endif

#if MODBUS_MASTER_FUNC_10_ENABLE
static modbus_status_t modbus_master_handle_10(modbus_frame_t *frame)
{
    return modbus_master_handle_write_multiple(frame,
                                               MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTERS,
                                               MODBUS_WRITE_REGISTER_MAX_QUANTITY,
                                               0U);
}
#endif

modbus_status_t modbus_master_port_register(const modbus_port_ops_t *ops)
{
    if ((ops == NULL) ||
        (ops->init == NULL) ||
        (ops->send == NULL) ||
        (ops->receive == NULL))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }
    if (s_modbus_master_busy != 0U)
    {
        return MODBUS_STATUS_BUSY;
    }

    s_modbus_port_ops = ops;
    return MODBUS_STATUS_OK;
}

modbus_status_t modbus_master_init(void)
{
    s_modbus_master_busy = 0U;
    if ((s_modbus_port_ops == NULL) || (s_modbus_port_ops->init == NULL))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }

    if (s_modbus_port_ops->init() == 0U)
    {
        return MODBUS_STATUS_PORT_ERROR;
    }

    return MODBUS_STATUS_OK;
}

modbus_status_t modbus_master_execute(modbus_frame_t *frame)
{
    modbus_status_t status;
    uint32_t index;
    uint8_t function_code;

    if ((frame == NULL) || (frame->length < 2U))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }
    if (s_modbus_port_ops == NULL)
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }
    if (s_modbus_master_busy != 0U)
    {
        return MODBUS_STATUS_BUSY;
    }

    function_code = frame->buffer[1];
    s_modbus_master_busy = 1U;
    status = MODBUS_STATUS_FUNCTION_ERROR;

    for (index = 0U;
         index < (sizeof(s_modbus_function_table) /
                  sizeof(s_modbus_function_table[0]));
         index++)
    {
        if (s_modbus_function_table[index].function_code == function_code)
        {
            status = s_modbus_function_table[index].handler(frame);
            break;
        }
    }

    s_modbus_master_busy = 0U;
    return status;
}
