/**
 * @file    dev_at24c02.c
 * @brief   AT24C02 protocol and wear-levelled parameter slots.
 */
#include "dev_at24c02.h"
#include "bsp_iic/bsp_iic.h"
#include "bsp_delay/bsp_delay.h"

#define AT24C02_I2C_ADDRESS          (0xA0U)
#define AT24C02_WRITE_CYCLE_GUARD_MS (10U)
#define AT24C02_PARAM_BASE           (0U)
#define AT24C02_SLOT_SIZE            DEV_AT24C02_PAGE_SIZE

static uint8_t s_at24c02_ready;

static uint16_t at24c02_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++)
    {
        crc ^= (uint16_t)data[index] << 8U;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x8000U) != 0U) ?
                  (uint16_t)((crc << 1U) ^ 0x1021U) :
                  (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static ret_t at24c02_page_write(uint16_t address,
                                const uint8_t *buffer,
                                uint8_t length)
{
    ret_t ret;

    if ((buffer == NULL) || (length == 0U) ||
        (length > DEV_AT24C02_PAGE_SIZE))
    {
        return RET_INVALID_PARAM;
    }
    ret = bsp_iic_bus_write(BSP_IIC_BUS_AT24C02,
                            AT24C02_I2C_ADDRESS,
                            address,
                            BSP_IIC_MEM_ADDR_8BIT,
                            buffer,
                            length);
    if (ret == RET_OK)
    {
        /* The bare-metal path keeps the write-cycle guard bounded and simple. */
        (void)bsp_delay_ms(AT24C02_WRITE_CYCLE_GUARD_MS);
    }
    return ret;
}

static ret_t at24c02_write_core(uint16_t address,
                                const uint8_t *buffer,
                                uint16_t length)
{
    uint16_t offset = 0U;

    while (offset < length)
    {
        uint16_t page_offset = (address + offset) % DEV_AT24C02_PAGE_SIZE;
        uint16_t page_remaining = DEV_AT24C02_PAGE_SIZE - page_offset;
        uint16_t remaining = length - offset;
        uint8_t chunk = (uint8_t)((remaining < page_remaining) ?
                                  remaining : page_remaining);
        ret_t ret = at24c02_page_write((uint16_t)(address + offset),
                                        buffer + offset,
                                        chunk);
        if (ret != RET_OK)
        {
            return ret;
        }
        offset = (uint16_t)(offset + chunk);
    }
    return RET_OK;
}

ret_t dev_at24c02_init(void)
{
    uint8_t probe;
    ret_t ret;

    if (s_at24c02_ready != 0U)
    {
        return RET_ALREADY_INIT;
    }
    ret = bsp_iic_bus_init(BSP_IIC_BUS_AT24C02);
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        return ret;
    }
    ret = bsp_iic_bus_read(BSP_IIC_BUS_AT24C02,
                           AT24C02_I2C_ADDRESS,
                           0U,
                           BSP_IIC_MEM_ADDR_8BIT,
                           &probe,
                           1U);
    if (ret != RET_OK)
    {
        return (ret == RET_NACK_ERROR) ? RET_DEVICE_NOT_FOUND : ret;
    }
    s_at24c02_ready = 1U;
    return RET_OK;
}

ret_t dev_at24c02_read(uint16_t address,
                       uint8_t *buffer,
                       uint16_t length)
{
    if (s_at24c02_ready == 0U)
    {
        return RET_NOT_INIT;
    }
    if ((buffer == NULL) || (length == 0U) ||
        ((uint32_t)address + length > DEV_AT24C02_TOTAL_SIZE))
    {
        return RET_INVALID_PARAM;
    }
    return bsp_iic_bus_read(BSP_IIC_BUS_AT24C02,
                            AT24C02_I2C_ADDRESS,
                            address,
                            BSP_IIC_MEM_ADDR_8BIT,
                            buffer,
                            length);
}

ret_t dev_at24c02_write(uint16_t address,
                        const uint8_t *buffer,
                        uint16_t length)
{
    if (s_at24c02_ready == 0U)
    {
        return RET_NOT_INIT;
    }
    if ((buffer == NULL) || (length == 0U) ||
        ((uint32_t)address + length > DEV_AT24C02_TOTAL_SIZE))
    {
        return RET_INVALID_PARAM;
    }
    return at24c02_write_core(address, buffer, length);
}

ret_t dev_at24c02_param_format(void)
{
    uint8_t slot[AT24C02_SLOT_SIZE] = {0U};
    uint16_t crc = at24c02_crc16(slot, 6U);
    slot[6] = (uint8_t)(crc >> 8U);
    slot[7] = (uint8_t)crc;
    return dev_at24c02_write(AT24C02_PARAM_BASE, slot, sizeof(slot));
}

ret_t dev_at24c02_param_write(uint16_t value)
{
    uint8_t slot[AT24C02_SLOT_SIZE];
    uint8_t scan[AT24C02_SLOT_SIZE];
    uint32_t max_sequence = 0U;
    uint8_t max_slot = (uint8_t)(DEV_AT24C02_PARAM_SLOTS - 1U);
    uint8_t have_valid = 0U;
    uint8_t index;
    ret_t ret;

    if (s_at24c02_ready == 0U)
    {
        return RET_NOT_INIT;
    }
    for (index = 0U; index < DEV_AT24C02_PARAM_SLOTS; index++)
    {
        ret = dev_at24c02_read((uint16_t)(AT24C02_PARAM_BASE +
                                          index * AT24C02_SLOT_SIZE),
                               scan,
                               sizeof(scan));
        if (ret != RET_OK)
        {
            return ret;
        }
        if ((at24c02_crc16(scan, 6U) ==
             (uint16_t)(((uint16_t)scan[6] << 8U) | scan[7])) &&
            !((scan[0] == 0xFFU) && (scan[1] == 0xFFU) &&
              (scan[2] == 0xFFU) && (scan[3] == 0xFFU)))
        {
            uint32_t sequence = (uint32_t)scan[0] |
                                 ((uint32_t)scan[1] << 8U) |
                                 ((uint32_t)scan[2] << 16U) |
                                 ((uint32_t)scan[3] << 24U);
            if ((have_valid == 0U) || (sequence > max_sequence))
            {
                max_sequence = sequence;
                max_slot = index;
                have_valid = 1U;
            }
        }
    }

    max_slot = (uint8_t)((max_slot + 1U) % DEV_AT24C02_PARAM_SLOTS);
    max_sequence += 1U;
    slot[0] = (uint8_t)max_sequence;
    slot[1] = (uint8_t)(max_sequence >> 8U);
    slot[2] = (uint8_t)(max_sequence >> 16U);
    slot[3] = (uint8_t)(max_sequence >> 24U);
    slot[4] = (uint8_t)value;
    slot[5] = (uint8_t)(value >> 8U);
    ret = at24c02_crc16(slot, 6U);
    slot[6] = (uint8_t)(ret >> 8U);
    slot[7] = (uint8_t)ret;
    return dev_at24c02_write((uint16_t)(AT24C02_PARAM_BASE +
                                        max_slot * AT24C02_SLOT_SIZE),
                             slot,
                             sizeof(slot));
}

ret_t dev_at24c02_param_read(uint16_t *value)
{
    uint8_t scan[AT24C02_SLOT_SIZE];
    uint32_t max_sequence = 0U;
    uint16_t best_value = 0U;
    uint8_t have_valid = 0U;
    uint8_t index;
    ret_t ret;

    if (value == NULL)
    {
        return RET_NULL_PTR;
    }
    if (s_at24c02_ready == 0U)
    {
        return RET_NOT_INIT;
    }
    for (index = 0U; index < DEV_AT24C02_PARAM_SLOTS; index++)
    {
        ret = dev_at24c02_read((uint16_t)(AT24C02_PARAM_BASE +
                                          index * AT24C02_SLOT_SIZE),
                               scan,
                               sizeof(scan));
        if (ret != RET_OK)
        {
            return ret;
        }
        if (at24c02_crc16(scan, 6U) ==
            (uint16_t)(((uint16_t)scan[6] << 8U) | scan[7]))
        {
            uint32_t sequence = (uint32_t)scan[0] |
                                 ((uint32_t)scan[1] << 8U) |
                                 ((uint32_t)scan[2] << 16U) |
                                 ((uint32_t)scan[3] << 24U);
            if ((have_valid == 0U) || (sequence > max_sequence))
            {
                max_sequence = sequence;
                best_value = (uint16_t)scan[4] | ((uint16_t)scan[5] << 8U);
                have_valid = 1U;
            }
        }
    }
    if (have_valid == 0U)
    {
        return RET_EMPTY;
    }
    *value = best_value;
    return RET_OK;
}
