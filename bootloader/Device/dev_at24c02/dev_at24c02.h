/**
 * @file    dev_at24c02.h
 * @brief   AT24C02 EEPROM device API.
 *
 * The Device layer exposes storage semantics only; I2C wire transactions are
 * hidden in the implementation and provided by the BSP.
 */
#ifndef BOOTLOADER_DEV_AT24C02_H
#define BOOTLOADER_DEV_AT24C02_H

#include "config.h"

#define DEV_AT24C02_TOTAL_SIZE   (256U)
#define DEV_AT24C02_PAGE_SIZE    (8U)
#define DEV_AT24C02_PARAM_SLOTS  (8U)

ret_t dev_at24c02_init(void);
ret_t dev_at24c02_read(uint16_t address, uint8_t *buffer, uint16_t length);
ret_t dev_at24c02_write(uint16_t address,
                        const uint8_t *buffer,
                        uint16_t length);
ret_t dev_at24c02_param_format(void);
ret_t dev_at24c02_param_write(uint16_t value);
ret_t dev_at24c02_param_read(uint16_t *value);

#endif /* BOOTLOADER_DEV_AT24C02_H */
