/**
 * @file    bsp_iic.h
 * @brief   Small, synchronous I2C bus abstraction for the bootloader.
 *
 * The bootloader is single-threaded, so a transaction is naturally serialized.
 */
#ifndef BOOTLOADER_BSP_IIC_H
#define BOOTLOADER_BSP_IIC_H

#include "config.h"

#define BSP_IIC_MODE_SOFT       (0U)
#define BSP_IIC_MODE_HARD       (1U)
#define BSP_IIC_MEM_ADDR_8BIT   (1U)
#define BSP_IIC_MEM_ADDR_16BIT  (2U)

typedef enum
{
    BSP_IIC_BUS_AT24C02 = 0U,
    BSP_IIC_BUS_COUNT
} bsp_iic_bus_id_t;

ret_t bsp_iic_init(void);
ret_t bsp_iic_bus_init(bsp_iic_bus_id_t bus_id);
ret_t bsp_iic_bus_write(bsp_iic_bus_id_t bus_id,
                        uint16_t slave_addr,
                        uint16_t mem_addr,
                        uint8_t mem_addr_size,
                        const uint8_t *buffer,
                        uint16_t len);
ret_t bsp_iic_bus_read(bsp_iic_bus_id_t bus_id,
                       uint16_t slave_addr,
                       uint16_t mem_addr,
                       uint8_t mem_addr_size,
                       uint8_t *buffer,
                       uint16_t len);

/** Raw object-driver transactions: slave_addr uses the 8-bit wire format. */
ret_t bsp_iic_raw_write(bsp_iic_bus_id_t bus_id,
                        uint16_t slave_addr,
                        const uint8_t *buffer,
                        uint16_t len);
ret_t bsp_iic_raw_read(bsp_iic_bus_id_t bus_id,
                       uint16_t slave_addr,
                       uint8_t *buffer,
                       uint16_t len);

#endif /* BOOTLOADER_BSP_IIC_H */
