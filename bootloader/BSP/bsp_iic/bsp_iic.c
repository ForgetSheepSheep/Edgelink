/**
 * @file    bsp_iic.c
 * @brief   Bounded software-I2C master for PB6/PB7 (AT24C02).
 */
#include "bsp_iic.h"
#include "bsp_delay/bsp_delay.h"
#include "gd32f30x.h"

#define IIC_SCL_PIN             GPIO_PIN_6
#define IIC_SDA_PIN             GPIO_PIN_7
#define IIC_PORT                GPIOB
#define IIC_CLOCK_TIMEOUT_US    (1000U)
#define IIC_HALF_PERIOD_US      (4U)

static uint8_t s_iic_initialized;

static void iic_scl_high(void)
{
    gpio_bit_set(IIC_PORT, IIC_SCL_PIN);
}

static void iic_scl_low(void)
{
    gpio_bit_reset(IIC_PORT, IIC_SCL_PIN);
}

static void iic_sda_high(void)
{
    gpio_bit_set(IIC_PORT, IIC_SDA_PIN);
}

static void iic_sda_low(void)
{
    gpio_bit_reset(IIC_PORT, IIC_SDA_PIN);
}

static uint8_t iic_sda_read(void)
{
    return (gpio_input_bit_get(IIC_PORT, IIC_SDA_PIN) == SET) ? 1U : 0U;
}

static uint8_t iic_scl_read(void)
{
    return (gpio_input_bit_get(IIC_PORT, IIC_SCL_PIN) == SET) ? 1U : 0U;
}

static ret_t iic_clock_high(void)
{
    uint32_t start;

    iic_scl_high();
    start = bsp_delay_cycle_get();
    while (iic_scl_read() == 0U)
    {
        if (bsp_delay_timeout_expired(start, IIC_CLOCK_TIMEOUT_US) != 0U)
        {
            iic_scl_low();
            return RET_TIMEOUT;
        }
    }
    return RET_OK;
}

static void iic_delay(void)
{
    (void)bsp_delay_us(IIC_HALF_PERIOD_US);
}

static ret_t iic_start(void)
{
    ret_t ret;

    iic_sda_high();
    ret = iic_clock_high();
    if (ret != RET_OK)
    {
        return ret;
    }
    iic_delay();
    iic_sda_low();
    iic_delay();
    iic_scl_low();
    return RET_OK;
}

static ret_t iic_stop(void)
{
    ret_t ret;

    iic_sda_low();
    iic_delay();
    ret = iic_clock_high();
    iic_delay();
    iic_sda_high();
    iic_delay();
    return ret;
}

static ret_t iic_write_byte(uint8_t value)
{
    uint8_t bit;
    ret_t ret;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((value & 0x80U) != 0U)
        {
            iic_sda_high();
        }
        else
        {
            iic_sda_low();
        }
        iic_delay();
        ret = iic_clock_high();
        if (ret != RET_OK)
        {
            return ret;
        }
        iic_delay();
        iic_scl_low();
        value <<= 1U;
    }

    /* Release SDA for the slave ACK bit. */
    iic_sda_high();
    iic_delay();
    ret = iic_clock_high();
    if (ret == RET_OK)
    {
        ret = (iic_sda_read() == 0U) ? RET_OK : RET_NACK_ERROR;
    }
    iic_delay();
    iic_scl_low();
    return ret;
}

static ret_t iic_read_byte(uint8_t *value, uint8_t acknowledge)
{
    uint8_t bit;
    uint8_t result = 0U;
    ret_t ret;

    if (value == NULL)
    {
        return RET_NULL_PTR;
    }

    iic_sda_high();
    for (bit = 0U; bit < 8U; bit++)
    {
        result <<= 1U;
        ret = iic_clock_high();
        if (ret != RET_OK)
        {
            return ret;
        }
        if (iic_sda_read() != 0U)
        {
            result |= 1U;
        }
        iic_delay();
        iic_scl_low();
        iic_delay();
    }

    if (acknowledge != 0U)
    {
        iic_sda_low();
    }
    else
    {
        iic_sda_high();
    }
    iic_delay();
    ret = iic_clock_high();
    iic_delay();
    iic_scl_low();
    iic_sda_high();
    *value = result;
    return ret;
}

static ret_t iic_send_mem_address(uint16_t address, uint8_t width)
{
    ret_t ret;

    if (width == BSP_IIC_MEM_ADDR_8BIT)
    {
        return iic_write_byte((uint8_t)address);
    }
    if (width != BSP_IIC_MEM_ADDR_16BIT)
    {
        return RET_INVALID_PARAM;
    }

    ret = iic_write_byte((uint8_t)(address >> 8U));
    if (ret == RET_OK)
    {
        ret = iic_write_byte((uint8_t)address);
    }
    return ret;
}

static ret_t iic_prepare(bsp_iic_bus_id_t bus_id,
                         uint16_t slave_addr,
                         uint16_t mem_addr,
                         uint8_t mem_addr_size,
                         uint8_t read)
{
    ret_t ret;

    if (bus_id != BSP_IIC_BUS_AT24C02)
    {
        return RET_INVALID_PARAM;
    }
    ret = iic_start();
    if (ret == RET_OK)
    {
        ret = iic_write_byte((uint8_t)(slave_addr | (read != 0U ? 1U : 0U)));
    }
    if ((ret == RET_OK) && (read == 0U))
    {
        ret = iic_send_mem_address(mem_addr, mem_addr_size);
    }
    return ret;
}

ret_t bsp_iic_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    (void)bsp_delay_init();
    gpio_init(IIC_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ,
              IIC_SCL_PIN | IIC_SDA_PIN);
    iic_scl_high();
    iic_sda_high();
    s_iic_initialized = 1U;
    return RET_OK;
}

ret_t bsp_iic_bus_init(bsp_iic_bus_id_t bus_id)
{
    if (bus_id != BSP_IIC_BUS_AT24C02)
    {
        return RET_INVALID_PARAM;
    }
    if (s_iic_initialized != 0U)
    {
        return RET_ALREADY_INIT;
    }
    return bsp_iic_init();
}

ret_t bsp_iic_bus_write(bsp_iic_bus_id_t bus_id,
                        uint16_t slave_addr,
                        uint16_t mem_addr,
                        uint8_t mem_addr_size,
                        const uint8_t *buffer,
                        uint16_t len)
{
    uint16_t index;
    ret_t ret;
    ret_t stop_ret;

    if ((buffer == NULL) || (len == 0U))
    {
        return RET_INVALID_PARAM;
    }
    if (s_iic_initialized == 0U)
    {
        return RET_NOT_INIT;
    }

    ret = iic_prepare(bus_id, slave_addr, mem_addr, mem_addr_size, 0U);
    for (index = 0U; (ret == RET_OK) && (index < len); index++)
    {
        ret = iic_write_byte(buffer[index]);
    }
    stop_ret = iic_stop();
    return (ret == RET_OK) ? stop_ret : ret;
}

ret_t bsp_iic_bus_read(bsp_iic_bus_id_t bus_id,
                       uint16_t slave_addr,
                       uint16_t mem_addr,
                       uint8_t mem_addr_size,
                       uint8_t *buffer,
                       uint16_t len)
{
    uint16_t index;
    ret_t ret;
    ret_t stop_ret;

    if ((buffer == NULL) || (len == 0U))
    {
        return RET_INVALID_PARAM;
    }
    if (s_iic_initialized == 0U)
    {
        return RET_NOT_INIT;
    }

    ret = iic_prepare(bus_id, slave_addr, mem_addr, mem_addr_size, 0U);
    if (ret == RET_OK)
    {
        ret = iic_start();
    }
    if (ret == RET_OK)
    {
        ret = iic_write_byte((uint8_t)(slave_addr | 1U));
    }
    for (index = 0U; (ret == RET_OK) && (index < len); index++)
    {
        ret = iic_read_byte(&buffer[index],
                            (index + 1U < len) ? 1U : 0U);
    }
    stop_ret = iic_stop();
    return (ret == RET_OK) ? stop_ret : ret;
}

ret_t bsp_iic_raw_write(bsp_iic_bus_id_t bus_id,
                        uint16_t slave_addr,
                        const uint8_t *buffer,
                        uint16_t len)
{
    uint16_t index;
    ret_t ret;
    ret_t stop_ret;

    if ((buffer == NULL) || (len == 0U) || (bus_id != BSP_IIC_BUS_AT24C02))
    {
        return RET_INVALID_PARAM;
    }
    if (s_iic_initialized == 0U)
    {
        return RET_NOT_INIT;
    }
    ret = iic_start();
    if (ret == RET_OK)
    {
        ret = iic_write_byte((uint8_t)(slave_addr & 0xFEU));
    }
    for (index = 0U; (ret == RET_OK) && (index < len); index++)
    {
        ret = iic_write_byte(buffer[index]);
    }
    stop_ret = iic_stop();
    return (ret == RET_OK) ? stop_ret : ret;
}

ret_t bsp_iic_raw_read(bsp_iic_bus_id_t bus_id,
                       uint16_t slave_addr,
                       uint8_t *buffer,
                       uint16_t len)
{
    uint16_t index;
    ret_t ret;
    ret_t stop_ret;

    if ((buffer == NULL) || (len == 0U) || (bus_id != BSP_IIC_BUS_AT24C02))
    {
        return RET_INVALID_PARAM;
    }
    if (s_iic_initialized == 0U)
    {
        return RET_NOT_INIT;
    }
    ret = iic_start();
    if (ret == RET_OK)
    {
        ret = iic_write_byte((uint8_t)(slave_addr | 1U));
    }
    for (index = 0U; (ret == RET_OK) && (index < len); index++)
    {
        ret = iic_read_byte(&buffer[index],
                            (index + 1U < len) ? 1U : 0U);
    }
    stop_ret = iic_stop();
    return (ret == RET_OK) ? stop_ret : ret;
}
