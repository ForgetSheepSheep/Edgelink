/**
 * @file    bsp_spi.h
 * @brief   SPI BSP 驱动接口（结构体表管理）
 *
 * @note    本模块通过 spi_info_t 结构体表管理多路 SPI 外设，
 *          上层通过 bsp_spi_id_t 枚举索引选择对应的 SPI。
 *          当前支持 SPI1（PB13=SCK, PB14=MISO, PB15=MOSI）。
 */
#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "config.h"

/**
 * @brief   SPI 设备 ID 枚举
 */
typedef enum
{
    BSP_SPI_FLASH = 0U,     /**< 外部 SPI Flash（GD25Q128） */
    BSP_SPI_MAX
} bsp_spi_id_t;

/**
 * @brief   初始化指定 SPI 外设
 * @param   spi_id  SPI 设备 ID
 * @retval  RET_OK 成功; RET_INVALID_PARAM 参数非法
 */
ret_t bsp_spi_init(bsp_spi_id_t spi_id);

/**
 * @brief   SPI 全双工读写一个字节
 * @param   spi_id   SPI 设备 ID
 * @param   tx_data  要发送的字节
 * @retval  接收到的字节（超时返回 0xFF）
 */
uint8_t bsp_spi_read_write_byte(bsp_spi_id_t spi_id, uint8_t tx_data);

/**
 * @brief   SPI 全双工读写一个字节，并返回明确的错误码
 * @param   spi_id   SPI 设备 ID
 * @param   tx_data  要发送的字节
 * @param   rx_data  接收字节输出地址
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针;
 *          RET_INVALID_PARAM 参数非法; RET_TIMEOUT SPI硬件等待超时
 */
ret_t bsp_spi_read_write_byte_ex(bsp_spi_id_t spi_id,
                                 uint8_t tx_data,
                                 uint8_t *rx_data);

#endif /* __BSP_SPI_H__ */
