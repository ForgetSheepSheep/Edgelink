#ifndef __OTA_CRC_H__
#define __OTA_CRC_H__

#include "config.h"

/*
 * 标准 CRC-32/ISO-HDLC 参数（与以太网/ZIP 同一款）。
 * 多项式取反射形式 0xEDB88320，初值与最终异或都是 0xFFFFFFFF。
 * 牢记一条：中间值和最终值是两个东西，中间值永不异或，最终值只异或一次。
 */
#define CRC32_INIT_VALUE 0xFFFFFFFFUL
#define CRC32_XOR_VALUE  0xFFFFFFFFUL
#define CRC32_POLY_VALUE 0xEDB88320UL

/*
 * @brief  把一段新数据续算进已有的 CRC 中间值。
 * @param  crc  首片传 CRC32_INIT_VALUE，后续片传上一次的返回值。
 * @param  buf  本片数据，允许包含 0x00；NULL 表示本片无数据，直接返回原值。
 * @param  len  本片字节数。
 * @return 仍未做最终异或的中间值，只能继续往下传，不能拿去和云端比对。
 */
uint32_t ota_crc32_update(uint32_t crc,
                          const uint8_t *buf,
                          uint32_t len);

/*
 * @brief  一次性算出一块连续内存的标准 CRC32。
 * @note   内部已处理初值和最终异或，返回值可直接与云端下发的 fw_crc32 比较。
 */
uint32_t ota_crc32_calc(const uint8_t *buf, uint32_t len);

#endif /* __OTA_CRC_H__ */
