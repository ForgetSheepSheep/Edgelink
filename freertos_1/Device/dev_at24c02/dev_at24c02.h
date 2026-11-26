/**
 * @file    dev_at24c02.h
 * @brief   AT24C02 EEPROM Device 层接口（256字节，8字节页）
 *
 * @note    App 只调用本层业务 API，不接触 I2C 原语和页拆分细节。
 *          参数区提供基于槽位轮换的磨损平衡写，掉电安全。
 */

#ifndef __DEV_AT24C02_H__
#define __DEV_AT24C02_H__

#include "ret_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 芯片总容量（字节）。 */
#define DEV_AT24C02_TOTAL_SIZE     (256U)
/** 页大小（字节）：单次写不得跨页，否则页内回卷覆盖本页开头。 */
#define DEV_AT24C02_PAGE_SIZE      (8U)
/** 参数区槽位数：每槽正好一个页。 */
#define DEV_AT24C02_PARAM_SLOTS    (8U)

ret_t dev_at24c02_init(void);

ret_t dev_at24c02_read(uint16_t address,
                       uint8_t *buffer,
                       uint16_t len);

ret_t dev_at24c02_write(uint16_t address,
                        const uint8_t *buffer,
                        uint16_t len);

ret_t dev_at24c02_param_format(void);

ret_t dev_at24c02_param_write(uint16_t value);

ret_t dev_at24c02_param_read(uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_AT24C02_H__ */
