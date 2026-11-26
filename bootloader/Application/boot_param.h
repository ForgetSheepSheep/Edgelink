#ifndef __BOOT_PARAM_H__
#define __BOOT_PARAM_H__

#include "boot_protocol.h"

/**
 * @brief  读取主备参数页并输出 generation 最新的有效副本。
 * @param  param 成功时接收完整启动参数。
 * @return RET_OK 表示读到有效副本；RET_MAGIC_ERROR 只表示两个记录都为全 0xFF；
 *         RET_CRC_ERROR 表示没有有效副本且至少一个记录不是擦除态；
 *         还可能返回 RET_FLASH_READ_ERROR 或 RET_NULL_PTR。
 */
ret_t boot_param_load(boot_param_t *param);

/**
 * @brief  把参数保存到无效页或 generation 较旧的页，并回读验证。
 * @param  param RAM 工作副本；函数会更新 generation 和 param_crc32。
 * @return RET_OK 或具体 Flash 错误。
 */
ret_t boot_param_save(boot_param_t *param);

/**
 * @brief  生成首次启动默认参数，只修改 RAM，不写 Flash。
 * @param  param 输出参数；默认 active_slot 为 Slot A。
 */
void boot_param_make_default(boot_param_t *param);

#endif /* __BOOT_PARAM_H__ */




