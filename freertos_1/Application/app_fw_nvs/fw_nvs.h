#ifndef __FW_NVS_H__
#define __FW_NVS_H__

#include "config.h"

/*
 * App 侧 OTA 固件参数 NVS：独占内部 Flash 一页。
 *
 * 地址：0x0807E000，2KB。保留区最后一页；BootLoader(<=0x08007FFF)、
 * SlotA(<=0x0803FFFF)、SlotB(<=0x08077FFF)、启动参数页(>=0x0807F000)
 * 都不触碰；0x0807E800 留作隔离带。
 *
 * 原理（F4 原语“槽位轮换 + seq + CRC”的 NOR 移植）：
 * EEPROM 版靠直接覆写，NOR 只能 1->0，所以页内改 append-only：
 * 每条记录 32 字节，1 页放 64 条；写找空槽追加，读扫全页取
 * “CRC 有效且 seq 最大”者；页满则整理（整页擦后把最新一条写回 0 号槽）。
 *
 * 掉电语义（单页方案的固有窗口，不隐瞒）：
 * - 追加被打断：最多毁正在写的槽，旧记录完好，读自动回退旧值；
 * - 整理时擦除瞬间掉电：整页变空，读返回 RET_NVS_NOT_FOUND，
 *   调用方按首次上电处理（版本号未知、回滚标志清零）。
 *
 * 并发：与 boot_param 一致，无内部锁；多任务同时 set 须由调用方串行化。
 * 上下文：erase/program 涉及 FMC 解锁，只允许任务上下文调用，禁止 ISR。
 */
#define FW_NVS_PAGE_ADDR 0x0807E000UL
#define FW_NVS_PAGE_SIZE 2048UL

/* 记录魔数：0x534E5746 小端字节为 "FWNS"（FirmWare NvS）。 */
#define FW_NVS_MAGIC 0x534E5746UL
#define FW_NVS_FORMAT_VERSION 1UL

/* 一条记录 32 字节，页内 64 槽。 */
#define FW_NVS_RECORD_SIZE 32UL
#define FW_NVS_SLOT_COUNT (FW_NVS_PAGE_SIZE / FW_NVS_RECORD_SIZE)

/* OTA 固件参数：固件版本(0x00010203=v1.2.3) + 回滚标志 + 上次确认槽。 */
typedef struct
{
    uint32_t fw_version;
    uint32_t rollback_flag; /* 0=未回滚过，1=发生过回滚 */
    uint32_t active_slot;   /* 记录写入时已确认的槽，仅供诊断 */
} fw_nvs_params_t;

/* @brief  初始化（dev_flash_init，重复调用视为成功）。 */
ret_t fw_nvs_init(void);

/* @brief  格式化：整页擦除。首次使用前或损坏无法恢复时调用一次。 */
ret_t fw_nvs_format(void);

/*
 * @brief  写入一组参数（追加新记录；页满自动整理）。
 * @return RET_OK / RET_NULL_PTR / RET_NVS_INVALID_DATA(字段非法) / Flash 错误。
 */
ret_t fw_nvs_set(const fw_nvs_params_t *params);

/*
 * @brief  读出最新一组参数。
 * @return RET_OK；RET_NVS_NOT_FOUND=整页空白(从未写入或整理时掉电)；
 *         RET_CRC_ERROR=有非空槽但无一有效(写入被打断且旧记录已被整理丢弃的极端情形)。
 */
ret_t fw_nvs_get(fw_nvs_params_t *params);

/* @brief  读-改-写：置位回滚标志（版本号与槽位保持最新值）。 */
ret_t fw_nvs_mark_rollback(void);

/* @brief  读-改-写：清零回滚标志（版本号与槽位保持最新值）。 */
ret_t fw_nvs_clear_rollback(void);

#endif /* __FW_NVS_H__ */
