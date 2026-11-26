#ifndef __APP_OTA_H__
#define __APP_OTA_H__

#include "config.h"

/*
 * 云端下发的固件包描述（传输层解析出该结构后交给 OTA）。
 * fw_crc32 必须是标准 CRC-32/ISO-HDLC 数值（与 ota_crc 算法一致）；传输层负责把云端的小端十六进制表示归一化后再填入。
 * fw_version 编码为 0x00010203 表示 v1.2.3，仅做记录透传，OTA 本身不做版本比较。
 * target_slot 必须是当前运行槽的对端：运行 A 则下载 B，反之亦然。
 */
typedef struct
{
    uint32_t fw_size;
    uint32_t fw_crc32;
    uint32_t fw_version;
    uint32_t target_slot;
} app_ota_meta_t;

/* OTA 同步状态机。单实例，任何时刻只允许一轮升级。 */
typedef enum
{
    APP_OTA_STATE_IDLE = 0, /* 空闲，可接受 start */
    APP_OTA_STATE_PREPARING,/* 正在擦除外部 Flash（短暂中间态） */
    APP_OTA_STATE_DOWNLOADING, /* 已建档，等待按序写入分片 */
    APP_OTA_STATE_VERIFYING,/* 分片收齐，正在全量复验（短暂中间态） */
    APP_OTA_STATE_COMMITTING, /* 复验通过，正在写信息头与启动参数（短暂中间态） */
    APP_OTA_STATE_ERROR     /* 出错保持，须 abort 回 IDLE 后才能重来 */
} app_ota_state_t;

/* 当前下载进度快照，供传输层做断点续传与进度条显示。 */
typedef struct
{
    uint32_t received;
    uint32_t total;
} app_ota_progress_t;

/*
 * @brief  初始化内部 Flash 与外部 GD25Q128，并校验 JEDEC ID。
 * @note   可重复调用；重复调用返回 RET_ALREADY_INIT 视为成功。
 */
ret_t app_ota_init(void);

/*
 * @brief  建档并擦除暂存区，开始一轮升级。
 * @param  meta 固件包描述。
 * @return RET_OK 则进入 DOWNLOADING；RET_BUSY 表示上一轮未结束；
 *         RET_LENGTH_ERROR / RET_PARTITION_ERROR / RET_INVALID_STATE 见实现。
 * @note   建档前要求启动参数处于 CONFIRMED 且 active 为本槽，
 *         防止在上一轮未确认时叠加开新一轮导致无法回滚。
 */
ret_t app_ota_start(const app_ota_meta_t *meta);

/*
 * @brief  按序写入一个分片（必须严格从 s_next_offset 续写，不支持乱序/重叠）。
 * @param  offset 本片在固件中的字节偏移，必须等于已接收字节数。
 * @param  data   本片数据；len 为 0 或超过 APP_OTA_CHUNK_MAX 视为非法。
 */
ret_t app_ota_write_chunk(uint32_t offset, const uint8_t *data, uint32_t len);

/*
 * @brief  全量复验并提交：流式 CRC → 外存回读 CRC → 向量头 → 信息头 → 启动参数。
 * @return RET_OK 表示已写入 DOWNLOAD_READY，调用方随后应复位生效；
 *         本函数自身不复位（见 app_ota_reboot）。
 */
ret_t app_ota_finish(void);

/* @brief  放弃当前轮次并回 IDLE。任何状态下均可调用，永不失败。 */
void app_ota_abort(void);

/*
 * @brief  复位生效（Bootloader 在下次上电时搬运并进入试运行）。
 * @note   仅在 app_ota_finish 返回 RET_OK 后调用。
 *         若注册了复位前钩子，会先执行钩子（用于把"即将重启"显示到
 *         LCD/日志）再复位。
 */
void app_ota_reboot(void);

/*
 * @brief  注册复位前钩子（可为 NULL）。
 * @note   在 ota_worker 任务上下文执行，允许短暂延时（如等 LCD 画完）；
 *         返回值忽略，钩子返回后立即 NVIC_SystemReset()。
 */
typedef void (*app_ota_reboot_hook_t)(void);
void app_ota_set_reboot_hook(app_ota_reboot_hook_t hook);

/*
 * @brief  新固件试运行自检通过后调用，把启动参数推进为 CONFIRMED。
 * @return 无试运行（状态非 TRIAL）时直接返回 RET_OK。
 */
ret_t app_boot_confirm_running(void);

app_ota_state_t app_ota_get_state(void);

/* @brief  读取当前进度；IDLE/ERROR 下 received 为 0。 */
void app_ota_get_progress(app_ota_progress_t *progress);

#endif /* __APP_OTA_H__ */
