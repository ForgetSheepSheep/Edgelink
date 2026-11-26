/**
 * @file    app_ui_snapshot.h
 * @brief   界面状态快照适配层
 *
 * @note    本模块只保存"供界面使用的系统快照"，不拥有任何业务状态：
 *          - 网络在线状态与云端业务摘要来自 app_net（组合根回调写入）；
 *          - OTA 状态来自 app_ota / ota_dl（健康任务周期轮询写入）；
 *          - 启动与 A/B 状态来自 Boot 服务（boot_param + fw_nvs）。
 *          快照按分区维护变化计数，显示任务按"最新快照 + 变化标记"
 *          拉取刷新，不经过消息队列，避免旧值覆盖新值或关键状态丢失。
 */
#ifndef __APP_UI_SNAPSHOT_H__
#define __APP_UI_SNAPSHOT_H__

#include "config.h"
#include "app_net/app_net.h"

/** 时间文本长度："YYYY-MM-DD HH:MM:SS" + '\0'。 */
#define APP_UI_SNAPSHOT_TIME_TEXT_SIZE 20U

/** 槽位编码；0/1 对应 A/B，0xFF 表示未知或没有待升级槽。 */
#define APP_UI_SNAPSHOT_SLOT_A     0U
#define APP_UI_SNAPSHOT_SLOT_B     1U
#define APP_UI_SNAPSHOT_SLOT_NONE  0xFFU

/** OTA 界面状态（与 app_ota_state_t 解耦，数值需保持稳定）。 */
typedef enum
{
    APP_UI_OTA_STATE_IDLE = 0U,
    APP_UI_OTA_STATE_DOWNLOADING,
    APP_UI_OTA_STATE_VERIFYING,
    APP_UI_OTA_STATE_COMMITTING,
    APP_UI_OTA_STATE_ERROR,
    APP_UI_OTA_STATE_REBOOTING
} app_ui_ota_state_t;

/** 云端业务摘要：只保留设备页与诊断页需要的字段。 */
typedef struct
{
    uint8_t valid_slave_mask;
    uint8_t poll_was_attempted;
    uint8_t led2_is_on;
    uint16_t control_holding_register_0;
    uint16_t control_holding_register_1;
    uint16_t control_status_register_1;
    uint16_t analog_input_register_0;
    uint8_t io_coil_states;
    uint8_t io_discrete_input_states;
} app_ui_cloud_t;

/** OTA 会话摘要；percent 按已写入字节数计算。 */
typedef struct
{
    uint8_t state;   /* app_ui_ota_state_t */
    uint8_t percent; /* 0..100 */
    uint32_t received;
    uint32_t total;
    uint32_t target_version; /* 云端声明的目标版本，0 表示未知 */
} app_ui_ota_t;

/** 启动与 A/B 摘要；valid=0 时其余字段无效。 */
typedef struct
{
    uint8_t valid;
    uint8_t active_slot;   /* APP_UI_SNAPSHOT_SLOT_* */
    uint8_t pending_slot;  /* APP_UI_SNAPSHOT_SLOT_* 或 NONE */
    uint8_t rollback_flag; /* 1 表示发生过回滚（fw_nvs 锁存） */
    uint32_t running_version;
    uint32_t generation;
} app_ui_ab_t;

/** 界面系统快照；值语义，消费者整份复制，不传递指针到业务层。 */
typedef struct
{
    uint8_t network_valid;
    uint8_t network_online;
    uint8_t cloud_valid;
    app_ui_cloud_t cloud;
    uint8_t time_valid;
    char time_text[APP_UI_SNAPSHOT_TIME_TEXT_SIZE];
    app_ui_ota_t ota;
    app_ui_ab_t ab;
} app_ui_snapshot_t;

/** 分区变化计数：与消费者上次记录的计数不同即表示该分区有新数据。 */
typedef struct
{
    uint32_t network;
    uint32_t cloud;
    uint32_t time;
    uint32_t ota;
    uint32_t ab;
} app_ui_snapshot_change_t;

/*
 * 以下写入接口由组合根或服务调用；可在调度器启动前后调用，
 * 不可在中断上下文调用。写入只更新快照与变化计数，不做任何绘制。
 */
void app_ui_snapshot_set_network_online(uint8_t is_online);
void app_ui_snapshot_update_cloud(const app_net_snapshot_t *snapshot);
void app_ui_snapshot_set_time(const char *time_text);
void app_ui_snapshot_poll_ota(void);
void app_ui_snapshot_mark_ota_rebooting(void);
void app_ui_snapshot_refresh_boot(void);

/* @brief 读取最新快照与变化计数；两个输出参数均可为 NULL。 */
void app_ui_snapshot_get(app_ui_snapshot_t *snapshot,
                         app_ui_snapshot_change_t *changes);

#endif /* __APP_UI_SNAPSHOT_H__ */
