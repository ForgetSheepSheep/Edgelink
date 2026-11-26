#include "app_ui_snapshot/app_ui_snapshot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_ota/app_ota.h"
#include "app_ota_dl/ota_dl.h"
#include "app_boot_ab/app_boot_ab.h"
#include "app_fw_nvs/fw_nvs.h"
#include "app_version.h"

/*
 * 快照与变化计数只由本模块持有；所有写入与读取都在临界区内完成，
 * 保证显示任务拿到的是同一时刻的一致副本。
 */
static app_ui_snapshot_t s_ui_snapshot;
static app_ui_snapshot_change_t s_ui_changes;

/* OTA 服务状态映射到界面状态；映射集中在本适配层，LCD 不依赖 app_ota。 */
static uint8_t ui_ota_state_from_service(app_ota_state_t state)
{
    switch (state)
    {
        case APP_OTA_STATE_PREPARING:
        case APP_OTA_STATE_DOWNLOADING:
            return (uint8_t)APP_UI_OTA_STATE_DOWNLOADING;
        case APP_OTA_STATE_VERIFYING:
            return (uint8_t)APP_UI_OTA_STATE_VERIFYING;
        case APP_OTA_STATE_COMMITTING:
            return (uint8_t)APP_UI_OTA_STATE_COMMITTING;
        case APP_OTA_STATE_ERROR:
            return (uint8_t)APP_UI_OTA_STATE_ERROR;
        case APP_OTA_STATE_IDLE:
        default:
            return (uint8_t)APP_UI_OTA_STATE_IDLE;
    }
}

void app_ui_snapshot_set_network_online(uint8_t is_online)
{
    uint8_t value = (is_online != 0U) ? 1U : 0U;

    taskENTER_CRITICAL();
    if ((s_ui_snapshot.network_valid == 0U) ||
        (s_ui_snapshot.network_online != value))
    {
        s_ui_snapshot.network_valid = 1U;
        s_ui_snapshot.network_online = value;
        s_ui_changes.network++;
    }
    taskEXIT_CRITICAL();
}

void app_ui_snapshot_update_cloud(const app_net_snapshot_t *snapshot)
{
    app_ui_cloud_t cloud;

    if (snapshot == NULL)
    {
        return;
    }

    /* 先清零再填字段：保证结构体内填充字节一致，memcmp 才能用于判变化。 */
    (void)memset(&cloud, 0, sizeof(cloud));
    cloud.valid_slave_mask = (uint8_t)(snapshot->valid_slave_mask & 0x07U);
    cloud.poll_was_attempted = (snapshot->poll_was_attempted != 0U) ? 1U : 0U;
    cloud.led2_is_on = (snapshot->led2_is_on != 0U) ? 1U : 0U;
    cloud.control_holding_register_0 =
        snapshot->control_holding_registers[0U];
    cloud.control_holding_register_1 =
        snapshot->control_holding_registers[1U];
    cloud.control_status_register_1 =
        snapshot->control_status_registers[1U];
    cloud.analog_input_register_0 = snapshot->analog_input_registers[0U];
    cloud.io_coil_states = snapshot->io_coil_states;
    cloud.io_discrete_input_states = snapshot->io_discrete_input_states;

    taskENTER_CRITICAL();
    if ((s_ui_snapshot.cloud_valid == 0U) ||
        (memcmp(&s_ui_snapshot.cloud, &cloud, sizeof(cloud)) != 0))
    {
        s_ui_snapshot.cloud_valid = 1U;
        s_ui_snapshot.cloud = cloud;
        s_ui_changes.cloud++;
    }
    taskEXIT_CRITICAL();
}

void app_ui_snapshot_set_time(const char *time_text)
{
    char text[APP_UI_SNAPSHOT_TIME_TEXT_SIZE];
    size_t length;

    if (time_text == NULL)
    {
        return;
    }

    length = strlen(time_text);
    if (length >= (size_t)APP_UI_SNAPSHOT_TIME_TEXT_SIZE)
    {
        return;
    }

    (void)memset(text, 0, sizeof(text));
    (void)memcpy(text, time_text, length + 1U);

    taskENTER_CRITICAL();
    if ((s_ui_snapshot.time_valid == 0U) ||
        (memcmp(s_ui_snapshot.time_text, text, sizeof(text)) != 0))
    {
        s_ui_snapshot.time_valid = 1U;
        (void)memcpy(s_ui_snapshot.time_text, text, sizeof(text));
        s_ui_changes.time++;
    }
    taskEXIT_CRITICAL();
}

void app_ui_snapshot_poll_ota(void)
{
    ota_dl_status_t status;
    app_ui_ota_t ota;

    ota_dl_get_status(&status);

    (void)memset(&ota, 0, sizeof(ota));
    ota.state = ui_ota_state_from_service(status.state);
    ota.received = status.received;
    ota.total = status.total;
    ota.target_version = status.target_version;
    if (status.total > 0U)
    {
        ota.percent = (uint8_t)(((uint64_t)status.received * 100ULL) /
                                (uint64_t)status.total);
    }
    if (ota.percent > 100U)
    {
        ota.percent = 100U;
    }

    taskENTER_CRITICAL();
    if (memcmp(&s_ui_snapshot.ota, &ota, sizeof(ota)) != 0)
    {
        s_ui_snapshot.ota = ota;
        s_ui_changes.ota++;
    }
    taskEXIT_CRITICAL();
}

void app_ui_snapshot_mark_ota_rebooting(void)
{
    taskENTER_CRITICAL();
    s_ui_snapshot.ota.state = (uint8_t)APP_UI_OTA_STATE_REBOOTING;
    s_ui_snapshot.ota.percent = 100U;
    s_ui_changes.ota++;
    taskEXIT_CRITICAL();
}

void app_ui_snapshot_refresh_boot(void)
{
    app_ui_ab_t ab;
    boot_param_t param;
    fw_nvs_params_t nvs;
    ret_t result;

    (void)memset(&ab, 0, sizeof(ab));
    ab.active_slot = APP_UI_SNAPSHOT_SLOT_NONE;
    ab.pending_slot = APP_UI_SNAPSHOT_SLOT_NONE;
    ab.running_version = (uint32_t)APP_FW_VERSION;

    result = boot_param_load(&param);
    if (result == RET_OK)
    {
        ab.valid = 1U;
        ab.active_slot = (uint8_t)param.active_slot;
        ab.pending_slot = (param.pending_slot == BOOT_SLOT_NONE) ?
                          APP_UI_SNAPSHOT_SLOT_NONE :
                          (uint8_t)param.pending_slot;
        ab.generation = param.generation;
    }

    result = fw_nvs_get(&nvs);
    if (result == RET_OK)
    {
        ab.rollback_flag = (uint8_t)nvs.rollback_flag;
    }

    taskENTER_CRITICAL();
    if (memcmp(&s_ui_snapshot.ab, &ab, sizeof(ab)) != 0)
    {
        s_ui_snapshot.ab = ab;
        s_ui_changes.ab++;
    }
    taskEXIT_CRITICAL();
}

void app_ui_snapshot_get(app_ui_snapshot_t *snapshot,
                         app_ui_snapshot_change_t *changes)
{
    taskENTER_CRITICAL();
    if (snapshot != NULL)
    {
        *snapshot = s_ui_snapshot;
    }
    if (changes != NULL)
    {
        *changes = s_ui_changes;
    }
    taskEXIT_CRITICAL();
}
