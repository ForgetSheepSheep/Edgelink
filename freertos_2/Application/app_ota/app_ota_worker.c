#include "app_ota_worker.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define OTA_WORKER_QUEUE_LEN 4U
#define OTA_WORKER_STACK_WORDS 512U
#define OTA_WORKER_PRIORITY 3U

typedef enum
{
    OTA_CMD_START = 0,
    OTA_CMD_CHUNK,
    OTA_CMD_FINISH,
    OTA_CMD_ABORT
} ota_cmd_type_t;

typedef struct
{
    ota_cmd_type_t type;
    app_ota_meta_t meta;
    uint32_t offset;
    uint32_t len;
    uint8_t payload[APP_OTA_CHUNK_MAX];
} ota_cmd_t;

static QueueHandle_t s_queue = NULL;
static TaskHandle_t s_task = NULL;
static app_ota_result_cb_t s_cb = NULL;

/* 把 core 当前进度连同事件一起上报，调用方无需再查一次。 */
static void ota_notify(app_ota_event_t event, ret_t result)
{
    app_ota_progress_t snap;

    if (s_cb == NULL)
    {
        return;
    }
    app_ota_get_progress(&snap);
    s_cb(event, result, snap.received, snap.total);
}

static void ota_task(void *arg)
{
    ota_cmd_t cmd;
    ret_t ret;

    (void)arg;

    for (;;)
    {
        if (xQueueReceive(s_queue, &cmd, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        switch (cmd.type)
        {
        case OTA_CMD_START:
            ret = app_ota_start(&cmd.meta);
            if (ret != RET_OK)
            {
                app_ota_abort();
            }
            ota_notify(APP_OTA_EVENT_STARTED, ret);
            break;

        case OTA_CMD_CHUNK:
            ret = app_ota_write_chunk(cmd.offset, cmd.payload, cmd.len);
            if (ret != RET_OK)
            {
                app_ota_abort();
            }
            ota_notify(APP_OTA_EVENT_CHUNK_WRITTEN, ret);
            break;

        case OTA_CMD_FINISH:
            ret = app_ota_finish();
            if (ret != RET_OK)
            {
                app_ota_abort();
            }
            ota_notify(APP_OTA_EVENT_FINISHED, ret);
            break;

        case OTA_CMD_ABORT:
            app_ota_abort();
            ota_notify(APP_OTA_EVENT_ABORTED, RET_OK);
            break;

        default:
            break;
        }
    }
}

ret_t app_ota_worker_init(app_ota_result_cb_t result_cb)
{
    ret_t ret;

    if ((s_queue != NULL) || (s_task != NULL))
    {
        return RET_ALREADY_INIT;
    }

    ret = app_ota_init();
    if (ret != RET_OK)
    {
        return ret;
    }

    s_queue = xQueueCreate(OTA_WORKER_QUEUE_LEN, sizeof(ota_cmd_t));
    if (s_queue == NULL)
    {
        return RET_NO_MEMORY;
    }

    s_cb = result_cb;

    if (xTaskCreate(ota_task, "ota_worker",
                    OTA_WORKER_STACK_WORDS, NULL,
                    OTA_WORKER_PRIORITY, &s_task) != pdPASS)
    {
        vQueueDelete(s_queue);
        s_queue = NULL;
        s_cb = NULL;
        return RET_TASK_CREATE_ERROR;
    }
    return RET_OK;
}

ret_t app_ota_post_start(const app_ota_meta_t *meta)
{
    ota_cmd_t cmd;

    if (meta == NULL)
    {
        return RET_NULL_PTR;
    }
    if (s_queue == NULL)
    {
        return RET_NOT_INIT;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.type = OTA_CMD_START;
    cmd.meta = *meta;
    return (xQueueSend(s_queue, &cmd, 0U) == pdTRUE) ? RET_OK : RET_QUEUE_FULL;
}

static ret_t ota_post_chunk_inner(uint32_t offset,
                                  const uint8_t *data,
                                  uint32_t len,
                                  TickType_t wait)
{
    ota_cmd_t cmd;

    if (data == NULL)
    {
        return RET_NULL_PTR;
    }
    if ((len == 0U) || (len > APP_OTA_CHUNK_MAX))
    {
        return RET_LENGTH_ERROR;
    }
    if (s_queue == NULL)
    {
        return RET_NOT_INIT;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.type = OTA_CMD_CHUNK;
    cmd.offset = offset;
    cmd.len = len;
    (void)memcpy(cmd.payload, data, len);
    return (xQueueSend(s_queue, &cmd, wait) == pdTRUE) ? RET_OK : RET_QUEUE_FULL;
}

ret_t app_ota_post_chunk(uint32_t offset, const uint8_t *data, uint32_t len)
{
    return ota_post_chunk_inner(offset, data, len, 0U);
}

ret_t app_ota_post_chunk_wait(uint32_t offset,
                              const uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms)
{
    return ota_post_chunk_inner(offset, data, len, pdMS_TO_TICKS(timeout_ms));
}

static ret_t ota_post_simple_inner(ota_cmd_type_t type, TickType_t wait)
{
    ota_cmd_t cmd;

    if (s_queue == NULL)
    {
        return RET_NOT_INIT;
    }

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.type = type;
    return (xQueueSend(s_queue, &cmd, wait) == pdTRUE) ? RET_OK : RET_QUEUE_FULL;
}

ret_t app_ota_post_finish(void)
{
    return ota_post_simple_inner(OTA_CMD_FINISH, 0U);
}

ret_t app_ota_post_abort(void)
{
    return ota_post_simple_inner(OTA_CMD_ABORT, 0U);
}

ret_t app_ota_post_finish_wait(uint32_t timeout_ms)
{
    return ota_post_simple_inner(OTA_CMD_FINISH, pdMS_TO_TICKS(timeout_ms));
}

ret_t app_ota_post_abort_wait(uint32_t timeout_ms)
{
    return ota_post_simple_inner(OTA_CMD_ABORT, pdMS_TO_TICKS(timeout_ms));
}
