#include "app_ota_dl/ota_dl.h"
#include "../app_boot_ab/boot_protocol.h"
#include "app_ota/app_ota_worker.h"
#include "dev_net/dev_net.h"
#include "cJSON/cJSON.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define OTA_DL_TOPIC_ATTRIBUTES "v1/devices/me/attributes"
#define OTA_DL_TOPIC_FW_RESPONSE "v2/fw/response/+/chunk/#"

#define OTA_DL_TIMEOUT_MS 15000U
#define OTA_DL_MAX_RETRY 3U
#define OTA_DL_REQUEST_GAP_MS 50U
#define OTA_DL_QUEUE_LENGTH 4U
#define OTA_DL_TEXT_SIZE 768U
#define OTA_DL_NOTIFY_JSON_SIZE 256U
/** worker 结果投递：队列满时最长等待，仍失败则转入兜底槽。 */
#define OTA_DL_RESULT_WAIT_MS 500U
/** FINISH/ABORT 控制命令投递：队列满时最长等待，失败进入重试状态。 */
#define OTA_DL_CONTROL_WAIT_MS 200U
/** 进入 finish 阶段（校验+提交）后的完成上限，超过即放弃本轮。 */
#define OTA_DL_FINISH_TIMEOUT_MS 30000U
/** ABORT 投递失败后的重试间隔。 */
#define OTA_DL_ABORT_RETRY_MS 500U

typedef enum
{
    OTA_DL_MSG_META = 0, /* 固件通知 JSON 文本 */
    OTA_DL_MSG_CHUNK,    /* 组装完的分片二进制 */
    OTA_DL_MSG_RESULT    /* worker 回调事件 */
} ota_dl_msg_type_t;

typedef struct
{
    ota_dl_msg_type_t type;
    app_ota_meta_t meta;
    app_ota_event_t event;
    ret_t result;
    uint32_t received;
    uint32_t total;
    uint32_t request_id;
    uint32_t chunk_id;
    uint32_t data_len;
    uint8_t data[OTA_DL_CHUNK_SIZE];
    char json[OTA_DL_NOTIFY_JSON_SIZE];
} ota_dl_msg_t;

typedef struct
{
    char text[OTA_DL_TEXT_SIZE];
    uint16_t text_len;
    uint16_t json_depth;
    uint8_t in_json;
    uint8_t json_in_string;
    uint8_t json_escape;
    uint8_t in_binary;
    uint32_t binary_remaining;
    uint32_t binary_expected;
    uint32_t binary_len;
    uint32_t request_id;
    uint32_t chunk_id;
    uint8_t binary[OTA_DL_CHUNK_SIZE];
} ota_dl_parser_t;

static QueueHandle_t s_dl_queue = NULL;
static ota_dl_parser_t s_parser;
static uint8_t s_downloading = 0U;
static uint8_t s_waiting_chunk = 0U;
static uint32_t s_session_id = 0U;
static uint32_t s_fw_size = 0U;
static uint32_t s_expected_chunk = 0U;
static uint32_t s_requested_offset = 0U;
static uint32_t s_retry_count = 0U;
static TickType_t s_request_tick = 0U;
static uint32_t s_last_percent = 0U;
static uint32_t s_target_version = 0U;
/** FINISH 已入 worker 队列，等待 FINISHED 事件。 */
static uint8_t s_finish_pending = 0U;
/** FINISH 入队失败，等待重试；与 s_finish_pending 互斥。 */
static uint8_t s_finish_post_pending = 0U;
/** 进入 finish 阶段的时刻，用于 OTA_DL_FINISH_TIMEOUT_MS 超时判定。 */
static TickType_t s_finish_tick = 0U;
/** ABORT 入队失败，等待重试。 */
static uint8_t s_abort_pending = 0U;
static TickType_t s_abort_retry_tick = 0U;
/**
 * worker 结果兜底槽：结果队列满时先存这里，ota_dl_tick 下一轮消费。
 * 同一时刻 worker 只会有一个命令在途，因此单槽足够；若在兜底槽未消费
 * 期间又收到新结果，保留最新事件（此时旧事件对应的会话已在收尾）。
 */
static ota_dl_msg_t s_pending_result;
static volatile uint8_t s_pending_result_valid = 0U;

static ret_t ota_dl_post(const ota_dl_msg_t *msg)
{
    if (s_dl_queue == NULL)
    {
        return RET_NOT_INIT;
    }
    return (xQueueSend(s_dl_queue, msg, 0U) == pdTRUE) ? RET_OK : RET_QUEUE_FULL;
}

/* worker 回调：ota_worker 任务上下文，只做投递。 */
static void ota_dl_on_worker(app_ota_event_t event,
                             ret_t result,
                             uint32_t received,
                             uint32_t total)
{
    ota_dl_msg_t msg;

    (void)memset(&msg, 0, sizeof(msg));
    msg.type = OTA_DL_MSG_RESULT;
    msg.event = event;
    msg.result = result;
    msg.received = received;
    msg.total = total;

    /*
     * 结果事件决定会话下一步，不能丢：先带等待入队让 app_net 任务
     * 消费出空位；仍失败则写入兜底槽，由 ota_dl_tick 保证最终被处理。
     */
    if ((s_dl_queue != NULL) &&
        (xQueueSend(s_dl_queue, &msg,
                    pdMS_TO_TICKS(OTA_DL_RESULT_WAIT_MS)) == pdTRUE))
    {
        return;
    }

    taskENTER_CRITICAL();
    s_pending_result = msg;
    s_pending_result_valid = 1U;
    taskEXIT_CRITICAL();

    DBG_Error("OTA result queue full; stored in fallback slot event=%u\r\n",
              (unsigned int)event);
}

static ret_t ota_dl_parse_crc32(const char *text, uint32_t *value)
{
    const char *digits;
    uint32_t digit_count = 0U;
    uint32_t acc = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return RET_NULL_PTR;
    }

    digits = text;
    if ((digits[0] == '0') && ((digits[1] == 'x') || (digits[1] == 'X')))
    {
        digits += 2;
    }

    while (digits[digit_count] != '\0')
    {
        char c = digits[digit_count];
        uint32_t nibble;

        if ((c >= '0') && (c <= '9'))
        {
            nibble = (uint32_t)(c - '0');
        }
        else if ((c >= 'a') && (c <= 'f'))
        {
            nibble = (uint32_t)(c - 'a') + 10U;
        }
        else if ((c >= 'A') && (c <= 'F'))
        {
            nibble = (uint32_t)(c - 'A') + 10U;
        }
        else
        {
            return RET_PARSE_ERROR;
        }

        if (digit_count >= 8U)
        {
            return RET_PARSE_ERROR;
        }
        acc = (acc << 4) | nibble;
        digit_count++;
    }

    if (digit_count == 0U)
    {
        return RET_PARSE_ERROR;
    }

    *value = ((acc & 0x000000FFUL) << 24) |
             ((acc & 0x0000FF00UL) << 8) |
             ((acc & 0x00FF0000UL) >> 8) |
             ((acc & 0xFF000000UL) >> 24);
    return RET_OK;
}

static ret_t ota_dl_parse_version(const char *text, uint32_t *value)
{
    unsigned long parts[3] = {0UL, 0UL, 0UL};
    const char *cursor;
    char *end;
    uint32_t count = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return RET_NULL_PTR;
    }

    cursor = text;
    if ((*cursor == 'V') || (*cursor == 'v'))
    {
        cursor++;
    }

    while (count < 3U)
    {
        if ((*cursor < '0') || (*cursor > '9'))
        {
            return RET_PARSE_ERROR;
        }
        parts[count] = strtoul(cursor, &end, 10);
        if ((end == cursor) || (parts[count] > 255UL))
        {
            return RET_PARSE_ERROR;
        }
        count++;

        if (*end == '\0')
        {
            break;
        }
        if (*end != '.')
        {
            return RET_PARSE_ERROR;
        }
        cursor = end + 1;
    }

    if ((count == 0U) || (*end != '\0'))
    {
        return RET_PARSE_ERROR;
    }

    *value = ((uint32_t)parts[0] << 16) |
             ((uint32_t)parts[1] << 8) |
             (uint32_t)parts[2];
    return RET_OK;
}

/* 文本头：v2/fw/response/{rid}/chunk/{cid},{bytes}, 之后进入二进制体。 */
static uint8_t ota_dl_try_enter_binary(void)
{
    const char *prefix;
    const char *cursor;
    char *end;
    unsigned long rid;
    unsigned long cid;
    unsigned long bytes;

    prefix = strstr(s_parser.text, "v2/fw/response/");
    if (prefix == NULL)
    {
        return 0U;
    }

    cursor = prefix + strlen("v2/fw/response/");
    rid = strtoul(cursor, &end, 10);
    if ((end == cursor) || (strncmp(end, "/chunk/", 7) != 0))
    {
        return 0U;
    }

    cursor = end + 7;
    cid = strtoul(cursor, &end, 10);
    if ((end == cursor) || (*end != ','))
    {
        return 0U;
    }

    cursor = end + 1;
    bytes = strtoul(cursor, &end, 10);
    if ((end == cursor) || (*end != ','))
    {
        return 0U;
    }

    /* 头不完整时等更多字节；非法长度直接丢弃本段文本。 */
    if ((bytes == 0UL) || (bytes > OTA_DL_CHUNK_SIZE))
    {
        (void)memset(&s_parser, 0, sizeof(s_parser));
        return 1U;
    }

    s_parser.request_id = (uint32_t)rid;
    s_parser.chunk_id = (uint32_t)cid;
    s_parser.binary_remaining = (uint32_t)bytes;
    s_parser.binary_expected = (uint32_t)bytes;
    s_parser.binary_len = 0U;
    s_parser.in_binary = 1U;
    s_parser.text_len = 0U;
    s_parser.text[0] = '\0';
    return 1U;
}

/* 完整 JSON 到达：含 fw 四元组即视为固件通知，其余丢弃。 */
static void ota_dl_try_handle_json(void)
{
    const char *json;
    ota_dl_msg_t msg;
    size_t copy_len;

    json = strchr(s_parser.text, '{');
    if ((json == NULL) ||
        (strstr(json, "fw_title") == NULL) ||
        (strstr(json, "fw_version") == NULL) ||
        (strstr(json, "fw_size") == NULL) ||
        (strstr(json, "fw_checksum") == NULL))
    {
        return;
    }

    (void)memset(&msg, 0, sizeof(msg));
    msg.type = OTA_DL_MSG_META;
    copy_len = strlen(json);
    if (copy_len > (sizeof(msg.json) - 1U))
    {
        copy_len = sizeof(msg.json) - 1U;
    }
    (void)memcpy(msg.json, json, copy_len);
    msg.json[copy_len] = '\0';

    if (ota_dl_post(&msg) != RET_OK)
    {
        DBG_Error("OTA notify queue full; dropped\r\n");
    }
}

void ota_dl_feed(uint8_t byte)
{
    if (s_parser.in_binary != 0U)
    {
        if (s_parser.binary_len < sizeof(s_parser.binary))
        {
            s_parser.binary[s_parser.binary_len++] = byte;
        }
        if (s_parser.binary_remaining > 0U)
        {
            s_parser.binary_remaining--;
        }

        if (s_parser.binary_remaining == 0U)
        {
            if (s_parser.binary_len == s_parser.binary_expected)
            {
                ota_dl_msg_t msg;

                (void)memset(&msg, 0, sizeof(msg));
                msg.type = OTA_DL_MSG_CHUNK;
                msg.request_id = s_parser.request_id;
                msg.chunk_id = s_parser.chunk_id;
                msg.data_len = s_parser.binary_len;
                (void)memcpy(msg.data, s_parser.binary, s_parser.binary_len);

                if (ota_dl_post(&msg) != RET_OK)
                {
                    DBG_Error("OTA chunk queue full; dropped\r\n");
                }
            }
            s_parser.in_binary = 0U;
            s_parser.binary_len = 0U;
            s_parser.binary_expected = 0U;
        }
        return;
    }

    if (s_parser.text_len >= (sizeof(s_parser.text) - 1U))
    {
        (void)memset(&s_parser, 0, sizeof(s_parser));
        return;
    }

    s_parser.text[s_parser.text_len++] = (char)byte;
    s_parser.text[s_parser.text_len] = '\0';

    if ((s_parser.in_json == 0U) && (ota_dl_try_enter_binary() != 0U))
    {
        return;
    }

    if (s_parser.in_json == 0U)
    {
        if (byte == '{')
        {
            s_parser.in_json = 1U;
            s_parser.json_depth = 1U;
            s_parser.json_in_string = 0U;
            s_parser.json_escape = 0U;
        }
        return;
    }

    if (s_parser.json_in_string != 0U)
    {
        if (s_parser.json_escape != 0U)
        {
            s_parser.json_escape = 0U;
        }
        else if (byte == '\\')
        {
            s_parser.json_escape = 1U;
        }
        else if (byte == '"')
        {
            s_parser.json_in_string = 0U;
        }
        return;
    }

    if (byte == '"')
    {
        s_parser.json_in_string = 1U;
    }
    else if (byte == '{')
    {
        s_parser.json_depth++;
    }
    else if (byte == '}')
    {
        if (s_parser.json_depth > 0U)
        {
            s_parser.json_depth--;
        }
        if (s_parser.json_depth == 0U)
        {
            ota_dl_try_handle_json();
            s_parser.text_len = 0U;
            s_parser.text[0] = '\0';
            s_parser.in_json = 0U;
            s_parser.json_in_string = 0U;
            s_parser.json_escape = 0U;
        }
    }
}

uint8_t ota_dl_in_binary(void)
{
    return s_parser.in_binary;
}

uint8_t ota_dl_active(void)
{
    return s_downloading;
}

void ota_dl_get_status(ota_dl_status_t *status)
{
    app_ota_progress_t progress;

    if (status == NULL)
    {
        return;
    }

    (void)memset(status, 0, sizeof(*status));
    status->state = app_ota_get_state();
    app_ota_get_progress(&progress);
    status->received = progress.received;
    status->total = progress.total;
    status->target_version = s_target_version;
}

static ret_t ota_dl_request(uint32_t offset)
{
    char topic[96];
    int topic_len;
    ret_t ret;

    if ((s_fw_size == 0U) || (offset >= s_fw_size) ||
        ((offset % OTA_DL_CHUNK_SIZE) != 0U))
    {
        return RET_INVALID_PARAM;
    }

    s_expected_chunk = offset / OTA_DL_CHUNK_SIZE;
    if ((s_waiting_chunk != 0U) && (offset == s_requested_offset))
    {
        s_retry_count++;
    }
    else
    {
        s_retry_count = 0U;
    }
    s_requested_offset = offset;
    s_waiting_chunk = 1U;
    s_request_tick = xTaskGetTickCount();

    topic_len = snprintf(topic, sizeof(topic),
                         "v2/fw/request/%lu/chunk/%lu",
                         (unsigned long)s_session_id,
                         (unsigned long)s_expected_chunk);
    if ((topic_len < 0) || (topic_len >= (int)sizeof(topic)))
    {
        return RET_OVERFLOW;
    }

    ret = dev_net_mqtt_publish(topic, "256");
    if (ret != RET_OK)
    {
        DBG_Error("OTA request publish failed offset=%lu ret=%ld\r\n",
                  (unsigned long)offset, (long)ret);
    }
    return ret;
}

static void ota_dl_abort_session(void)
{
    ret_t ret;

    s_downloading = 0U;
    s_waiting_chunk = 0U;
    s_finish_pending = 0U;
    s_finish_post_pending = 0U;

    /*
     * ABORT 必须最终送达 worker，否则 app_ota 会停在旧会话状态，
     * 下一轮 start 只会拿到 RET_BUSY。投递失败转入重试状态，
     * 由 ota_dl_tick 按 OTA_DL_ABORT_RETRY_MS 周期补投（ABORT 幂等）。
     */
    ret = app_ota_post_abort_wait(OTA_DL_CONTROL_WAIT_MS);
    if (ret == RET_OK)
    {
        s_abort_pending = 0U;
    }
    else
    {
        s_abort_pending = 1U;
        s_abort_retry_tick = xTaskGetTickCount();
        DBG_Error("OTA abort post failed ret=%ld; will retry\r\n", (long)ret);
    }
}

/* 分片收齐：投递 FINISH；失败则进入重试状态，超时由 tick 兜底。 */
static void ota_dl_begin_finish(void)
{
    ret_t ret;

    s_finish_tick = xTaskGetTickCount();
    ret = app_ota_post_finish_wait(OTA_DL_CONTROL_WAIT_MS);
    if (ret == RET_OK)
    {
        s_finish_pending = 1U;
        s_finish_post_pending = 0U;
        DBG_log("OTA finish posted; waiting verify/commit\r\n");
    }
    else
    {
        s_finish_pending = 0U;
        s_finish_post_pending = 1U;
        DBG_Warning("OTA finish post failed ret=%ld; retry pending\r\n",
                    (long)ret);
    }
}

static void ota_dl_handle_meta(const ota_dl_msg_t *msg)
{
    cJSON *root;
    cJSON *title;
    cJSON *version;
    cJSON *size;
    cJSON *checksum;
    app_ota_meta_t meta;
    double fw_size_num;
    ret_t ret;

    if (s_downloading != 0U)
    {
        DBG_Warning("ignore OTA notify: session busy\r\n");
        return;
    }

    /*
     * 上一轮 ABORT 还没送达 worker 时不接受新通知：否则 ABORT 可能
     * 排在新 START 之后，把刚开始的新会话立刻杀掉。
     */
    if (s_abort_pending != 0U)
    {
        if (app_ota_post_abort() == RET_OK)
        {
            s_abort_pending = 0U;
        }
        DBG_Warning("ignore OTA notify: abort delivery pending\r\n");
        return;
    }

    root = cJSON_Parse(msg->json);
    if (root == NULL)
    {
        return;
    }

    title = cJSON_GetObjectItemCaseSensitive(root, "fw_title");
    version = cJSON_GetObjectItemCaseSensitive(root, "fw_version");
    size = cJSON_GetObjectItemCaseSensitive(root, "fw_size");
    checksum = cJSON_GetObjectItemCaseSensitive(root, "fw_checksum");

    ret = RET_PARSE_ERROR;
    if (cJSON_IsString(title) && cJSON_IsString(version) &&
        cJSON_IsNumber(size) && cJSON_IsString(checksum))
    {
        fw_size_num = size->valuedouble;
        if ((fw_size_num >= 8.0) &&
            (fw_size_num <= (double)BOOT_SLOT_SIZE) &&
            (fw_size_num == (double)(uint32_t)fw_size_num))
        {
            (void)memset(&meta, 0, sizeof(meta));
            meta.fw_size = (uint32_t)fw_size_num;
            meta.target_slot = ((uint32_t)APP_CURRENT_SLOT) ^ 1UL;

            if ((ota_dl_parse_crc32(checksum->valuestring,
                                    &meta.fw_crc32) == RET_OK) &&
                (ota_dl_parse_version(version->valuestring,
                                      &meta.fw_version) == RET_OK))
            {
                ret = RET_OK;
            }
        }
    }
    if (ret == RET_OK)
    {
        DBG_log("OTA title=%s size=%lu crc=0x%08lX target=%lu\r\n",
                title->valuestring,
                (unsigned long)meta.fw_size,
                (unsigned long)meta.fw_crc32,
                (unsigned long)meta.target_slot);
    }
    cJSON_Delete(root);

    if (ret != RET_OK)
    {
        DBG_Warning("OTA notify invalid; ignored\r\n");
        return;
    }

    s_session_id = ((uint32_t)xTaskGetTickCount() & 0xFFFFUL) + 1UL;
    s_fw_size = meta.fw_size;
    s_target_version = meta.fw_version;
    s_expected_chunk = 0U;
    s_requested_offset = 0U;
    s_retry_count = 0U;
    s_waiting_chunk = 0U;
    s_last_percent = 0U;
    s_finish_pending = 0U;
    s_finish_post_pending = 0U;

    ret = app_ota_post_start(&meta);
    if (ret == RET_OK)
    {
        s_downloading = 1U;
    }
    else
    {
        DBG_Error("OTA post start failed ret=%ld\r\n", (long)ret);
    }
}

static void ota_dl_handle_chunk(const ota_dl_msg_t *msg)
{
    uint32_t offset;
    uint32_t need;
    ret_t ret;

    if ((s_downloading == 0U) || (s_waiting_chunk == 0U))
    {
        return;
    }
    if ((msg->request_id != s_session_id) ||
        (msg->chunk_id != s_expected_chunk))
    {
        return;
    }
    if (msg->chunk_id > (UINT32_MAX / OTA_DL_CHUNK_SIZE))
    {
        return;
    }

    offset = msg->chunk_id * OTA_DL_CHUNK_SIZE;
    if ((offset != s_requested_offset) || (offset >= s_fw_size))
    {
        return;
    }

    /* 尾片按剩余长度收；短于需要的分片视为丢包，等超时重发。 */
    need = s_fw_size - offset;
    if (need > OTA_DL_CHUNK_SIZE)
    {
        need = OTA_DL_CHUNK_SIZE;
    }
    if (msg->data_len < need)
    {
        DBG_Warning("short chunk got=%lu need=%lu\r\n",
                    (unsigned long)msg->data_len,
                    (unsigned long)need);
        return;
    }

    ret = app_ota_post_chunk_wait(offset,
                                  msg->data,
                                  need,
                                  OTA_DL_CONTROL_WAIT_MS);
    if (ret == RET_OK)
    {
        s_waiting_chunk = 0U;
    }
    else
    {
        DBG_Error("OTA post chunk failed ret=%ld\r\n", (long)ret);
    }
}

static void ota_dl_handle_result(const ota_dl_msg_t *msg)
{
    uint32_t percent;
    ret_t ret;

    /*
     * 会话已结束（abort/finish 后）仍可能收到在途 worker 结果：
     * 只允许 ABORTED 用来清理状态，其余事件一律忽略，
     * 避免已放弃的会话被旧分片结果重新推进。
     */
    if ((s_downloading == 0U) && (msg->event != APP_OTA_EVENT_ABORTED))
    {
        DBG_Warning("OTA stale event=%u result=%ld ignored\r\n",
                    (unsigned int)msg->event,
                    (long)msg->result);
        return;
    }

    if (msg->result != RET_OK)
    {
        DBG_Error("OTA event=%u failed ret=%ld received=%lu\r\n",
                  (unsigned int)msg->event,
                  (long)msg->result,
                  (unsigned long)msg->received);
        ota_dl_abort_session();
        return;
    }

    switch (msg->event)
    {
    case APP_OTA_EVENT_STARTED:
        DBG_log("OTA download: 0%% (0/%lu bytes)\r\n",
                (unsigned long)s_fw_size);
        ret = ota_dl_request(0U);
        if (ret != RET_OK)
        {
            ota_dl_abort_session();
        }
        break;

    case APP_OTA_EVENT_CHUNK_WRITTEN:
        if (s_fw_size == 0U)
        {
            break;
        }
        percent = (uint32_t)(((uint64_t)msg->received * 100ULL) / s_fw_size);
        if (percent > 100U)
        {
            percent = 100U;
        }
        if (percent != s_last_percent)
        {
            s_last_percent = percent;
            DBG_log("OTA download: %lu%% (%lu/%lu bytes)\r\n",
                    (unsigned long)percent,
                    (unsigned long)msg->received,
                    (unsigned long)s_fw_size);
        }
        if (msg->received < s_fw_size)
        {
            vTaskDelay(pdMS_TO_TICKS(OTA_DL_REQUEST_GAP_MS));
            ret = ota_dl_request(msg->received);
            if (ret != RET_OK)
            {
                ota_dl_abort_session();
            }
        }
        else if (msg->received == s_fw_size)
        {
            ota_dl_begin_finish();
        }
        else
        {
            DBG_Error("OTA offset overflow received=%lu size=%lu\r\n",
                      (unsigned long)msg->received,
                      (unsigned long)s_fw_size);
            ota_dl_abort_session();
        }
        break;

    case APP_OTA_EVENT_FINISHED:
        DBG_log("OTA verified 100%%; reboot to bootloader\r\n");
        s_downloading = 0U;
        s_waiting_chunk = 0U;
        s_finish_pending = 0U;
        s_finish_post_pending = 0U;
        app_ota_reboot();
        break;

    case APP_OTA_EVENT_ABORTED:
    default:
        s_downloading = 0U;
        s_waiting_chunk = 0U;
        s_finish_pending = 0U;
        s_finish_post_pending = 0U;
        s_abort_pending = 0U;
        break;
    }
}

/* 兜底槽里的 worker 结果：与队列消息同样处理。 */
static void ota_dl_handle_pending_result(void)
{
    ota_dl_msg_t msg;
    uint8_t has_pending = 0U;

    taskENTER_CRITICAL();
    if (s_pending_result_valid != 0U)
    {
        msg = s_pending_result;
        s_pending_result_valid = 0U;
        has_pending = 1U;
    }
    taskEXIT_CRITICAL();

    if (has_pending != 0U)
    {
        DBG_Warning("OTA result recovered from fallback slot\r\n");
        ota_dl_handle_result(&msg);
    }
}

/*
 * FINISH/ABORT 投递失败的恢复状态：
 * - FINISH 重试到 OTA_DL_FINISH_TIMEOUT_MS 仍无法投递或完成 → 放弃本轮；
 * - ABORT 按固定间隔补投，直到 worker 收下（ABORT 幂等）。
 */
static void ota_dl_check_pending_posts(void)
{
    TickType_t now = xTaskGetTickCount();
    ret_t ret;

    if ((s_finish_pending != 0U) || (s_finish_post_pending != 0U))
    {
        if ((now - s_finish_tick) >= pdMS_TO_TICKS(OTA_DL_FINISH_TIMEOUT_MS))
        {
            DBG_Error("OTA finish timeout; abort session\r\n");
            ota_dl_abort_session();
        }
        else if (s_finish_post_pending != 0U)
        {
            ret = app_ota_post_finish_wait(OTA_DL_CONTROL_WAIT_MS);
            if (ret == RET_OK)
            {
                s_finish_pending = 1U;
                s_finish_post_pending = 0U;
                DBG_log("OTA finish post recovered\r\n");
            }
        }
    }

    if ((s_abort_pending != 0U) &&
        ((now - s_abort_retry_tick) >= pdMS_TO_TICKS(OTA_DL_ABORT_RETRY_MS)))
    {
        if (app_ota_post_abort() == RET_OK)
        {
            s_abort_pending = 0U;
            DBG_log("OTA abort post recovered\r\n");
        }
        else
        {
            s_abort_retry_tick = now;
        }
    }
}

static void ota_dl_check_timeout(void)
{
    TickType_t now = xTaskGetTickCount();
    ret_t ret;

    if ((s_downloading == 0U) || (s_waiting_chunk == 0U))
    {
        return;
    }
    if ((now - s_request_tick) < pdMS_TO_TICKS(OTA_DL_TIMEOUT_MS))
    {
        return;
    }

    if (s_retry_count < OTA_DL_MAX_RETRY)
    {
        DBG_Warning("OTA chunk timeout offset=%lu retry=%lu\r\n",
                    (unsigned long)s_requested_offset,
                    (unsigned long)(s_retry_count + 1U));
        ret = ota_dl_request(s_requested_offset);
        if (ret != RET_OK)
        {
            ota_dl_abort_session();
        }
    }
    else
    {
        DBG_Error("OTA retries exhausted offset=%lu\r\n",
                  (unsigned long)s_requested_offset);
        ota_dl_abort_session();
    }
}

void ota_dl_tick(void)
{
    ota_dl_msg_t msg;

    while (xQueueReceive(s_dl_queue, &msg, 0U) == pdTRUE)
    {
        switch (msg.type)
        {
        case OTA_DL_MSG_META:
            ota_dl_handle_meta(&msg);
            break;
        case OTA_DL_MSG_CHUNK:
            ota_dl_handle_chunk(&msg);
            break;
        case OTA_DL_MSG_RESULT:
            ota_dl_handle_result(&msg);
            break;
        default:
            break;
        }
    }

    ota_dl_handle_pending_result();
    ota_dl_check_pending_posts();
    ota_dl_check_timeout();
}

ret_t ota_dl_init(void)
{
    ret_t ret;

    if (s_dl_queue != NULL)
    {
        return RET_ALREADY_INIT;
    }

    s_dl_queue = xQueueCreate(OTA_DL_QUEUE_LENGTH, sizeof(ota_dl_msg_t));
    if (s_dl_queue == NULL)
    {
        return RET_NO_MEMORY;
    }

    (void)memset(&s_parser, 0, sizeof(s_parser));

    ret = app_ota_worker_init(ota_dl_on_worker);
    if (ret != RET_OK)
    {
        vQueueDelete(s_dl_queue);
        s_dl_queue = NULL;
        return ret;
    }
    return RET_OK;
}

ret_t ota_dl_subscribe(void)
{
    ret_t ret;

    ret = dev_net_mqtt_subscribe(OTA_DL_TOPIC_ATTRIBUTES);
    if (ret != RET_OK)
    {
        return ret;
    }
    return dev_net_mqtt_subscribe(OTA_DL_TOPIC_FW_RESPONSE);
}
