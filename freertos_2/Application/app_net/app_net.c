/**
 * @file    app_net.c
 * @brief   MQTT 云端控制、Modbus 轮询上报和设备页 LED 控制
 *
 * @details 本模块只拥有“网络业务流程”：MQTT 连接、RPC 解析、遥测组包
 *          和周期调度。具体的 Modbus、GPIO、LCD 实现通过
 *          app_net_dependencies_t 在 main.c 中装配。
 *
 *          数据流如下：
 *          1. MQTT 下行数据 -> 接收回调 -> JSON 队列 -> app_net_task；
 *          2. app_net_task -> 业务回调 -> Modbus/GPIO 适配层；
 *          3. Modbus 快照 -> 快照回调 -> LCD 消息队列 -> Display Task。
 *
 * @note    s_app_net_state_mutex 只保护快照和 LED 状态；MQTT 串口事务由
 *          dev_net 自己管理，Modbus 总线互斥由 app_modbus 管理。
 */
#include "app_net/app_net.h"
#include "dev_net/dev_net.h"
#include "app_ota_dl/ota_dl.h"
#include "cJSON/cJSON.h"
#include "SEGGER_RTT.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#define APP_NET_TOPIC_RPC_REQUEST          "v1/devices/me/rpc/request/+"
#define APP_NET_TOPIC_RPC_RESPONSE_PREFIX  "v1/devices/me/rpc/response/"
#define APP_NET_TOPIC_TELEMETRY            "v1/devices/me/telemetry"

/*
 * 下行订阅：本工程只订 RPC 一路。
 *
 * 两个方向必须分清（曾混淆过）：
 *   - telemetry 是【上行】（设备 → 平台），只发布、不订阅；
 *   - rpc/request/+ 是【下行】（平台 → 设备），设备必须显式订阅才能收到。
 *     它是云端控制设备的唯一入口，app_net_process_rpc() 负责解析与响应。
 *
 * 曾照搬参考例程额外订阅过两路，实测本工程【都没有任何处理代码】，
 * 订了只是白占模组订阅表，故删除：
 *   - v1/devices/me/attributes        （平台下发共享属性）
 *   - v2/fw/response/+/chunk/#        （OTA 分片响应）
 * OTA 已由 app_ota_dl 接入：上述两路订阅在 SUBSCRIBING 状态由
 * ota_dl_subscribe() 补订，下行字节经 ota_dl_feed 分流
 *（模组订阅上限 NET_MAX_SUBSCRIPTIONS = 10，空间不是约束）。
 */

#define APP_NET_TASK_STACK_DEPTH           (1024U)
#define APP_NET_TASK_PRIORITY              (2U)
#define APP_NET_RPC_QUEUE_LENGTH           (8U)
#define APP_NET_RPC_JSON_SIZE              (192U)
#define APP_NET_RX_PREFIX_SIZE             (160U)
#define APP_NET_RPC_RESPONSE_SIZE          (160U)
#define APP_NET_TELEMETRY_JSON_SIZE        (512U)
#define APP_NET_LINK_EVENT_QUEUE_LENGTH    (8U)
#define APP_NET_FSM_TICK_MS                (100U)
#define APP_NET_RECONNECT_BACKOFF_MS       (5000U)
#define APP_NET_CONNECT_TIMEOUT_MS         (180000U)

/* ------------------------------------------------------------------
 * 假数据上报开关（联调/演示用）
 *
 * =1 时 app_net_task 不再走 Modbus 485 轮询，而是每周期直接把一份
 * 伪造的"设备快照"组包发到 MQTT 遥测主题。用途：
 *   - 485 从站尚未接入 / 从站无响应时，先验证「固件 → 模组 → 云平台」
 *     这一段链路是否打通（看平台能否收到 points）；
 *   - 演示时保证平台侧数据永远在动。
 *
 * 假数据的字段与真实 Modbus 快照**完全同名同结构**，只是数值随时间
 * 缓变，因此平台侧仪表盘/规则链不需要为假数据做任何适配。
 *
 * ★ 本开关必须定义在【所有 #if 使用点之前】！
 *   预处理器把"未定义的标识符"当 0。若定义写在使用点之后，前面的
 *   #if 会静默走 #else 分支，且【编译没有任何警告】。本项目曾因此
 *   让假数据模式下的上报周期变成 5000ms 而非预期的 2000ms。
 *
 * 当前取值：0 = 走真实 485 轮询（从站已接入）。
 * ------------------------------------------------------------------ */
#define APP_NET_FAKE_TELEMETRY_ENABLE      (0U)

/* 上报周期。假数据联调时缩短到 2s，方便在平台侧立刻看到曲线；
 * 切回真实 485 轮询后建议恢复 5000U（Modbus 三从站一轮事务本身
 * 就要几百毫秒，过密会挤占总线和模组发送窗口）。 */
#if APP_NET_FAKE_TELEMETRY_ENABLE
#define APP_NET_POLL_PERIOD_MS             (2000U)
#else
#define APP_NET_POLL_PERIOD_MS             (5000U)
#endif
#define APP_NET_RPC_QUEUE_WAIT_MS          (50U)

/* ------------------------------------------------------------------
 * 遥测连发节流间隔（毫秒）
 *
 * 一轮上报要往同一个遥测主题连发三条 JSON（从站1/2/3），每条 200~300
 * 字节。dev_net_mqtt_publish() 是"发完即返回、不等发布 ACK"的透传式
 * 命令，三条背靠背发出去时中间没有任何间隙，模组内部的发送队列会在
 * 一个 tick 内被灌满，于是模组主动断开 MQTT 重连——现象就是 RTT 里
 * 反复出现的：
 *
 *     MQTT telemetry UART send complete, slave=3
 *     [rx] idx=15: MQTT_DISCONNECT
 *     [rx] idx=31: MQTT_CONNECT:xthings.cloud:1883
 *
 * 规律极稳定：每一次 MQTT_DISCONNECT 都紧跟在 slave=3 发送完成之后。
 * 注意这期间模组**不会**回 ERROR（实测整段日志零 ERROR）——它只是
 * 默默断开，所以从 AT 应答上完全看不出问题，只能靠时序对齐识别。
 *
 * 在两条遥测之间插入本间隔，把突发改造成均匀流，给模组留出打包和
 * 入队的时间。50ms 对 5s 的上报周期来说完全无感，但足以消除突发。
 * ------------------------------------------------------------------ */
#define APP_NET_PUBLISH_GAP_MS             (50U)

#define APP_NET_SLAVE_CONTROL              (1U)
#define APP_NET_SLAVE_ANALOG               (2U)
#define APP_NET_SLAVE_IO                   (3U)
#define APP_NET_CONTROL_HR_LAST             (6U)

typedef struct
{
    uint32_t request_id;
    uint8_t has_request_id;
    char json_text[APP_NET_RPC_JSON_SIZE];
} app_net_rpc_msg_t;

typedef enum
{
    APP_NET_STATE_CONFIGURING = 0U, /* 执行 MQTT 配置步骤 */
    APP_NET_STATE_SUBSCRIBING,      /* 订阅 RPC 与 OTA 主题 */
    APP_NET_STATE_ONLINE,           /* 允许遥测和 RPC */
    APP_NET_STATE_BACKOFF,          /* 重试退避 */
    APP_NET_STATE_FAULT             /* 未知状态，停机 */
} app_net_state_t;

typedef struct
{
    char topic_prefix[APP_NET_RX_PREFIX_SIZE];
    uint16_t topic_prefix_length;
    uint32_t request_id;
    uint8_t has_request_id;
    char json_text[APP_NET_RPC_JSON_SIZE];
    uint16_t json_text_length;
    uint16_t json_nesting_depth;
    uint8_t is_collecting_json;
    uint8_t is_in_string;
    uint8_t is_escaped;
} app_net_rx_parser_t;

/** MQTT RPC 消息队列；接收回调只入队，JSON 解析在业务任务中完成。 */
static QueueHandle_t s_app_net_rpc_queue = NULL;
/** Device 层链路事件队列；状态只由 app_net 任务修改。 */
static QueueHandle_t s_app_net_link_event_queue = NULL;
/** 网络业务任务句柄；任务创建成功后由 app_net 持有。 */
static TaskHandle_t s_app_net_task_handle = NULL;
/** 快照和 LED 状态互斥锁；不用于保护 Modbus 或 MQTT。 */
static SemaphoreHandle_t s_app_net_state_mutex = NULL;
/** MQTT 接收回调与网络业务任务之间共享的增量 JSON 解析状态。 */
static app_net_rx_parser_t s_app_net_rx_parser;
/** 最近一次采集快照；写入和读取都必须经过 s_app_net_state_mutex。 */
static app_net_snapshot_t s_app_net_snapshot;
/** 由组合根提供的业务适配器表，初始化后只读。 */
static app_net_dependencies_t s_app_net_dependencies;
/** MQTT 链路状态；只由网络业务流程更新。 */
static uint8_t s_app_net_is_online;
/** 连续上报失败计数（调试器 Watch 观察用；成功一轮即清零） */
static uint32_t s_app_net_publish_fail_count;
static app_net_state_t s_app_net_state = APP_NET_STATE_CONFIGURING;
static TickType_t s_app_net_state_deadline = 0U;
static uint32_t s_app_net_reconnect_count = 0U;

static void app_net_task(void *task_parameter);
static void app_net_rx_callback(const char *received_data,
                                uint16_t received_length);
static void app_net_link_callback(dev_net_link_event_t event);
static void app_net_feed_byte(uint8_t received_byte);
static void app_net_reset_rx_parser(void);
static ret_t app_net_prepare(void);
static void app_net_set_state(app_net_state_t next_state);
static void app_net_handle_link_event(dev_net_link_event_t event);
static void app_net_run_state_machine(void);
#if !APP_NET_FAKE_TELEMETRY_ENABLE
static ret_t app_net_poll_and_publish(void);
#endif
static void app_net_process_rpc(const app_net_rpc_msg_t *rpc_message);
static ret_t app_net_validate_dependencies(
    const app_net_dependencies_t *dependencies);
static void app_net_notify_snapshot(void);
static ret_t app_net_set_led_state(uint8_t is_on);
static uint32_t app_net_registers_to_u32(const uint16_t registers[2U]);
static int32_t app_net_registers_to_i32(const uint16_t registers[2U]);
static int32_t app_net_register_to_i16(uint16_t value);
static float app_net_registers_to_float32(const uint16_t registers[2U]);
static ret_t app_net_json_start(char *payload,
                                uint16_t capacity,
                                uint16_t *used,
                                uint8_t slave,
                                uint8_t online);
static ret_t app_net_json_append_i32(char *payload,
                                     uint16_t capacity,
                                     uint16_t *used,
                                     const char *key,
                                     int32_t value);
static ret_t app_net_json_append_float2(char *payload,
                                        uint16_t capacity,
                                        uint16_t *used,
                                        const char *key,
                                        float value);
static ret_t app_net_json_append_bool(char *payload,
                                      uint16_t capacity,
                                      uint16_t *used,
                                      const char *key,
                                      uint8_t value);
static ret_t app_net_json_append_u16_array(char *payload,
                                           uint16_t capacity,
                                           uint16_t *used,
                                           const char *key,
                                           const uint16_t *values,
                                           uint16_t count);
static ret_t app_net_json_finish(char *payload,
                                 uint16_t capacity,
                                 uint16_t *used);
static ret_t app_net_build_control_payload(const app_net_snapshot_t *snapshot,
                                           char *payload,
                                           uint16_t capacity);
#if APP_NET_FAKE_TELEMETRY_ENABLE
static ret_t app_net_fake_poll_and_publish(void);
#endif
static ret_t app_net_build_analog_payload(const app_net_snapshot_t *snapshot,
                                          char *payload,
                                          uint16_t capacity);
static ret_t app_net_build_io_payload(const app_net_snapshot_t *snapshot,
                                      char *payload,
                                      uint16_t capacity);
static ret_t app_net_publish_control(const app_net_snapshot_t *snapshot);
static ret_t app_net_publish_analog(const app_net_snapshot_t *snapshot);
static ret_t app_net_publish_io(const app_net_snapshot_t *snapshot);

static void *app_net_json_malloc(size_t size)
{
    return pvPortMalloc(size);
}

static void app_net_json_free(void *pointer)
{
    vPortFree(pointer);
}

/**
 * @brief 从 MQTT RPC 主题后缀中提取请求序号
 * @param topic_prefix 已接收的主题前缀
 * @param request_id   输出的请求序号
 * @retval 1 提取成功；0 主题格式错误、缺少数字或数值溢出
 * @note 请求序号用于拼接 response 主题，不能用 JSON 中的业务字段替代。
 */
static uint8_t app_net_extract_request_id(const char *topic_prefix,
                                          uint32_t *request_id)
{
    const char *request_id_text;
    uint32_t request_id_value;
    uint8_t request_digit_count;

    if ((topic_prefix == NULL) || (request_id == NULL))
    {
        return 0U;
    }

    request_id_text = strstr(topic_prefix, "request/");
    if (request_id_text == NULL)
    {
        return 0U;
    }
    request_id_text += (uint16_t)strlen("request/");

    request_id_value = 0U;
    request_digit_count = 0U;
    while ((*request_id_text >= '0') && (*request_id_text <= '9'))
    {
        uint8_t request_digit =
            (uint8_t)(*request_id_text - '0');
        if (request_id_value >
            ((UINT32_MAX - request_digit) / 10U))
        {
            return 0U;
        }
        request_id_value = (request_id_value * 10U) + request_digit;
        request_digit_count++;
        request_id_text++;
    }

    if (request_digit_count == 0U)
    {
        return 0U;
    }

    *request_id = request_id_value;
    return 1U;
}

static void app_net_reset_rx_parser(void)
{
    (void)memset(&s_app_net_rx_parser,
                 0,
                 sizeof(s_app_net_rx_parser));
}

/**
 * @brief 向增量 JSON 解析器输入一个字节
 * @note dev_net 回调可能一次给出半包或多包数据，因此这里不能假设一次
 *       回调就是一条完整 JSON。解析完成后只把值拷贝进队列，不把解析器
 *       内部缓冲区指针交给其他任务。
 */
static void app_net_feed_byte(uint8_t received_byte)
{
    app_net_rx_parser_t *parser_state = &s_app_net_rx_parser;

    if (parser_state->is_collecting_json == 0U)
    {
        if (parser_state->topic_prefix_length <
            (APP_NET_RX_PREFIX_SIZE - 1U))
        {
            parser_state->topic_prefix[parser_state->topic_prefix_length++] =
                (char)received_byte;
            parser_state->topic_prefix[parser_state->topic_prefix_length] =
                '\0';
            if (app_net_extract_request_id(parser_state->topic_prefix,
                                           &parser_state->request_id) != 0U)
            {
                parser_state->has_request_id = 1U;
            }
        }
        else
        {
            app_net_reset_rx_parser();
        }

        if (received_byte != '{')
        {
            return;
        }

        parser_state->is_collecting_json = 1U;
        parser_state->is_in_string = 0U;
        parser_state->is_escaped = 0U;
        parser_state->json_nesting_depth = 1U;
        parser_state->json_text_length = 1U;
        parser_state->json_text[0U] = '{';
        parser_state->json_text[1U] = '\0';
        return;
    }

    if (parser_state->json_text_length >= (APP_NET_RPC_JSON_SIZE - 1U))
    {
        DBG_Warning("MQTT RPC JSON too long; discard\r\n");
        app_net_reset_rx_parser();
        return;
    }

    parser_state->json_text[parser_state->json_text_length++] =
        (char)received_byte;
    parser_state->json_text[parser_state->json_text_length] = '\0';

    if (parser_state->is_in_string != 0U)
    {
        if (parser_state->is_escaped != 0U)
        {
            parser_state->is_escaped = 0U;
        }
        else if (received_byte == '\\')
        {
            parser_state->is_escaped = 1U;
        }
        else if (received_byte == '"')
        {
            parser_state->is_in_string = 0U;
        }
        return;
    }

    if (received_byte == '"')
    {
        parser_state->is_in_string = 1U;
    }
    else if (received_byte == '{')
    {
        parser_state->json_nesting_depth++;
    }
    else if (received_byte == '}')
    {
        if (parser_state->json_nesting_depth > 0U)
        {
            parser_state->json_nesting_depth--;
        }

        if (parser_state->json_nesting_depth == 0U)
        {
            app_net_rpc_msg_t rpc_message;
            BaseType_t queue_result;

            rpc_message.request_id = parser_state->request_id;
            rpc_message.has_request_id = parser_state->has_request_id;
            (void)memcpy(rpc_message.json_text,
                         parser_state->json_text,
                         (uint32_t)parser_state->json_text_length + 1U);

            if ((rpc_message.has_request_id != 0U) &&
                (s_app_net_rpc_queue != NULL))
            {
                queue_result = xQueueSend(s_app_net_rpc_queue,
                                          &rpc_message,
                                          pdMS_TO_TICKS(
                                              APP_NET_RPC_QUEUE_WAIT_MS));
                if (queue_result != pdPASS)
                {
                    DBG_Error("MQTT RPC queue full; request=%lu dropped\r\n",
                              (unsigned long)rpc_message.request_id);
                }
            }
            else
            {
                DBG_Warning("MQTT JSON ignored: RPC request id missing\r\n");
            }

            app_net_reset_rx_parser();
        }
    }
}

static void app_net_rx_callback(const char *received_data,
                                 uint16_t received_length)
{
    uint16_t byte_index;

    if (received_data == NULL)
    {
        return;
    }

    /*
     * 双路分流：OTA 会话先看每个字节；一旦进入二进制体，
     * RPC 侧跳过这些字节——固件二进制里的 '{' '}' 不能污染 JSON 解析。
     * 文本头字节两路都可见（无花括号，对 RPC 解析无害）。
     */
    for (byte_index = 0U;
         byte_index < received_length;
         byte_index++)
    {
        uint8_t byte_value = (uint8_t)received_data[byte_index];

        ota_dl_feed(byte_value);
        if (ota_dl_in_binary() == 0U)
        {
            app_net_feed_byte(byte_value);
        }
    }
}

static void app_net_link_callback(dev_net_link_event_t event)
{
    if (s_app_net_link_event_queue == NULL)
    {
        return;
    }

    if (xQueueSend(s_app_net_link_event_queue, &event, 0U) != pdPASS)
    {
        DBG_Warning("network link event queue full, event=%u\r\n",
                    (unsigned int)event);
    }
}

/**
 * @brief  准备网络设备；连接生命周期由 app_net 状态机驱动。
 * @note   此函数只执行 UART/模块初始化和回调注册，不等待 MQTT_CONNECT，
 *         不执行连接配置和主题订阅。
 */
static ret_t app_net_prepare(void)
{
    ret_t ret;

    ret = dev_net_init();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        DBG_Error("dev_net_init failed ret=%ld\r\n", (long)ret);
        return ret;
    }

    dev_net_register_link_cb(app_net_link_callback);

    /* 在连接状态机启动前先注册回调，避免第一条云端命令早到后无人接收。 */
    (void)memset(&s_app_net_rx_parser, 0, sizeof(s_app_net_rx_parser));
    dev_net_register_recv_cb(app_net_rx_callback);

    return RET_OK;
}

static ret_t app_net_validate_dependencies(
    const app_net_dependencies_t *dependencies)
{
    if (dependencies == NULL)
    {
        return RET_NULL_PTR;
    }

    if ((dependencies->read_registers == NULL) ||
        (dependencies->write_register == NULL) ||
        (dependencies->write_coil == NULL) ||
        (dependencies->write_led2 == NULL))
    {
        return RET_INVALID_PARAM;
    }

#if !APP_NET_FAKE_TELEMETRY_ENABLE
    if (dependencies->read_bits == NULL)
    {
        return RET_INVALID_PARAM;
    }
#endif

    return RET_OK;
}

/**
 * @brief 更新 MQTT 在线状态并通知消费者
 * @note 状态通知只表达网络流程已完成/失效，不代表某一条遥测一定发送成功。
 *       因此发布失败不会在这里被误判为链路断开。
 */
static void app_net_set_online(uint8_t is_online)
{
    s_app_net_is_online = (is_online != 0U) ? 1U : 0U;

    if (s_app_net_dependencies.on_online_changed != NULL)
    {
        s_app_net_dependencies.on_online_changed(
            s_app_net_dependencies.context,
            s_app_net_is_online);
    }
}

static void app_net_set_state(app_net_state_t next_state)
{
    app_net_state_t previous_state = s_app_net_state;

    if (previous_state == next_state)
    {
        return;
    }

    s_app_net_state = next_state;
    if ((next_state == APP_NET_STATE_ONLINE) &&
        (previous_state != APP_NET_STATE_ONLINE))
    {
        s_app_net_reconnect_count = 0U;
        app_net_set_online(1U);
    }
    if ((previous_state == APP_NET_STATE_ONLINE) &&
        (next_state != APP_NET_STATE_ONLINE))
    {
        app_net_set_online(0U);
    }

    DBG_log("network state %u -> %u\r\n",
            (unsigned int)previous_state,
            (unsigned int)next_state);
}

static void app_net_enter_backoff(ret_t reason)
{
    s_app_net_reconnect_count++;
    s_app_net_state_deadline = xTaskGetTickCount() +
                               pdMS_TO_TICKS(APP_NET_RECONNECT_BACKOFF_MS);
    DBG_Warning("network backoff count=%lu reason=%ld\r\n",
                (unsigned long)s_app_net_reconnect_count,
                (long)reason);
    app_net_set_state(APP_NET_STATE_BACKOFF);
}

static void app_net_handle_link_event(dev_net_link_event_t event)
{
    switch (event)
    {
        case DEV_NET_LINK_CONNECTED:
            switch (s_app_net_state)
            {
                case APP_NET_STATE_SUBSCRIBING:
                    break;
                case APP_NET_STATE_BACKOFF:
                    /* 模组自愈重连：不等退避计时，立刻重新订阅。 */
                    app_net_set_state(APP_NET_STATE_SUBSCRIBING);
                    break;
                case APP_NET_STATE_ONLINE:
                    break;
                default:
                    /* CONFIGURING 中的 CONNECT 属于早到事件，忽略即可。 */
                    break;
            }
            break;

        case DEV_NET_LINK_DISCONNECTED:
            switch (s_app_net_state)
            {
                case APP_NET_STATE_ONLINE:
                case APP_NET_STATE_SUBSCRIBING:
                    app_net_enter_backoff(RET_NO_RESPONSE);
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
}

static void app_net_run_state_machine(void)
{
    TickType_t now = xTaskGetTickCount();
    ret_t ret;

    switch (s_app_net_state)
    {
        case APP_NET_STATE_CONFIGURING:
            ret = dev_net_mqtt_connect();
            if (ret == RET_OK)
            {
                s_app_net_state_deadline = now;
                /*
                 * 不复位模组：模组断线后会自己重连。MCU 单独复位（如 OTA 升级）
                 * 后模组会话仍在、不会再发 MQTT_CONNECT URC，因此这里直接
                 * 用订阅做乐观探测：订阅成功即视为在线，失败再退避重试。
                 */
                app_net_set_state(APP_NET_STATE_SUBSCRIBING);
            }
            else
            {
                app_net_enter_backoff(ret);
            }
            break;

        case APP_NET_STATE_SUBSCRIBING:
            /* 会话重建后旧订阅槽失效，从 0 号槽重新订阅。 */
            dev_net_mqtt_subscribe_reset();
            ret = dev_net_mqtt_subscribe(APP_NET_TOPIC_RPC_REQUEST);
            if (ret == RET_OK)
            {
                DBG_log("MQTT subscribed: %s\r\n",
                        APP_NET_TOPIC_RPC_REQUEST);
                /* OTA 两路订阅：attributes 通知 + 分片响应，失败同样退避重试。 */
                ret = ota_dl_subscribe();
            }
            if (ret == RET_OK)
            {
                app_net_set_state(APP_NET_STATE_ONLINE);
            }
            else
            {
                /* 订阅失败通常意味着 MQTT 会话还没建立，退避后重试。 */
                app_net_enter_backoff(ret);
            }
            break;

        case APP_NET_STATE_ONLINE:
            break;

        case APP_NET_STATE_BACKOFF:
            if ((int32_t)(now - s_app_net_state_deadline) >= 0)
            {
                app_net_set_state(APP_NET_STATE_CONFIGURING);
            }
            break;

        case APP_NET_STATE_FAULT:
            break;

        default:
            app_net_set_state(APP_NET_STATE_FAULT);
            break;
    }
}

/**
 * @brief 把受互斥锁保护的快照复制给外部消费者
 * @note 先解锁再调用外部回调，避免 LCD 队列满或其他消费者逻辑阻塞网络状态。
 */
static void app_net_notify_snapshot(void)
{
    app_net_snapshot_t snapshot;

    if ((s_app_net_state_mutex == NULL) ||
        (s_app_net_dependencies.on_snapshot_updated == NULL))
    {
        return;
    }

    if (xSemaphoreTake(s_app_net_state_mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }
    snapshot = s_app_net_snapshot;
    (void)xSemaphoreGive(s_app_net_state_mutex);

    s_app_net_dependencies.on_snapshot_updated(
        s_app_net_dependencies.context,
        &snapshot);
}

/**
 * @brief 写入 LED2 并同步内部快照
 * @param is_on 目标电平，非 0 表示点亮
 * @note 外部硬件写成功后才更新快照；失败时保留旧状态，避免界面显示假状态。
 */
static ret_t app_net_set_led_state(uint8_t is_on)
{
    ret_t result;

    if (s_app_net_state_mutex == NULL)
    {
        return RET_NOT_INIT;
    }

    if (s_app_net_dependencies.write_led2 == NULL)
    {
        return RET_NOT_SUPPORT;
    }

    is_on = (is_on != 0U) ? 1U : 0U;
    if (xSemaphoreTake(s_app_net_state_mutex,
                       pdMS_TO_TICKS(100U)) != pdTRUE)
    {
        return RET_TIMEOUT;
    }

    result = s_app_net_dependencies.write_led2(
        s_app_net_dependencies.context,
        is_on);
    if (result == RET_OK)
    {
        s_app_net_snapshot.led2_is_on = is_on;
    }
    (void)xSemaphoreGive(s_app_net_state_mutex);

    if (result == RET_OK)
    {
        app_net_notify_snapshot();
    }
    return result;
}

/**
 * @brief 翻转 LED2
 * @note 触摸任务和云端 RPC 都通过此入口操作 LED，硬件写入与快照更新保持一致。
 */
ret_t app_net_led_toggle(void)
{
    uint8_t next_state;

    if (s_app_net_state_mutex == NULL)
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_app_net_state_mutex,
                       pdMS_TO_TICKS(100U)) != pdTRUE)
    {
        return RET_TIMEOUT;
    }

    next_state = (s_app_net_snapshot.led2_is_on == 0U) ? 1U : 0U;
    (void)xSemaphoreGive(s_app_net_state_mutex);

    return app_net_set_led_state(next_state);
}

/**
 * @brief   按 Modbus 大字在前顺序合成 32 位原始数据
 */
static uint32_t app_net_registers_to_u32(const uint16_t registers[2U])
{
    return (((uint32_t)registers[0U] << 16U) | (uint32_t)registers[1U]);
}

/**
 * @brief   按 Modbus 大字在前顺序合成有符号 32 位数据
 */
static int32_t app_net_registers_to_i32(const uint16_t registers[2U])
{
    uint32_t raw;

    raw = app_net_registers_to_u32(registers);
    if (raw <= 0x7FFFFFFFU)
    {
        return (int32_t)raw;
    }

    return (int32_t)((int64_t)raw - 0x100000000LL);
}

/**
 * @brief   将 16 位寄存器按 int16_t 二进制补码解释
 */
static int32_t app_net_register_to_i16(uint16_t value)
{
    if ((value & 0x8000U) != 0U)
    {
        return (int32_t)value - 0x10000L;
    }

    return (int32_t)value;
}

/**
 * @brief   按大字在前将两个寄存器转换为 IEEE-754 binary32
 */
static float app_net_registers_to_float32(const uint16_t registers[2U])
{
    uint32_t raw;
    float value;

    raw = app_net_registers_to_u32(registers);
    (void)memcpy(&value, &raw, sizeof(value));
    return value;
}

static ret_t app_net_json_start(char *payload,
                                uint16_t capacity,
                                uint16_t *used,
                                uint8_t slave,
                                uint8_t online)
{
    int length;

    if ((payload == NULL) || (used == NULL) || (capacity == 0U))
    {
        return RET_INVALID_PARAM;
    }

    length = snprintf(payload,
                      capacity,
                      "{\"addr\":\"%02u\",\"online\":%s",
                      (unsigned int)slave,
                      (online != 0U) ? "true" : "false");
    if ((length < 0) || (length >= (int)capacity))
    {
        return RET_OVERFLOW;
    }

    *used = (uint16_t)length;
    return RET_OK;
}

static ret_t app_net_json_append_i32(char *payload,
                                     uint16_t capacity,
                                     uint16_t *used,
                                     const char *key,
                                     int32_t value)
{
    int length;
    uint16_t remaining;

    if ((payload == NULL) || (used == NULL) || (key == NULL) ||
        (*used >= capacity))
    {
        return RET_INVALID_PARAM;
    }

    remaining = (uint16_t)(capacity - *used);
    length = snprintf(&payload[*used],
                      remaining,
                      ",\"%s\":%ld",
                      key,
                      (long)value);
    if ((length < 0) || (length >= (int)remaining))
    {
        return RET_OVERFLOW;
    }

    *used = (uint16_t)(*used + (uint16_t)length);
    return RET_OK;
}

/**
 * @brief   将有限范围内的浮点工程量输出为 JSON 两位小数
 * @note    用定点字符串拼接，避免依赖 printf 的浮点格式支持。
 */
static ret_t app_net_json_append_float2(char *payload,
                                        uint16_t capacity,
                                        uint16_t *used,
                                        const char *key,
                                        float value)
{
    int32_t scaled;
    int32_t magnitude;
    int32_t whole;
    int32_t fraction;
    int length;
    uint16_t remaining;

    if ((payload == NULL) || (used == NULL) || (key == NULL) ||
        (*used >= capacity))
    {
        return RET_INVALID_PARAM;
    }

    if (!(value >= -1000000.0F) || !(value <= 1000000.0F))
    {
        remaining = (uint16_t)(capacity - *used);
        length = snprintf(&payload[*used],
                          remaining,
                          ",\"%s\":null",
                          key);
        if ((length < 0) || (length >= (int)remaining))
        {
            return RET_OVERFLOW;
        }

        *used = (uint16_t)(*used + (uint16_t)length);
        return RET_OK;
    }

    scaled = (int32_t)((value * 100.0F) +
                       ((value >= 0.0F) ? 0.5F : -0.5F));
    magnitude = (scaled < 0) ? -scaled : scaled;
    whole = magnitude / 100;
    fraction = magnitude % 100;
    remaining = (uint16_t)(capacity - *used);
    length = snprintf(&payload[*used],
                      remaining,
                      ",\"%s\":%s%ld.%02ld",
                      key,
                      (scaled < 0) ? "-" : "",
                      (long)whole,
                      (long)fraction);
    if ((length < 0) || (length >= (int)remaining))
    {
        return RET_OVERFLOW;
    }

    *used = (uint16_t)(*used + (uint16_t)length);
    return RET_OK;
}

static ret_t app_net_json_append_bool(char *payload,
                                      uint16_t capacity,
                                      uint16_t *used,
                                      const char *key,
                                      uint8_t value)
{
    int length;
    uint16_t remaining;

    if ((payload == NULL) || (used == NULL) || (key == NULL) ||
        (*used >= capacity))
    {
        return RET_INVALID_PARAM;
    }

    remaining = (uint16_t)(capacity - *used);
    length = snprintf(&payload[*used],
                      remaining,
                      ",\"%s\":%s",
                      key,
                      (value != 0U) ? "true" : "false");
    if ((length < 0) || (length >= (int)remaining))
    {
        return RET_OVERFLOW;
    }

    *used = (uint16_t)(*used + (uint16_t)length);
    return RET_OK;
}

static ret_t app_net_json_append_u16_array(char *payload,
                                           uint16_t capacity,
                                           uint16_t *used,
                                           const char *key,
                                           const uint16_t *values,
                                           uint16_t count)
{
    uint16_t index;
    uint16_t remaining;
    int length;

    if ((payload == NULL) || (used == NULL) || (key == NULL) ||
        (values == NULL) || (count == 0U) || (*used >= capacity))
    {
        return RET_INVALID_PARAM;
    }

    remaining = (uint16_t)(capacity - *used);
    length = snprintf(&payload[*used], remaining, ",\"%s\":[", key);
    if ((length < 0) || (length >= (int)remaining))
    {
        return RET_OVERFLOW;
    }
    *used = (uint16_t)(*used + (uint16_t)length);

    for (index = 0U; index < count; index++)
    {
        remaining = (uint16_t)(capacity - *used);
        length = snprintf(&payload[*used],
                          remaining,
                          "%s%u",
                          (index == 0U) ? "" : ",",
                          (unsigned int)values[index]);
        if ((length < 0) || (length >= (int)remaining))
        {
            return RET_OVERFLOW;
        }
        *used = (uint16_t)(*used + (uint16_t)length);
    }

    if (*used >= capacity)
    {
        return RET_OVERFLOW;
    }
    payload[*used] = ']';
    (*used)++;
    if (*used >= capacity)
    {
        return RET_OVERFLOW;
    }
    payload[*used] = '\0';
    return RET_OK;
}

static ret_t app_net_json_finish(char *payload,
                                 uint16_t capacity,
                                 uint16_t *used)
{
    if ((payload == NULL) || (used == NULL) ||
        ((uint32_t)*used + 2U > (uint32_t)capacity))
    {
        return RET_OVERFLOW;
    }

    payload[*used] = '}';
    (*used)++;
    payload[*used] = '\0';
    return RET_OK;
}

static ret_t app_net_build_control_payload(const app_net_snapshot_t *snapshot,
                                           char *payload,
                                           uint16_t capacity)
{
    ret_t ret;
    uint16_t used;
    uint8_t online;

    if ((snapshot == NULL) || (payload == NULL))
    {
        return RET_NULL_PTR;
    }

    online = ((snapshot->valid_slave_mask & 0x01U) != 0U) ? 1U : 0U;
    ret = app_net_json_start(payload, capacity, &used,
                             APP_NET_SLAVE_CONTROL, online);
    if (ret != RET_OK)
    {
        return ret;
    }

    if (online != 0U)
    {
        ret = app_net_json_append_u16_array(payload, capacity, &used,
                                             "hr",
                                             snapshot->control_holding_registers,
                                             APP_NET_CONTROL_HOLDING_REGISTER_COUNT);
        if (ret == RET_OK)
        {
            ret = app_net_json_append_u16_array(payload, capacity, &used,
                                                 "status",
                                                 snapshot->control_status_registers,
                                                 APP_NET_CONTROL_STATUS_REGISTER_COUNT);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used,
                                           "run_command",
                                           (int32_t)snapshot->control_holding_registers[0U]);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used,
                                           "target_speed_rpm",
                                           (int32_t)snapshot->control_holding_registers[1U]);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(
                payload, capacity, &used, "position_setpoint_pulse",
                app_net_registers_to_i32(
                    &snapshot->control_holding_registers[2U]));
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "frequency_setpoint_hz",
                app_net_registers_to_float32(
                    &snapshot->control_holding_registers[4U]) *
                0.01F);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used,
                                           "acceleration_time_s",
                                           (int32_t)snapshot->control_holding_registers[6U]);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used,
                                           "run_state",
                                           (int32_t)snapshot->control_status_registers[0U]);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used,
                                           "actual_speed_rpm",
                                           (int32_t)snapshot->control_status_registers[1U]);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "bus_voltage_v",
                app_net_registers_to_float32(
                    &snapshot->control_status_registers[2U]));
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "output_current_a",
                (float)snapshot->control_status_registers[4U] * 0.1F);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used,
                                           "fault_code",
                                           (int32_t)snapshot->control_status_registers[5U]);
        }
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return app_net_json_finish(payload, capacity, &used);
}

static ret_t app_net_build_analog_payload(const app_net_snapshot_t *snapshot,
                                          char *payload,
                                          uint16_t capacity)
{
    ret_t ret;
    uint16_t used;
    uint8_t online;

    if ((snapshot == NULL) || (payload == NULL))
    {
        return RET_NULL_PTR;
    }

    online = ((snapshot->valid_slave_mask & 0x02U) != 0U) ? 1U : 0U;
    ret = app_net_json_start(payload, capacity, &used,
                             APP_NET_SLAVE_ANALOG, online);
    if (ret != RET_OK)
    {
        return ret;
    }

    if (online != 0U)
    {
        ret = app_net_json_append_u16_array(payload, capacity, &used,
                                             "ir",
                                             snapshot->analog_input_registers,
                                             APP_NET_ANALOG_INPUT_REGISTER_COUNT);
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "temperature_c",
                app_net_registers_to_float32(
                    &snapshot->analog_input_registers[0U]));
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "pressure_mpa",
                app_net_registers_to_float32(
                    &snapshot->analog_input_registers[2U]));
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "flow_m3h",
                app_net_registers_to_float32(
                    &snapshot->analog_input_registers[4U]));
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "voltage_v",
                (float)snapshot->analog_input_registers[6U] * 0.1F);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "current_a",
                (float)snapshot->analog_input_registers[7U] * 0.1F);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(
                payload, capacity, &used, "liquid_level_mm",
                app_net_register_to_i16(snapshot->analog_input_registers[8U]));
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_float2(
                payload, capacity, &used, "motor_temperature_c",
                (float)app_net_register_to_i16(
                    snapshot->analog_input_registers[9U]) * 0.1F);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used,
                                           "speed_feedback_rpm",
                                           (int32_t)snapshot->analog_input_registers[10U]);
        }
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return app_net_json_finish(payload, capacity, &used);
}

static ret_t app_net_build_io_payload(const app_net_snapshot_t *snapshot,
                                      char *payload,
                                      uint16_t capacity)
{
    static const char *const do_names[8U] =
    {
        "do_motor_start", "do_solenoid_valve", "do_cooling_fan",
        "do_alarm_lamp", "do_heater", "do_spare_1", "do_spare_2",
        "do_spare_3"
    };
    static const char *const di_names[8U] =
    {
        "di_emergency_stop", "di_start_button", "di_stop_button",
        "di_upper_limit", "di_lower_limit", "di_door_open",
        "di_level_high_alarm", "di_ready"
    };
    ret_t ret;
    uint16_t used;
    uint8_t index;
    uint8_t online;

    if ((snapshot == NULL) || (payload == NULL))
    {
        return RET_NULL_PTR;
    }

    online = ((snapshot->valid_slave_mask & 0x04U) != 0U) ? 1U : 0U;
    ret = app_net_json_start(payload, capacity, &used,
                             APP_NET_SLAVE_IO, online);
    if (ret != RET_OK)
    {
        return ret;
    }

    if (online != 0U)
    {
        ret = app_net_json_append_i32(payload, capacity, &used, "co",
                                      (int32_t)snapshot->io_coil_states);
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used, "di",
                                          (int32_t)snapshot->io_discrete_input_states);
        }
        if (ret == RET_OK)
        {
            ret = app_net_json_append_i32(payload, capacity, &used, "led2",
                                          (int32_t)snapshot->led2_is_on);
        }
        for (index = 0U; (index < 8U) && (ret == RET_OK); index++)
        {
            ret = app_net_json_append_bool(
                payload, capacity, &used, do_names[index],
                (uint8_t)((snapshot->io_coil_states >> index) & 0x01U));
        }
        for (index = 0U; (index < 8U) && (ret == RET_OK); index++)
        {
            ret = app_net_json_append_bool(
                payload, capacity, &used, di_names[index],
                (uint8_t)((snapshot->io_discrete_input_states >> index) & 0x01U));
        }
        if (ret != RET_OK)
        {
            return ret;
        }
    }
    else
    {
        ret = app_net_json_append_i32(payload, capacity, &used, "led2",
                                      (int32_t)snapshot->led2_is_on);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return app_net_json_finish(payload, capacity, &used);
}

static ret_t app_net_read_registers(uint8_t slave_address,
                                    app_net_register_area_t area,
                                    uint16_t start_address,
                                    uint16_t quantity,
                                    uint16_t *register_values)
{
    if (s_app_net_dependencies.read_registers == NULL)
    {
        return RET_NOT_SUPPORT;
    }

    return s_app_net_dependencies.read_registers(
        s_app_net_dependencies.context,
        slave_address,
        area,
        start_address,
        quantity,
        register_values);
}

#if !APP_NET_FAKE_TELEMETRY_ENABLE
static ret_t app_net_read_bits(uint8_t slave_address,
                               app_net_bit_area_t area,
                               uint16_t start_address,
                               uint16_t quantity,
                               uint8_t *bit_values)
{
    if (s_app_net_dependencies.read_bits == NULL)
    {
        return RET_NOT_SUPPORT;
    }

    return s_app_net_dependencies.read_bits(
        s_app_net_dependencies.context,
        slave_address,
        area,
        start_address,
        quantity,
        bit_values);
}
#endif /* !APP_NET_FAKE_TELEMETRY_ENABLE */

static ret_t app_net_write_register(uint8_t slave_address,
                                    uint16_t register_address,
                                    uint16_t register_value)
{
    if (s_app_net_dependencies.write_register == NULL)
    {
        return RET_NOT_SUPPORT;
    }

    return s_app_net_dependencies.write_register(
        s_app_net_dependencies.context,
        slave_address,
        register_address,
        register_value);
}

static ret_t app_net_write_coil(uint8_t slave_address,
                                uint16_t coil_address,
                                uint8_t is_on)
{
    if (s_app_net_dependencies.write_coil == NULL)
    {
        return RET_NOT_SUPPORT;
    }

    return s_app_net_dependencies.write_coil(
        s_app_net_dependencies.context,
        slave_address,
        coil_address,
        is_on);
}

#if !APP_NET_FAKE_TELEMETRY_ENABLE
/**
 * @brief 读取三个业务从站并生成一份一致的业务快照
 * @param snapshot 输出快照；函数返回前会完整初始化
 * @retval RET_OK 三个从站均读取成功；RET_TIMEOUT 表示至少一路失败
 * @note 每个从站的局部数组只有在该从站整组读取成功后才写入快照，
 *       避免把半组数据发布到云端。失败从站的 valid_slave_mask 位保持为 0。
 */
static ret_t app_net_poll_modbus(app_net_snapshot_t *snapshot)
{
    uint16_t control_holding_registers[
        APP_NET_CONTROL_HOLDING_REGISTER_COUNT];
    uint16_t control_status_registers[
        APP_NET_CONTROL_STATUS_REGISTER_COUNT];
    uint16_t analog_input_registers[
        APP_NET_ANALOG_INPUT_REGISTER_COUNT];
    uint8_t coil_states = 0U;
    uint8_t discrete_input_states = 0U;
    ret_t result;

    if (snapshot == NULL)
    {
        return RET_NULL_PTR;
    }

    (void)memset(snapshot, 0, sizeof(*snapshot));
    snapshot->poll_was_attempted = 1U;
    if (s_app_net_state_mutex != NULL)
    {
        (void)xSemaphoreTake(s_app_net_state_mutex, portMAX_DELAY);
        snapshot->led2_is_on = s_app_net_snapshot.led2_is_on;
        (void)xSemaphoreGive(s_app_net_state_mutex);
    }

    result = app_net_read_registers(
        APP_NET_SLAVE_CONTROL,
        APP_NET_REGISTER_AREA_HOLDING,
        0U,
        APP_NET_CONTROL_HOLDING_REGISTER_COUNT,
        control_holding_registers);
    if (result == RET_OK)
    {
        result = app_net_read_registers(
            APP_NET_SLAVE_CONTROL,
            APP_NET_REGISTER_AREA_HOLDING,
            10U,
            APP_NET_CONTROL_STATUS_REGISTER_COUNT,
            control_status_registers);
    }
    if (result == RET_OK)
    {
        (void)memcpy(snapshot->control_holding_registers,
                     control_holding_registers,
                     sizeof(control_holding_registers));
        (void)memcpy(snapshot->control_status_registers,
                     control_status_registers,
                     sizeof(control_status_registers));
        snapshot->valid_slave_mask |= 0x01U;
    }
    else
    {
        DBG_Warning("Modbus slave1 poll failed ret=%ld\r\n",
                    (long)result);
    }

    result = app_net_read_registers(
        APP_NET_SLAVE_ANALOG,
        APP_NET_REGISTER_AREA_INPUT,
        0U,
        APP_NET_ANALOG_INPUT_REGISTER_COUNT,
        analog_input_registers);
    if (result == RET_OK)
    {
        (void)memcpy(snapshot->analog_input_registers,
                     analog_input_registers,
                     sizeof(analog_input_registers));
        snapshot->valid_slave_mask |= 0x02U;
    }
    else
    {
        DBG_Warning("Modbus slave2 poll failed ret=%ld\r\n",
                    (long)result);
    }

    result = app_net_read_bits(APP_NET_SLAVE_IO,
                               APP_NET_BIT_AREA_COIL,
                               0U,
                               APP_NET_IO_POINT_COUNT,
                               &coil_states);
    if (result == RET_OK)
    {
        result = app_net_read_bits(APP_NET_SLAVE_IO,
                                   APP_NET_BIT_AREA_DISCRETE_INPUT,
                                   0U,
                                   APP_NET_IO_POINT_COUNT,
                                   &discrete_input_states);
    }
    if (result == RET_OK)
    {
        snapshot->io_coil_states = coil_states;
        snapshot->io_discrete_input_states = discrete_input_states;
        snapshot->valid_slave_mask |= 0x04U;
    }
    else
    {
        DBG_Warning("Modbus slave3 poll failed ret=%ld\r\n",
                    (long)result);
    }

    if (s_app_net_state_mutex != NULL)
    {
        (void)xSemaphoreTake(s_app_net_state_mutex, portMAX_DELAY);
        s_app_net_snapshot = *snapshot;
        (void)xSemaphoreGive(s_app_net_state_mutex);
    }
    app_net_notify_snapshot();

    return (snapshot->valid_slave_mask == APP_NET_ALL_SLAVES_VALID_MASK) ?
           RET_OK : RET_TIMEOUT;
}
#endif /* !APP_NET_FAKE_TELEMETRY_ENABLE */

/**
 * @brief 发布控制从站遥测
 * @note 控制、模拟量、IO 三条消息由 app_net_poll_and_publish() 依次调用，
 *       调用者在消息之间留出间隔，避免连续写入模组发送队列。
 */
static ret_t app_net_publish_control(const app_net_snapshot_t *snapshot)
{
    char payload[APP_NET_TELEMETRY_JSON_SIZE];
    ret_t ret;

    if (snapshot == NULL)
    {
        return RET_NULL_PTR;
    }

    ret = app_net_build_control_payload(snapshot,
                                        payload,
                                        (uint16_t)sizeof(payload));
    if (ret != RET_OK)
    {
        return ret;
    }
    DBG_log("MQTT telemetry payload, slave=%u: %s\r\n",
            (unsigned int)APP_NET_SLAVE_CONTROL,
            payload);
    ret = dev_net_mqtt_publish(APP_NET_TOPIC_TELEMETRY, payload);
    if (ret != RET_OK)
    {
        return ret;
    }

    DBG_log("MQTT telemetry UART send complete, slave=%u\r\n",
            (unsigned int)APP_NET_SLAVE_CONTROL);
    return RET_OK;
}

static ret_t app_net_publish_analog(const app_net_snapshot_t *snapshot)
{
    char payload[APP_NET_TELEMETRY_JSON_SIZE];
    ret_t ret;

    if (snapshot == NULL)
    {
        return RET_NULL_PTR;
    }

    ret = app_net_build_analog_payload(snapshot,
                                       payload,
                                       (uint16_t)sizeof(payload));
    if (ret != RET_OK)
    {
        return ret;
    }
    DBG_log("MQTT telemetry payload, slave=%u: %s\r\n",
            (unsigned int)APP_NET_SLAVE_ANALOG,
            payload);
    ret = dev_net_mqtt_publish(APP_NET_TOPIC_TELEMETRY, payload);
    if (ret != RET_OK)
    {
        return ret;
    }

    DBG_log("MQTT telemetry UART send complete, slave=%u\r\n",
            (unsigned int)APP_NET_SLAVE_ANALOG);
    return RET_OK;
}

static ret_t app_net_publish_io(const app_net_snapshot_t *snapshot)
{
    char payload[APP_NET_TELEMETRY_JSON_SIZE];
    ret_t ret;

    if (snapshot == NULL)
    {
        return RET_NULL_PTR;
    }

    ret = app_net_build_io_payload(snapshot,
                                   payload,
                                   (uint16_t)sizeof(payload));
    if (ret != RET_OK)
    {
        return ret;
    }
    DBG_log("MQTT telemetry payload, slave=%u: %s\r\n",
            (unsigned int)APP_NET_SLAVE_IO,
            payload);
    ret = dev_net_mqtt_publish(APP_NET_TOPIC_TELEMETRY, payload);
    if (ret != RET_OK)
    {
        return ret;
    }

    DBG_log("MQTT telemetry UART send complete, slave=%u\r\n",
            (unsigned int)APP_NET_SLAVE_IO);
    return RET_OK;
}

#if APP_NET_FAKE_TELEMETRY_ENABLE
/**
 * @brief   把 32 位浮点写成 Modbus 双寄存器（大端，与从站约定一致）
 * @note    与 app_net_registers_to_float32() 严格互逆：
 *          取 float 的 IEEE754 位模式 → 高 16 位存 registers[0]。
 *          这样假数据走的是和真实数据完全相同的解码路径，
 *          不会出现"假数据看着对、真数据接上后格式不对"的落差。
 */
static void app_net_float_to_registers(float value, uint16_t registers[2U])
{
    uint32_t raw;

    (void)memcpy(&raw, &value, sizeof(raw));
    registers[0U] = (uint16_t)(raw >> 16U);
    registers[1U] = (uint16_t)(raw & 0xFFFFU);
}

/**
 * @brief   生成一份随时间缓变的伪造设备快照
 * @param   snapshot  输出的快照
 * @param   tick      单调递增的轮次计数（决定波形相位）
 * @note    数值刻意做成"缓慢漂移 + 小幅抖动"，而不是随机跳变：
 *          平台侧的趋势曲线、报警规则看起来才像真实工况。
 *          valid_slave_mask 三路全置位，等价于"三个从站全部在线"。
 */
static void app_net_fake_fill_snapshot(app_net_snapshot_t *snapshot,
                                       uint32_t tick)
{
    float temperature;
    float pressure;
    float flow;
    float motor_temp;
    uint16_t speed;
    uint16_t level;

    /* 用轮次做相位，构造周期约 60 个轮次（5s x 60 = 5min）的缓变波形。
     * 只做整数运算，避免在 MCU 上引入浮点三角函数开销。 */
    {
        uint32_t phase = tick % 60U;
        /* 三角波：0..29 上升、30..59 下降，峰值 29。 */
        uint32_t tri = (phase < 30U) ? phase : (59U - phase);

        temperature = 24.5F + (float)tri * 0.15F;        /* 24.5 ~ 28.85 °C */
        pressure    = 0.42F + (float)tri * 0.006F;       /* 0.42 ~ 0.594 MPa */
        flow        = 12.0F + (float)tri * 0.2F;         /* 12 ~ 17.8 m3/h */
        motor_temp  = 41.0F + (float)tri * 0.25F;        /* 41 ~ 48.25 °C */
        speed       = (uint16_t)(1450U + tri * 3U);      /* 1450 ~ 1537 rpm */
        level       = (uint16_t)(620U + tri * 4U);       /* 620 ~ 736 mm */
    }

    (void)memset(snapshot, 0, sizeof(*snapshot));

    /* ---- 从站1：控制参数（保持寄存器 0~6） ---- */
    snapshot->control_holding_registers[0U] = 1U;        /* run_command  */
    snapshot->control_holding_registers[1U] = (uint16_t)1500; /* target_speed_rpm */
    snapshot->control_holding_registers[2U] = 0x0000U;   /* position_setpoint_pulse 高字 */
    snapshot->control_holding_registers[3U] = 0x1388U;   /* position_setpoint_pulse 低字 = 5000 */
    app_net_float_to_registers(50.0F,
                               &snapshot->control_holding_registers[4U]);
    snapshot->control_holding_registers[6U] = 3U;        /* acceleration_time_s */

    /* ---- 从站1：运行状态（保持寄存器 10~15） ---- */
    snapshot->control_status_registers[0U] = 3U;         /* run_state = 运行中 */
    snapshot->control_status_registers[1U] = speed;      /* actual_speed_rpm */
    app_net_float_to_registers(380.0F + (float)(tick % 10U),
                               &snapshot->control_status_registers[2U]);
    snapshot->control_status_registers[4U] =
        (uint16_t)(35U + (tick % 8U));                   /* output_current_a * 10 */
    snapshot->control_status_registers[5U] = 0U;          /* fault_code = 无故障 */

    /* ---- 从站2：模拟量（输入寄存器 0~10） ---- */
    app_net_float_to_registers(temperature,
                               &snapshot->analog_input_registers[0U]);
    app_net_float_to_registers(pressure,
                               &snapshot->analog_input_registers[2U]);
    app_net_float_to_registers(flow,
                               &snapshot->analog_input_registers[4U]);
    snapshot->analog_input_registers[6U] =
        (uint16_t)(2200U + (tick % 20U));                  /* voltage_v * 10 = 220.0~222.0 */
    snapshot->analog_input_registers[7U] =
        (uint16_t)(85U + (tick % 15U));                    /* current_a * 10 */
    snapshot->analog_input_registers[8U] = level;           /* liquid_level_mm */
    snapshot->analog_input_registers[9U] =
        (uint16_t)((int16_t)(motor_temp * 10.0F));         /* motor_temperature_c * 10 */
    snapshot->analog_input_registers[10U] = speed;         /* speed_feedback_rpm */

    /* ---- 从站3：开关量 ---- */
    snapshot->io_coil_states = (uint8_t)(0x03U | ((tick % 4U) << 2U));
    snapshot->io_discrete_input_states =
        (uint8_t)(0x81U | ((tick % 2U) << 2U));

    /* 三路全部置位 = 三个从站都"在线"，平台侧三组 point 都会展开 */
    snapshot->valid_slave_mask = APP_NET_ALL_SLAVES_VALID_MASK;
    snapshot->poll_was_attempted = 1U;

    if (s_app_net_state_mutex != NULL)
    {
        (void)xSemaphoreTake(s_app_net_state_mutex, portMAX_DELAY);
        snapshot->led2_is_on = s_app_net_snapshot.led2_is_on;
        (void)xSemaphoreGive(s_app_net_state_mutex);
    }
}

/**
 * @brief   假数据版一轮上报：不碰 485，直接组包发三个遥测主题
 * @note    与 app_net_poll_and_publish() 一一对应，只是数据来源换成
 *          本地生成的快照。复用同一批 build_*_payload 与 publish_*，
 *          所以发出去的 JSON 结构与真实链路完全一致。
 */
static ret_t app_net_fake_poll_and_publish(void)
{
    app_net_snapshot_t snapshot;
    static uint32_t fake_tick = 0U;
    ret_t publish_status;
    ret_t ret;

    fake_tick++;
    app_net_fake_fill_snapshot(&snapshot, fake_tick);

    DBG_log("FAKE telemetry tick=%lu mask=0x%02X\r\n",
            (unsigned long)fake_tick,
            (unsigned int)snapshot.valid_slave_mask);

    publish_status = RET_OK;

    ret = app_net_publish_control(&snapshot);
    if (ret != RET_OK)
    {
        DBG_Error("MQTT control telemetry publish failed ret=%ld\r\n",
                  (long)ret);
        publish_status = ret;
    }

    /* 三条遥测之间留出模组消化的间隙，防止突发灌满其发送队列
     * 导致模组主动断开重连（详见 APP_NET_PUBLISH_GAP_MS 注释）。 */
    vTaskDelay(pdMS_TO_TICKS(APP_NET_PUBLISH_GAP_MS));

    ret = app_net_publish_analog(&snapshot);
    if (ret != RET_OK)
    {
        DBG_Error("MQTT analog telemetry publish failed ret=%ld\r\n",
                  (long)ret);
        if (publish_status == RET_OK)
        {
            publish_status = ret;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(APP_NET_PUBLISH_GAP_MS));

    ret = app_net_publish_io(&snapshot);
    if (ret != RET_OK)
    {
        DBG_Error("MQTT IO telemetry publish failed ret=%ld\r\n",
                  (long)ret);
        if (publish_status == RET_OK)
        {
            publish_status = ret;
        }
    }

    /* 让屏幕上的"云状态/网络状态"跟着一起刷新，便于肉眼确认链路在动。 */
    app_net_notify_snapshot();

    return publish_status;
}
#endif /* APP_NET_FAKE_TELEMETRY_ENABLE */

#if !APP_NET_FAKE_TELEMETRY_ENABLE
static ret_t app_net_poll_and_publish(void)
{
    app_net_snapshot_t snapshot;
    ret_t poll_result;
    ret_t publish_status;
    ret_t ret;

    poll_result = app_net_poll_modbus(&snapshot);
    DBG_log("Modbus poll mask=0x%02X result=%ld\r\n",
            (unsigned int)snapshot.valid_slave_mask,
            (long)poll_result);

    publish_status = RET_OK;

    ret = app_net_publish_control(&snapshot);
    if (ret != RET_OK)
    {
        DBG_Error("MQTT control telemetry publish failed ret=%ld\r\n",
                  (long)ret);
        publish_status = ret;
    }

    /* 同 app_net_fake_poll_and_publish()：三条遥测不可背靠背发。 */
    vTaskDelay(pdMS_TO_TICKS(APP_NET_PUBLISH_GAP_MS));

    ret = app_net_publish_analog(&snapshot);
    if (ret != RET_OK)
    {
        DBG_Error("MQTT analog telemetry publish failed ret=%ld\r\n",
                  (long)ret);
        if (publish_status == RET_OK)
        {
            publish_status = ret;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(APP_NET_PUBLISH_GAP_MS));

    ret = app_net_publish_io(&snapshot);
    if (ret != RET_OK)
    {
        DBG_Error("MQTT IO telemetry publish failed ret=%ld\r\n",
                  (long)ret);
        if (publish_status == RET_OK)
        {
            publish_status = ret;
        }
    }

    return publish_status;
}
#endif /* !APP_NET_FAKE_TELEMETRY_ENABLE */

static uint8_t app_net_json_u16(cJSON *object,
                                const char *name,
                                uint16_t minimum,
                                uint16_t maximum,
                                uint16_t *value)
{
    cJSON *item;

    if ((object == NULL) || (name == NULL) || (value == NULL))
    {
        return 0U;
    }

    item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (!cJSON_IsNumber(item) ||
        (item->valuedouble < (double)minimum) ||
        (item->valuedouble > (double)maximum) ||
        ((double)item->valueint != item->valuedouble))
    {
        return 0U;
    }

    *value = (uint16_t)item->valueint;
    return 1U;
}

static uint8_t app_net_json_bool(cJSON *object,
                                const char *name,
                                uint8_t *value)
{
    cJSON *item;

    if ((object == NULL) || (name == NULL) || (value == NULL))
    {
        return 0U;
    }

    item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (cJSON_IsBool(item))
    {
        *value = cJSON_IsTrue(item) ? 1U : 0U;
        return 1U;
    }

    if (cJSON_IsNumber(item) &&
        ((item->valueint == 0) || (item->valueint == 1)) &&
        ((double)item->valueint == item->valuedouble))
    {
        *value = (uint8_t)item->valueint;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 执行一次云端寄存器读取 RPC 并生成响应 JSON
 * @param area 由业务层指定的寄存器区，组合层再负责协议映射
 * @note response_payload 属于当前任务栈，函数不会保存其地址。
 */
static ret_t app_net_rpc_read_registers(cJSON *params,
                                        app_net_register_area_t area,
                                        char *response_payload,
                                        uint16_t response_capacity)
{
    uint16_t slave_address;
    uint16_t start_address;
    uint16_t register_quantity;
    uint16_t register_values[16U];
    uint16_t register_index;
    uint16_t payload_length;
    int formatted_length;
    ret_t result;

    if (!app_net_json_u16(params, "slave", 1U, 247U, &slave_address) ||
        !app_net_json_u16(params, "address", 0U, 65535U, &start_address) ||
        !app_net_json_u16(params, "quantity", 1U, 16U,
                          &register_quantity))
    {
        return RET_INVALID_PARAM;
    }

    result = app_net_read_registers((uint8_t)slave_address,
                                    area,
                                    start_address,
                                    register_quantity,
                                    register_values);
    if (result != RET_OK)
    {
        return result;
    }

    formatted_length = snprintf(response_payload,
                                response_capacity,
                                "{\"success\":true,\"values\":[");
    if ((formatted_length < 0) ||
        (formatted_length >= (int)response_capacity))
    {
        return RET_OVERFLOW;
    }
    payload_length = (uint16_t)formatted_length;

    for (register_index = 0U;
         register_index < register_quantity;
         register_index++)
    {
        formatted_length = snprintf(&response_payload[payload_length],
                                    response_capacity - payload_length,
                                    "%s%u",
                                    (register_index == 0U) ? "" : ",",
                                    (unsigned int)register_values[
                                        register_index]);
        if ((formatted_length < 0) ||
            (formatted_length >=
             (int)(response_capacity - payload_length)))
        {
            return RET_OVERFLOW;
        }
        payload_length = (uint16_t)(payload_length +
                                    (uint16_t)formatted_length);
    }

    formatted_length = snprintf(&response_payload[payload_length],
                                response_capacity - payload_length,
                                "]}");
    if ((formatted_length < 0) ||
        (formatted_length >=
         (int)(response_capacity - payload_length)))
    {
        return RET_OVERFLOW;
    }
    return RET_OK;
}

/**
 * @brief 在网络任务上下文中处理一条 RPC
 * @note 接收回调不做 JSON 解析、不访问 Modbus；所有可能阻塞的业务调用
 *       都在这里串行执行，完成后通过请求序号发布 response 主题。
 */
static void app_net_process_rpc(const app_net_rpc_msg_t *rpc_message)
{
    cJSON *root_object;
    cJSON *method_item;
    cJSON *params_object;
    uint16_t slave_address;
    uint16_t target_address;
    uint16_t target_value;
    uint8_t led2_is_on;
    uint8_t target_is_on;
    uint8_t is_read_response;
    ret_t result;
    char response_payload[APP_NET_RPC_RESPONSE_SIZE];
    char response_topic[64U];
    int formatted_length;

    if (rpc_message == NULL)
    {
        return;
    }

    root_object = cJSON_Parse(rpc_message->json_text);
    method_item = (root_object != NULL) ?
                  cJSON_GetObjectItemCaseSensitive(root_object, "method") :
                  NULL;
    params_object = (root_object != NULL) ?
                    cJSON_GetObjectItemCaseSensitive(root_object, "params") :
                    NULL;
    result = RET_INVALID_PARAM;
    is_read_response = 0U;
    (void)snprintf(response_payload,
                   sizeof(response_payload),
                   "{\"success\":false,\"error\":\"invalid_request\"}");

    if (cJSON_IsString(method_item) && cJSON_IsObject(params_object))
    {
        if (strcmp(method_item->valuestring, "setLed") == 0)
        {
            uint16_t led_id;

            if (app_net_json_u16(params_object, "led", 2U, 2U, &led_id) &&
                app_net_json_bool(params_object, "state", &led2_is_on))
            {
                result = app_net_set_led_state(led2_is_on);
            }
        }
        else if (strcmp(method_item->valuestring,
                         "writeHoldingRegister") == 0)
        {
            if (app_net_json_u16(params_object,
                                 "slave",
                                 1U,
                                 247U,
                                 &slave_address) &&
                app_net_json_u16(params_object,
                                 "address",
                                 0U,
                                 65535U,
                                 &target_address) &&
                app_net_json_u16(params_object,
                                 "value",
                                 0U,
                                 65535U,
                                 &target_value) &&
                (slave_address == APP_NET_SLAVE_CONTROL) &&
                (target_address <= APP_NET_CONTROL_HR_LAST))
            {
                result = app_net_write_register((uint8_t)slave_address,
                                                target_address,
                                                target_value);
            }
        }
        else if (strcmp(method_item->valuestring, "writeCoil") == 0)
        {
            if (app_net_json_u16(params_object,
                                 "slave",
                                 1U,
                                 247U,
                                 &slave_address) &&
                app_net_json_u16(params_object,
                                 "address",
                                 0U,
                                 65535U,
                                 &target_address) &&
                app_net_json_bool(params_object, "value", &target_is_on) &&
                (slave_address == APP_NET_SLAVE_IO) &&
                (target_address < APP_NET_IO_POINT_COUNT))
            {
                result = app_net_write_coil((uint8_t)slave_address,
                                            target_address,
                                            target_is_on);
            }
        }
        else if (strcmp(method_item->valuestring,
                        "readHoldingRegisters") == 0)
        {
            result = app_net_rpc_read_registers(
                params_object,
                APP_NET_REGISTER_AREA_HOLDING,
                response_payload,
                (uint16_t)sizeof(response_payload));
            is_read_response = 1U;
        }
        else if (strcmp(method_item->valuestring,
                        "readInputRegisters") == 0)
        {
            result = app_net_rpc_read_registers(
                params_object,
                APP_NET_REGISTER_AREA_INPUT,
                response_payload,
                (uint16_t)sizeof(response_payload));
            is_read_response = 1U;
        }
    }

    if ((result == RET_OK) && (is_read_response == 0U))
    {
        (void)snprintf(response_payload,
                       sizeof(response_payload),
                       "{\"success\":true}");
    }
    else if (result != RET_OK)
    {
        (void)snprintf(response_payload,
                       sizeof(response_payload),
                       "{\"success\":false,\"status\":%ld}",
                       (long)result);
    }

    if (root_object != NULL)
    {
        cJSON_Delete(root_object);
    }

    formatted_length = snprintf(response_topic,
                                sizeof(response_topic),
                                "%s%lu",
                                APP_NET_TOPIC_RPC_RESPONSE_PREFIX,
                                (unsigned long)rpc_message->request_id);
    if ((formatted_length < 0) ||
        (formatted_length >= (int)sizeof(response_topic)))
    {
        DBG_Error("MQTT RPC response topic overflow\r\n");
        return;
    }

    result = dev_net_mqtt_publish(response_topic, response_payload);
    if (result != RET_OK)
    {
        DBG_Error("MQTT RPC response failed id=%lu ret=%ld\r\n",
                  (unsigned long)rpc_message->request_id,
                  (long)result);
        /*
         * 这里曾经调用 app_net_set_online(0U)，那是"发布失败即判离线"的
         * 误判源头：模组长期在线，一笔响应没发出去不代表链路断了。
         * 旧实现会因此连锁触发 AT+REST 整机重启，把一次丢帧放大成
         * 4~8 秒的上报黑洞，并由此自激。现在只记日志，不改变链路状态；
         * 恢复只由 MQTT_DISCONNECT 或连接超时驱动的状态机负责。
         */
        s_app_net_publish_fail_count++;
    }
    else
    {
        DBG_log("MQTT RPC response id=%lu payload=%s\r\n",
                (unsigned long)rpc_message->request_id,
                response_payload);
    }
}

/**
 * @brief 网络业务主任务
 * @note 任务同时承担周期遥测和 RPC 响应。RPC 队列只保存值拷贝，避免
 *       接收回调的临时缓冲区被异步使用。
 */
static void app_net_task(void *task_parameter)
{
    app_net_rpc_msg_t rpc_message;
    dev_net_link_event_t link_event;
    ret_t result;
    TickType_t next_poll_tick;
    TickType_t current_tick;

    (void)task_parameter;
    next_poll_tick = xTaskGetTickCount() +
                     pdMS_TO_TICKS(APP_NET_POLL_PERIOD_MS);

    for (;;)
    {
        while (xQueueReceive(s_app_net_link_event_queue,
                             &link_event,
                             0U) == pdPASS)
        {
            app_net_handle_link_event(link_event);
        }

        app_net_run_state_machine();
        current_tick = xTaskGetTickCount();

        /*
         * OTA 会话推进：消费下行队列、发分片请求、超时重试。
         * 下载期间暂停周期遥测，避免分片请求与遥测竞争模组发送队列。
         */
        ota_dl_tick();

        if ((s_app_net_state == APP_NET_STATE_ONLINE) &&
            (ota_dl_active() == 0U) &&
            ((int32_t)(current_tick - next_poll_tick) >= 0))
        {
#if APP_NET_FAKE_TELEMETRY_ENABLE
            result = app_net_fake_poll_and_publish();
#else
            result = app_net_poll_and_publish();
#endif
            if (result != RET_OK)
            {
                /*
                 * 单轮上报失败只记一笔，不做任何链路恢复。
                 * 参考工程同样是失败即返回、下一周期重试：模组长期在线，
                 * 偶发丢帧不需要（也不应该）靠重启模组来"修"。
                 */
                s_app_net_publish_fail_count++;
                DBG_Warning("telemetry publish failed ret=%ld, fail_count=%lu\r\n",
                            (long)result,
                            (unsigned long)s_app_net_publish_fail_count);
            }
            else
            {
                s_app_net_publish_fail_count = 0U;
            }

            next_poll_tick = xTaskGetTickCount() +
                             pdMS_TO_TICKS(APP_NET_POLL_PERIOD_MS);
        }

        if (s_app_net_state == APP_NET_STATE_ONLINE)
        {
            while (xQueueReceive(s_app_net_rpc_queue,
                                 &rpc_message,
                                 0U) == pdPASS)
            {
                app_net_process_rpc(&rpc_message);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(APP_NET_FSM_TICK_MS));
    }
}

/**
 * @brief 创建网络业务所需的内核对象并启动网络状态机
 * @note 初始化顺序是：校验依赖 -> 创建队列/锁 -> 初始化 LED -> 准备
 *       网络 Device -> 创建网络任务。配置、reset、连接等待和订阅由
 *       任务中的状态机继续执行；中途失败会释放已创建的对象。
 */
ret_t app_net_init(const app_net_dependencies_t *dependencies)
{
    cJSON_Hooks hooks;
    ret_t result;

    if ((s_app_net_rpc_queue != NULL) ||
        (s_app_net_task_handle != NULL))
    {
        return RET_ALREADY_INIT;
    }

    result = app_net_validate_dependencies(dependencies);
    if (result != RET_OK)
    {
        return result;
    }

    s_app_net_dependencies = *dependencies;

    (void)memset(&hooks, 0, sizeof(hooks));
    hooks.malloc_fn = app_net_json_malloc;
    hooks.free_fn = app_net_json_free;
    cJSON_InitHooks(&hooks);

    s_app_net_rpc_queue =
        xQueueCreate(APP_NET_RPC_QUEUE_LENGTH, sizeof(app_net_rpc_msg_t));
    if (s_app_net_rpc_queue == NULL)
    {
        return RET_NO_MEMORY;
    }

    s_app_net_link_event_queue =
        xQueueCreate(APP_NET_LINK_EVENT_QUEUE_LENGTH,
                     sizeof(dev_net_link_event_t));
    if (s_app_net_link_event_queue == NULL)
    {
        vQueueDelete(s_app_net_rpc_queue);
        s_app_net_rpc_queue = NULL;
        return RET_NO_MEMORY;
    }

    s_app_net_state_mutex = xSemaphoreCreateMutex();
    if (s_app_net_state_mutex == NULL)
    {
        vQueueDelete(s_app_net_link_event_queue);
        s_app_net_link_event_queue = NULL;
        vQueueDelete(s_app_net_rpc_queue);
        s_app_net_rpc_queue = NULL;
        return RET_NO_MEMORY;
    }

    (void)memset(&s_app_net_snapshot, 0, sizeof(s_app_net_snapshot));
    s_app_net_is_online = 0U;
    s_app_net_publish_fail_count = 0U;
    s_app_net_state = APP_NET_STATE_CONFIGURING;
    s_app_net_state_deadline = 0U;
    s_app_net_reconnect_count = 0U;

    /* OTA 会话队列 + worker 任务：必须在订阅与网络任务启动前就绪。 */
    result = ota_dl_init();
    if ((result != RET_OK) && (result != RET_ALREADY_INIT))
    {
        vQueueDelete(s_app_net_link_event_queue);
        s_app_net_link_event_queue = NULL;
        vSemaphoreDelete(s_app_net_state_mutex);
        s_app_net_state_mutex = NULL;
        vQueueDelete(s_app_net_rpc_queue);
        s_app_net_rpc_queue = NULL;
        return result;
    }

    result = app_net_set_led_state(0U);
    if (result != RET_OK)
    {
        vQueueDelete(s_app_net_link_event_queue);
        s_app_net_link_event_queue = NULL;
        vSemaphoreDelete(s_app_net_state_mutex);
        vQueueDelete(s_app_net_rpc_queue);
        s_app_net_state_mutex = NULL;
        s_app_net_rpc_queue = NULL;
        return result;
    }

    /* 仅准备 UART 和模块配置；reset/连接等待/订阅由 app_net 状态机执行。 */
    result = app_net_prepare();
    if (result != RET_OK)
    {
        DBG_Error("network prepare failed ret=%ld\r\n", (long)result);
        dev_net_register_link_cb(NULL);
        dev_net_register_recv_cb(NULL);
        vQueueDelete(s_app_net_link_event_queue);
        s_app_net_link_event_queue = NULL;
        vSemaphoreDelete(s_app_net_state_mutex);
        vQueueDelete(s_app_net_rpc_queue);
        s_app_net_state_mutex = NULL;
        s_app_net_rpc_queue = NULL;
        return result;
    }

    app_net_notify_snapshot();

    if (xTaskCreate(app_net_task,
                    "app_net",
                    APP_NET_TASK_STACK_DEPTH,
                    NULL,
                    APP_NET_TASK_PRIORITY,
                    &s_app_net_task_handle) != pdPASS)
    {
        dev_net_register_link_cb(NULL);
        dev_net_register_recv_cb(NULL);
        vQueueDelete(s_app_net_link_event_queue);
        s_app_net_link_event_queue = NULL;
        vSemaphoreDelete(s_app_net_state_mutex);
        vQueueDelete(s_app_net_rpc_queue);
        s_app_net_state_mutex = NULL;
        s_app_net_rpc_queue = NULL;
        s_app_net_task_handle = NULL;
        return RET_TASK_CREATE_ERROR;
    }

    DBG_log("MQTT cloud app task created\r\n");
    return RET_OK;
}
