/**
 * @file    dev_net.c
 * @brief   ML307R 网络模组 Device 层实现
 *
 * @note    分层契约：Device 层负责 AT 事务、响应分类和 MQTT 命令封装；
 *          BSP 层只负责 UART/GPIO 与字节搬运；App 层只处理业务数据。
 *
 * @note    设计不变量（阅读与修改都以此为准）：
 *          1. 先挂牌（expect/waiting）再发命令——防响应早到被当残渣；
 *          2. 事件位发送前清一次（防过去）、clearOnExit 清一次（防未来）；
 *          3. 两关验收：结果位=结论码分流，正文含 expect=业务判定，
 *             两关全过才算成功（反例：AT+CPIN? 没插卡也回 OK）；
 *          4. 事件组只公告当前 AT 事务的结果，字节正文在 rx_buffer；
 *          5. MQTT URC（模组主动上报）走链路回调，不混入 AT 结果位。
 *
 * @note    并发分工（三把钥匙各管一扇门）：
 *          - 事务互斥锁：同一时刻只允许一笔完整 AT 事务（清场→挂牌→
 *            发送→等待→撤牌→还锁是原子单元）；
 *          - 短临界区：rx_buffer/ring 的两处写入点（解析任务批量搬移、
 *            命令任务清场）用 taskENTER_CRITICAL 保护，微秒级、无阻塞；
 *          - 事件组：FreeRTOS 自带并发安全，只做跨任务公告，不加锁。
 */

#include "dev_net/dev_net.h"
#include "bsp_gpio/bsp_gpio.h"
#include "bsp_uart/bsp_uart.h"
#include "SEGGER_RTT.h"
#include "config.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"

/* ======================== 协议与资源配置 ======================== */

#define NET_RX_BUF_SIZE                 1024U
#define NET_SUB_CMD_BUF_SIZE            128U
/* 发布从站工程量时需要容纳完整 JSON、主题和命令前缀。 */
#define NET_PUB_CMD_BUF_SIZE            640U

#define NET_RX_TASK_STACK_SIZE          512U
#define NET_RX_TASK_PRIORITY            3U

#define NET_AT_TEST                     "AT\r\n"
#define NET_AT_MQTT_MULTI               "AT+MQMULTEN=1\r\n"
#define NET_AT_MQTT_FILTER              "AT+MQTTFILTER=0\r\n"
#define NET_AT_RESET                    "AT+REST\r\n"
#define NET_AT_TIME_STANDARD             "AT+TIME=1\r\n"

#define NET_EXPECT_OK                   "OK\r\n"
#define NET_EXPECT_ERROR                "ERROR"
#define NET_EXPECT_MQTT_CONNECT         "MQTT_CONNECT:"
#define NET_EXPECT_MQTT_DISCONNECT      "MQTT_DISCONNECT"
#define NET_EXPECT_TIME                 "+TIME=\""

#define NET_CMD_DEFAULT_TIMEOUT_MS      2000U
#define NET_RESET_ACK_TIMEOUT_MS        3000U
#define NET_MAX_SUBSCRIPTIONS           10U

/* 命令终态：成功、模组明确拒绝、接收缓冲溢出。 */
#define NET_EVENT_CMD_OK                (1U << 0)
#define NET_EVENT_CMD_ERROR             (1U << 1)
#define NET_EVENT_RX_OVERFLOW           (1U << 2)

#define NET_EVENT_CMD_RESULT            (NET_EVENT_CMD_OK       \
                                         | NET_EVENT_CMD_ERROR  \
                                         | NET_EVENT_RX_OVERFLOW)

/* ======================== 内部数据结构 ======================== */

/** 当前 AT 事务的累计接收正文。 */
typedef struct
{
    uint16_t rx_index;
    uint8_t rx_buffer[NET_RX_BUF_SIZE];
} net_handle_t;

/** 一条可复用的 AT 命令步骤：命令、期望响应和超时彼此独立。 */
typedef struct
{
    const char *cmd;
    const char *expect;
    uint32_t timeout_ms;
} net_cmd_step_t;

static const net_cmd_step_t s_net_connect_steps[] =
{
    {NET_AT_MQTT_MULTI,  NET_EXPECT_OK, 2000U},
    {NET_AT_MQTT_FILTER, NET_EXPECT_OK, 2000U},
};

static const net_cmd_step_t s_net_reset_step =
{
    NET_AT_RESET,
    NET_EXPECT_OK,
    NET_RESET_ACK_TIMEOUT_MS
};

/* ======================== 模块私有状态 ======================== */

static net_handle_t s_net_handle;
static TaskHandle_t s_net_rx_task_handle = NULL;
static EventGroupHandle_t s_net_events = NULL;
static SemaphoreHandle_t s_net_cmd_mutex = NULL;
/** 同步 AT 响应中已转交给 App 的 MQTT URC 字节位置，防止累计缓冲重复投递。 */
static uint16_t s_net_async_delivered_index = 0U;

/* 链路生命周期由上层状态机拥有；Device 只负责识别并顺序上报 URC。 */
static dev_net_link_cb_t s_net_link_cb = NULL;
static volatile uint8_t s_net_link_online = 0U;
static volatile uint8_t s_net_accept_connect_event = 1U;
static volatile uint8_t s_net_connect_event_pending = 0U;

/* waiting/expect 描述当前唯一一笔同步 AT 事务。
   expect 指针本身被命令任务写、解析任务读，必须 volatile 修饰指针自身。 */
static volatile uint8_t s_net_cmd_waiting = 0U;
static const char * volatile s_net_cmd_expect = NULL;

static uint8_t s_net_subscribe_index = 0U;
static dev_net_recv_cb_t s_net_recv_cb = NULL;

/* ======================== 内部接口声明 ======================== */

static ret_t dev_net_send(uint8_t *data,
                          uint16_t length,
                          uint16_t timeout_ms);
static ret_t dev_net_rcv(uint8_t *byte);
static ret_t net_check_expect(const char *expect);

static void dev_net_report_link_event(dev_net_link_event_t event)
{
    if (event == DEV_NET_LINK_CONNECTED)
    {
        if (s_net_link_online != 0U)
        {
            return;
        }
        s_net_link_online = 1U;
        if (s_net_accept_connect_event == 0U)
        {
            s_net_connect_event_pending = 1U;
            return;
        }

        if (s_net_link_cb != NULL)
        {
            s_net_link_cb(event);
        }
    }
    else
    {
        s_net_connect_event_pending = 0U;
        if (s_net_link_online == 0U)
        {
            return;
        }

        s_net_link_online = 0U;
        if (s_net_link_cb != NULL)
        {
            s_net_link_cb(event);
        }
    }
}

static void dev_net_reset_rx_state(void);
static void dev_net_process_rx(void);
static ret_t dev_net_send_cmd(const char *cmd,
                              const char *expect,
                              uint32_t timeout_ms);
static ret_t dev_net_send_cmd_response(const char *cmd,
                                       const char *expect,
                                       uint32_t timeout_ms,
                                       char *response,
                                       uint16_t response_size);
static ret_t dev_net_run_step(const net_cmd_step_t *step);
static void dev_net_notify_task(void *param);

/* ======================== 传输适配层 ======================== */

/**
 * @brief  通过网络 UART 发送字节
 * @note   参数检查后调用 bsp_uart_transmit()，原样传播错误码。
 *         本函数只证明 UART 发送完成，不等待模组响应。
 */
static ret_t dev_net_send(uint8_t *data,
                          uint16_t length,
                          uint16_t timeout_ms)
{
    if (data == NULL)
    {
        return RET_NULL_PTR;
    }

    if ((length == 0U) || (timeout_ms == 0U))
    {
        return RET_INVALID_PARAM;
    }

    return bsp_uart_transmit(BSP_UART_NET, data, length, timeout_ms);
}

/**
 * @brief  从 BSP 网络 UART 环形缓冲区取一个字节
 * @note   检查 byte 后调用 bsp_uart_receive(BSP_UART_NET, byte)。
 *         每次只取一个字节；无数据时应原样返回 RET_EMPTY。
 */
static ret_t dev_net_rcv(uint8_t *byte)
{
    if (byte == NULL)
    {
        return RET_NULL_PTR;
    }

    return bsp_uart_receive(BSP_UART_NET, byte);
}

/* ======================== 响应缓冲与分类 ======================== */

/**
 * @brief  重置当前 AT 响应正文
 * @note   将 rx_index 归零，并保证 rx_buffer[0] 为 '\0'。
 *         只能在开始一笔新 AT 事务时调用，不能每次通知都清零。
 */
static void dev_net_reset_rx_state(void)
{
    s_net_handle.rx_index = 0U;
    s_net_handle.rx_buffer[0] = '\0';
    s_net_async_delivered_index = 0U;
}

/**
 * @brief  检查累计正文是否包含期待字符串
 * @note   expect 为空返回 RET_NULL_PTR；匹配成功返回 RET_OK；
 *         否则返回 RET_NO_RESPONSE。
 */
static ret_t net_check_expect(const char *expect)
{
    if (expect == NULL)
    {
        return RET_NULL_PTR;
    }

    if (strstr((const char *)s_net_handle.rx_buffer, expect) == NULL)
    {
        return RET_NO_RESPONSE;
    }

    return RET_OK;
}

/**
 * @brief  分类当前累计接收数据并设置事务事件位或调用 App 回调
 *
 * @note   处理顺序：
 *         1. 独立检查 MQTT_CONNECT:/MQTT_DISCONNECT，交给上层状态机；
 *         2. waiting 有效时，检查 expect 与 ERROR，设置命令结果位；
 *         3. 没有同步命令等待时，才把异步下行交给 App 回调；
 *         4. 事件组只传 AT 事务结果，响应正文仍保存在 s_net_handle 中。
 */
static void dev_net_process_rx(void)
{
    const char *rx_text;

    /* 没有正文时无需分类。 */
    if (s_net_handle.rx_index == 0U)
    {
        return;
    }

    rx_text = (const char *)s_net_handle.rx_buffer;

    /*
     * MQTT_CONNECT: 是模组主动上报的 URC，不属于某一条 AT 命令。
     * 即使当前正在等待 AT 响应，也必须独立识别这个连接事件。
     */
    if (strstr(rx_text, NET_EXPECT_MQTT_CONNECT) != NULL)
    {
        dev_net_report_link_event(DEV_NET_LINK_CONNECTED);
    }
    if (strstr(rx_text, NET_EXPECT_MQTT_DISCONNECT) != NULL)
    {
        dev_net_report_link_event(DEV_NET_LINK_DISCONNECTED);
    }

    /*
     * waiting 有效时，这批数据优先属于当前同步 AT 事务。
     * ERROR 是明确的否定终态，因此先于 expect 判断。
     * 响应可能尚未收完整；两者都没找到时保留缓冲，等待后续通知。
     */
    if (s_net_cmd_waiting != 0U)
    {
        if ((s_net_events == NULL) || (s_net_cmd_expect == NULL))
        {
            return;
        }

        /* MQTT URC 可能恰好与 AT 命令响应交错到达。AT 缓冲是累计的，
         * 因此只把新增字节转交一次；App 回调只做复制/投递，不解析阻塞。 */
        if ((s_net_recv_cb != NULL) &&
            (s_net_async_delivered_index < s_net_handle.rx_index) &&
            (strstr(rx_text, "+MQTTMSG:") != NULL))
        {
            s_net_recv_cb(
                (const char *)&s_net_handle.rx_buffer[s_net_async_delivered_index],
                (uint16_t)(s_net_handle.rx_index -
                           s_net_async_delivered_index));
            s_net_async_delivered_index = s_net_handle.rx_index;
        }

        if (strstr(rx_text, NET_EXPECT_ERROR) != NULL)
        {
            (void)xEventGroupSetBits(s_net_events,
                                     NET_EVENT_CMD_ERROR);
        }
        else if (strstr(rx_text, s_net_cmd_expect) != NULL)
        {
            (void)xEventGroupSetBits(s_net_events,
                                     NET_EVENT_CMD_OK);
        }

        return;
    }

    /*
     * 没有同步 AT 事务等待时，这批数据属于异步上报。
     * 回调在 net_rx 任务上下文执行，只能快速复制或投递数据。
     */
    if (s_net_recv_cb != NULL)
    {
        s_net_recv_cb(rx_text, s_net_handle.rx_index);
    }

    /* 异步数据交付结束，清空缓冲，为下一批数据做准备。 */
    dev_net_reset_rx_state();
}

/* ======================== 同步 AT 事务引擎 ======================== */

/**
 * @brief  发送一条 AT 命令并同步等待结果
 *
 * @note   严格保持顺序：
 *         参数检查 -> 初始化检查 -> 获取事务锁 -> 清旧结果位
 *         -> 清旧正文 -> 挂 expect/waiting -> 发送 -> 等待结果
 *         -> 判断 OK/ERROR/OVERFLOW/超时 -> 清挂牌 -> 释放锁。
 *
 *         获取锁后禁止直接 return；所有出口必须经过统一清理段。
 *         互斥锁覆盖完整请求/响应事务，接收任务不得获取此锁。
 */
static ret_t dev_net_send_cmd(const char *cmd,
                              const char *expect,
                              uint32_t timeout_ms)
{
    return dev_net_send_cmd_response(cmd,
                                     expect,
                                     timeout_ms,
                                     NULL,
                                     0U);
}

/**
 * @brief  执行同步 AT 事务，并可在释放事务锁前复制完整响应
 * @note   response 为 NULL 时不复制正文；非 NULL 时 response_size 必须非零。
 */
static ret_t dev_net_send_cmd_response(const char *cmd,
                                       const char *expect,
                                       uint32_t timeout_ms,
                                       char *response,
                                       uint16_t response_size)
{
    EventBits_t event_bits;
    ret_t ret;
    uint8_t discard_byte;

    if ((cmd == NULL) || (expect == NULL))
    {
        return RET_NULL_PTR;
    }

    if (((response == NULL) && (response_size != 0U)) ||
        ((response != NULL) && (response_size == 0U)))
    {
        return RET_INVALID_PARAM;
    }

    if (response != NULL)
    {
        response[0] = '\0';
    }

    if ((timeout_ms == 0U) || (timeout_ms > UINT16_MAX))
    {
        return RET_INVALID_PARAM;
    }

    if ((s_net_events == NULL) || (s_net_cmd_mutex == NULL))
    {
        return RET_NOT_INIT;
    }

    if (xSemaphoreTake(s_net_cmd_mutex,
                       pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return RET_TIMEOUT;
    }

    /* 清除历史结果；MQTT_CONNECTED 是独立状态，不在本掩码内。 */
    (void)xEventGroupClearBits(s_net_events, NET_EVENT_CMD_RESULT);

    /*
     * 清场：先排空 BSP 环形缓冲，再清响应正文，然后才挂牌。
     * 顺序不能颠倒——挂牌之后到达的字节才属于本条命令；
     * 上一条命令残留在环形缓冲里的旧数据（可能含 "OK\r\n"）若被
     * 解析任务搬进新事务正文，模组还没应答就会伪成功。
     * 临界区屏蔽调度与 UART 中断，使"排空+清缓冲+挂牌"相对解析任务原子。
     */
    taskENTER_CRITICAL();
    while (dev_net_rcv(&discard_byte) == RET_OK)
    {
        /* 丢弃：这些字节属于上一个时代。 */
    }
    dev_net_reset_rx_state();
    s_net_cmd_expect = expect;
    s_net_cmd_waiting = 1U;
    taskEXIT_CRITICAL();

    ret = dev_net_send((uint8_t *)cmd,
                       (uint16_t)strlen(cmd),
                       (uint16_t)timeout_ms);
    if (ret != RET_OK)
    {
        goto cmd_exit;
    }

    event_bits = xEventGroupWaitBits(s_net_events,
                                     NET_EVENT_CMD_RESULT,
                                     pdTRUE,
                                     pdFALSE,
                                     pdMS_TO_TICKS(timeout_ms));

    if ((event_bits & NET_EVENT_RX_OVERFLOW) != 0U)
    {
        ret = RET_OVERFLOW;
    }
    else if ((event_bits & NET_EVENT_CMD_ERROR) != 0U)
    {
        ret = RET_NACK_ERROR;
    }
    else if ((event_bits & NET_EVENT_CMD_OK) != 0U)
    {
        ret = net_check_expect(expect);

        if ((ret == RET_OK) && (response != NULL))
        {
            if ((uint32_t)s_net_handle.rx_index + 1U > response_size)
            {
                ret = RET_LENGTH_ERROR;
            }
            else
            {
                (void)memcpy(response,
                             s_net_handle.rx_buffer,
                             (uint32_t)s_net_handle.rx_index + 1U);
            }
        }
    }
    else
    {
        ret = RET_TIMEOUT;
    }

cmd_exit:
    /* 在释放事务锁前撤销本次挂牌，避免破坏下一笔事务。 */
    taskENTER_CRITICAL();
    s_net_cmd_waiting = 0U;
    s_net_cmd_expect = NULL;
    dev_net_reset_rx_state();
    taskEXIT_CRITICAL();

    if (xSemaphoreGive(s_net_cmd_mutex) != pdTRUE)
    {
        if (ret == RET_OK)
        {
            ret = RET_OS_ERROR;
        }
    }

    return ret;
}

/**
 * @brief  执行表驱动配置中的一个步骤
 * @note   校验 step，调用 dev_net_send_cmd() 并传播结果。
 */
static ret_t dev_net_run_step(const net_cmd_step_t *step)
{
    if (step == NULL)
    {
        return RET_NULL_PTR;
    }

    return dev_net_send_cmd(step->cmd,
                            step->expect,
                            step->timeout_ms);
}

/* ======================== 接收任务 ======================== */

/**
 * @brief  网络 UART 接收与解析任务
 *
 * @note   *         1. 永久阻塞等待 UART ISR 的任务通知；
 *         2. while 调用 dev_net_rcv()，直到返回 RET_EMPTY；
 *         3. 字节追加时始终为 '\0' 预留一个位置；
 *         4. 缓冲区满时继续排空 BSP，并设置 RX_OVERFLOW；
 *         5. 排空后调用 dev_net_process_rx()；
 *         6. 不打印原始协议字符串，不在此任务中长时间阻塞。
 */
static void dev_net_notify_task(void *param)
{
    uint8_t byte;
    uint8_t overflowed;
    ret_t ret;

    (void)param;

    while (1)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        overflowed = 0U;

        /*
         * 批量搬移在短临界区内完成：解析任务是 rx_buffer 的日常写入者，
         * 命令任务的"清场"也在临界区里做，两边才不会互相踩踏。
         * 循环只搬运字节、无阻塞，临界区时长为微秒级。
         */
        taskENTER_CRITICAL();
        while ((ret = dev_net_rcv(&byte)) == RET_OK)
        {
            if (s_net_handle.rx_index < (NET_RX_BUF_SIZE - 1U))
            {
                s_net_handle.rx_buffer[s_net_handle.rx_index++] = byte;
                s_net_handle.rx_buffer[s_net_handle.rx_index] = '\0';
            }
            else
            {
                overflowed = 1U;
            }
        }
        taskEXIT_CRITICAL();

        /*
         * RTT 监控：每收到一批数据就打印 rx_index 和前 64 字节预览。
         * 预览截断必须先保存再还原原字节——直接写 '\0' 不还原会把
         * 64 字节之后的真实响应正文破坏掉，expect 匹配随之失效。
         */
        if (s_net_handle.rx_index > 0U)
        {
            uint8_t saved_byte;
            uint16_t preview_len = s_net_handle.rx_index;

            if (preview_len > 64U)
            {
                preview_len = 64U;
            }

            saved_byte = s_net_handle.rx_buffer[preview_len];
            s_net_handle.rx_buffer[preview_len] = '\0';
            SEGGER_RTT_printf(0, "[rx] idx=%u: %s\r\n",
                              (unsigned)s_net_handle.rx_index,
                              (const char *)s_net_handle.rx_buffer);
            s_net_handle.rx_buffer[preview_len] = saved_byte;
        }

        if ((ret != RET_EMPTY) && (ret != RET_OK))
        {
            DBG_Error("dev_net receive failed ret=%ld", (long)ret);
        }

        if (overflowed != 0U)
        {
            if ((s_net_cmd_waiting != 0U) && (s_net_events != NULL))
            {
                (void)xEventGroupSetBits(s_net_events,
                                         NET_EVENT_RX_OVERFLOW);
            }
            else
            {
                DBG_Warning("dev_net async receive overflow");
            }
        }

        dev_net_process_rx();
    }
}

/* ======================== 公共接口 ======================== */

/**
 * @brief  初始化 ML307R Device 层并执行 AT 在线检测
 *
 * @note   顺序：UART -> GPIO -> 硬件稳定延时 -> AT事务结果事件组
 *         -> 事务互斥锁 -> 接收任务 -> BSP通知注册 -> AT测试。
 *         每个 RTOS 对象都要防止重复创建。
 */
ret_t dev_net_init(void)
{
    ret_t ret;

    ret = bsp_uart_init(BSP_UART_NET);
    if (ret != RET_OK)
    {
        DBG_Error("dev_net UART init failed ret=%ld", (long)ret);
        return ret;
    }

    ret = bsp_gpio_write(BSP_GPIO_WIFI_CTRL, BSP_GPIO_OFF);
    if (ret != RET_OK)
    {
        DBG_Error("dev_net control GPIO failed ret=%ld", (long)ret);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(500U));

    if (s_net_events == NULL)
    {
        s_net_events = xEventGroupCreate();
        if (s_net_events == NULL)
        {
            return RET_NO_MEMORY;
        }
    }

    if (s_net_cmd_mutex == NULL)
    {
        s_net_cmd_mutex = xSemaphoreCreateMutex();
        if (s_net_cmd_mutex == NULL)
        {
            return RET_NO_MEMORY;
        }
    }

    if (s_net_rx_task_handle == NULL)
    {
        if (xTaskCreate(dev_net_notify_task,
                        "net_rx",
                        NET_RX_TASK_STACK_SIZE,
                        NULL,
                        NET_RX_TASK_PRIORITY,
                        &s_net_rx_task_handle) != pdPASS)
        {
            return RET_TASK_CREATE_ERROR;
        }
    }

    ret = bsp_uart_register_notify_task(BSP_UART_NET,
                                         s_net_rx_task_handle);
    if (ret != RET_OK)
    {
        return ret;
    }

    DBG_log("dev_net sending AT probe...");

    ret = dev_net_send_cmd(NET_AT_TEST,
                           NET_EXPECT_OK,
                           NET_CMD_DEFAULT_TIMEOUT_MS);
    if (ret != RET_OK)
    {
        DBG_Error("dev_net AT probe failed ret=%ld", (long)ret);
        return ret;
    }

    DBG_log("dev_net init ok");
    return RET_OK;
}

/**
 * @brief  配置 MQTT 多主题与过滤器
 * @note   遍历 s_net_connect_steps；任一步失败即返回。
 */
ret_t dev_net_mqtt_connect(void)
{
    uint32_t index;
    ret_t ret;

    for (index = 0U;
         index < (sizeof(s_net_connect_steps)
                  / sizeof(s_net_connect_steps[0]));
         index++)
    {
        ret = dev_net_run_step(&s_net_connect_steps[index]);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return RET_OK;
}

/**
 * @brief  复位模组并等待 AT+REST 应答
 * @note   MQTT_CONNECT/MQTT_DISCONNECT 由 RX 任务通知上层状态机。
 */
ret_t dev_net_reset(void)
{
    ret_t ret;

    if ((s_net_events == NULL) || (s_net_cmd_mutex == NULL))
    {
        return RET_NOT_INIT;
    }

    /* 进入新一代连接流程；连接 URC 在 AT+REST 应答前只记 pending，
     * 不直接交给上层，避免旧会话或残留 URC 完成本次 reset。 */
    s_net_accept_connect_event = 0U;
    s_net_connect_event_pending = 0U;
    s_net_link_online = 0U;

    ret = dev_net_run_step(&s_net_reset_step);
    if (ret != RET_OK)
    {
        DBG_Error("dev_net reset ACK failed ret=%ld", (long)ret);
        s_net_accept_connect_event = 1U;
        s_net_connect_event_pending = 0U;
        return ret;
    }

    s_net_accept_connect_event = 1U;
    if (s_net_connect_event_pending != 0U)
    {
        s_net_connect_event_pending = 0U;
        if (s_net_link_cb != NULL)
        {
            s_net_link_cb(DEV_NET_LINK_CONNECTED);
        }
    }

    /* 模组重启后旧订阅全部失效，复位成功才把订阅编号清零，
       重新订阅从 0 开始；mqtt_connect 不做无条件清零。 */
    s_net_subscribe_index = 0U;

    return RET_OK;
}

/**
 * @brief  查询并解析模块标准时间
 */
ret_t dev_net_get_time(char *time_text, uint16_t buffer_size)
{
    char response[64U];
    const char *time_start;
    const char *time_end;
    uint16_t index;
    ret_t ret;

    if (time_text == NULL)
    {
        return RET_NULL_PTR;
    }

    if (buffer_size < DEV_NET_TIME_TEXT_SIZE)
    {
        return RET_LENGTH_ERROR;
    }

    time_text[0] = '\0';
    ret = dev_net_send_cmd_response(NET_AT_TIME_STANDARD,
                                    NET_EXPECT_OK,
                                    NET_CMD_DEFAULT_TIMEOUT_MS,
                                    response,
                                    (uint16_t)sizeof(response));
    if (ret != RET_OK)
    {
        return ret;
    }

    time_start = strstr(response, NET_EXPECT_TIME);
    if (time_start == NULL)
    {
        return RET_NO_RESPONSE;
    }
    time_start += strlen(NET_EXPECT_TIME);
    time_end = strchr(time_start, '"');
    if ((time_end == NULL) ||
        ((uint32_t)(time_end - time_start) !=
         (DEV_NET_TIME_TEXT_SIZE - 1U)))
    {
        return RET_LENGTH_ERROR;
    }

    /* 固定分隔符之外必须全是数字，拒绝把残缺或异常时间交给页面。 */
    for (index = 0U; index < (DEV_NET_TIME_TEXT_SIZE - 1U); index++)
    {
        if ((index == 4U) || (index == 7U))
        {
            if (time_start[index] != '-')
            {
                return RET_DATA_FORMAT_ERROR;
            }
        }
        else if (index == 10U)
        {
            if (time_start[index] != ' ')
            {
                return RET_DATA_FORMAT_ERROR;
            }
        }
        else if ((index == 13U) || (index == 16U))
        {
            if (time_start[index] != ':')
            {
                return RET_DATA_FORMAT_ERROR;
            }
        }
        else if ((time_start[index] < '0') || (time_start[index] > '9'))
        {
            return RET_DATA_FORMAT_ERROR;
        }
    }

    (void)memcpy(time_text, time_start, DEV_NET_TIME_TEXT_SIZE - 1U);
    time_text[DEV_NET_TIME_TEXT_SIZE - 1U] = '\0';
    return RET_OK;
}

/**
 * @brief  订阅一个 MQTT 主题
 * @note   检查 topic 与订阅数量；使用 snprintf() 生成命令；
 *         检查截断；同步等待 OK；成功后才递增订阅编号。
 */
ret_t dev_net_mqtt_subscribe(const char *topic)
{
    char cmd_buffer[NET_SUB_CMD_BUF_SIZE];
    net_cmd_step_t step;
    int cmd_length;
    ret_t ret;

    if (topic == NULL)
    {
        return RET_NULL_PTR;
    }

    if (s_net_subscribe_index >= NET_MAX_SUBSCRIPTIONS)
    {
        return RET_FULL;
    }

    cmd_length = snprintf(cmd_buffer,
                          sizeof(cmd_buffer),
                          "AT+MQSUBM=%u,1,0,4,\"%s\"\r\n",
                          (unsigned int)s_net_subscribe_index,
                          topic);
    if ((cmd_length < 0)
        || (cmd_length >= (int)sizeof(cmd_buffer)))
    {
        return RET_OVERFLOW;
    }

    step.cmd = cmd_buffer;
    step.expect = NET_EXPECT_OK;
    step.timeout_ms = 3000U;
    ret = dev_net_run_step(&step);
    if (ret != RET_OK)
    {
        return ret;
    }

    s_net_subscribe_index++;

    return RET_OK;
}

/**
 * @brief  发布 MQTT 消息
 * @note   检查 topic/payload；使用 snprintf() 生成命令并检查
 *         截断；当前版本只发送，不等待发布 ACK。
 *         命令体是 "MQPUB,1,<topic>,<payload>"，字段 1 是固定的
 *         "是否等待应答"，与 MQTT QoS 无关——模组固件不接受 QoS 入参，
 *         因此本接口刻意不设 qos 形参。
 */
ret_t dev_net_mqtt_publish(const char *topic,
                           const char *payload)
{
    char cmd_buffer[NET_PUB_CMD_BUF_SIZE];
    int cmd_length;
    ret_t ret;

    if ((topic == NULL) || (payload == NULL))
    {
        return RET_NULL_PTR;
    }

    cmd_length = snprintf(cmd_buffer,
                          sizeof(cmd_buffer),
                          "MQPUB,1,%s,%s",
                          topic,
                          payload);
    if ((cmd_length < 0)
        || (cmd_length >= (int)sizeof(cmd_buffer)))
    {
        return RET_OVERFLOW;
    }

    /*
     * 发布与 AT 命令共用同一把事务锁：防止发布正文与某条 AT 命令的
     * 字节在 UART 上交叉串扰。只发送不等待 ACK（当前协议约定）。
     * 命令格式沿用参考固件约定（无 AT+ 前缀），待模组固件确认后校准。
     */
    if (xSemaphoreTake(s_net_cmd_mutex,
                       pdMS_TO_TICKS(NET_CMD_DEFAULT_TIMEOUT_MS)) != pdTRUE)
    {
        return RET_TIMEOUT;
    }

    ret = dev_net_send((uint8_t *)cmd_buffer,
                       (uint16_t)cmd_length,
                       NET_CMD_DEFAULT_TIMEOUT_MS);

    (void)xSemaphoreGive(s_net_cmd_mutex);

    return ret;
}

/**
 * @brief  注册或取消注册异步下行回调
 * @note   保存 cb。回调在 net_rx 任务上下文执行，App 必须快速返回。
 */
void dev_net_register_recv_cb(dev_net_recv_cb_t cb)
{
    s_net_recv_cb = cb;
}

void dev_net_register_link_cb(dev_net_link_cb_t cb)
{
    /* NULL 是明确的注销操作；拒绝它会让 app_net 失败清理时留下悬空回调。 */
    s_net_link_cb = cb;
}

uint8_t dev_net_link_is_online(void)
{
    return s_net_link_online;
}

void dev_net_mqtt_subscribe_reset(void)
{
    s_net_subscribe_index = 0U;
}
