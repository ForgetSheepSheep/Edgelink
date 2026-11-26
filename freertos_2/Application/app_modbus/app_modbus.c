/**
 * @file    app_modbus.c
 * @brief   Modbus RTU 主站服务所有者
 *
 * @note    本层拥有主站互斥和协议帧细节，对上只提供语义化的读写接口；
 *          上层业务不需要知道 Modbus 帧布局或串口端口对象。
 *
 * @details 调用链为：app_net 业务回调 -> 本模块生成请求帧 ->
 *          modbus_master 校验/收发 -> port_serial 访问 RS485。
 *          app_modbus_execute_frame() 是唯一进入主站事务的内部入口，
 *          因此同一时刻不会有两笔请求交叉使用串口收发缓冲区。
 */
#include "app_modbus/app_modbus.h"
#include "config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "SEGGER_RTT.h"
#include "../../Middleware/modbus_rtu/modbus_crc.h"
#include "../../Middleware/modbus_rtu/modbus_master/modbus_master.h"
#include "../../Middleware/modbus_rtu/prot/port_serial.h"

#define APP_MODBUS_INIT_TASK_STACK_DEPTH       (256U)
#define APP_MODBUS_INIT_TASK_PRIORITY          (3U)
#define APP_MODBUS_INIT_RETRY_DELAY_MS         (1000U)
#define APP_MODBUS_TRANSACTION_TIMEOUT_MS      (2000U)

/* 485 原码跟踪只属于 Modbus 服务，不再由网络业务层承担。 */
#define APP_MODBUS_TRACE_ENABLE                (1U)
#define APP_MODBUS_TRACE_BYTES_PER_LINE        (16U)
#define APP_MODBUS_TRACE_TX_MAX                (8U)

#define MODBUS_FUNCTION_READ_COILS             (0x01U)
#define MODBUS_FUNCTION_READ_DISCRETE_INPUTS   (0x02U)
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS (0x03U)
#define MODBUS_FUNCTION_READ_INPUT_REGISTERS   (0x04U)
#define MODBUS_FUNCTION_WRITE_SINGLE_COIL      (0x05U)
#define MODBUS_FUNCTION_WRITE_SINGLE_REGISTER  (0x06U)
#define MODBUS_REQUEST_LENGTH                  (6U)
#define MODBUS_CRC_LENGTH                      (2U)
#define MODBUS_RESPONSE_MIN_LENGTH             (5U)
#define MODBUS_READ_REGISTER_MAX_QUANTITY      (125U)
#define MODBUS_READ_BIT_MAX_QUANTITY           (2000U)

static SemaphoreHandle_t s_app_modbus_mutex = NULL;
static TaskHandle_t s_app_modbus_init_task_handle = NULL;
static volatile uint8_t s_app_modbus_is_ready = 0U;

static void app_modbus_startup_task(void *task_parameter);
static modbus_status_t app_modbus_execute_frame(modbus_frame_t *frame);
static ret_t app_modbus_status_to_ret(modbus_status_t status);
static void app_modbus_write_u16(uint8_t *data, uint16_t value);
#if APP_MODBUS_TRACE_ENABLE
static void app_modbus_trace_bytes(const char *tag,
                                   const uint8_t *data,
                                   uint16_t length);
#endif

static void app_modbus_startup_task(void *task_parameter)
{
    modbus_status_t init_status;

    (void)task_parameter;

    /* 先注册端口，再初始化主站；注册失败时释放本模块已创建的锁，
     * 允许上层稍后重新调用 app_modbus_init() 重试。 */
    init_status = modbus_master_port_register(&g_modbus_port_serial_ops);
    if (init_status != MODBUS_STATUS_OK)
    {
        DBG_Error("MODBUS port register failed, status=%u\r\n",
                  (unsigned int)init_status);
        s_app_modbus_init_task_handle = NULL;
        vSemaphoreDelete(s_app_modbus_mutex);
        s_app_modbus_mutex = NULL;
        vTaskDelete(NULL);
        return;
    }

    /* 串口初始化失败不退出系统，持续等待硬件/驱动恢复。 */
    do
    {
        init_status = modbus_master_init();
        if (init_status != MODBUS_STATUS_OK)
        {
            DBG_Error("MODBUS port init failed, status=%u; retry\r\n",
                      (unsigned int)init_status);
            vTaskDelay(pdMS_TO_TICKS(APP_MODBUS_INIT_RETRY_DELAY_MS));
        }
    } while (init_status != MODBUS_STATUS_OK);

    s_app_modbus_is_ready = 1U;
    DBG_log("MODBUS service ready\r\n");

    s_app_modbus_init_task_handle = NULL;
    vTaskDelete(NULL);
}

ret_t app_modbus_init(void)
{
    if ((s_app_modbus_mutex != NULL) ||
        (s_app_modbus_init_task_handle != NULL) ||
        (s_app_modbus_is_ready != 0U))
    {
        return RET_ALREADY_INIT;
    }

    s_app_modbus_mutex = xSemaphoreCreateMutex();
    if (s_app_modbus_mutex == NULL)
    {
        return RET_NO_MEMORY;
    }

    if (xTaskCreate(app_modbus_startup_task,
                    "modbus_init",
                    APP_MODBUS_INIT_TASK_STACK_DEPTH,
                    NULL,
                    APP_MODBUS_INIT_TASK_PRIORITY,
                    &s_app_modbus_init_task_handle) != pdPASS)
    {
        vSemaphoreDelete(s_app_modbus_mutex);
        s_app_modbus_mutex = NULL;
        s_app_modbus_init_task_handle = NULL;
        return RET_TASK_CREATE_ERROR;
    }

    return RET_OK;
}

uint8_t app_modbus_is_ready(void)
{
    return s_app_modbus_is_ready;
}

static void app_modbus_write_u16(uint8_t *data, uint16_t value)
{
    data[0U] = (uint8_t)(value >> 8U);
    data[1U] = (uint8_t)value;
}

static ret_t app_modbus_status_to_ret(modbus_status_t status)
{
    switch (status)
    {
        case MODBUS_STATUS_OK:
            return RET_OK;
        case MODBUS_STATUS_INVALID_PARAM:
            return RET_INVALID_PARAM;
        case MODBUS_STATUS_BUSY:
            return RET_BUSY;
        case MODBUS_STATUS_SEND_FAILED:
            return RET_SEND_ERROR;
        case MODBUS_STATUS_TIMEOUT:
            return RET_TIMEOUT;
        case MODBUS_STATUS_CRC_ERROR:
            return RET_CRC_ERROR;
        case MODBUS_STATUS_ADDRESS_ERROR:
            return RET_ADDR_ERROR;
        case MODBUS_STATUS_LENGTH_ERROR:
            return RET_DATA_LENGTH_ERROR;
        case MODBUS_STATUS_PORT_ERROR:
            return RET_RS485_ERROR;
        case MODBUS_STATUS_FUNCTION_ERROR:
        case MODBUS_STATUS_EXCEPTION:
        default:
            return RET_MODBUS_ERROR;
    }
}

#if APP_MODBUS_TRACE_ENABLE
static void app_modbus_trace_bytes(const char *tag,
                                   const uint8_t *data,
                                   uint16_t length)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    char line[(APP_MODBUS_TRACE_BYTES_PER_LINE * 3U) + 1U];
    uint16_t data_index = 0U;
    uint16_t line_length = 0U;
    uint8_t line_byte_count = 0U;

    if ((tag == NULL) || (data == NULL) || (length == 0U))
    {
        return;
    }

    (void)SEGGER_RTT_WriteString(0, tag);

    while (data_index < length)
    {
        line[line_length++] = hex_digits[(data[data_index] >> 4U) & 0x0FU];
        line[line_length++] = hex_digits[data[data_index] & 0x0FU];
        line[line_length++] = ' ';
        data_index++;
        line_byte_count++;

        if ((line_byte_count >= APP_MODBUS_TRACE_BYTES_PER_LINE) ||
            (data_index == length))
        {
            line[line_length] = '\0';
            (void)SEGGER_RTT_WriteString(0, line);
            line_length = 0U;
            line_byte_count = 0U;

            if (data_index < length)
            {
                (void)SEGGER_RTT_WriteString(0, "\r\n         ");
            }
        }
    }

    (void)SEGGER_RTT_WriteString(0, "\r\n");
}
#endif

static modbus_status_t app_modbus_execute_frame(modbus_frame_t *frame)
{
    modbus_status_t status;
#if APP_MODBUS_TRACE_ENABLE
    uint8_t request_frame[APP_MODBUS_TRACE_TX_MAX];
    uint16_t request_length;
    uint16_t crc;
#endif

    /* frame->length 表示不含 CRC 的请求长度；CRC 由主站中间件补上。
     * 函数返回成功时，同一个 frame 已被覆盖为完整响应帧。 */
    if ((frame == NULL) || (frame->length < 2U))
    {
        return MODBUS_STATUS_INVALID_PARAM;
    }

#if APP_MODBUS_TRACE_ENABLE
    /* 在副本上补 CRC，不能修改上层传入的请求长度。 */
    request_length = frame->length;
    if ((request_length + MODBUS_CRC_LENGTH) <= sizeof(request_frame))
    {
        (void)memcpy(request_frame, frame->buffer, request_length);
        crc = modbus_crc16(request_frame, request_length);
        request_frame[request_length] = (uint8_t)(crc & 0x00FFU);
        request_frame[request_length + 1U] = (uint8_t)(crc >> 8U);
        app_modbus_trace_bytes("[485] TX ",
                               request_frame,
                               (uint16_t)(request_length + MODBUS_CRC_LENGTH));
    }
    else
    {
        app_modbus_trace_bytes("[485] TX ", frame->buffer, frame->length);
    }
#endif

    if ((s_app_modbus_is_ready == 0U) ||
        (s_app_modbus_mutex == NULL))
    {
        return MODBUS_STATUS_BUSY;
    }

    if (xSemaphoreTake(s_app_modbus_mutex,
                       pdMS_TO_TICKS(APP_MODBUS_TRANSACTION_TIMEOUT_MS)) !=
        pdTRUE)
    {
        return MODBUS_STATUS_BUSY;
    }

    status = modbus_master_execute(frame);
    (void)xSemaphoreGive(s_app_modbus_mutex);

#if APP_MODBUS_TRACE_ENABLE
    if (status == MODBUS_STATUS_OK)
    {
        app_modbus_trace_bytes("[485] RX ", frame->buffer, frame->length);
    }
    else
    {
        (void)SEGGER_RTT_printf(0,
                                "[485] RX <no-response> status=%u\r\n",
                                (unsigned int)status);
    }
#endif

    return status;
}

ret_t app_modbus_read_registers(uint8_t slave_address,
                                uint8_t function_code,
                                uint16_t start_address,
                                uint16_t quantity,
                                uint16_t *register_values)
{
    modbus_frame_t frame;
    modbus_status_t status;
    uint16_t expected_byte_count;
    uint16_t expected_response_length;
    uint16_t register_index;

    if ((register_values == NULL) ||
        (quantity == 0U) ||
        (quantity > MODBUS_READ_REGISTER_MAX_QUANTITY) ||
        ((function_code != MODBUS_FUNCTION_READ_HOLDING_REGISTERS) &&
         (function_code != MODBUS_FUNCTION_READ_INPUT_REGISTERS)))
    {
        return RET_INVALID_PARAM;
    }

    /* 只在本层组装协议帧，调用者只需要关心地址、数量和输出数组。 */
    frame.buffer[0U] = slave_address;
    frame.buffer[1U] = function_code;
    app_modbus_write_u16(&frame.buffer[2U], start_address);
    app_modbus_write_u16(&frame.buffer[4U], quantity);
    frame.length = MODBUS_REQUEST_LENGTH;

    status = app_modbus_execute_frame(&frame);
    if (status != MODBUS_STATUS_OK)
    {
        return app_modbus_status_to_ret(status);
    }

    expected_byte_count = (uint16_t)(quantity * 2U);
    expected_response_length = (uint16_t)(expected_byte_count +
                                          MODBUS_RESPONSE_MIN_LENGTH);
    if ((frame.length != expected_response_length) ||
        (frame.buffer[1U] != function_code) ||
        (frame.buffer[2U] != (uint8_t)expected_byte_count))
    {
        return RET_DATA_LENGTH_ERROR;
    }

    for (register_index = 0U;
         register_index < quantity;
         register_index++)
    {
        register_values[register_index] =
            (uint16_t)(((uint16_t)frame.buffer[3U +
                                              (register_index * 2U)] << 8U) |
                       frame.buffer[4U + (register_index * 2U)]);
    }

    return RET_OK;
}

ret_t app_modbus_read_bits(uint8_t slave_address,
                           uint8_t function_code,
                           uint16_t start_address,
                           uint16_t quantity,
                           uint8_t *bit_values)
{
    modbus_frame_t frame;
    modbus_status_t status;
    uint16_t expected_byte_count;
    uint16_t expected_response_length;

    if ((bit_values == NULL) ||
        (quantity == 0U) ||
        (quantity > MODBUS_READ_BIT_MAX_QUANTITY) ||
        ((function_code != MODBUS_FUNCTION_READ_COILS) &&
         (function_code != MODBUS_FUNCTION_READ_DISCRETE_INPUTS)))
    {
        return RET_INVALID_PARAM;
    }

    /* Modbus 位响应按字节打包，解析后复制为调用者可直接使用的位图。 */
    frame.buffer[0U] = slave_address;
    frame.buffer[1U] = function_code;
    app_modbus_write_u16(&frame.buffer[2U], start_address);
    app_modbus_write_u16(&frame.buffer[4U], quantity);
    frame.length = MODBUS_REQUEST_LENGTH;

    status = app_modbus_execute_frame(&frame);
    if (status != MODBUS_STATUS_OK)
    {
        return app_modbus_status_to_ret(status);
    }

    expected_byte_count = (uint16_t)((quantity + 7U) / 8U);
    expected_response_length = (uint16_t)(expected_byte_count +
                                          MODBUS_RESPONSE_MIN_LENGTH);
    if ((frame.length != expected_response_length) ||
        (frame.buffer[1U] != function_code) ||
        (frame.buffer[2U] != (uint8_t)expected_byte_count))
    {
        return RET_DATA_LENGTH_ERROR;
    }

    (void)memcpy(bit_values,
                 &frame.buffer[3U],
                 expected_byte_count);
    return RET_OK;
}

ret_t app_modbus_write_register(uint8_t slave_address,
                                uint16_t register_address,
                                uint16_t register_value)
{
    modbus_frame_t frame;
    modbus_status_t status;

    frame.buffer[0U] = slave_address;
    frame.buffer[1U] = MODBUS_FUNCTION_WRITE_SINGLE_REGISTER;
    app_modbus_write_u16(&frame.buffer[2U], register_address);
    app_modbus_write_u16(&frame.buffer[4U], register_value);
    frame.length = MODBUS_REQUEST_LENGTH;

    status = app_modbus_execute_frame(&frame);
    return app_modbus_status_to_ret(status);
}

ret_t app_modbus_write_coil(uint8_t slave_address,
                            uint16_t coil_address,
                            uint8_t is_on)
{
    modbus_frame_t frame;
    modbus_status_t status;

    frame.buffer[0U] = slave_address;
    frame.buffer[1U] = MODBUS_FUNCTION_WRITE_SINGLE_COIL;
    app_modbus_write_u16(&frame.buffer[2U], coil_address);
    app_modbus_write_u16(&frame.buffer[4U],
                         (is_on != 0U) ? 0xFF00U : 0x0000U);
    frame.length = MODBUS_REQUEST_LENGTH;

    status = app_modbus_execute_frame(&frame);
    return app_modbus_status_to_ret(status);
}
