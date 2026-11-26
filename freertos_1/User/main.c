/**
 * @file    main.c
 * @brief   工程入口（FreeRTOS 启动�?
 *
 * @note    启动流程：设置向量表 -> 配置中断分组 -> 初始�?GPIO ->
 *          创建应用任务 -> 启动 FreeRTOS 调度器�?
 *          故障 Hook（栈溢出/内存分配失败）仅记录故障码后停机�?
 */
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gd32f30x.h"
#include "bsp_fwdgt/bsp_fwdgt.h"
#include "bsp_gpio/bsp_gpio.h"
#include "bsp_iic/bsp_iic.h"
#include "app_boot_ab/app_boot_ab.h"
#include "app_lcd_plan_page/app_lcd_plan_page.h"
#include "app_modbus/app_modbus.h"
#include "app_net/app_net.h"
#include "app_ota/app_ota.h"
#include "app_ui_snapshot/app_ui_snapshot.h"
#include "SEGGER_RTT.h"

/* 跨复位故障现场（0x2000FF00 位于 .bss/栈之上，IWDG 复位不清除）：
 * [0]=magic [1]=code(1=栈溢出 2=堆失败 3..6=硬件异常 7=assert) [2]=detail
 * [3]=tick；复位后用调试器读取即可定位。 */
#define FAULT_TRACE ((volatile uint32_t *)0x2000FF00UL)
#define HEALTH_TRACE ((volatile uint32_t *)0x2000FF20UL)

static void fault_trace_save(uint32_t code, uint32_t detail)
{
    FAULT_TRACE[0] = 0xF0F0F0F0UL;
    FAULT_TRACE[1] = code;
    FAULT_TRACE[2] = detail;
    FAULT_TRACE[3] = (uint32_t)xTaskGetTickCount();
}

/** 故障码：无故�?*/
#define FAULT_NONE     0U
/** 故障码：任务栈溢�?*/
#define FAULT_STACK    1U
/** 故障码：堆内存分配失�?*/
#define FAULT_MALLOC   2U

/** 健康任务参数：喂狗周期与试运行确认延�?*/
#define SYSTEM_HEALTH_PERIOD_MS        1000U
#define SYSTEM_HEALTH_CONFIRM_DELAY_MS 10000U
#define SYSTEM_HEALTH_STACK_DEPTH      512U
#define SYSTEM_HEALTH_PRIORITY         5U

/** LED 心跳周期：每 2s 一个短脉冲（原 1Hz 方波视觉抖动过强�?*/
#define LED_HEARTBEAT_PERIOD_MS        2000U
/** 心跳脉冲宽度：亮 100ms 后熄�?*/
#define LED_HEARTBEAT_PULSE_MS         100U
/** 网络初始化含阻塞�?FreeRTOS 等待，必须在调度器启动后执行�?*/
#define APP_NET_INIT_TASK_STACK_DEPTH  1024U
#define APP_NET_INIT_TASK_PRIORITY     4U

/** 当前故障码（调试�?Watch 观察用） */
static volatile uint32_t fault_code = FAULT_NONE;

/* ------------------------------------------------------------------
 * 系统组合�?
 *
 * 各业务模块只依赖抽象接口；具体的 Modbus、GPIO �?LCD 连接关系�?
 * main.c 统一装配。这�?app_net 不再直接引用其他 App/BSP 模块，LCD
 * 也不需要反向包�?app_net.h�?
 * ------------------------------------------------------------------ */
static ret_t main_app_net_read_registers(void *context,
                                         uint8_t slave_address,
                                         app_net_register_area_t area,
                                         uint16_t start_address,
                                         uint16_t quantity,
                                         uint16_t *register_values)
{
    uint8_t modbus_function_code;

    (void)context;

    /*
     * 这是组合根中的协议适配器：app_net 只表达“保持寄存器/输入寄存器”，
     * Modbus 功能码在这里集中转换，避免业务层散落 0x03/0x04 等协议常量�?
     */
    if (area == APP_NET_REGISTER_AREA_HOLDING)
    {
        modbus_function_code = 0x03U;
    }
    else if (area == APP_NET_REGISTER_AREA_INPUT)
    {
        modbus_function_code = 0x04U;
    }
    else
    {
        return RET_INVALID_PARAM;
    }

    return app_modbus_read_registers(slave_address,
                                     modbus_function_code,
                                     start_address,
                                     quantity,
                                     register_values);
}

static ret_t main_app_net_read_bits(void *context,
                                    uint8_t slave_address,
                                    app_net_bit_area_t area,
                                    uint16_t start_address,
                                    uint16_t quantity,
                                    uint8_t *bit_values)
{
    uint8_t modbus_function_code;

    (void)context;

    /* 线圈和离散输入的功能码同样只在适配层转换�?*/
    if (area == APP_NET_BIT_AREA_COIL)
    {
        modbus_function_code = 0x01U;
    }
    else if (area == APP_NET_BIT_AREA_DISCRETE_INPUT)
    {
        modbus_function_code = 0x02U;
    }
    else
    {
        return RET_INVALID_PARAM;
    }

    return app_modbus_read_bits(slave_address,
                                modbus_function_code,
                                start_address,
                                quantity,
                                bit_values);
}

static ret_t main_app_net_write_register(void *context,
                                         uint8_t slave_address,
                                         uint16_t register_address,
                                         uint16_t register_value)
{
    (void)context;
    return app_modbus_write_register(slave_address,
                                     register_address,
                                     register_value);
}

static ret_t main_app_net_write_coil(void *context,
                                     uint8_t slave_address,
                                     uint16_t coil_address,
                                     uint8_t is_on)
{
    (void)context;
    return app_modbus_write_coil(slave_address, coil_address, is_on);
}

static ret_t main_app_net_write_led2(void *context, uint8_t is_on)
{
    (void)context;
    return bsp_gpio_write(BSP_GPIO_LED2,
                          (is_on != 0U) ? BSP_GPIO_ON : BSP_GPIO_OFF);
}

static void main_app_net_online_changed(void *context, uint8_t is_online)
{
    (void)context;

    /* 网络模块不直接依赖 LCD；状态写入界面快照，由 Display Task 拉取。 */
    app_ui_snapshot_set_network_online(is_online);
}

static void main_app_net_snapshot_updated(
    void *context,
    const app_net_snapshot_t *snapshot)
{
    (void)context;

    /* 回调只做快照搬运，不在这里组装任何显示消息。 */
    app_ui_snapshot_update_cloud(snapshot);
}

static ret_t main_app_lcd_led_toggle(void *context)
{
    (void)context;
    return app_net_led_toggle();
}

static const app_net_dependencies_t s_app_net_dependencies =
{
    /* 依赖表为静态对象，生命周期覆盖整个系统运行期�?*/
    NULL,
    main_app_net_read_registers,
    main_app_net_read_bits,
    main_app_net_write_register,
    main_app_net_write_coil,
    main_app_net_write_led2,
    main_app_net_online_changed,
    main_app_net_snapshot_updated
};

/**
 * @brief  LED 心跳任务（每 2s 一�?100ms 短脉冲，证明调度器活着�?
 * @note   不再�?1Hz 方波�?0% 占空比的长亮长灭视觉上像"抖动/闪屏"�?
 *         改成标准心跳脉冲后既安静又能证明任务在跑�?
 *         脉冲时刻仍打一�?RTT�? �?/ 2s），保留可验证证据�?
 *         监控方式：GPIOA OCTL(0x4001080C) bit8（GD32 �?OSTATE 寄存器）�?
 */
static void led_heartbeat_task(void *parameter)
{
    TickType_t last_wake_tick = xTaskGetTickCount();

    (void)parameter;

    for (;;)
    {
        (void)bsp_gpio_write(BSP_GPIO_LED1, BSP_GPIO_ON);
        vTaskDelayUntil(&last_wake_tick,
                        pdMS_TO_TICKS(LED_HEARTBEAT_PULSE_MS));

        (void)bsp_gpio_write(BSP_GPIO_LED1, BSP_GPIO_OFF);
        vTaskDelayUntil(&last_wake_tick,
                        pdMS_TO_TICKS(LED_HEARTBEAT_PERIOD_MS -
                                      LED_HEARTBEAT_PULSE_MS));

        SEGGER_RTT_printf(0, "[hb] tick=%lu\r\n",
                          (unsigned long)xTaskGetTickCount());
    }
}

/**
 * @brief  在调度器启动后完成网络模块初始化
 * @note   app_net_init() 会等�?UART 接收任务处理 AT 响应，不能在 main()
 *         中同步调用。初始化失败时保持其他任务运行，网络保持离线�?
 */
static void app_net_init_task(void *parameter)
{
    ret_t init_result;

    (void)parameter;

    init_result = app_net_init(&s_app_net_dependencies);
    if ((init_result != RET_OK) && (init_result != RET_ALREADY_INIT))
    {
        DBG_Error("app_net_init failed ret=%ld; system continues offline\r\n",
                  (long)init_result);
    }
    else
    {
        DBG_log("app_net_init task finished ret=%ld\r\n",
                (long)init_result);
    }

    vTaskDelete(NULL);
}

/**
 * @brief  OTA 复位前钩子：先把"即将重启"写入界面快照再复位
 * @note   运行在 ota_worker 任务上下文；写入快照后短暂延时，
 *         等 Display Task 拉到 REBOOTING 状态并画完。
 */
static void main_ota_reboot_hook(void)
{
    app_ui_snapshot_mark_ota_rebooting();
    vTaskDelay(pdMS_TO_TICKS(600U));
}

/**
 * @brief  系统健康任务：喂�?+ 试运行确�?
 * @note   独立看门狗约 10s 超时�?0kHz/128，重�?3124）�?
 *         - 上电后每秒喂狗一次；新固件若跑飞不喂�?0s 后复位回
 *           BootLoader，由 trial_count 触发回滚�?
 *         - 稳定运行 10s 后调�?app_boot_service_confirm()�?
 *           BootLoader �?TRIAL 态把 pending 槽转正，同时把当�?
 *           已确认版本写�?fw_nvs（失败则 1s 后重试）�?
 *         本任务优先级最高，保证任何业务阻塞时喂狗不被饿死�?
 */
static void system_health_task(void *parameter)
{
    TickType_t start_tick;
    ret_t confirm_result;
    uint8_t confirmed = 0U;

    (void)parameter;
    start_tick = xTaskGetTickCount();

    for (;;)
    {
        if ((confirmed == 0U) &&
            ((xTaskGetTickCount() - start_tick) >=
             pdMS_TO_TICKS(SYSTEM_HEALTH_CONFIRM_DELAY_MS)))
        {
            confirm_result = app_boot_service_confirm();
            if (confirm_result == RET_OK)
            {
                confirmed = 1U;
                DBG_log("boot confirm ok; fw_nvs updated\r\n");
                app_ui_snapshot_refresh_boot();
            }
            else
            {
                DBG_Error("boot confirm failed ret=%ld\r\n",
                          (long)confirm_result);
                vTaskDelay(pdMS_TO_TICKS(SYSTEM_HEALTH_PERIOD_MS));
                continue;
            }
        }

        app_ui_snapshot_poll_ota();
        HEALTH_TRACE[0] = (uint32_t)xTaskGetTickCount();
        (void)bsp_fwdgt_feed();
        vTaskDelay(pdMS_TO_TICKS(SYSTEM_HEALTH_PERIOD_MS));
    }
}

/**
 * @brief   启动失败停机（关中断后死循环�?
 */
static void startup_halt(void)
{
    __disable_irq();

    while (1)
    {
    }
}

/************************************************************
 * @brief   主函数（系统初始�?+ 任务创建 + 启动调度器）
 * @retval  不返回（进入调度器或停机�?
 ************************************************************/
int main(void)
{
    ret_t init_result;

    /* 保留上一次复位前的最后喂狗时刻（跨 IWDG 复位保留） */
    HEALTH_TRACE[1] = HEALTH_TRACE[0];

    /*
     * 向量表基址必须与当前镜像槽�?scatter 文件 ER_IROM1 一致�?     * freertos_1 �?Boot A/B �?B 槽镜像，工程配置�?APP_VECTOR_ADDR
     * 传入�?0x08008000UL；这里的缺省值也保持�?B 槽一致，防止脱离
     * Keil 工程配置编译时意外回到主槽地址�?     */
#ifndef APP_VECTOR_ADDR
#define APP_VECTOR_ADDR 0x08008000UL
#endif

    SCB->VTOR = (uint32_t)APP_VECTOR_ADDR;
    __enable_irq();
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /*
     * 独立看门狗先于业务初始化：约 10s 超时�?
     * 试运行的新固件必须在窗口内启动健康任务并持续喂狗�?
     * 否则复位�?BootLoader，由 trial_count 触发回滚�?
     */
    init_result = bsp_fwdgt_init();
    if (init_result != RET_OK)
    {
        DBG_Error("bsp_fwdgt_init failed ret=%ld", (long)init_result);
        startup_halt();
    }

    /* 启动服务：消�?BootLoader 留下的回滚标志并落成持久记录�?*/
    init_result = app_boot_service_init();
    if (init_result != RET_OK)
    {
        DBG_Error("app_boot_service_init failed ret=%ld", (long)init_result);
        startup_halt();
    }

    init_result = bsp_gpio_init();
    if (init_result != RET_OK)
    {
        DBG_Error("bsp_gpio_init failed ret=%ld", (long)init_result);
        startup_halt();
    }

    init_result = bsp_iic_init();
    if ((init_result != RET_OK) && (init_result != RET_ALREADY_INIT))
    {
        DBG_Error("bsp_iic_init failed ret=%ld", (long)init_result);
        startup_halt();
    }

    init_result = app_lcd_plan_page_init();
    if (init_result != RET_OK)
    {
        DBG_Error("app_lcd_plan_page_init failed ret=%ld",
                  (long)init_result);
        startup_halt();
    }

    init_result = app_lcd_plan_page_set_led_toggle_handler(
        main_app_lcd_led_toggle,
        NULL);
    if (init_result != RET_OK)
    {
        DBG_Error("LED2 display handler register failed ret=%ld",
                  (long)init_result);
        startup_halt();
    }

    /* LCD 队列就绪后立刻同步一�?A/B 与版本状态（调度器启动前入队）�?*/
    app_ui_snapshot_refresh_boot();

    /* OTA 复位前钩子：把 REBOOTING 状态画到 LCD 再复位。 */
    app_ota_set_reboot_hook(main_ota_reboot_hook);

    init_result = app_modbus_init();
    if ((init_result != RET_OK) && (init_result != RET_ALREADY_INIT))
    {
        DBG_Error("app_modbus_init failed ret=%ld", (long)init_result);
        startup_halt();
    }

    /* 网络初始化会阻塞等待 AT 响应，放�?scheduler task context�?*/
    if (xTaskCreate(app_net_init_task,
                    "net_init",
                    APP_NET_INIT_TASK_STACK_DEPTH,
                    NULL,
                    APP_NET_INIT_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        DBG_Error("app_net_init task create failed; system continues offline\r\n");
    }

    /* 健康任务优先级最高：喂狗与试运行确认不能被业务阻塞饿死�?*/
    if (xTaskCreate(system_health_task,
                    "sys_health",
                    SYSTEM_HEALTH_STACK_DEPTH,
                    NULL,
                    SYSTEM_HEALTH_PRIORITY,
                    NULL) != pdPASS)
    {
        DBG_Error("system_health task create failed");
        startup_halt();
    }

    if (xTaskCreate(led_heartbeat_task,
                    "led_hb",
                    128U,
                    NULL,
                    3U,
                    NULL) != pdPASS)
    {
        DBG_Error("led_heartbeat task create failed");
        startup_halt();
    }

    SEGGER_RTT_printf(0, "[sys] FreeRTOS scheduler starting\r\n");
    vTaskStartScheduler();
    startup_halt();

    return 0;
}

/*
 * 故障 Hook 不执行日志调用：栈可能已经损坏，任何额外调用都可能扩大故障�?
 * 只记录最小故障标记（FAULT_STACK=1 / FAULT_MALLOC=2），随后关中断停机�?
 * 故障标记可通过调试器的 Watch 窗口查看。
 */
/************************************************************
 * @brief   任务栈溢�?Hook（FreeRTOS 调用�?
 * @param   task       溢出任务句柄
 * @param   task_name  溢出任务�?
 * @note    栈可能已损坏，仅记录故障码后关中断停机，不做日志调用
 ************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char *task_name)
{
    (void)task;

    fault_trace_save(1U, (uint32_t)task_name);
    fault_code = FAULT_STACK;

    taskDISABLE_INTERRUPTS();

    while (1)
    {
    }
}

/************************************************************
 * @brief   堆内存分配失�?Hook（FreeRTOS 调用�?
 * @note    记录故障码后关中断停�?
 ************************************************************/
void vApplicationMallocFailedHook(void)
{
    fault_trace_save(2U, 0U);
    fault_code = FAULT_MALLOC;

    taskDISABLE_INTERRUPTS();

    while (1)
    {
    }
}
