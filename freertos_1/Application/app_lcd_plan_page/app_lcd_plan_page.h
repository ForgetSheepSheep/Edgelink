/**
 * @file    app_lcd_plan_page.h
 * @brief   LCD P0 计划页应用层入口
 *
 * @note    当前模块由一个 Display Task 独占 LCD 设备接口，其他任务通过
 *          语义消息队列提交页面数值；Touch Task 只提交原始触摸按下/抬起
 *          事件，页面命中判定、页面切换与业务回调全部由 Display Task 串行执行。
 *          网络、OTA、启动等系统状态由 app_ui_snapshot 提供，Display Task
 *          按"最新快照 + 变化标记"拉取，不再走消息队列。
 */
#ifndef __APP_LCD_PLAN_PAGE_H__
#define __APP_LCD_PLAN_PAGE_H__

#include "config.h"
#include "dev_gt911/dev_gt911.h"
#include "dev_gd25q128/dev_gd25q128.h"

/** 页面时间消息长度："YYYY-MM-DD HH:MM:SS" + '\0'。 */
#define APP_LCD_PLAN_TIME_TEXT_SIZE  20U

/**
 * 页面显示字段标识；消息只携带语义数据，不携带 LCD 命令或指针。
 * Display Task 是唯一可以调用 dev_lcd 的任务，其他模块只能投递这些消息。
 *
 * NET_STATUS / TIME / CLOUD_STATUS / OTA_STATUS / AB_STATUS 为保留值：
 * 这些系统状态改由 app_ui_snapshot 提供，Display Task 不再消费对应消息。
 */
typedef enum
{
    APP_LCD_PLAN_FIELD_TOUCH_ID = 2U,
    APP_LCD_PLAN_FIELD_PAGE = 3U,
    APP_LCD_PLAN_FIELD_FLASH_ID = 4U,
    APP_LCD_PLAN_FIELD_FLASH_TEST = 5U,
    APP_LCD_PLAN_FIELD_NET_STATUS = 6U,   /* 保留 */
    APP_LCD_PLAN_FIELD_TIME = 7U,         /* 保留 */
    APP_LCD_PLAN_FIELD_CLOUD_STATUS = 9U, /* 保留 */
    APP_LCD_PLAN_FIELD_HOME_SLIDE = 10U,
    APP_LCD_PLAN_FIELD_OTA_STATUS = 11U,  /* 保留 */
    APP_LCD_PLAN_FIELD_AB_STATUS = 12U,   /* 保留 */
    APP_LCD_PLAN_FIELD_TOUCH_EVENT = 13U
} app_lcd_plan_field_t;

/** 触摸事件类型：Touch Task 只上报按下/抬起两种原始事件。 */
#define APP_LCD_PLAN_TOUCH_EVENT_DOWN 0U
#define APP_LCD_PLAN_TOUCH_EVENT_UP   1U

/** OTA 会话状态（LCD 语义，与 app_ota_state_t 解耦，数值需保持稳定）。 */
#define APP_LCD_PLAN_OTA_STATE_IDLE        0U
#define APP_LCD_PLAN_OTA_STATE_DOWNLOADING 1U
#define APP_LCD_PLAN_OTA_STATE_VERIFYING   2U
#define APP_LCD_PLAN_OTA_STATE_COMMITTING  3U
#define APP_LCD_PLAN_OTA_STATE_ERROR       4U
#define APP_LCD_PLAN_OTA_STATE_REBOOTING   5U /* 校验通过，等待复位进 BootLoader */

/** A/B 槽位编码（与 boot_protocol.h 一致，LCD 层不反向依赖 Boot 头文件）。 */
#define APP_LCD_PLAN_SLOT_A     0U
#define APP_LCD_PLAN_SLOT_B     1U
#define APP_LCD_PLAN_SLOT_NONE  0xFFU

/** HOME 内部五张轮播页；与底部主导航页面 ID 相互独立。 */
typedef enum
{
    APP_LCD_PLAN_HOME_SLIDE_OVERVIEW = 0U,
    APP_LCD_PLAN_HOME_SLIDE_COMM_DIAG,
    APP_LCD_PLAN_HOME_SLIDE_OTA_PROGRESS,
    APP_LCD_PLAN_HOME_SLIDE_AB_STATUS,
    APP_LCD_PLAN_HOME_SLIDE_DEVICE_LOG,
    APP_LCD_PLAN_HOME_SLIDE_COUNT
} app_lcd_plan_home_slide_t;

/** 底部导航对应的五个页面。 */
typedef enum
{
    APP_LCD_PLAN_PAGE_HOME = 0U,
    APP_LCD_PLAN_PAGE_DEVICES,
    APP_LCD_PLAN_PAGE_OTA,
    APP_LCD_PLAN_PAGE_ALERTS,
    APP_LCD_PLAN_PAGE_SETTINGS,
    APP_LCD_PLAN_PAGE_COUNT
} app_lcd_plan_page_id_t;

/**
 * 页面消息载荷；数据使用值传递，不在队列中传递指针。
 * cloud 只保留当前设备页需要显示的摘要，不把完整网络快照倒灌进 LCD 层。
 */
typedef union
{
    char touch_id[DEV_GT911_PRODUCT_ID_LENGTH];
    dev_gd25q128_jedec_id_t flash_id;
    ret_t flash_test_result;
    uint8_t network_online;
    char time_text[APP_LCD_PLAN_TIME_TEXT_SIZE];
    uint32_t page;
    uint8_t home_slide;
    struct
    {
        uint8_t event; /* APP_LCD_PLAN_TOUCH_EVENT_* */
        uint16_t x;    /* UP 事件携带最后一次已知坐标，供滑动手势判定 */
        uint16_t y;
    } touch;
    struct
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
    } cloud;
    struct
    {
        uint8_t state;   /* APP_LCD_PLAN_OTA_STATE_* */
        uint8_t percent; /* 0..100，按已写入字节数计算 */
        uint32_t received;
        uint32_t total;
        uint32_t target_version; /* 云端通知里的目标版本，0 表示未知 */
    } ota;
    struct
    {
        uint8_t valid;          /* 0 时其余字段无效，界面显示未知 */
        uint8_t active_slot;    /* APP_LCD_PLAN_SLOT_* */
        uint8_t pending_slot;   /* APP_LCD_PLAN_SLOT_* 或 NONE */
        uint8_t rollback_flag;  /* 1 表示发生过回滚（fw_nvs 锁存） */
        uint32_t running_version; /* 当前已确认运行的固件版本 */
        uint32_t generation;    /* 启动参数 generation，每次参数落盘 +1 */
    } ab;
} app_lcd_plan_value_t;

/** 页面显示消息；队列会复制整个消息内容，生产者无需保持 message 指针有效。 */
typedef struct
{
    app_lcd_plan_field_t field;
    app_lcd_plan_value_t value;
} app_lcd_plan_msg_t;

/**
 * 设备页 LED2 交互处理器；由系统组合根注册。
 * Display Task 命中按钮后调用本回调，硬件和业务状态由回调拥有者处理。
 */
typedef ret_t (*app_lcd_plan_led_toggle_handler_t)(void *context);

/************************************************************
 * @brief   初始化 P0 计划页显示队列、Display Task 和 Touch Task
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始化;
 *          RET_NO_MEMORY 内存不足; RET_TASK_CREATE_ERROR 任务创建失败
 ************************************************************/
ret_t app_lcd_plan_page_init(void);

/**
 * 注册设备页 LED2 操作处理器；传入 NULL 可取消注册。
 * @note 应在创建调度器前完成注册，回调运行在 Display Task 上下文中。
 */
ret_t app_lcd_plan_page_set_led_toggle_handler(
    app_lcd_plan_led_toggle_handler_t handler,
    void *context);

/************************************************************
 * @brief   向P0页面显示队列投递一条语义消息
 * @param   message 显示消息指针；函数返回前会复制消息内容
 * @note    普通状态更新使用非阻塞投递；调用者不应在回调或中断中直接操作 LCD。
 * @note    普通测量值为0 tick非阻塞投递；触摸事件与页面切换等关键事件
 *          在队列满时允许短暂等待后再返回。
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_NOT_INIT 未初始化;
 *          RET_NOT_READY Display Task未就绪; RET_QUEUE_FULL 队列已满
 ************************************************************/
ret_t app_lcd_plan_page_post(const app_lcd_plan_msg_t *message);

#endif /* __APP_LCD_PLAN_PAGE_H__ */
