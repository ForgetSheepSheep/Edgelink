/**
 * @file    app_lcd_plan_page.c
 * @brief   LCD P0 计划页应用层实现
 *
 * @note    页面任务只调�?LCD Device 的清屏、矩形和文本语义接口�?
 *          不直接访�?EXMC 地址、ILI9488 命令或总线原语�?
 */

#include "app_lcd_plan_page/app_lcd_plan_page.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "dev_lcd/dev_lcd.h"
#include "dev_gt911/dev_gt911.h"
#include "dev_gd25q128/dev_gd25q128.h"
#include "app_ui_snapshot/app_ui_snapshot.h"

/** RGB565 页面颜色：深海军蓝底色与青绿色强调色�?*/
#define PLAN_COLOR_BACKGROUND        (0x08C5U)
#define PLAN_COLOR_HEADER            (0xFFFFU)
#define PLAN_COLOR_CARD              (0x11AAU)
#define PLAN_COLOR_BORDER            (0x1A0FU)
#define PLAN_COLOR_TEXT              (0xE73FU)
#define PLAN_COLOR_MUTED             (0x9DBBU)
#define PLAN_COLOR_GREEN             (0x07B4U)
#define PLAN_COLOR_BLUE              (0x35BFU)
#define PLAN_COLOR_ORANGE            (0xFD20U)
#define PLAN_COLOR_GRAY              (0x52D7U)
#define PLAN_COLOR_FOOTER            (PLAN_COLOR_BACKGROUND)

/** 启动页专用浅蓝主题，参�?Smart Life 留白与柔和波浪风格�?*/
#define PLAN_BOOT_COLOR_BACKGROUND   (0xF7FFU)
#define PLAN_BOOT_COLOR_SURFACE      (0xEFFFU)
#define PLAN_BOOT_COLOR_TRACK        (0xD69FU)
#define PLAN_BOOT_COLOR_TEXT         (0x294DU)
#define PLAN_BOOT_COLOR_MUTED        (0x5B32U)
#define PLAN_BOOT_COLOR_ACCENT       (0x249FU)
#define PLAN_BOOT_COLOR_LEAF_GREEN   (0x66CFU)
#define PLAN_BOOT_COLOR_COMPLETE     (0x262BU)
#define PLAN_BOOT_COLOR_WAVE_BACK    (0xDFFFU)
#define PLAN_BOOT_COLOR_WAVE_MIDDLE  (0xB67FU)
#define PLAN_BOOT_COLOR_WAVE_FRONT   (0x8DFFU)

/** Display Task栈深�?*/
#define LCD_PLAN_TASK_STACK_DEPTH    256U
/** Display Task优先�?*/
#define LCD_PLAN_TASK_PRIORITY       2U
/** 页面显示消息队列长度 */
#define LCD_PLAN_QUEUE_LENGTH         8U
/** GT911 Touch Task 栈深�?*/
#define LCD_PLAN_TOUCH_TASK_STACK_DEPTH 256U
/** GT911 Touch Task 优先�?*/
#define LCD_PLAN_TOUCH_TASK_PRIORITY    3U
/** Touch Task 在没有收�?INT 时的参考轮询周期�?*/
#define LCD_PLAN_TOUCH_SCAN_PERIOD_MS 20U
/** GT911 初始化失败时的重试次数与重试间隔（毫秒） */
#define LCD_PLAN_TOUCH_INIT_RETRY          3U
#define LCD_PLAN_TOUCH_INIT_RETRY_DELAY_MS 100U
/**
 * 松开确认帧数（�?20ms = 200ms）�?
 *
 * GT911 在手指静止时不再发送新帧，驱动�?READY=0 返回 UP�?
 * 若把这种"无新数据"当成松开，一次按压会�?GT911 每次重报�?
 * 重新判定为按下沿，LED2 按钮/导航会被重复触发（表现为灯闪）�?
 * 只有连续 N �?UP 才确认手指真的离开并重新武装按钮�?
 */
#define LCD_PLAN_TOUCH_RELEASE_FRAMES 10U
/** 页面切换属于关键事件，队列满时允许短暂等待�?*/
#define LCD_PLAN_PAGE_EVENT_WAIT_MS   100U
/** Display Task ??????????????????????????????????????*/
#define LCD_PLAN_SNAPSHOT_POLL_MS     100U
/** 启动动画固定�?3 秒：30 × 100ms，避免上电等待过久�?*/
#define LCD_PLAN_BOOT_DURATION_MS     3000U
#define LCD_PLAN_BOOT_STEP_COUNT      30U
#define LCD_PLAN_BOOT_STEP_MS         (LCD_PLAN_BOOT_DURATION_MS / \
                                       LCD_PLAN_BOOT_STEP_COUNT)
/** 页面显示消息队列句柄 */
static QueueHandle_t s_lcd_plan_queue = NULL;
/** Display Task句柄 */
static TaskHandle_t s_lcd_plan_task = NULL;
/** GT911 Touch Task 句柄；由 PB12 INT �?20ms 轮询唤醒 */
static TaskHandle_t s_lcd_plan_touch_task = NULL;
/** Display Task 当前显示页；只由 Display Task 访问�?*/
static app_lcd_plan_page_id_t s_lcd_plan_current_page = APP_LCD_PLAN_PAGE_HOME;
/** HOME 五张轮播页索引；只由 Display Task 更新�?*/
static app_lcd_plan_home_slide_t s_lcd_plan_home_slide =
    APP_LCD_PLAN_HOME_SLIDE_OVERVIEW;
/** 触摸按下状态与起点坐标：只由 Display Task 维护，Touch Task 只上报原始事件 */
static uint8_t s_lcd_plan_touch_active = 0U;
static uint16_t s_lcd_plan_touch_start_x = 0U;
static uint16_t s_lcd_plan_touch_start_y = 0U;
/** ????????????????????Display Task ??? */
static app_ui_snapshot_change_t s_lcd_plan_seen_changes;
/** 最�?GT911 产品 ID 文本；切页后返回 HOME 时重新绘制�?*/
static char s_lcd_plan_touch_id[DEV_GT911_PRODUCT_ID_TEXT_SIZE] = "WAIT";
/** 产品 ID 状态颜色；WAIT/FAIL 为橙色，读取成功为绿色�?*/
static uint16_t s_lcd_plan_touch_id_color = PLAN_COLOR_ORANGE;
/** 最�?GD25Q128 JEDEC ID 文本，格式为“XX XX XX”�?*/
static char s_lcd_plan_flash_id[10U] = "WAIT";
/** GD25Q128 ID 状态颜色�?*/
static uint16_t s_lcd_plan_flash_status_color = PLAN_COLOR_ORANGE;
/** GD25Q128 Page Program 测试状态文本�?*/
static char s_lcd_plan_flash_test_status[5U] = "WAIT";
/** GD25Q128 Page Program 测试状态颜色�?*/
static uint16_t s_lcd_plan_flash_test_status_color = PLAN_COLOR_ORANGE;
/** MQTT网络状态快照；valid�?时界面显示WAIT�?*/
static uint8_t s_lcd_plan_network_online = 0U;
static uint8_t s_lcd_plan_network_status_valid = 0U;
/** 云端数据快照；只�?Display Task 更新并绘制�?*/
static uint8_t s_lcd_plan_cloud_status_valid = 0U;
/** Modbus轮询有效位：bit0=控制从站、bit1=模拟量从站、bit2=IO从站�?*/
static uint8_t s_lcd_plan_cloud_valid_slave_mask = 0U;
static uint8_t s_lcd_plan_cloud_poll_was_attempted = 0U;
static uint8_t s_lcd_plan_cloud_led2_is_on = 0U;
static uint16_t s_lcd_plan_cloud_control_holding_register_0 = 0U;
static uint16_t s_lcd_plan_cloud_control_holding_register_1 = 0U;
static uint16_t s_lcd_plan_cloud_control_holding_register_11 = 0U;
static uint16_t s_lcd_plan_cloud_analog_input_register_0 = 0U;
static uint8_t s_lcd_plan_cloud_io_coil_states = 0U;
static uint8_t s_lcd_plan_cloud_io_discrete_input_states = 0U;
/** 顶栏时间快照，显示格式为 "MM-DD HH:MM"�?*/
static char s_lcd_plan_clock[12U] = "--/-- --:--";
/** OTA 会话快照；由组合根经消息投递，Display Task 只读绘制�?*/
static uint8_t s_lcd_plan_ota_state = APP_LCD_PLAN_OTA_STATE_IDLE;
static uint8_t s_lcd_plan_ota_percent = 0U;
static uint32_t s_lcd_plan_ota_received = 0U;
static uint32_t s_lcd_plan_ota_total = 0U;
static uint32_t s_lcd_plan_ota_target_version = 0U;
/** A/B 分区与启动状态快照�?*/
static uint8_t s_lcd_plan_ab_valid = 0U;
static uint8_t s_lcd_plan_ab_active_slot = APP_LCD_PLAN_SLOT_NONE;
static uint8_t s_lcd_plan_ab_pending_slot = APP_LCD_PLAN_SLOT_NONE;
static uint8_t s_lcd_plan_ab_rollback_flag = 0U;
static uint32_t s_lcd_plan_ab_running_version = 0U;
static uint32_t s_lcd_plan_ab_generation = 0U;

/**
 * 设备�?LED2 操作处理器，由系统组合根注入�?
 * Touch Task 只识别“用户按下按钮”这一事件，不直接依赖网络�?GPIO�?
 */
static app_lcd_plan_led_toggle_handler_t s_lcd_plan_led_toggle_handler = NULL;
static void *s_lcd_plan_led_toggle_context = NULL;

static ret_t plan_format_flash_id(const dev_gd25q128_jedec_id_t *id,
                                  char *buffer,
                                  uint32_t buffer_size);
static ret_t plan_apply_message(const app_lcd_plan_msg_t *message);
static ret_t plan_draw_boot_leaf_logo(void);
static ret_t plan_draw_boot_waves(void);
static ret_t plan_draw_startup_animation(void);
static ret_t plan_draw_bottom_nav(app_lcd_plan_page_id_t page);
static ret_t plan_draw_header(const char *page_title);
static const char *plan_page_title(app_lcd_plan_page_id_t page);
static ret_t plan_draw_home_header(void);
static ret_t plan_page_draw_home(void);
static ret_t plan_draw_home_link_status(void);
static ret_t plan_draw_home_system_info(void);
static ret_t plan_draw_home_ota_status(void);
static ret_t plan_draw_home_runtime_info(void);
static ret_t plan_draw_home_comm_diagnostics(void);
static ret_t plan_draw_home_ota_progress(void);
static ret_t plan_draw_home_ab_status(void);
static ret_t plan_draw_home_device_log(void);
static ret_t plan_draw_home_pagination(uint8_t slide);
static ret_t plan_format_u16_hex(uint16_t value, char *buffer, uint32_t buffer_size);
static ret_t plan_draw_nav_icon(uint8_t page,
                                uint16_t center_x,
                                uint16_t y,
                                uint16_t color);
static const char *plan_link_state_text(uint8_t valid, uint8_t online);
static uint16_t plan_link_state_color(uint8_t valid, uint8_t online);
static const char *plan_modbus_state_text(uint8_t snapshot_is_valid,
                                          uint8_t poll_was_attempted,
                                          uint8_t valid_slave_mask);
static uint16_t plan_modbus_state_color(uint8_t snapshot_is_valid,
                                        uint8_t poll_was_attempted,
                                        uint8_t valid_slave_mask);
static ret_t plan_page_draw_devices(void);
static ret_t plan_draw_cloud_modbus_card(void);
static ret_t plan_page_draw_alerts(void);
static ret_t plan_page_draw_ota(void);
static ret_t plan_page_draw_settings(void);
static ret_t plan_draw_link_status(void);
static ret_t plan_page_draw(app_lcd_plan_page_id_t page);
static void lcd_plan_touch_task(void *argument);
static ret_t plan_handle_touch_event(const app_lcd_plan_msg_t *message);

/**
 * @brief   绘制圆角填充矩形（页面层几何辅助�?
 * @retval  RET_OK 成功；其他值由 LCD Device 返回
 */
static ret_t plan_fill_round_rect(uint16_t x_start,
                                  uint16_t y_start,
                                  uint16_t x_end,
                                  uint16_t y_end,
                                  uint16_t radius,
                                  uint16_t color)
{
    ret_t ret;
    uint16_t row;
    uint16_t inset;
    uint16_t delta_y;
    uint16_t delta_x;
    uint16_t radius_squared;

    if ((x_start > x_end) || (y_start > y_end) ||
        (radius > 10U) ||
        ((uint32_t)(x_end - x_start + 1U) < (uint32_t)radius * 2U + 1U) ||
        ((uint32_t)(y_end - y_start + 1U) < (uint32_t)radius * 2U + 1U))
    {
        return RET_INVALID_PARAM;
    }

    if (radius == 0U)
    {
        return dev_lcd_fill_rect(x_start, y_start, x_end, y_end, color);
    }

    radius_squared = (uint16_t)(radius * radius);

    /* 以整数圆方程求每一行边缘，避免浮点运算和帧缓冲�?*/
    for (row = 0U; row < radius; row++)
    {
        delta_y = (uint16_t)(radius - row);
        inset = radius;
        while (inset > 0U)
        {
            delta_x = (uint16_t)(radius - inset);
            if (((uint32_t)delta_x * delta_x) +
                ((uint32_t)delta_y * delta_y) <= radius_squared)
            {
                break;
            }
            inset--;
        }

        ret = dev_lcd_fill_rect((uint16_t)(x_start + inset),
                                (uint16_t)(y_start + row),
                                (uint16_t)(x_end - inset),
                                (uint16_t)(y_start + row),
                                color);
        if (ret != RET_OK)
        {
            return ret;
        }

        ret = dev_lcd_fill_rect((uint16_t)(x_start + inset),
                                (uint16_t)(y_end - row),
                                (uint16_t)(x_end - inset),
                                (uint16_t)(y_end - row),
                                color);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return dev_lcd_fill_rect(x_start,
                             (uint16_t)(y_start + radius),
                             x_end,
                             (uint16_t)(y_end - radius),
                             color);
}

static ret_t plan_draw_card(uint16_t x_start,
                            uint16_t y_start,
                            uint16_t x_end,
                            uint16_t y_end,
                            uint16_t color)
{
    ret_t ret;

    ret = plan_fill_round_rect(x_start,
                               y_start,
                               x_end,
                               y_end,
                               8U,
                               PLAN_COLOR_BORDER);
    if (ret != RET_OK)
    {
        return ret;
    }

    return plan_fill_round_rect((uint16_t)(x_start + 1U),
                                (uint16_t)(y_start + 1U),
                                (uint16_t)(x_end - 1U),
                                (uint16_t)(y_end - 1U),
                                7U,
                                color);
}

static ret_t plan_draw_status_tag(uint16_t y,
                                  const char *text,
                                  uint16_t color,
                                  uint16_t text_color)
{
    const char *cursor;
    uint32_t text_px;
    uint16_t tag_x;
    ret_t ret;

    if (text == NULL)
    {
        return RET_NULL_PTR;
    }

    text_px = 0U;
    cursor = text;
    while (*cursor != '\0')
    {
        text_px += DEV_LCD_FONT_WIDTH;
        cursor++;
    }

    /* 标签默认右对齐固定在 252~296；文字比标签宽时向左扩框，
     * 避免长状态名（如 DOWNLOADING）越界导致整页刷新中止。 */
    tag_x = 252U;
    if ((text_px + 8U) > (296U - 252U + 1U))
    {
        tag_x = (uint16_t)(296U - (text_px + 8U));
        if (tag_x < 4U)
        {
            tag_x = 4U;
        }
    }

    ret = plan_fill_round_rect(tag_x,
                               y,
                               296U,
                               (uint16_t)(y + 18U),
                               4U,
                               color);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_draw_text((uint16_t)(tag_x + 4U),
                             (uint16_t)(y + 1U),
                             text,
                             text_color,
                             color);
}

/**
 * @brief   清除固定字符槽并绘制页面数�?
 * @param   x            区域左上�?X 坐标
 * @param   y            区域左上�?Y 坐标
 * @param   slot_count   预留字符槽位数量
 * @param   text         要绘制的 ASCII 文本
 * @param   text_color   文本前景�?
 * @param   bg_color     区域背景�?
 * @retval  RET_OK 成功；RET_NULL_PTR 空指针；RET_INVALID_PARAM 参数非法�?
 *          RET_OUT_OF_RANGE 区域或文本超出允许范围；其他值由 LCD Device 返回
 *
 * @note    该辅助函数属于页面布局层。它只负责固定区域的清除和刷新，
 *          不接�?LCD 命令、EXMC 地址�?GRAM 原语�?
 */
static ret_t plan_draw_value(uint16_t x,
                             uint16_t y,
                             uint16_t slot_count,
                             const char *text,
                             uint16_t text_color,
                             uint16_t bg_color)
{
    const char *cursor;
    uint32_t text_length;
    uint32_t region_width;
    uint32_t region_x_end;
    uint32_t region_y_end;
    ret_t ret;

    if (text == NULL)
    {
        return RET_NULL_PTR;
    }

    if (slot_count == 0U)
    {
        return RET_INVALID_PARAM;
    }

    /* 手工 strlen：不用标准库 strlen，减少对库的依赖�?
     * 逻辑�?strlen 完全一致（数到 '\0'）�?*/
    text_length = 0U;
    cursor = text;
    while (*cursor != '\0')
    {
        text_length++;
        cursor++;
    }

    /* 文本不许超出预留槽位：超了说明调用者槽位算小了�?
     * 直接报错比截断显示半截数字更安全（半截数字会误导读数）�?*/
    if (text_length > slot_count)
    {
        return RET_OUT_OF_RANGE;
    }

    /* 槽位转像素区：宽=槽位数�?，算出右下角并做屏幕越界检查�?
     * �?uint32 中间计算再转�?uint16，x+width �?16 位下可能溢出�?
     * 先拓宽再比较是防溢出的标准做法�?*/
    region_width = (uint32_t)slot_count * DEV_LCD_FONT_WIDTH;
    region_x_end = (uint32_t)x + region_width - 1U;
    region_y_end = (uint32_t)y + DEV_LCD_FONT_HEIGHT - 1U;
    if ((region_x_end >= DEV_LCD_WIDTH) ||
        (region_y_end >= DEV_LCD_HEIGHT))
    {
        return RET_OUT_OF_RANGE;
    }

    /* 先按背景色填满整个槽位，再写新文本：新文本比旧文本短�?
     * （如 "125.0" -> "9.5"），多余的旧笔画被背景覆盖，
     * 不会留下拖影——这就是“槽位刷新”防残影的原理�?*/
    ret = dev_lcd_fill_rect(x,
                            y,
                            (uint16_t)region_x_end,
                            (uint16_t)region_y_end,
                            bg_color);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_draw_text(x,
                             y,
                             text,
                             text_color,
                             bg_color);
}

/**
 * @brief   将三字节 JEDEC ID 格式化为 LCD 可显示的 ASCII 文本
 */
static ret_t plan_format_flash_id(const dev_gd25q128_jedec_id_t *id,
                                  char *buffer,
                                  uint32_t buffer_size)
{
    static const char hex[] = "0123456789ABCDEF";

    if ((id == NULL) || (buffer == NULL))
    {
        return RET_NULL_PTR;
    }

    /* “XX XX XX”共8个字符，另加结尾 '\0'�?*/
    if (buffer_size < 9U)
    {
        return RET_LENGTH_ERROR;
    }

    buffer[0] = hex[(id->manufacturer >> 4U) & 0x0FU];
    buffer[1] = hex[id->manufacturer & 0x0FU];
    buffer[2] = ' ';
    buffer[3] = hex[(id->memory_type >> 4U) & 0x0FU];
    buffer[4] = hex[id->memory_type & 0x0FU];
    buffer[5] = ' ';
    buffer[6] = hex[(id->capacity >> 4U) & 0x0FU];
    buffer[7] = hex[id->capacity & 0x0FU];
    buffer[8] = '\0';

    return RET_OK;
}

/** 绘制双色叶片标志：逐行填充多边形，不需要位图或帧缓冲�?*/
static ret_t plan_draw_boot_leaf_logo(void)
{
    uint32_t y;
    uint32_t left;
    uint32_t right;
    uint32_t vein_x;
    ret_t ret;

    /* 主叶：顶端、最宽处、叶根三段线性插值形成柔和轮廓�?*/
    for (y = 78U; y <= 188U; y++)
    {
        if (y <= 112U)
        {
            left = 198U - ((44U * (y - 78U)) / 34U);
            right = 198U + ((22U * (y - 78U)) / 34U);
        }
        else if (y <= 148U)
        {
            left = 154U - ((20U * (y - 112U)) / 36U);
            right = 220U - ((14U * (y - 112U)) / 36U);
        }
        else
        {
            left = 134U + ((24U * (y - 148U)) / 40U);
            right = 206U - ((48U * (y - 148U)) / 40U);
        }

        ret = dev_lcd_fill_rect((uint16_t)left, (uint16_t)y,
                                (uint16_t)right, (uint16_t)y,
                                PLAN_BOOT_COLOR_ACCENT);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    /* 辅叶：青绿色，与主叶在根部汇合�?*/
    for (y = 128U; y <= 188U; y++)
    {
        if (y <= 158U)
        {
            left = 106U + ((6U * (y - 128U)) / 30U);
            right = 106U + ((50U * (y - 128U)) / 30U);
        }
        else
        {
            left = 112U + ((46U * (y - 158U)) / 30U);
            right = 156U + ((2U * (y - 158U)) / 30U);
        }

        ret = dev_lcd_fill_rect((uint16_t)left, (uint16_t)y,
                                (uint16_t)right, (uint16_t)y,
                                PLAN_BOOT_COLOR_LEAF_GREEN);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    /* 用背景色切出两条叶脉，保留参考图轻盈的负形结构�?*/
    for (y = 108U; y <= 188U; y++)
    {
        vein_x = 201U - ((43U * (y - 108U)) / 80U);
        ret = dev_lcd_fill_rect((uint16_t)vein_x, (uint16_t)y,
                                (uint16_t)(vein_x + 2U), (uint16_t)y,
                                PLAN_BOOT_COLOR_BACKGROUND);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    for (y = 148U; y <= 188U; y++)
    {
        vein_x = 122U + ((36U * (y - 148U)) / 40U);
        ret = dev_lcd_fill_rect((uint16_t)vein_x, (uint16_t)y,
                                (uint16_t)(vein_x + 1U), (uint16_t)y,
                                PLAN_BOOT_COLOR_BACKGROUND);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return RET_OK;
}

/** 绘制底部三层浅蓝波浪�?*/
static ret_t plan_draw_boot_waves(void)
{
    uint32_t x;
    uint32_t x_end;
    int32_t distance;
    uint32_t y_start;
    ret_t ret;

    for (x = 0U; x < DEV_LCD_WIDTH; x += 4U)
    {
        x_end = x + 3U;
        if (x_end >= DEV_LCD_WIDTH)
        {
            x_end = DEV_LCD_WIDTH - 1U;
        }

        distance = (int32_t)((x + 20U) % 160U) - 80;
        y_start = 358U +
                  (uint32_t)((distance * distance) / 320);
        ret = dev_lcd_fill_rect((uint16_t)x, (uint16_t)y_start,
                                (uint16_t)x_end, 479U,
                                PLAN_BOOT_COLOR_WAVE_BACK);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    for (x = 0U; x < DEV_LCD_WIDTH; x += 4U)
    {
        x_end = x + 3U;
        if (x_end >= DEV_LCD_WIDTH)
        {
            x_end = DEV_LCD_WIDTH - 1U;
        }

        distance = (int32_t)((x + 92U) % 176U) - 88;
        y_start = 390U +
                  (uint32_t)((distance * distance) / 390);
        ret = dev_lcd_fill_rect((uint16_t)x, (uint16_t)y_start,
                                (uint16_t)x_end, 479U,
                                PLAN_BOOT_COLOR_WAVE_MIDDLE);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    for (x = 0U; x < DEV_LCD_WIDTH; x += 4U)
    {
        x_end = x + 3U;
        if (x_end >= DEV_LCD_WIDTH)
        {
            x_end = DEV_LCD_WIDTH - 1U;
        }

        distance = (int32_t)((x + 45U) % 144U) - 72;
        y_start = 425U +
                  (uint32_t)((distance * distance) / 300);
        ret = dev_lcd_fill_rect((uint16_t)x, (uint16_t)y_start,
                                (uint16_t)x_end, 479U,
                                PLAN_BOOT_COLOR_WAVE_FRONT);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return RET_OK;
}

/**
 * @brief   绘制�?3 秒的 Smart Life 风格启动动画
 * @retval  RET_OK 成功；其他值由 LCD Device 返回
 */
static ret_t plan_draw_startup_animation(void)
{
    static const char *const phase_text[5U] =
    {
        "  INITIALIZING DISPLAY",
        "  CHECKING TOUCH PANEL",
        "    CHECKING STORAGE",
        "   STARTING 4G MODEM",
        "  WAITING DEVICE STATUS"
    };
    TickType_t last_wake_tick;
    char percent_text[5U];
    uint32_t step;
    uint32_t percent;
    uint32_t filled_width;
    uint32_t previous_width;
    uint32_t phase;
    uint32_t previous_phase;
    ret_t ret;

    ret = dev_lcd_clear(PLAN_BOOT_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(172U, 24U, "EXPLORE . CREATE",
                            PLAN_BOOT_COLOR_MUTED,
                            PLAN_BOOT_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }
    ret = dev_lcd_draw_text(164U, 48U, "MAKE LIFE SMARTER",
                            PLAN_BOOT_COLOR_MUTED,
                            PLAN_BOOT_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }
    ret = dev_lcd_fill_rect(260U, 76U, 299U, 77U,
                            PLAN_BOOT_COLOR_MUTED);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_boot_leaf_logo();
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text_scaled(80U, 208U, "Smart", 2U,
                                   PLAN_BOOT_COLOR_TEXT,
                                   PLAN_BOOT_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }
    ret = dev_lcd_draw_text_scaled(160U, 208U, " Life", 2U,
                                   PLAN_BOOT_COLOR_ACCENT,
                                   PLAN_BOOT_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(56U, 250U,
                            "SMART | EFFICIENT | BETTER",
                            PLAN_BOOT_COLOR_MUTED,
                            PLAN_BOOT_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_fill_round_rect(28U, 298U, 255U, 322U, 8U,
                               PLAN_BOOT_COLOR_TRACK);
    if (ret != RET_OK)
    {
        return ret;
    }
    ret = dev_lcd_fill_rect(36U, 306U, 247U, 314U,
                            PLAN_BOOT_COLOR_SURFACE);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_boot_waves();
    if (ret != RET_OK)
    {
        return ret;
    }

    percent_text[0U] = ' ';
    percent_text[1U] = ' ';
    percent_text[2U] = '0';
    percent_text[3U] = '%';
    percent_text[4U] = '\0';
    ret = plan_draw_value(268U, 303U, 4U, percent_text,
                          PLAN_BOOT_COLOR_TEXT,
                          PLAN_BOOT_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(60U, 334U, 25U, phase_text[0U],
                          PLAN_BOOT_COLOR_MUTED,
                          PLAN_BOOT_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(76U, 448U, "WITH A BETTER FUTURE",
                            PLAN_BOOT_COLOR_ACCENT,
                            PLAN_BOOT_COLOR_WAVE_FRONT);
    if (ret != RET_OK)
    {
        return ret;
    }

    previous_width = 0U;
    previous_phase = 0U;
    last_wake_tick = xTaskGetTickCount();

    for (step = 1U; step <= LCD_PLAN_BOOT_STEP_COUNT; step++)
    {
        percent = (step * 100U) / LCD_PLAN_BOOT_STEP_COUNT;
        filled_width = (step * 212U) / LCD_PLAN_BOOT_STEP_COUNT;

        if (filled_width > previous_width)
        {
            ret = dev_lcd_fill_rect((uint16_t)(36U + previous_width),
                                    306U,
                                    (uint16_t)(35U + filled_width),
                                    314U,
                                    (step == LCD_PLAN_BOOT_STEP_COUNT) ?
                                    PLAN_BOOT_COLOR_COMPLETE :
                                    PLAN_BOOT_COLOR_ACCENT);
            if (ret != RET_OK)
            {
                return ret;
            }
            previous_width = filled_width;
        }

        if (percent == 100U)
        {
            percent_text[0U] = '1';
            percent_text[1U] = '0';
            percent_text[2U] = '0';
        }
        else
        {
            percent_text[0U] = ' ';
            percent_text[1U] = (percent >= 10U) ?
                               (char)('0' + (percent / 10U)) : ' ';
            percent_text[2U] = (char)('0' + (percent % 10U));
        }

        ret = plan_draw_value(268U, 303U, 4U, percent_text,
                              (percent == 100U) ?
                              PLAN_BOOT_COLOR_COMPLETE :
                              PLAN_BOOT_COLOR_TEXT,
                              PLAN_BOOT_COLOR_BACKGROUND);
        if (ret != RET_OK)
        {
            return ret;
        }

        phase = ((step - 1U) * 5U) / LCD_PLAN_BOOT_STEP_COUNT;
        if (phase > 4U)
        {
            phase = 4U;
        }
        if (phase != previous_phase)
        {
            ret = plan_draw_value(60U, 334U, 25U, phase_text[phase],
                                  PLAN_BOOT_COLOR_MUTED,
                                  PLAN_BOOT_COLOR_BACKGROUND);
            if (ret != RET_OK)
            {
                return ret;
            }
            previous_phase = phase;
        }

        if (step == LCD_PLAN_BOOT_STEP_COUNT)
        {
            ret = dev_lcd_fill_rect(36U, 306U, 247U, 314U,
                                    PLAN_BOOT_COLOR_COMPLETE);
            if (ret != RET_OK)
            {
                return ret;
            }
            ret = plan_draw_value(60U, 334U, 25U,
                                  "   OPENING DASHBOARD",
                                  PLAN_BOOT_COLOR_COMPLETE,
                                  PLAN_BOOT_COLOR_BACKGROUND);
            if (ret != RET_OK)
            {
                return ret;
            }
        }

        vTaskDelayUntil(&last_wake_tick,
                        pdMS_TO_TICKS(LCD_PLAN_BOOT_STEP_MS));
    }

    return RET_OK;
}

/**
 * @brief   �?Modbus 原始 16 位值格式化为四位十六进制文�?
 * @note    不在没有量纲/比例定义时把原始寄存器伪装成工程量�?
 */
static ret_t plan_format_u16_hex(uint16_t value,
                                 char *buffer,
                                 uint32_t buffer_size)
{
    static const char hex[] = "0123456789ABCDEF";

    if (buffer == NULL)
    {
        return RET_NULL_PTR;
    }
    if (buffer_size < 5U)
    {
        return RET_LENGTH_ERROR;
    }

    buffer[0] = hex[(value >> 12U) & 0x0FU];
    buffer[1] = hex[(value >> 8U) & 0x0FU];
    buffer[2] = hex[(value >> 4U) & 0x0FU];
    buffer[3] = hex[value & 0x0FU];
    buffer[4] = '\0';
    return RET_OK;
}

/**
 * @brief   �?0x00010203 编码的固件版本格式化�?"V1.2.3"
 * @param   version     版本值；0 表示未知
 * @param   buffer      输出缓冲区，至少 13 字节
 * @param   buffer_size 缓冲区长�?
 * @note    仅做显示格式化，不做版本比较（比较属�?OTA 业务，不�?LCD 层）�?
 */
static ret_t plan_format_version(uint32_t version,
                                 char *buffer,
                                 uint32_t buffer_size)
{
    int length;

    if (buffer == NULL)
    {
        return RET_NULL_PTR;
    }
    if (buffer_size < 13U)
    {
        return RET_LENGTH_ERROR;
    }

    if (version == 0U)
    {
        buffer[0] = '-';
        buffer[1] = '-';
        buffer[2] = '\0';
        return RET_OK;
    }

    length = snprintf(buffer,
                      buffer_size,
                      "V%lu.%lu.%lu",
                      (unsigned long)((version >> 16U) & 0xFFU),
                      (unsigned long)((version >> 8U) & 0xFFU),
                      (unsigned long)(version & 0xFFU));
    if ((length < 0) || ((uint32_t)length >= buffer_size))
    {
        return RET_OVERFLOW;
    }
    return RET_OK;
}

/** OTA 状态标签文本（供概览卡�?OTA 页共用）�?*/
static const char *plan_ota_state_text(uint8_t state)
{
    switch (state)
    {
        case APP_LCD_PLAN_OTA_STATE_DOWNLOADING:
            return "DOWNLOADING";
        case APP_LCD_PLAN_OTA_STATE_VERIFYING:
            return "VERIFYING";
        case APP_LCD_PLAN_OTA_STATE_COMMITTING:
            return "COMMITTING";
        case APP_LCD_PLAN_OTA_STATE_REBOOTING:
            return "REBOOTING";
        case APP_LCD_PLAN_OTA_STATE_ERROR:
            return "ERROR";
        case APP_LCD_PLAN_OTA_STATE_IDLE:
        default:
            return "IDLE";
    }
}

/** OTA 状态标签颜色：进行中蓝、成功绿、错误橙、空闲灰�?*/
static uint16_t plan_ota_state_color(uint8_t state)
{
    switch (state)
    {
        case APP_LCD_PLAN_OTA_STATE_DOWNLOADING:
        case APP_LCD_PLAN_OTA_STATE_VERIFYING:
        case APP_LCD_PLAN_OTA_STATE_COMMITTING:
        case APP_LCD_PLAN_OTA_STATE_REBOOTING:
            return PLAN_COLOR_BLUE;
        case APP_LCD_PLAN_OTA_STATE_ERROR:
            return PLAN_COLOR_ORANGE;
        case APP_LCD_PLAN_OTA_STATE_IDLE:
        default:
            return PLAN_COLOR_GRAY;
    }
}

/** 槽位短名�?A"/"B"/"--"�?*/
static const char *plan_slot_text(uint8_t slot)
{
    if (slot == APP_LCD_PLAN_SLOT_A)
    {
        return "A";
    }
    if (slot == APP_LCD_PLAN_SLOT_B)
    {
        return "B";
    }
    return "--";
}

/** 槽位角色：活�?待切�?备用/未知�?*/
static const char *plan_slot_role_text(uint8_t slot)
{
    if (s_lcd_plan_ab_valid == 0U)
    {
        return "ROLE: --";
    }
    if (slot == s_lcd_plan_ab_active_slot)
    {
        return "ROLE: ACTIVE";
    }
    if (slot == s_lcd_plan_ab_pending_slot)
    {
        return "ROLE: PENDING";
    }
    return "ROLE: STANDBY";
}

static const char *plan_link_state_text(uint8_t valid, uint8_t online)
{
    if (valid == 0U)
    {
        return "WAIT";
    }

    return (online != 0U) ? "ON" : "OFF";
}

static uint16_t plan_link_state_color(uint8_t valid, uint8_t online)
{
    if (valid == 0U)
    {
        return PLAN_COLOR_ORANGE;
    }

    return (online != 0U) ? PLAN_COLOR_GREEN : PLAN_COLOR_GRAY;
}

static const char *plan_modbus_state_text(uint8_t snapshot_is_valid,
                                          uint8_t poll_was_attempted,
                                          uint8_t valid_slave_mask)
{
    if ((snapshot_is_valid == 0U) || (poll_was_attempted == 0U))
    {
        return "WAIT";
    }

    if (valid_slave_mask == 0x07U)
    {
        return "OK";
    }

    return (valid_slave_mask == 0U) ? "FAIL" : "PART";
}

static uint16_t plan_modbus_state_color(uint8_t snapshot_is_valid,
                                        uint8_t poll_was_attempted,
                                        uint8_t valid_slave_mask)
{
    if ((snapshot_is_valid != 0U) &&
        (poll_was_attempted != 0U) &&
        (valid_slave_mask == 0x07U))
    {
        return PLAN_COLOR_GREEN;
    }

    return ((snapshot_is_valid == 0U) || (poll_was_attempted == 0U)) ?
           PLAN_COLOR_GRAY : PLAN_COLOR_ORANGE;
}

static const char *plan_page_title(app_lcd_plan_page_id_t page)
{
    static const char *const titles[APP_LCD_PLAN_PAGE_COUNT] =
    {
        "HOME", "COMM", "OTA", "LOG", "SYSTEM"
    };

    if (page >= APP_LCD_PLAN_PAGE_COUNT)
    {
        return "HOME";
    }

    return titles[page];
}

/** 共用顶部品牌栏：缩放字标建立层级，时间与页面名保持小字号�?*/
static ret_t plan_draw_header(const char *page_title)
{
    ret_t ret;

    if (page_title == NULL)
    {
        return RET_NULL_PTR;
    }

    ret = dev_lcd_fill_rect(0U, 0U, 319U, 43U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text_scaled(12U,
                                   4U,
                                   "Edge",
                                   2U,
                                   PLAN_COLOR_TEXT,
                                   PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text_scaled(76U,
                                   4U,
                                   "Link",
                                   2U,
                                   PLAN_COLOR_GREEN,
                                   PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(150U,
                            13U,
                            page_title,
                            PLAN_COLOR_MUTED,
                            PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(216U,
                          13U,
                          11U,
                          s_lcd_plan_clock,
                          PLAN_COLOR_MUTED,
                          PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_fill_rect(0U, 42U, 319U, 43U, PLAN_COLOR_BORDER);
}

static ret_t plan_draw_home_header(void)
{
    const char *state_text;
    uint16_t state_color;
    uint16_t state_x;
    ret_t ret;

    if ((s_lcd_plan_network_status_valid != 0U) &&
        (s_lcd_plan_network_online == 0U))
    {
        state_text = "OFFLINE";
        state_color = PLAN_COLOR_ORANGE;
        state_x = 229U;
    }
    else if ((s_lcd_plan_network_status_valid != 0U) &&
             (s_lcd_plan_network_online != 0U) &&
             (s_lcd_plan_cloud_status_valid != 0U) &&
             (s_lcd_plan_cloud_poll_was_attempted != 0U) &&
             (s_lcd_plan_cloud_valid_slave_mask == 0x07U))
    {
        state_text = "RUNNING";
        state_color = PLAN_COLOR_GREEN;
        state_x = 229U;
    }
    else if ((s_lcd_plan_cloud_status_valid != 0U) &&
             (s_lcd_plan_cloud_poll_was_attempted != 0U) &&
             (s_lcd_plan_cloud_valid_slave_mask != 0x07U))
    {
        state_text = "CHECK";
        state_color = PLAN_COLOR_ORANGE;
        state_x = 237U;
    }
    else
    {
        state_text = "WAIT";
        state_color = PLAN_COLOR_GRAY;
        state_x = 241U;
    }

    ret = dev_lcd_fill_rect(0U, 0U, 319U, 77U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text_scaled(14U, 7U, "Edge", 2U,
                                   PLAN_COLOR_TEXT, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text_scaled(78U, 7U, "Link", 2U,
                                   PLAN_COLOR_GREEN, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(216U, 8U, 11U, s_lcd_plan_clock,
                          PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(16U, 43U, "INDUSTRIAL EDGE TERMINAL",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(16U, 59U, "GD32F303ZE | FreeRTOS | MQTT",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_fill_round_rect(210U, 31U, 307U, 57U, 10U, state_color);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(state_x, 37U, state_text,
                            PLAN_COLOR_HEADER, state_color);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_fill_rect(0U, 76U, 319U, 77U, PLAN_COLOR_BORDER);
}

static ret_t plan_draw_home_link_status(void)
{
    const char *mqtt_text;
    uint16_t mqtt_color;
    uint16_t mqtt_x;
    ret_t ret;

    if (s_lcd_plan_network_status_valid == 0U)
    {
        mqtt_text = "WAIT";
        mqtt_color = PLAN_COLOR_GRAY;
        mqtt_x = 211U;
    }
    else if (s_lcd_plan_network_online != 0U)
    {
        mqtt_text = "ONLINE";
        mqtt_color = PLAN_COLOR_GREEN;
        mqtt_x = 203U;
    }
    else
    {
        mqtt_text = "OFFLINE";
        mqtt_color = PLAN_COLOR_ORANGE;
        mqtt_x = 199U;
    }

    ret = plan_draw_card(12U, 82U, 307U, 152U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(22U, 88U, "COMMUNICATION STATUS",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(20U, 106U, 158U, 145U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(165U, 106U, 299U, 145U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(30U, 110U, "4G NETWORK",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text_scaled(30U, 126U, "N/A", 1U,
                                   PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(175U, 110U, "MQTT SERVER",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_fill_round_rect(175U, 126U, 288U, 141U, 6U, mqtt_color);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_draw_text(mqtt_x, 126U, mqtt_text,
                             PLAN_COLOR_HEADER, mqtt_color);
}

static ret_t plan_draw_home_system_info(void)
{
    ret_t ret;

    ret = plan_draw_card(12U, 158U, 307U, 238U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(22U, 166U, "SYSTEM INFO",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_fill_rect(106U, 190U, 107U, 228U, PLAN_COLOR_BORDER);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_fill_rect(201U, 190U, 202U, 228U, PLAN_COLOR_BORDER);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 190U, "FIRMWARE",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 209U, "--",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(116U, 190U, "RUN SLOT",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(116U, 209U, "--",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(210U, 190U, "EXT FLASH",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(210U, 205U, "GD25Q128",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_draw_text(210U, 220U, s_lcd_plan_flash_id,
                             s_lcd_plan_flash_status_color, PLAN_COLOR_CARD);
}

/** HOME 1/5 概览卡：OTA 会话状态与目标版本�?*/
static ret_t plan_draw_home_ota_status(void)
{
    char version_text[13U];
    char progress_text[32U];
    const char *state_text;
    uint16_t state_color;
    int length;
    ret_t ret;

    state_text = plan_ota_state_text(s_lcd_plan_ota_state);
    state_color = plan_ota_state_color(s_lcd_plan_ota_state);

    ret = plan_draw_card(12U, 244U, 307U, 326U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(22U, 252U, "OTA STATUS",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_fill_round_rect(210U, 249U, 297U, 267U, 5U, state_color);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(218U, 250U, state_text,
                            PLAN_COLOR_HEADER, state_color);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_format_version(s_lcd_plan_ota_target_version,
                              version_text,
                              (uint32_t)sizeof(version_text));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text_scaled(24U, 273U, version_text, 1U,
                                   PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    if ((s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_DOWNLOADING) &&
        (s_lcd_plan_ota_total > 0U))
    {
        length = snprintf(progress_text,
                          sizeof(progress_text),
                          "%lu%%  %lu/%lu KB",
                          (unsigned long)s_lcd_plan_ota_percent,
                          (unsigned long)(s_lcd_plan_ota_received / 1024U),
                          (unsigned long)(s_lcd_plan_ota_total / 1024U));
        if ((length < 0) || ((uint32_t)length >= sizeof(progress_text)))
        {
            return RET_OVERFLOW;
        }
    }
    else
    {
        (void)snprintf(progress_text,
                       sizeof(progress_text),
                       "TARGET VERSION");
    }

    return dev_lcd_draw_text(24U, 302U, progress_text,
                             PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
}

static ret_t plan_draw_home_runtime_info(void)
{
    const char *poll_text;
    uint16_t poll_color;
    ret_t ret;

    poll_text = plan_modbus_state_text(s_lcd_plan_cloud_status_valid,
                                       s_lcd_plan_cloud_poll_was_attempted,
                                       s_lcd_plan_cloud_valid_slave_mask);
    poll_color = plan_modbus_state_color(s_lcd_plan_cloud_status_valid,
                                         s_lcd_plan_cloud_poll_was_attempted,
                                         s_lcd_plan_cloud_valid_slave_mask);

    ret = plan_draw_card(12U, 334U, 307U, 404U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(22U, 341U, "RUNNING INFO",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(20U, 358U, 158U, 398U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(165U, 358U, 299U, 398U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(29U, 361U, "LAST REPORT",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text_scaled(29U, 378U, "--", 1U,
                                   PLAN_COLOR_TEXT, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(174U, 361U, "MODBUS POLL",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_draw_text_scaled(174U, 378U, poll_text, 1U,
                                    poll_color, PLAN_COLOR_BACKGROUND);
}

/** HOME 2/5：通信链路与最近状态�?*/
static ret_t plan_draw_home_comm_diagnostics(void)
{
    const char *mqtt_text;
    const char *poll_text;
    uint16_t mqtt_color;
    uint16_t poll_color;
    ret_t ret;

    mqtt_text = plan_link_state_text(s_lcd_plan_network_status_valid,
                                     s_lcd_plan_network_online);
    mqtt_color = plan_link_state_color(s_lcd_plan_network_status_valid,
                                       s_lcd_plan_network_online);
    poll_text = plan_modbus_state_text(s_lcd_plan_cloud_status_valid,
                                       s_lcd_plan_cloud_poll_was_attempted,
                                       s_lcd_plan_cloud_valid_slave_mask);
    poll_color = plan_modbus_state_color(s_lcd_plan_cloud_status_valid,
                                         s_lcd_plan_cloud_poll_was_attempted,
                                         s_lcd_plan_cloud_valid_slave_mask);

    ret = plan_draw_card(12U, 82U, 307U, 222U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 90U, "COMM STATUS / DIAGNOSTICS",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(20U, 111U, 158U, 214U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = plan_draw_card(165U, 111U, 299U, 214U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;

    ret = dev_lcd_draw_text(29U, 119U, "4G MODULE",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text_scaled(29U, 137U, "N/A", 1U,
                                   PLAN_COLOR_GRAY, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(29U, 165U, "SIGNAL  --",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(29U, 188U, "IP      --",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;

    ret = dev_lcd_draw_text(174U, 119U, "MQTT SERVER",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(174U, 139U, mqtt_text,
                            mqtt_color, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(174U, 165U, "ADDR    --",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(174U, 188U, "PORT    --",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 228U, 307U, 312U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 236U, "CONNECTION FLOW",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    ret = plan_fill_round_rect(22U, 260U, 97U, 294U, 7U,
                               PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = plan_fill_round_rect(120U, 260U, 199U, 294U, 7U,
                               PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = plan_fill_round_rect(222U, 260U, 299U, 294U, 7U,
                               PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(40U, 269U, "4G N/A",
                            PLAN_COLOR_GRAY, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(137U, 269U, "MQTT",
                            mqtt_color, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(236U, 269U, "REPORT",
                            poll_color, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(101U, 269U, ">",
                            PLAN_COLOR_GREEN, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(202U, 269U, ">",
                            PLAN_COLOR_GREEN, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 318U, 307U, 404U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 326U, "RECENT RESULTS",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = plan_fill_round_rect(217U, 322U, 298U, 343U, 6U,
                               PLAN_COLOR_BLUE);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(224U, 325U, "VIEW LOG",
                            PLAN_COLOR_HEADER, PLAN_COLOR_BLUE);
    if (ret != RET_OK) return ret;

    ret = dev_lcd_draw_text(26U, 355U, "MQTT",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(26U, 375U, mqtt_text,
                            mqtt_color, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(164U, 355U, "MODBUS POLL",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(164U, 375U, poll_text,
                            poll_color, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    return dev_lcd_draw_text(26U, 389U, "LAST ERROR / TIME: --",
                             PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
}

/** HOME 3/5：OTA进度；数据来自组合根投递的 OTA/A-B 状态消息�?*/
static ret_t plan_draw_home_ota_progress(void)
{
    static const uint16_t step_center_x[7U] =
    {
        31U, 74U, 117U, 160U, 203U, 246U, 289U
    };
    static const char *const step_label[7U] =
    {
        "INFO", "DATA", "CRC", "RESET", "BOOT", "TEST", "OK"
    };
    char current_version[13U];
    char target_version[13U];
    char download_text[32U];
    const char *state_text;
    uint8_t index;
    uint8_t steps_done;
    uint16_t step_x;
    uint16_t step_color;
    uint16_t bar_width;
    int length;
    ret_t ret;

    /*
     * 流程步映射：INFO/DATA 对应下载，CRC 对应全量校验�?
     * RESET 对应提交；BOOT/TEST/OK 属于 BootLoader 搬运与新固件试运行�?
     */
    switch (s_lcd_plan_ota_state)
    {
        case APP_LCD_PLAN_OTA_STATE_DOWNLOADING:
            steps_done = 2U;
            break;
        case APP_LCD_PLAN_OTA_STATE_VERIFYING:
            steps_done = 3U;
            break;
        case APP_LCD_PLAN_OTA_STATE_COMMITTING:
        case APP_LCD_PLAN_OTA_STATE_REBOOTING:
            steps_done = 4U;
            break;
        default:
            steps_done = 0U;
            break;
    }

    state_text = plan_ota_state_text(s_lcd_plan_ota_state);

    ret = plan_format_version(s_lcd_plan_ab_running_version,
                              current_version,
                              (uint32_t)sizeof(current_version));
    if (ret != RET_OK) return ret;
    ret = plan_format_version(s_lcd_plan_ota_target_version,
                              target_version,
                              (uint32_t)sizeof(target_version));
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 82U, 307U, 177U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 90U, "OTA UPDATE",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = plan_fill_round_rect(248U, 87U, 296U, 107U, 6U,
                               plan_ota_state_color(s_lcd_plan_ota_state));
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(252U, 89U, state_text,
                            PLAN_COLOR_HEADER,
                            plan_ota_state_color(s_lcd_plan_ota_state));
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(24U, 122U, "CURRENT VERSION",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(24U, 142U, current_version,
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(170U, 122U, "TARGET VERSION",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(170U, 142U, target_version,
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(24U, 160U,
                            (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_IDLE) ?
                            "OTA IDLE; WAITING FOR NOTIFY" :
                            "SESSION ACTIVE; DO NOT POWER OFF",
                            (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_ERROR) ?
                            PLAN_COLOR_ORANGE :
                            ((s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_REBOOTING) ?
                             PLAN_COLOR_BLUE : PLAN_COLOR_MUTED),
                            PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 183U, 307U, 289U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 191U, "IMAGE TRANSFER",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(24U, 216U, "DOWNLOADED",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    if (s_lcd_plan_ota_total > 0U)
    {
        length = snprintf(download_text,
                          sizeof(download_text),
                          "%lu / %lu KB",
                          (unsigned long)(s_lcd_plan_ota_received / 1024U),
                          (unsigned long)(s_lcd_plan_ota_total / 1024U));
        if ((length < 0) || ((uint32_t)length >= sizeof(download_text)))
        {
            return RET_OVERFLOW;
        }
    }
    else
    {
        (void)snprintf(download_text, sizeof(download_text), "-- / --");
    }
    ret = dev_lcd_draw_text(215U, 216U, download_text,
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    /* 进度条：外框固定，内部按百分比填充�?*/
    ret = plan_fill_round_rect(24U, 242U, 295U, 260U, 8U,
                               PLAN_COLOR_BORDER);
    if (ret != RET_OK) return ret;
    bar_width = (uint16_t)((269UL * (uint32_t)s_lcd_plan_ota_percent) / 100UL);
    if (bar_width > 0U)
    {
        ret = plan_fill_round_rect(26U, 244U,
                                   (uint16_t)(26U + bar_width - 1U), 258U,
                                   6U, PLAN_COLOR_GREEN);
        if (ret != RET_OK) return ret;
    }
    length = snprintf(download_text,
                      sizeof(download_text),
                      "PROGRESS: %lu%%",
                      (unsigned long)s_lcd_plan_ota_percent);
    if ((length < 0) || ((uint32_t)length >= sizeof(download_text)))
    {
        return RET_OVERFLOW;
    }
    ret = dev_lcd_draw_text(24U, 267U, download_text,
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 295U, 307U, 404U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 303U, "UPGRADE FLOW",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_fill_rect(31U, 340U, 289U, 342U, PLAN_COLOR_BORDER);
    if (ret != RET_OK) return ret;

    for (index = 0U; index < 7U; index++)
    {
        step_x = step_center_x[index];
        if (index < steps_done)
        {
            step_color = PLAN_COLOR_GREEN;
        }
        else if ((index == steps_done) &&
                 (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_DOWNLOADING))
        {
            step_color = PLAN_COLOR_BLUE;
        }
        else
        {
            step_color = PLAN_COLOR_GRAY;
        }

        ret = plan_fill_round_rect((uint16_t)(step_x - 10U),
                                   329U,
                                   (uint16_t)(step_x + 10U),
                                   350U,
                                   8U,
                                   step_color);
        if (ret != RET_OK) return ret;
        ret = dev_lcd_draw_text((uint16_t)(step_x - 4U),
                                332U,
                                (index == 0U) ? "1" :
                                (index == 1U) ? "2" :
                                (index == 2U) ? "3" :
                                (index == 3U) ? "4" :
                                (index == 4U) ? "5" :
                                (index == 5U) ? "6" : "7",
                                PLAN_COLOR_HEADER,
                                step_color);
        if (ret != RET_OK) return ret;
        ret = dev_lcd_draw_text((uint16_t)(step_x - 16U),
                                358U,
                                step_label[index],
                                (index <= steps_done) ?
                                PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                                PLAN_COLOR_CARD);
        if (ret != RET_OK) return ret;
    }

    return dev_lcd_draw_text(24U, 386U,
                             (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_ERROR) ?
                             "LAST SESSION FAILED; RETRY VIA CLOUD" :
                             "TRIAL RUN CONFIRMS AFTER REBOOT",
                             PLAN_COLOR_MUTED,
                             PLAN_COLOR_CARD);
}

/** HOME 4/5：A/B分区与启动状态；数据来自 fw_nvs + 启动参数快照�?*/
static ret_t plan_draw_home_ab_status(void)
{
    char version_text[13U];
    char line_text[40U];
    const char *slot_a_state;
    const char *slot_b_state;
    const char *rollback_text;
    uint16_t rollback_color;
    uint16_t slot_a_color;
    uint16_t slot_b_color;
    int length;
    ret_t ret;

    ret = plan_format_version(s_lcd_plan_ab_running_version,
                              version_text,
                              (uint32_t)sizeof(version_text));
    if (ret != RET_OK) return ret;

    /* 角色文本�?plan_slot_role_text() 依据当前快照给出�?*/
    slot_a_color = (s_lcd_plan_ab_active_slot == APP_LCD_PLAN_SLOT_A) ?
                   PLAN_COLOR_GREEN : PLAN_COLOR_MUTED;
    slot_b_color = (s_lcd_plan_ab_active_slot == APP_LCD_PLAN_SLOT_B) ?
                   PLAN_COLOR_GREEN : PLAN_COLOR_MUTED;

    if (s_lcd_plan_ab_valid == 0U)
    {
        slot_a_state = "STATE: UNKNOWN";
        slot_b_state = "STATE: UNKNOWN";
    }
    else if (s_lcd_plan_ab_pending_slot == APP_LCD_PLAN_SLOT_A)
    {
        slot_a_state = "STATE: TRIAL";
        slot_b_state = "STATE: RUNNING";
    }
    else if (s_lcd_plan_ab_pending_slot == APP_LCD_PLAN_SLOT_B)
    {
        slot_a_state = "STATE: RUNNING";
        slot_b_state = "STATE: TRIAL";
    }
    else
    {
        slot_a_state = "STATE: CONFIRMED";
        slot_b_state = "STATE: CONFIRMED";
    }

    ret = plan_draw_card(12U, 82U, 307U, 216U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 90U, "A/B PARTITION STATUS",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = plan_draw_card(20U, 114U, 158U, 207U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = plan_draw_card(165U, 114U, 299U, 207U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(31U, 124U, "SLOT A",
                            PLAN_COLOR_TEXT, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(31U, 151U, plan_slot_role_text(APP_LCD_PLAN_SLOT_A),
                            slot_a_color, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(31U, 177U, slot_a_state,
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(176U, 124U, "SLOT B",
                            PLAN_COLOR_TEXT, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(176U, 151U, plan_slot_role_text(APP_LCD_PLAN_SLOT_B),
                            slot_b_color, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(176U, 177U, slot_b_state,
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 222U, 307U, 330U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 230U, "BOOT STATUS",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    length = snprintf(line_text, sizeof(line_text),
                      "CURRENT SLOT       %s",
                      plan_slot_text(s_lcd_plan_ab_active_slot));
    if ((length < 0) || ((uint32_t)length >= sizeof(line_text)))
    {
        return RET_OVERFLOW;
    }
    ret = dev_lcd_draw_text(24U, 258U, line_text,
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    length = snprintf(line_text, sizeof(line_text),
                      "RUNNING VERSION    %s",
                      version_text);
    if ((length < 0) || ((uint32_t)length >= sizeof(line_text)))
    {
        return RET_OVERFLOW;
    }
    ret = dev_lcd_draw_text(24U, 281U, line_text,
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    rollback_text = (s_lcd_plan_ab_rollback_flag != 0U) ?
                    "ROLLBACK STATE     OCCURRED" :
                    "ROLLBACK STATE     NONE";
    rollback_color = (s_lcd_plan_ab_rollback_flag != 0U) ?
                     PLAN_COLOR_ORANGE : PLAN_COLOR_GREEN;
    ret = dev_lcd_draw_text(24U, 304U, rollback_text,
                            rollback_color, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 336U, 307U, 404U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 344U, "BOOTLOADER METADATA",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(24U, 368U,
                            (s_lcd_plan_ab_valid != 0U) ?
                            "FW NVS: VALID RECORD" :
                            "FW NVS: NO RECORD YET",
                            (s_lcd_plan_ab_valid != 0U) ?
                            PLAN_COLOR_GREEN : PLAN_COLOR_ORANGE,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    length = snprintf(line_text, sizeof(line_text),
                      "PARAM GEN          %lu",
                      (unsigned long)s_lcd_plan_ab_generation);
    if ((length < 0) || ((uint32_t)length >= sizeof(line_text)))
    {
        return RET_OVERFLOW;
    }
    return dev_lcd_draw_text(24U, 388U, line_text,
                             PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
}

/** HOME 5/5：已轮询的原始Modbus值、设备状态和故障记录服务状态�?*/
static ret_t plan_draw_home_device_log(void)
{
    char input_text[5U] = "----";
    char holding_text[5U] = "----";
    char coil_text[5U] = "----";
    char discrete_text[5U] = "----";
    const char *control_state;
    const char *analog_state;
    const char *io_state;
    uint16_t control_color;
    uint16_t analog_color;
    uint16_t io_color;
    ret_t ret;

    if ((s_lcd_plan_cloud_poll_was_attempted != 0U) &&
        ((s_lcd_plan_cloud_valid_slave_mask & 0x02U) != 0U))
    {
        ret = plan_format_u16_hex(s_lcd_plan_cloud_analog_input_register_0,
                                  input_text,
                                  (uint32_t)sizeof(input_text));
        if (ret != RET_OK) return ret;
    }
    if ((s_lcd_plan_cloud_poll_was_attempted != 0U) &&
        ((s_lcd_plan_cloud_valid_slave_mask & 0x01U) != 0U))
    {
        ret = plan_format_u16_hex(s_lcd_plan_cloud_control_holding_register_0,
                                  holding_text,
                                  (uint32_t)sizeof(holding_text));
        if (ret != RET_OK) return ret;
    }
    if ((s_lcd_plan_cloud_poll_was_attempted != 0U) &&
        ((s_lcd_plan_cloud_valid_slave_mask & 0x04U) != 0U))
    {
        ret = plan_format_u16_hex((uint16_t)s_lcd_plan_cloud_io_coil_states,
                                  coil_text,
                                  (uint32_t)sizeof(coil_text));
        if (ret != RET_OK) return ret;
        ret = plan_format_u16_hex((uint16_t)s_lcd_plan_cloud_io_discrete_input_states,
                                  discrete_text,
                                  (uint32_t)sizeof(discrete_text));
        if (ret != RET_OK) return ret;
    }

    if (s_lcd_plan_cloud_poll_was_attempted == 0U)
    {
        control_state = "WAIT";
        analog_state = "WAIT";
        io_state = "WAIT";
        control_color = PLAN_COLOR_GRAY;
        analog_color = PLAN_COLOR_GRAY;
        io_color = PLAN_COLOR_GRAY;
    }
    else
    {
        control_state = ((s_lcd_plan_cloud_valid_slave_mask & 0x01U) != 0U) ?
                        "OK" : "FAIL";
        analog_state = ((s_lcd_plan_cloud_valid_slave_mask & 0x02U) != 0U) ?
                       "OK" : "FAIL";
        io_state = ((s_lcd_plan_cloud_valid_slave_mask & 0x04U) != 0U) ?
                   "OK" : "FAIL";
        control_color = ((s_lcd_plan_cloud_valid_slave_mask & 0x01U) != 0U) ?
                        PLAN_COLOR_GREEN : PLAN_COLOR_ORANGE;
        analog_color = ((s_lcd_plan_cloud_valid_slave_mask & 0x02U) != 0U) ?
                       PLAN_COLOR_GREEN : PLAN_COLOR_ORANGE;
        io_color = ((s_lcd_plan_cloud_valid_slave_mask & 0x04U) != 0U) ?
                   PLAN_COLOR_GREEN : PLAN_COLOR_ORANGE;
    }

    ret = plan_draw_card(12U, 82U, 307U, 128U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 90U, "DEVICE OVERVIEW",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 108U, "MODBUS SLAVES 1 / 2 / 3",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 134U, 307U, 238U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 140U, "RAW MODBUS SNAPSHOT",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = plan_draw_card(20U, 160U, 158U, 195U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = plan_draw_card(165U, 160U, 299U, 195U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = plan_draw_card(20U, 199U, 158U, 234U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = plan_draw_card(165U, 199U, 299U, 234U, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;

    ret = dev_lcd_draw_text(27U, 160U, "AI0 RAW",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(27U, 177U, input_text,
                            PLAN_COLOR_TEXT, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(172U, 160U, "HR0 RAW",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(172U, 177U, holding_text,
                            PLAN_COLOR_TEXT, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(27U, 199U, "COILS RAW",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(27U, 216U, coil_text,
                            PLAN_COLOR_TEXT, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(172U, 199U, "DI RAW",
                            PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(172U, 216U, discrete_text,
                            PLAN_COLOR_TEXT, PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 244U, 307U, 277U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 253U, "SYSTEM LOAD",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(218U, 253U, "N/A",
                            PLAN_COLOR_GRAY, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 283U, 307U, 363U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 289U, "DEVICE STATUS",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(24U, 307U, "CONTROL SLAVE 1",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = plan_draw_status_tag(305U, control_state, control_color,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(24U, 326U, "ANALOG SLAVE 2",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = plan_draw_status_tag(324U, analog_state, analog_color,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(24U, 345U, "IO SLAVE 3",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = plan_draw_status_tag(343U, io_state, io_color,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK) return ret;

    ret = plan_draw_card(12U, 366U, 307U, 404U, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    ret = dev_lcd_draw_text(22U, 369U, "FAULT RECORD",
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK) return ret;
    return dev_lcd_draw_text(22U, 386U, "LOG SERVICE NOT CONNECTED",
                             PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
}

static ret_t plan_draw_home_pagination(uint8_t slide)
{
    static const uint16_t dot_center_x[5U] =
    {
        116U, 131U, 146U, 161U, 176U
    };
    static const char *const page_label[5U] =
    {
        "1/5", "2/5", "3/5", "4/5", "5/5"
    };
    uint8_t index;
    ret_t ret;

    if (slide >= APP_LCD_PLAN_HOME_SLIDE_COUNT)
    {
        return RET_INVALID_PARAM;
    }

    for (index = 0U; index < 5U; index++)
    {
        if (index == slide)
        {
            ret = plan_fill_round_rect(
                (uint16_t)(dot_center_x[index] - 4U), 412U,
                (uint16_t)(dot_center_x[index] + 3U), 419U,
                3U, PLAN_COLOR_GREEN);
        }
        else
        {
            ret = plan_fill_round_rect(
                (uint16_t)(dot_center_x[index] - 2U), 414U,
                (uint16_t)(dot_center_x[index] + 1U), 417U,
                1U, PLAN_COLOR_MUTED);
        }

        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return dev_lcd_draw_text(190U, 410U, page_label[slide],
                             PLAN_COLOR_MUTED, PLAN_COLOR_BACKGROUND);
}

/**
 * @brief   绘制 HOME 首页 1/5
 * @retval  RET_OK 成功；其他值由 LCD Device 返回
 */
static ret_t plan_page_draw_home(void)
{
    ret_t ret;

    ret = dev_lcd_clear(PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_home_header();
    if (ret != RET_OK)
    {
        return ret;
    }

    switch (s_lcd_plan_home_slide)
    {
        case APP_LCD_PLAN_HOME_SLIDE_OVERVIEW:
            /* 首屏总览：只呈现已有的MQTT、Modbus和Flash数据�?*/
            ret = plan_draw_home_link_status();
            if (ret != RET_OK) return ret;
            ret = plan_draw_home_system_info();
            if (ret != RET_OK) return ret;
            ret = plan_draw_home_ota_status();
            if (ret != RET_OK) return ret;
            ret = plan_draw_home_runtime_info();
            break;

        case APP_LCD_PLAN_HOME_SLIDE_COMM_DIAG:
            ret = plan_draw_home_comm_diagnostics();
            break;

        case APP_LCD_PLAN_HOME_SLIDE_OTA_PROGRESS:
            ret = plan_draw_home_ota_progress();
            break;

        case APP_LCD_PLAN_HOME_SLIDE_AB_STATUS:
            ret = plan_draw_home_ab_status();
            break;

        case APP_LCD_PLAN_HOME_SLIDE_DEVICE_LOG:
            ret = plan_draw_home_device_log();
            break;

        default:
            return RET_INVALID_PARAM;
    }
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_home_pagination((uint8_t)s_lcd_plan_home_slide);
    if (ret != RET_OK)
    {
        return ret;
    }

    return plan_draw_bottom_nav(APP_LCD_PLAN_PAGE_HOME);
}

/************************************************************
 * @brief   绘制五个页面共用的底部导�?
 * @note    每项�?64 像素；触摸命中边界也使用 x=64/128/192/256�?
 ************************************************************/
static ret_t plan_draw_nav_icon(uint8_t page,
                                uint16_t center_x,
                                uint16_t y,
                                uint16_t color)
{
    uint16_t half_width;
    uint16_t line_y;
    uint16_t knob_x;
    uint8_t index;
    ret_t ret;

    switch (page)
    {
        case APP_LCD_PLAN_PAGE_HOME:
            for (index = 0U; index < 5U; index++)
            {
                half_width = (uint16_t)(2U + ((uint16_t)index * 2U));
                ret = dev_lcd_fill_rect((uint16_t)(center_x - half_width),
                                        (uint16_t)(y + index),
                                        (uint16_t)(center_x + half_width),
                                        (uint16_t)(y + index), color);
                if (ret != RET_OK) return ret;
            }
            ret = dev_lcd_fill_rect((uint16_t)(center_x - 7U),
                                    (uint16_t)(y + 5U),
                                    (uint16_t)(center_x + 7U),
                                    (uint16_t)(y + 15U), color);
            if (ret != RET_OK) return ret;
            return dev_lcd_fill_rect((uint16_t)(center_x - 2U),
                                     (uint16_t)(y + 10U),
                                     (uint16_t)(center_x + 2U),
                                     (uint16_t)(y + 15U), PLAN_COLOR_FOOTER);

        case APP_LCD_PLAN_PAGE_DEVICES:
            for (index = 0U; index < 4U; index++)
            {
                uint16_t x = (uint16_t)(center_x - 8U +
                                        ((uint16_t)index * 5U));
                uint16_t height = (uint16_t)(5U + ((uint16_t)index * 3U));
                ret = dev_lcd_fill_rect(x,
                                        (uint16_t)(y + 15U - height),
                                        (uint16_t)(x + 3U),
                                        (uint16_t)(y + 15U), color);
                if (ret != RET_OK) return ret;
            }
            return RET_OK;

        case APP_LCD_PLAN_PAGE_OTA:
            ret = dev_lcd_fill_rect((uint16_t)(center_x - 1U), y,
                                    (uint16_t)(center_x + 1U),
                                    (uint16_t)(y + 11U), color);
            if (ret != RET_OK) return ret;
            ret = dev_lcd_fill_rect((uint16_t)(center_x - 5U),
                                    (uint16_t)(y + 4U),
                                    (uint16_t)(center_x + 5U),
                                    (uint16_t)(y + 6U), color);
            if (ret != RET_OK) return ret;
            ret = dev_lcd_fill_rect((uint16_t)(center_x - 5U),
                                    (uint16_t)(y + 4U),
                                    (uint16_t)(center_x - 3U),
                                    (uint16_t)(y + 8U), color);
            if (ret != RET_OK) return ret;
            ret = dev_lcd_fill_rect((uint16_t)(center_x + 3U),
                                    (uint16_t)(y + 4U),
                                    (uint16_t)(center_x + 5U),
                                    (uint16_t)(y + 8U), color);
            if (ret != RET_OK) return ret;
            return dev_lcd_fill_rect((uint16_t)(center_x - 8U),
                                     (uint16_t)(y + 12U),
                                     (uint16_t)(center_x + 8U),
                                     (uint16_t)(y + 15U), color);

        case APP_LCD_PLAN_PAGE_ALERTS:
            ret = dev_lcd_fill_rect((uint16_t)(center_x - 6U), y,
                                    (uint16_t)(center_x + 5U),
                                    (uint16_t)(y + 15U), color);
            if (ret != RET_OK) return ret;
            ret = dev_lcd_fill_rect((uint16_t)(center_x - 3U),
                                    (uint16_t)(y + 3U),
                                    (uint16_t)(center_x + 4U),
                                    (uint16_t)(y + 4U), PLAN_COLOR_FOOTER);
            if (ret != RET_OK) return ret;
            ret = dev_lcd_fill_rect((uint16_t)(center_x - 3U),
                                    (uint16_t)(y + 7U),
                                    (uint16_t)(center_x + 4U),
                                    (uint16_t)(y + 8U), PLAN_COLOR_FOOTER);
            if (ret != RET_OK) return ret;
            return dev_lcd_fill_rect((uint16_t)(center_x - 3U),
                                     (uint16_t)(y + 11U),
                                     (uint16_t)(center_x + 2U),
                                     (uint16_t)(y + 12U), PLAN_COLOR_FOOTER);

        case APP_LCD_PLAN_PAGE_SETTINGS:
            for (index = 0U; index < 3U; index++)
            {
                line_y = (uint16_t)(y + 3U + ((uint16_t)index * 5U));
                knob_x = (index & 1U) ?
                         (uint16_t)(center_x + 4U) :
                         (uint16_t)(center_x - 4U);
                ret = dev_lcd_fill_rect((uint16_t)(center_x - 9U), line_y,
                                        (uint16_t)(center_x + 9U),
                                        (uint16_t)(line_y + 1U), color);
                if (ret != RET_OK) return ret;
                ret = dev_lcd_fill_rect((uint16_t)(knob_x - 2U),
                                        (uint16_t)(line_y - 2U),
                                        (uint16_t)(knob_x + 2U),
                                        (uint16_t)(line_y + 3U), color);
                if (ret != RET_OK) return ret;
            }
            return RET_OK;

        default:
            return RET_INVALID_PARAM;
    }
}

static ret_t plan_draw_bottom_nav(app_lcd_plan_page_id_t page)
{
    static const char *const labels[APP_LCD_PLAN_PAGE_COUNT] =
    {
        "HOME", "COMM", "OTA", "LOG", "SYS"
    };
    static const uint8_t label_lengths[APP_LCD_PLAN_PAGE_COUNT] =
    {
        4U, 4U, 3U, 3U, 3U
    };
    uint16_t column_x;
    uint16_t center_x;
    uint16_t label_x;
    uint8_t index;
    ret_t ret;

    if (page >= APP_LCD_PLAN_PAGE_COUNT)
    {
        return RET_INVALID_PARAM;
    }

    ret = dev_lcd_fill_rect(0U, 430U, 319U, 479U, PLAN_COLOR_FOOTER);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_fill_rect(0U, 430U, 319U, 431U, PLAN_COLOR_BORDER);
    if (ret != RET_OK)
    {
        return ret;
    }

    for (index = 0U; index < APP_LCD_PLAN_PAGE_COUNT; index++)
    {
        column_x = (uint16_t)((uint16_t)index * 64U);
        center_x = (uint16_t)(column_x + 32U);
        label_x = (uint16_t)(column_x +
                             (uint16_t)((64U -
                             ((uint16_t)label_lengths[index] * 8U)) / 2U));

        if (index == (uint8_t)page)
        {
            ret = dev_lcd_fill_rect((uint16_t)(column_x + 19U),
                                    430U,
                                    (uint16_t)(column_x + 44U),
                                    432U,
                                    PLAN_COLOR_GREEN);
            if (ret != RET_OK)
            {
                return ret;
            }
        }

        ret = plan_draw_nav_icon(index,
                                 center_x,
                                 436U,
                                 (index == (uint8_t)page) ?
                                 PLAN_COLOR_GREEN : PLAN_COLOR_MUTED);
        if (ret != RET_OK)
        {
            return ret;
        }

        ret = dev_lcd_draw_text(label_x,
                                458U,
                                labels[index],
                                (index == (uint8_t)page) ?
                                PLAN_COLOR_GREEN : PLAN_COLOR_MUTED,
                                PLAN_COLOR_FOOTER);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    return RET_OK;
}

/**
 * @brief   绘制设备页的云端/Modbus状态卡
 * @note    寄存器和位图以十六进制原始值显示，便于和RTU报文直接核对�?
 */
static ret_t plan_draw_cloud_modbus_card(void)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    char holding_text[28U] = "HR0=---- HR1=---- HR11=----";
    char io_text[21U] = "IR0=---- CO=-- DI=--";
    const char *mqtt_text;
    const char *modbus_text;
    const char *led_text;
    uint16_t mqtt_color;
    uint16_t modbus_color;
    uint16_t led_color;
    uint8_t i;
    ret_t ret;

    if (s_lcd_plan_network_status_valid == 0U)
    {
        mqtt_text = "MQTT:WAIT";
        mqtt_color = PLAN_COLOR_GRAY;
    }
    else
    {
        mqtt_text = (s_lcd_plan_network_online != 0U) ?
                    "MQTT:ONLINE" : "MQTT:OFFLINE";
        mqtt_color = (s_lcd_plan_network_online != 0U) ?
                     PLAN_COLOR_GREEN : PLAN_COLOR_GRAY;
    }

    if ((s_lcd_plan_cloud_status_valid == 0U) ||
        (s_lcd_plan_cloud_poll_was_attempted == 0U))
    {
        modbus_text = "MB:WAIT";
    }
    else if (s_lcd_plan_cloud_valid_slave_mask == 0x07U)
    {
        modbus_text = "MB:LASTOK";
    }
    else if (s_lcd_plan_cloud_valid_slave_mask == 0U)
    {
        modbus_text = "MB:FAIL";
    }
    else
    {
        modbus_text = "MB:PART";
    }
    modbus_color = plan_modbus_state_color(s_lcd_plan_cloud_status_valid,
                                           s_lcd_plan_cloud_poll_was_attempted,
                                           s_lcd_plan_cloud_valid_slave_mask);

    if (s_lcd_plan_cloud_status_valid == 0U)
    {
        led_text = "LED2:WAIT";
        led_color = PLAN_COLOR_GRAY;
    }
    else
    {
        led_text = (s_lcd_plan_cloud_led2_is_on != 0U) ? "LED2:ON" : "LED2:OFF";
        led_color = (s_lcd_plan_cloud_led2_is_on != 0U) ?
                    PLAN_COLOR_GREEN : PLAN_COLOR_GRAY;
    }

    if ((s_lcd_plan_cloud_poll_was_attempted != 0U) &&
        ((s_lcd_plan_cloud_valid_slave_mask & 0x01U) != 0U))
    {
        for (i = 0U; i < 4U; i++)
        {
            holding_text[4U + i] =
                hex_digits[(s_lcd_plan_cloud_control_holding_register_0 >>
                            ((3U - i) * 4U)) & 0x0FU];
            holding_text[13U + i] =
                hex_digits[(s_lcd_plan_cloud_control_holding_register_1 >>
                            ((3U - i) * 4U)) & 0x0FU];
            holding_text[23U + i] =
                hex_digits[(s_lcd_plan_cloud_control_holding_register_11 >>
                            ((3U - i) * 4U)) & 0x0FU];
        }
    }

    if ((s_lcd_plan_cloud_poll_was_attempted != 0U) &&
        ((s_lcd_plan_cloud_valid_slave_mask & 0x02U) != 0U))
    {
        for (i = 0U; i < 4U; i++)
        {
            io_text[4U + i] =
                hex_digits[(s_lcd_plan_cloud_analog_input_register_0 >>
                            ((3U - i) * 4U)) & 0x0FU];
        }
    }
    if ((s_lcd_plan_cloud_poll_was_attempted != 0U) &&
        ((s_lcd_plan_cloud_valid_slave_mask & 0x04U) != 0U))
    {
        io_text[12U] = hex_digits[(s_lcd_plan_cloud_io_coil_states >> 4U) & 0x0FU];
        io_text[13U] = hex_digits[s_lcd_plan_cloud_io_coil_states & 0x0FU];
        io_text[18U] =
            hex_digits[(s_lcd_plan_cloud_io_discrete_input_states >> 4U) & 0x0FU];
        io_text[19U] =
            hex_digits[s_lcd_plan_cloud_io_discrete_input_states & 0x0FU];
    }

    ret = plan_draw_card(12U, 322U, 307U, 420U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U,
                            329U,
                            "CLOUD / MODBUS",
                            PLAN_COLOR_TEXT,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(224U,
                            329U,
                            led_text,
                            led_color,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U,
                            349U,
                            mqtt_text,
                            mqtt_color,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(144U,
                            349U,
                            modbus_text,
                            modbus_color,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(24U,
                          367U,
                          27U,
                          holding_text,
                          PLAN_COLOR_TEXT,
                          PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(24U,
                          385U,
                          20U,
                          io_text,
                          PLAN_COLOR_TEXT,
                          PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_fill_round_rect(218U,
                               382U,
                               296U,
                               414U,
                               5U,
                               PLAN_COLOR_BLUE);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_draw_text(233U,
                             390U,
                             "TOGGLE",
                             PLAN_COLOR_HEADER,
                             PLAN_COLOR_BLUE);
}

/************************************************************
 * @brief   绘制 DEVICES 页面
 ************************************************************/
static ret_t plan_page_draw_devices(void)
{
    const char *touch_status_text;
    const char *network_detail;
    const char *network_status_text;
    uint16_t touch_status_color;
    uint16_t network_status_color;
    ret_t ret;

    if (s_lcd_plan_touch_id[0U] == 'W')
    {
        touch_status_text = "WAIT";
        touch_status_color = PLAN_COLOR_GRAY;
    }
    else if (s_lcd_plan_touch_id[0U] == 'F')
    {
        touch_status_text = "FAIL";
        touch_status_color = PLAN_COLOR_ORANGE;
    }
    else
    {
        touch_status_text = "READY";
        touch_status_color = PLAN_COLOR_GREEN;
    }

    if (s_lcd_plan_network_status_valid == 0U)
    {
        network_detail = "4G: N/A  MQTT: WAIT";
        network_status_text = "WAIT";
        network_status_color = PLAN_COLOR_GRAY;
    }
    else if (s_lcd_plan_network_online != 0U)
    {
        network_detail = "4G: N/A  MQTT: ONLINE";
        network_status_text = "ON";
        network_status_color = PLAN_COLOR_GREEN;
    }
    else
    {
        network_detail = "4G: N/A  MQTT: OFFLINE";
        network_status_text = "OFF";
        network_status_color = PLAN_COLOR_GRAY;
    }

    ret = dev_lcd_clear(PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_header(plan_page_title(APP_LCD_PLAN_PAGE_DEVICES));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 56U, 307U, 132U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 70U, "LCD PANEL", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 94U, "ILI9488 320X480", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_status_tag(88U,
                               "READY",
                               PLAN_COLOR_GREEN,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 146U, 307U, 222U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 160U, "GT911 TOUCH", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U,
                            184U,
                            "PRODUCT ID",
                            PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(112U,
                          184U,
                          DEV_GT911_PRODUCT_ID_LENGTH,
                          s_lcd_plan_touch_id,
                          s_lcd_plan_touch_id_color,
                          PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_status_tag(178U,
                               touch_status_text,
                               touch_status_color,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 236U, 307U, 312U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 250U, "4G / MQTT LINK", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(24U,
                          274U,
                          27U,
                          network_detail,
                          PLAN_COLOR_MUTED,
                          PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_status_tag(268U,
                               network_status_text,
                               network_status_color,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_cloud_modbus_card();
    if (ret != RET_OK)
    {
        return ret;
    }

    return plan_draw_bottom_nav(APP_LCD_PLAN_PAGE_DEVICES);
}

static ret_t plan_draw_link_status(void)
{
    const char *mqtt_text;
    const char *modbus_text;
    uint16_t mqtt_color;
    uint16_t modbus_color;
    ret_t ret;

    ret = plan_draw_card(12U, 292U, 307U, 390U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U,
                            304U,
                            "CLOUD / MODBUS STATUS",
                            PLAN_COLOR_TEXT,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    mqtt_text = plan_link_state_text(s_lcd_plan_network_status_valid,
                                    s_lcd_plan_network_online);
    mqtt_color = plan_link_state_color(s_lcd_plan_network_status_valid,
                                       s_lcd_plan_network_online);
    ret = dev_lcd_draw_text(24U,
                            326U,
                            "MQTT LINK",
                            PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_status_tag(322U,
                               mqtt_text,
                               mqtt_color,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    modbus_text = plan_modbus_state_text(s_lcd_plan_cloud_status_valid,
                                         s_lcd_plan_cloud_poll_was_attempted,
                                         s_lcd_plan_cloud_valid_slave_mask);
    modbus_color = plan_modbus_state_color(s_lcd_plan_cloud_status_valid,
                                           s_lcd_plan_cloud_poll_was_attempted,
                                           s_lcd_plan_cloud_valid_slave_mask);
    ret = dev_lcd_draw_text(24U,
                            350U,
                            "MODBUS RTU",
                            PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }
    ret = plan_draw_status_tag(346U,
                               modbus_text,
                               modbus_color,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    return dev_lcd_draw_text(24U,
                             372U,
                             "REMOTE CONTROL + TELEMETRY",
                             PLAN_COLOR_MUTED,
                             PLAN_COLOR_CARD);
}

/************************************************************
 * @brief   绘制 ALERTS 页面
 ************************************************************/
static ret_t plan_page_draw_alerts(void)
{
    ret_t ret;

    ret = dev_lcd_clear(PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_header(plan_page_title(APP_LCD_PLAN_PAGE_ALERTS));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 56U, 307U, 154U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 72U, "FAULT RECORD", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 104U, "FLASH TEST", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_status_tag(112U,
                               s_lcd_plan_flash_test_status,
                               s_lcd_plan_flash_test_status_color,
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 174U, 307U, 272U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 190U, "LOG STORAGE", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 222U, "LOG SERVICE NOT CONNECTED", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_link_status();
    if (ret != RET_OK)
    {
        return ret;
    }

    return plan_draw_bottom_nav(APP_LCD_PLAN_PAGE_ALERTS);
}

/************************************************************
 * @brief   绘制 OTA / A-B 分区页面
 * @note    OTA 进度、目标版本、A/B 槽位与回滚标志均由组合根经消�?
 *          投递；本页只读快照绘制，不直接访问 OTA �?Flash 模块�?
 ************************************************************/
static ret_t plan_page_draw_ota(void)
{
    char current_version[13U];
    char target_version[13U];
    char line_text[40U];
    const char *state_text;
    uint16_t state_color;
    uint16_t bar_width;
    ret_t ret;
    uint8_t step;
    uint16_t step_x;
    uint8_t steps_done;
    uint16_t step_color;
    int length;

    switch (s_lcd_plan_ota_state)
    {
        case APP_LCD_PLAN_OTA_STATE_DOWNLOADING:
            steps_done = 2U;
            break;
        case APP_LCD_PLAN_OTA_STATE_VERIFYING:
            steps_done = 3U;
            break;
        case APP_LCD_PLAN_OTA_STATE_COMMITTING:
        case APP_LCD_PLAN_OTA_STATE_REBOOTING:
            steps_done = 4U;
            break;
        default:
            steps_done = 0U;
            break;
    }

    state_text = plan_ota_state_text(s_lcd_plan_ota_state);
    state_color = plan_ota_state_color(s_lcd_plan_ota_state);

    ret = plan_format_version(s_lcd_plan_ab_running_version,
                              current_version,
                              (uint32_t)sizeof(current_version));
    if (ret != RET_OK) return ret;
    ret = plan_format_version(s_lcd_plan_ota_target_version,
                              target_version,
                              (uint32_t)sizeof(target_version));
    if (ret != RET_OK) return ret;

    ret = dev_lcd_clear(PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_header(plan_page_title(APP_LCD_PLAN_PAGE_OTA));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 50U, 307U, 157U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 60U, "OTA UPDATE", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_status_tag(56U, state_text, state_color, PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    length = snprintf(line_text, sizeof(line_text),
                      "CURRENT VERSION: %s", current_version);
    if ((length < 0) || ((uint32_t)length >= sizeof(line_text)))
    {
        return RET_OVERFLOW;
    }
    ret = dev_lcd_draw_text(24U, 88U, line_text,
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    length = snprintf(line_text, sizeof(line_text),
                      "TARGET VERSION: %s", target_version);
    if ((length < 0) || ((uint32_t)length >= sizeof(line_text)))
    {
        return RET_OVERFLOW;
    }
    ret = dev_lcd_draw_text(24U, 116U, line_text,
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 下载中显示实时进度，其余状态显示提示�?*/
    if ((s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_DOWNLOADING) &&
        (s_lcd_plan_ota_total > 0U))
    {
        length = snprintf(line_text, sizeof(line_text),
                          "%lu%%  %lu/%lu KB",
                          (unsigned long)s_lcd_plan_ota_percent,
                          (unsigned long)(s_lcd_plan_ota_received / 1024U),
                          (unsigned long)(s_lcd_plan_ota_total / 1024U));
        if ((length < 0) || ((uint32_t)length >= sizeof(line_text)))
        {
            return RET_OVERFLOW;
        }
    }
    else
    {
        if (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_REBOOTING)
        {
            (void)snprintf(line_text, sizeof(line_text), "REBOOTING TO BOOTLOADER");
        }
        else
        {
            (void)snprintf(line_text, sizeof(line_text),
                           (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_ERROR) ?
                           "LAST SESSION FAILED" : "WAITING FOR CLOUD NOTIFY");
        }
    }
    ret = dev_lcd_draw_text(24U, 136U, line_text,
                            (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_ERROR) ?
                            PLAN_COLOR_ORANGE :
                            ((s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_REBOOTING) ?
                             PLAN_COLOR_BLUE : PLAN_COLOR_MUTED),
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 167U, 307U, 277U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 177U, "A/B PARTITION", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 203U, "ACTIVE SLOT",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(224U, 203U,
                            plan_slot_text(s_lcd_plan_ab_active_slot),
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 229U, "PENDING SLOT",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(224U, 229U,
                            plan_slot_text(s_lcd_plan_ab_pending_slot),
                            PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 255U, "ROLLBACK",
                            PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(224U, 255U,
                            (s_lcd_plan_ab_rollback_flag != 0U) ?
                            "OCCURRED" : "NONE",
                            (s_lcd_plan_ab_rollback_flag != 0U) ?
                            PLAN_COLOR_ORANGE : PLAN_COLOR_GREEN,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 287U, 307U, 420U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 297U, "UPGRADE FLOW", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 进度条：外框固定，内部按百分比填充�?*/
    ret = plan_fill_round_rect(24U, 316U, 295U, 334U, 8U, PLAN_COLOR_BORDER);
    if (ret != RET_OK)
    {
        return ret;
    }
    bar_width = (uint16_t)((269UL * (uint32_t)s_lcd_plan_ota_percent) / 100UL);
    if (bar_width > 0U)
    {
        ret = plan_fill_round_rect(26U, 318U,
                                   (uint16_t)(26U + bar_width - 1U), 332U,
                                   6U, PLAN_COLOR_GREEN);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    ret = dev_lcd_fill_rect(43U, 342U, 277U, 344U, PLAN_COLOR_BORDER);
    if (ret != RET_OK)
    {
        return ret;
    }

    for (step = 0U; step < 4U; step++)
    {
        step_x = (uint16_t)(34U + ((uint16_t)step * 72U));
        if (step < steps_done)
        {
            step_color = PLAN_COLOR_GREEN;
        }
        else if ((step == steps_done) &&
                 (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_DOWNLOADING))
        {
            step_color = PLAN_COLOR_BLUE;
        }
        else
        {
            step_color = PLAN_COLOR_GRAY;
        }

        ret = plan_fill_round_rect(step_x,
                                   331U,
                                   (uint16_t)(step_x + 22U),
                                   355U,
                                   8U,
                                   step_color);
        if (ret != RET_OK)
        {
            return ret;
        }

        ret = dev_lcd_draw_text((uint16_t)(step_x + 7U),
                                335U,
                                (step == 0U) ? "1" :
                                (step == 1U) ? "2" :
                                (step == 2U) ? "3" : "4",
                                PLAN_COLOR_HEADER,
                                step_color);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    ret = dev_lcd_draw_text(28U, 369U, "INFO",
                            (steps_done >= 1U) ? PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(100U, 369U, "DATA",
                            (steps_done >= 2U) ? PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(172U, 369U, "CHECK",
                            (steps_done >= 3U) ? PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(244U, 369U, "BOOT",
                            (steps_done >= 4U) ? PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U,
                            397U,
                            "BOOTLOADER COPIES ON NEXT RESET",
                            PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    return plan_draw_bottom_nav(APP_LCD_PLAN_PAGE_OTA);
}

/**
 * @brief 只重�?OTA 页的动态区域（状态标�?/ 百分比行 / 进度�?/ 流程步）
 * @note  OTA 下载期间每秒都有进度更新，整�?clear+重画会持续闪屏；
 *        这里只刷新会变化的区域，静态卡片和文字保持不动�?
 */
static ret_t plan_draw_ota_dynamic(void)
{
    char line_text[40U];
    const char *state_text;
    uint16_t state_color;
    uint16_t bar_width;
    uint16_t step_color;
    uint16_t step_x;
    uint8_t steps_done;
    uint8_t step;
    int length;
    ret_t ret;

    switch (s_lcd_plan_ota_state)
    {
        case APP_LCD_PLAN_OTA_STATE_DOWNLOADING:
            steps_done = 2U;
            break;
        case APP_LCD_PLAN_OTA_STATE_VERIFYING:
            steps_done = 3U;
            break;
        case APP_LCD_PLAN_OTA_STATE_COMMITTING:
        case APP_LCD_PLAN_OTA_STATE_REBOOTING:
            steps_done = 4U;
            break;
        default:
            steps_done = 0U;
            break;
    }

    state_text = plan_ota_state_text(s_lcd_plan_ota_state);
    state_color = plan_ota_state_color(s_lcd_plan_ota_state);

    ret = plan_draw_status_tag(56U, state_text, state_color, PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 百分�?/ 已下载字节行（与整页绘制保持同一位置与文案）�?*/
    if ((s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_DOWNLOADING) &&
        (s_lcd_plan_ota_total > 0U))
    {
        length = snprintf(line_text, sizeof(line_text),
                          "%lu%%  %lu/%lu KB",
                          (unsigned long)s_lcd_plan_ota_percent,
                          (unsigned long)(s_lcd_plan_ota_received / 1024U),
                          (unsigned long)(s_lcd_plan_ota_total / 1024U));
        if ((length < 0) || ((uint32_t)length >= sizeof(line_text)))
        {
            return RET_OVERFLOW;
        }
    }
    else
    {
        if (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_REBOOTING)
        {
            (void)snprintf(line_text, sizeof(line_text), "REBOOTING TO BOOTLOADER");
        }
        else
        {
            (void)snprintf(line_text, sizeof(line_text),
                           (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_ERROR) ?
                           "LAST SESSION FAILED" : "WAITING FOR CLOUD NOTIFY");
        }
    }
    ret = dev_lcd_draw_text(24U, 136U, line_text,
                            (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_ERROR) ?
                            PLAN_COLOR_ORANGE :
                            ((s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_REBOOTING) ?
                             PLAN_COLOR_BLUE : PLAN_COLOR_MUTED),
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 进度条：外框固定，内部按百分比填充�?*/
    ret = plan_fill_round_rect(24U, 316U, 295U, 334U, 8U, PLAN_COLOR_BORDER);
    if (ret != RET_OK)
    {
        return ret;
    }
    bar_width = (uint16_t)((269UL * (uint32_t)s_lcd_plan_ota_percent) / 100UL);
    if (bar_width > 0U)
    {
        ret = plan_fill_round_rect(26U, 318U,
                                   (uint16_t)(26U + bar_width - 1U), 332U,
                                   6U, PLAN_COLOR_GREEN);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    /* 四个流程步与标签 */
    for (step = 0U; step < 4U; step++)
    {
        step_x = (uint16_t)(34U + ((uint16_t)step * 72U));
        if (step < steps_done)
        {
            step_color = PLAN_COLOR_GREEN;
        }
        else if ((step == steps_done) &&
                 (s_lcd_plan_ota_state == APP_LCD_PLAN_OTA_STATE_DOWNLOADING))
        {
            step_color = PLAN_COLOR_BLUE;
        }
        else
        {
            step_color = PLAN_COLOR_GRAY;
        }

        ret = plan_fill_round_rect(step_x, 331U,
                                   (uint16_t)(step_x + 22U), 355U,
                                   8U, step_color);
        if (ret != RET_OK)
        {
            return ret;
        }
        ret = dev_lcd_draw_text((uint16_t)(step_x + 7U), 335U,
                                (step == 0U) ? "1" :
                                (step == 1U) ? "2" :
                                (step == 2U) ? "3" : "4",
                                PLAN_COLOR_HEADER, step_color);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    ret = dev_lcd_draw_text(28U, 369U, "INFO",
                            (steps_done >= 1U) ? PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }
    ret = dev_lcd_draw_text(100U, 369U, "DATA",
                            (steps_done >= 2U) ? PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }
    ret = dev_lcd_draw_text(172U, 369U, "CHECK",
                            (steps_done >= 3U) ? PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                            PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }
    return dev_lcd_draw_text(244U, 369U, "BOOT",
                             (steps_done >= 4U) ? PLAN_COLOR_TEXT : PLAN_COLOR_MUTED,
                             PLAN_COLOR_CARD);
}

/************************************************************
 * @brief   绘制系统设置 / 设备信息页面
 ************************************************************/
static ret_t plan_page_draw_settings(void)
{
    ret_t ret;

    ret = dev_lcd_clear(PLAN_COLOR_BACKGROUND);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_header(plan_page_title(APP_LCD_PLAN_PAGE_SETTINGS));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 50U, 307U, 177U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 60U, "SYSTEM INFO", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 86U, "MCU", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(128U, 86U, "GD32F303ZE", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 112U, "RTOS", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(128U, 112U, "FreeRTOS", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 138U, "LCD", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(128U, 138U, "ILI9488 320X480", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 187U, 307U, 308U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 197U, "DEVICE IDs", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 223U, "GT911", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(112U,
                          223U,
                          DEV_GT911_PRODUCT_ID_LENGTH,
                          s_lcd_plan_touch_id,
                          s_lcd_plan_touch_id_color,
                          PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 249U, "GD25Q128", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_value(112U,
                          249U,
                          9U,
                          s_lcd_plan_flash_id,
                          PLAN_COLOR_TEXT,
                          PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_card(12U, 318U, 307U, 420U, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 328U, "LINK STATUS", PLAN_COLOR_TEXT, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 354U, "MQTT", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_status_tag(350U,
                               plan_link_state_text(s_lcd_plan_network_status_valid,
                                                    s_lcd_plan_network_online),
                               plan_link_state_color(s_lcd_plan_network_status_valid,
                                                     s_lcd_plan_network_online),
                               PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = dev_lcd_draw_text(24U, 382U, "MODBUS RTU", PLAN_COLOR_MUTED, PLAN_COLOR_CARD);
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = plan_draw_status_tag(
        378U,
        plan_modbus_state_text(s_lcd_plan_cloud_status_valid,
                                s_lcd_plan_cloud_poll_was_attempted,
                                s_lcd_plan_cloud_valid_slave_mask),
        plan_modbus_state_color(s_lcd_plan_cloud_status_valid,
                                s_lcd_plan_cloud_poll_was_attempted,
                                s_lcd_plan_cloud_valid_slave_mask),
        PLAN_COLOR_HEADER);
    if (ret != RET_OK)
    {
        return ret;
    }

    return plan_draw_bottom_nav(APP_LCD_PLAN_PAGE_SETTINGS);
}

/************************************************************
 * @brief   根据页面 ID 分发绘制函数
 ************************************************************/
static ret_t plan_page_draw(app_lcd_plan_page_id_t page)
{
    ret_t ret;

    switch (page)
    {
        case APP_LCD_PLAN_PAGE_HOME:
            ret = plan_page_draw_home();
            break;

        case APP_LCD_PLAN_PAGE_DEVICES:
            ret = plan_page_draw_devices();
            break;

        case APP_LCD_PLAN_PAGE_OTA:
            ret = plan_page_draw_ota();
            break;

        case APP_LCD_PLAN_PAGE_ALERTS:
            ret = plan_page_draw_alerts();
            break;

        case APP_LCD_PLAN_PAGE_SETTINGS:
            ret = plan_page_draw_settings();
            break;

        default:
            return RET_INVALID_PARAM;
    }

    if (ret == RET_OK)
    {
        s_lcd_plan_current_page = page;
    }

    return ret;
}

/**
 * @brief   在 Display Task 内处理原始触摸事件（命中判定与页面动作）
 * @param   message Touch Task 投递的按下/抬起事件
 * @retval  RET_OK 成功；RET_INVALID_PARAM 事件非法；其他为页面绘制错误
 *
 * @note    页面状态（当前页、HOME 轮播索引、按下起点）全部由 Display Task
 *          拥有；Touch Task 不再读取这些状态，避免跨任务读写导致误命中。
 */
static ret_t plan_handle_touch_event(const app_lcd_plan_msg_t *message)
{
    uint16_t x;
    uint16_t y;
    uint16_t delta_x;
    uint8_t home_slide;
    uint8_t dot_index;
    ret_t ret;

    if (message->value.touch.event == APP_LCD_PLAN_TOUCH_EVENT_DOWN)
    {
        x = message->value.touch.x;
        y = message->value.touch.y;
        s_lcd_plan_touch_start_x = x;
        s_lcd_plan_touch_start_y = y;
        s_lcd_plan_touch_active = 1U;

        /* 底部导航命中判定：每项 64 像素，与 plan_draw_bottom_nav() 分栏一致 */
        if ((x < 320U) && (y >= 432U) && (y < 480U))
        {
            app_lcd_plan_page_id_t page =
                (app_lcd_plan_page_id_t)(x / 64U);

            if (page >= APP_LCD_PLAN_PAGE_COUNT)
            {
                return RET_INVALID_PARAM;
            }
            if (page == s_lcd_plan_current_page)
            {
                return RET_OK;
            }

            ret = plan_page_draw(page);
            DBG_log("TOUCH page request=%lu x=%u y=%u ret=%ld\r\n",
                    (unsigned long)page,
                    (unsigned int)x,
                    (unsigned int)y,
                    (long)ret);
            return ret;
        }

        /* HOME 轮播圆点：与 plan_draw_home_pagination() 的圆心保持一致 */
        if ((s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME) &&
            (y >= 408U) && (y < 427U) &&
            (x >= 110U) && (x <= 182U))
        {
            static const uint16_t dot_center_x[5U] =
            {
                116U, 131U, 146U, 161U, 176U
            };

            for (dot_index = 0U; dot_index < 5U; dot_index++)
            {
                delta_x = (x >= dot_center_x[dot_index]) ?
                          (uint16_t)(x - dot_center_x[dot_index]) :
                          (uint16_t)(dot_center_x[dot_index] - x);
                if (delta_x <= 5U)
                {
                    if (dot_index == (uint8_t)s_lcd_plan_home_slide)
                    {
                        return RET_OK;
                    }
                    s_lcd_plan_home_slide =
                        (app_lcd_plan_home_slide_t)dot_index;
                    DBG_log("TOUCH home dot=%u\r\n",
                            (unsigned int)dot_index);
                    return plan_page_draw_home();
                }
            }
            return RET_OK;
        }

        /* 通信诊断页"查看设备日志"入口 */
        if ((s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME) &&
            (s_lcd_plan_home_slide == APP_LCD_PLAN_HOME_SLIDE_COMM_DIAG) &&
            (x >= 217U) && (x <= 298U) &&
            (y >= 322U) && (y <= 343U))
        {
            DBG_log("TOUCH open log page\r\n");
            return plan_page_draw(APP_LCD_PLAN_PAGE_ALERTS);
        }

        /* 设备页 LED2 按钮：回调由组合根注入，运行在 Display Task 上下文 */
        if ((s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_DEVICES) &&
            (x >= 218U) && (x <= 296U) &&
            (y >= 382U) && (y <= 414U))
        {
            if (s_lcd_plan_led_toggle_handler != NULL)
            {
                ret = s_lcd_plan_led_toggle_handler(
                    s_lcd_plan_led_toggle_context);
            }
            else
            {
                ret = RET_NOT_SUPPORT;
            }
            DBG_log("TOUCH LED2 toggle request ret=%ld\r\n", (long)ret);
            return RET_OK;
        }

        return RET_OK;
    }

    /* UP：用按下起点与最后坐标判定 HOME 内容区左右滑动 */
    if (s_lcd_plan_touch_active == 0U)
    {
        return RET_OK;
    }
    s_lcd_plan_touch_active = 0U;

    if ((s_lcd_plan_current_page != APP_LCD_PLAN_PAGE_HOME) ||
        (s_lcd_plan_touch_start_y < 78U) ||
        (s_lcd_plan_touch_start_y >= 430U))
    {
        return RET_OK;
    }

    x = message->value.touch.x;
    delta_x = (x >= s_lcd_plan_touch_start_x) ?
              (uint16_t)(x - s_lcd_plan_touch_start_x) :
              (uint16_t)(s_lcd_plan_touch_start_x - x);
    if (delta_x < 40U)
    {
        return RET_OK;
    }

    home_slide = (uint8_t)s_lcd_plan_home_slide;
    if ((x < s_lcd_plan_touch_start_x) &&
        (home_slide + 1U < APP_LCD_PLAN_HOME_SLIDE_COUNT))
    {
        home_slide++;
    }
    else if ((x > s_lcd_plan_touch_start_x) && (home_slide > 0U))
    {
        home_slide--;
    }

    if (home_slide == (uint8_t)s_lcd_plan_home_slide)
    {
        return RET_OK;
    }

    s_lcd_plan_home_slide = (app_lcd_plan_home_slide_t)home_slide;
    DBG_log("TOUCH home slide=%u\r\n", (unsigned int)home_slide);
    return plan_page_draw_home();
}

/**
 * @brief   把界面快照的变化分区应用到页面（最新快照 + 变化标记）
 * @retval  RET_OK 成功；其他为页面绘制错误
 *
 * @note    只由 Display Task 调用。分区计数与本地已应用计数比对，
 *          无变化的分区不做任何重绘，避免旧值覆盖新值或重复刷屏。
 */
static ret_t plan_apply_snapshot(void)
{
    app_ui_snapshot_t snapshot;
    app_ui_snapshot_change_t changes;

    app_ui_snapshot_get(&snapshot, &changes);

    if (changes.network != s_lcd_plan_seen_changes.network)
    {
        s_lcd_plan_seen_changes.network = changes.network;
        s_lcd_plan_network_online = snapshot.network_online;
        s_lcd_plan_network_status_valid = snapshot.network_valid;

        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME)
        {
            return plan_page_draw_home();
        }
        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_DEVICES)
        {
            return plan_page_draw(APP_LCD_PLAN_PAGE_DEVICES);
        }
        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_ALERTS)
        {
            return plan_draw_link_status();
        }
        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_SETTINGS)
        {
            return plan_draw_status_tag(
                350U,
                plan_link_state_text(s_lcd_plan_network_status_valid,
                                     s_lcd_plan_network_online),
                plan_link_state_color(s_lcd_plan_network_status_valid,
                                      s_lcd_plan_network_online),
                PLAN_COLOR_HEADER);
        }
    }

    if (changes.cloud != s_lcd_plan_seen_changes.cloud)
    {
        s_lcd_plan_seen_changes.cloud = changes.cloud;
        s_lcd_plan_cloud_status_valid = snapshot.cloud_valid;
        s_lcd_plan_cloud_valid_slave_mask = snapshot.cloud.valid_slave_mask;
        s_lcd_plan_cloud_poll_was_attempted =
            snapshot.cloud.poll_was_attempted;
        s_lcd_plan_cloud_led2_is_on = snapshot.cloud.led2_is_on;
        s_lcd_plan_cloud_control_holding_register_0 =
            snapshot.cloud.control_holding_register_0;
        s_lcd_plan_cloud_control_holding_register_1 =
            snapshot.cloud.control_holding_register_1;
        s_lcd_plan_cloud_control_holding_register_11 =
            snapshot.cloud.control_status_register_1;
        s_lcd_plan_cloud_analog_input_register_0 =
            snapshot.cloud.analog_input_register_0;
        s_lcd_plan_cloud_io_coil_states = snapshot.cloud.io_coil_states;
        s_lcd_plan_cloud_io_discrete_input_states =
            snapshot.cloud.io_discrete_input_states;

        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_DEVICES)
        {
            return plan_draw_cloud_modbus_card();
        }
        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME)
        {
            return plan_page_draw_home();
        }
        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_ALERTS)
        {
            return plan_draw_link_status();
        }
        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_SETTINGS)
        {
            return plan_draw_status_tag(
                378U,
                plan_modbus_state_text(s_lcd_plan_cloud_status_valid,
                                       s_lcd_plan_cloud_poll_was_attempted,
                                       s_lcd_plan_cloud_valid_slave_mask),
                plan_modbus_state_color(s_lcd_plan_cloud_status_valid,
                                        s_lcd_plan_cloud_poll_was_attempted,
                                        s_lcd_plan_cloud_valid_slave_mask),
                PLAN_COLOR_HEADER);
        }
    }

    if (changes.time != s_lcd_plan_seen_changes.time)
    {
        s_lcd_plan_seen_changes.time = changes.time;
        if ((snapshot.time_valid != 0U) &&
            (snapshot.time_text[4U] == '-') &&
            (snapshot.time_text[7U] == '-') &&
            (snapshot.time_text[10U] == ' ') &&
            (snapshot.time_text[13U] == ':') &&
            (snapshot.time_text[19U] == '\0'))
        {
            s_lcd_plan_clock[0U] = snapshot.time_text[5U];
            s_lcd_plan_clock[1U] = snapshot.time_text[6U];
            s_lcd_plan_clock[2U] = '-';
            s_lcd_plan_clock[3U] = snapshot.time_text[8U];
            s_lcd_plan_clock[4U] = snapshot.time_text[9U];
            s_lcd_plan_clock[5U] = ' ';
            s_lcd_plan_clock[6U] = snapshot.time_text[11U];
            s_lcd_plan_clock[7U] = snapshot.time_text[12U];
            s_lcd_plan_clock[8U] = ':';
            s_lcd_plan_clock[9U] = snapshot.time_text[14U];
            s_lcd_plan_clock[10U] = snapshot.time_text[15U];
            s_lcd_plan_clock[11U] = '\0';

            if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME)
            {
                return plan_page_draw_home();
            }
            return plan_page_draw(s_lcd_plan_current_page);
        }
    }

    if (changes.ota != s_lcd_plan_seen_changes.ota)
    {
        uint8_t previous_state = s_lcd_plan_ota_state;

        s_lcd_plan_seen_changes.ota = changes.ota;
        s_lcd_plan_ota_state = snapshot.ota.state;
        s_lcd_plan_ota_percent = snapshot.ota.percent;
        if (s_lcd_plan_ota_percent > 100U)
        {
            s_lcd_plan_ota_percent = 100U;
        }
        s_lcd_plan_ota_received = snapshot.ota.received;
        s_lcd_plan_ota_total = snapshot.ota.total;
        s_lcd_plan_ota_target_version = snapshot.ota.target_version;

        /* 会话开始：自动跳到 OTA 页；下载中只重画动态区，避免整页闪屏。 */
        if ((previous_state == APP_LCD_PLAN_OTA_STATE_IDLE) &&
            (s_lcd_plan_ota_state != APP_LCD_PLAN_OTA_STATE_IDLE) &&
            (s_lcd_plan_current_page != APP_LCD_PLAN_PAGE_OTA))
        {
            return plan_page_draw(APP_LCD_PLAN_PAGE_OTA);
        }
        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_OTA)
        {
            return plan_draw_ota_dynamic();
        }
        if ((s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME) &&
            (s_lcd_plan_home_slide == APP_LCD_PLAN_HOME_SLIDE_OVERVIEW))
        {
            return plan_draw_home_ota_status();
        }
    }

    if (changes.ab != s_lcd_plan_seen_changes.ab)
    {
        s_lcd_plan_seen_changes.ab = changes.ab;
        s_lcd_plan_ab_valid = snapshot.ab.valid;
        s_lcd_plan_ab_active_slot = snapshot.ab.active_slot;
        s_lcd_plan_ab_pending_slot = snapshot.ab.pending_slot;
        s_lcd_plan_ab_rollback_flag = snapshot.ab.rollback_flag;
        s_lcd_plan_ab_running_version = snapshot.ab.running_version;
        s_lcd_plan_ab_generation = snapshot.ab.generation;

        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME)
        {
            return plan_page_draw_home();
        }
        if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_OTA)
        {
            return plan_page_draw_ota();
        }
    }

    return RET_OK;
}

/**
 * @brief   将一条语义消息转换为页面局部刷�?
 * @param   message Display Task从队列复制出来的消息
 * @retval  RET_OK 成功；RET_NULL_PTR 空指针；RET_INVALID_PARAM 字段非法�?
 *          其他值表示格式化或LCD刷新失败
 *
 * @note    只有Display Task调用本函数，因此所有dev_lcd_*调用都在同一
 *          个任务上下文中串行执行，生产者不直接触碰LCD设备�?
 */
static ret_t plan_apply_message(const app_lcd_plan_msg_t *message)
{
    char touch_id_text[DEV_GT911_PRODUCT_ID_TEXT_SIZE];
    char flash_id_text[10U];
    uint16_t index;
    ret_t ret;

    if (message == NULL)
    {
        return RET_NULL_PTR;
    }

    /* 快照+按需重绘模式：每个分支都“先更新快照，再看当前页”�?
     * 若消息对应的页不在前台，只存快照不画（切页回来时整页重画会用上）�?
     * 避免后台页的像素被偷改——这是多页面 UI 不闪屏、不串画的关键�?*/
    switch (message->field)
    {
        case APP_LCD_PLAN_FIELD_TOUCH_ID:
            for (index = 0U; index < DEV_GT911_PRODUCT_ID_LENGTH; index++)
            {
                touch_id_text[index] = message->value.touch_id[index];
                s_lcd_plan_touch_id[index] = message->value.touch_id[index];
            }
            touch_id_text[DEV_GT911_PRODUCT_ID_LENGTH] = '\0';
            s_lcd_plan_touch_id[DEV_GT911_PRODUCT_ID_LENGTH] = '\0';
            s_lcd_plan_touch_id_color =
                (message->value.touch_id[0U] == 'F') ?
                PLAN_COLOR_ORANGE : PLAN_COLOR_GREEN;

            if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME)
            {
                return plan_page_draw_home();
            }

            if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_DEVICES)
            {
                return plan_page_draw(APP_LCD_PLAN_PAGE_DEVICES);
            }

            if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_SETTINGS)
            {
                return plan_draw_value(
                    112U,
                    223U,
                    DEV_GT911_PRODUCT_ID_LENGTH,
                    touch_id_text,
                    s_lcd_plan_touch_id_color,
                    PLAN_COLOR_CARD);
            }

            return RET_OK;

        case APP_LCD_PLAN_FIELD_FLASH_ID:
            ret = plan_format_flash_id(&message->value.flash_id,
                                       flash_id_text,
                                       (uint32_t)sizeof(flash_id_text));
            if (ret != RET_OK)
            {
                return ret;
            }

            for (index = 0U; index < 9U; index++)
            {
                s_lcd_plan_flash_id[index] = flash_id_text[index];
            }
            s_lcd_plan_flash_id[9U] = '\0';

            s_lcd_plan_flash_status_color = PLAN_COLOR_GREEN;

            if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME)
            {
                return plan_page_draw_home();
            }

            if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_SETTINGS)
            {
                return plan_draw_value(112U,
                                       249U,
                                       9U,
                                       flash_id_text,
                                       PLAN_COLOR_TEXT,
                                       PLAN_COLOR_CARD);
            }

            return RET_OK;

        case APP_LCD_PLAN_FIELD_FLASH_TEST:
            if (message->value.flash_test_result == RET_OK)
            {
                s_lcd_plan_flash_test_status[0U] = 'P';
                s_lcd_plan_flash_test_status[1U] = 'A';
                s_lcd_plan_flash_test_status[2U] = 'S';
                s_lcd_plan_flash_test_status[3U] = 'S';
                s_lcd_plan_flash_test_status_color = PLAN_COLOR_GREEN;
            }
            else
            {
                s_lcd_plan_flash_test_status[0U] = 'F';
                s_lcd_plan_flash_test_status[1U] = 'A';
                s_lcd_plan_flash_test_status[2U] = 'I';
                s_lcd_plan_flash_test_status[3U] = 'L';
                s_lcd_plan_flash_test_status_color = PLAN_COLOR_ORANGE;
            }
            s_lcd_plan_flash_test_status[4U] = '\0';

            if (s_lcd_plan_current_page != APP_LCD_PLAN_PAGE_ALERTS)
            {
                return RET_OK;
            }

            return plan_draw_status_tag(112U,
                                         s_lcd_plan_flash_test_status,
                                         s_lcd_plan_flash_test_status_color,
                                         PLAN_COLOR_HEADER);

        case APP_LCD_PLAN_FIELD_HOME_SLIDE:
            if (message->value.home_slide >= APP_LCD_PLAN_HOME_SLIDE_COUNT)
            {
                return RET_INVALID_PARAM;
            }

            if (message->value.home_slide ==
                (uint8_t)s_lcd_plan_home_slide)
            {
                return RET_OK;
            }

            s_lcd_plan_home_slide =
                (app_lcd_plan_home_slide_t)message->value.home_slide;
            if (s_lcd_plan_current_page == APP_LCD_PLAN_PAGE_HOME)
            {
                return plan_page_draw_home();
            }
            return RET_OK;

        case APP_LCD_PLAN_FIELD_PAGE:
            if (message->value.page >= APP_LCD_PLAN_PAGE_COUNT)
            {
                return RET_INVALID_PARAM;
            }

            /* 同一页面点击保持幂等：不清屏、不重画，只返回成功�?*/
            if (message->value.page == (uint32_t)s_lcd_plan_current_page)
            {
                return RET_OK;
            }

            ret = plan_page_draw((app_lcd_plan_page_id_t)message->value.page);
            return ret;

        case APP_LCD_PLAN_FIELD_TOUCH_EVENT:
            return plan_handle_touch_event(message);

        default:
            return RET_INVALID_PARAM;
    }
}

/**
 * @brief   Display Task：初始化LCD、绘制P0并串行处理显示消�?
 * @param   argument 未使�?
 */
static void lcd_plan_task(void *argument)
{
    ret_t ret;
    app_lcd_plan_msg_t received;
    uint8_t page_unchanged;

    (void)argument;

    ret = dev_lcd_init();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        DBG_log("P0 dev_lcd_init failed, ret=%ld\r\n", (long)ret);
        s_lcd_plan_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ret = dev_lcd_init_controller();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        DBG_log("P0 dev_lcd_init_controller failed, ret=%ld\r\n", (long)ret);
        s_lcd_plan_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ret = plan_draw_startup_animation();
    if (ret != RET_OK)
    {
        DBG_log("DISPLAY startup animation failed, ret=%ld\r\n",
                (long)ret);
    }

    DBG_log("P0 plan page draw start, state=BOOT_PLAN\r\n");

    ret = plan_page_draw(APP_LCD_PLAN_PAGE_HOME);
    if (ret != RET_OK)
    {
        DBG_log("P0 plan page draw failed, ret=%ld\r\n", (long)ret);
    }
    else
    {
        DBG_log("P0 plan page draw pass, refresh=BOOT_ONCE\r\n");
    }

    DBG_log("DISPLAY task ready, waiting queue length=%lu\r\n",
            (unsigned long)LCD_PLAN_QUEUE_LENGTH);

    for (;;)
    {
        if (xQueueReceive(s_lcd_plan_queue,
                          &received,
                          pdMS_TO_TICKS(LCD_PLAN_SNAPSHOT_POLL_MS)) == pdPASS)
        {
            page_unchanged = 0U;
            if ((received.field == APP_LCD_PLAN_FIELD_PAGE) &&
                (received.value.page == (uint32_t)s_lcd_plan_current_page))
            {
                page_unchanged = 1U;
            }

            ret = plan_apply_message(&received);
            if (ret != RET_OK)
            {
                if (received.field == APP_LCD_PLAN_FIELD_TOUCH_ID)
                {
                    DBG_log("DISPLAY apply failed, field=TOUCH_ID ret=%ld\r\n",
                            (long)ret);
                }
                else if (received.field == APP_LCD_PLAN_FIELD_FLASH_ID)
                {
                    DBG_log("DISPLAY apply failed, field=FLASH_ID ret=%ld\r\n",
                            (long)ret);
                }
                else if (received.field == APP_LCD_PLAN_FIELD_FLASH_TEST)
                {
                    DBG_log("DISPLAY apply failed, field=FLASH_TEST ret=%ld\r\n",
                            (long)ret);
                }
                else if (received.field == APP_LCD_PLAN_FIELD_PAGE)
                {
                    DBG_log("DISPLAY page apply failed, page=%lu ret=%ld\r\n",
                            (unsigned long)received.value.page,
                            (long)ret);
                }
                else if (received.field == APP_LCD_PLAN_FIELD_TIME)
                {
                    DBG_log("DISPLAY time apply failed ret=%ld\r\n",
                            (long)ret);
                }
                else if (received.field == APP_LCD_PLAN_FIELD_NET_STATUS)
                {
                    DBG_log("DISPLAY net status apply failed online=%u ret=%ld\r\n",
                            (unsigned int)received.value.network_online,
                            (long)ret);
                }
                else if (received.field == APP_LCD_PLAN_FIELD_CLOUD_STATUS)
                {
                    DBG_log("DISPLAY cloud status apply failed ret=%ld\r\n",
                            (long)ret);
                }
                else
                {
                    DBG_log("DISPLAY apply failed, field=%u ret=%ld\r\n",
                            (unsigned int)received.field,
                            (long)ret);
                }
            }
            else
            {
                if (received.field == APP_LCD_PLAN_FIELD_TOUCH_EVENT)
                {
                    /* 触摸事件的动作日志在命中判定内输出，避免逐帧刷屏 */
                }
                else if ((received.field == APP_LCD_PLAN_FIELD_TOUCH_ID) &&
                    (s_lcd_plan_current_page != APP_LCD_PLAN_PAGE_HOME))
                {
                    DBG_log("DISPLAY cache update pass, field=TOUCH_ID page=%u\r\n",
                            (unsigned int)s_lcd_plan_current_page);
                }
                else if (received.field == APP_LCD_PLAN_FIELD_TOUCH_ID)
                {
                    DBG_log("DISPLAY refresh pass, field=TOUCH_ID\r\n");
                }
                else if (received.field == APP_LCD_PLAN_FIELD_FLASH_ID)
                {
                    DBG_log("DISPLAY refresh pass, field=FLASH_ID\r\n");
                }
                else if (received.field == APP_LCD_PLAN_FIELD_FLASH_TEST)
                {
                    if (s_lcd_plan_current_page != APP_LCD_PLAN_PAGE_ALERTS)
                    {
                        DBG_log("DISPLAY cache update pass, field=FLASH_TEST page=%u\r\n",
                                (unsigned int)s_lcd_plan_current_page);
                    }
                    else
                    {
                        DBG_log("DISPLAY refresh pass, field=FLASH_TEST result=%ld\r\n",
                                (long)received.value.flash_test_result);
                    }
                }
                else if (received.field == APP_LCD_PLAN_FIELD_PAGE)
                {
                    if (page_unchanged != 0U)
                    {
                        DBG_log("DISPLAY page unchanged, page=%lu\r\n",
                                (unsigned long)received.value.page);
                    }
                    else
                    {
                        DBG_log("DISPLAY page refresh pass, page=%lu\r\n",
                                (unsigned long)received.value.page);
                    }
                }
                else if (received.field == APP_LCD_PLAN_FIELD_TIME)
                {
                    /* 5 秒周期刷新不输出成功日志，避免持续刷屏�?*/
                }
                else if (received.field == APP_LCD_PLAN_FIELD_NET_STATUS)
                {
                    /* 状态由页面呈现，不输出重复的成功日志�?*/
                }
                else if (received.field == APP_LCD_PLAN_FIELD_CLOUD_STATUS)
                {
                    /* 周期云端快照只更新卡片，不持续打印成功日志�?*/
                }
                else
                {
                    DBG_log("DISPLAY refresh pass, field=%u\r\n",
                            (unsigned int)received.field);
                }
            }
        }

        ret = plan_apply_snapshot();
        if (ret != RET_OK)
        {
            DBG_log("DISPLAY snapshot apply failed ret=%ld\r\n", (long)ret);
        }
    }
}

/**
 * @brief   GT911 Touch Task：初始化设备并上报原始触摸事件
 * @param   argument 未使用
 *
 * @note    初始化成功后常驻；INT 到达时立即唤醒，平时按例程每20ms轮询一次，
 *          兼容部分模块 INT 脉冲过窄或触发极性配置不同的情况。
 *          本任务只做去抖与原始事件投递，不读取页面状态、不做命中判定。
 */
static void lcd_plan_touch_task(void *argument)
{
    char product_id[DEV_GT911_PRODUCT_ID_TEXT_SIZE];
    app_lcd_plan_msg_t message;
    dev_gt911_touch_info_t touch_info;
    dev_gt911_touch_state_t last_state;
    uint32_t notify_count;
    uint16_t touch_start_x;
    uint16_t touch_start_y;
    uint16_t touch_last_x;
    uint16_t touch_last_y;
    uint8_t press_latched;
    uint8_t release_frames;
    uint8_t is_press_edge;
    uint8_t init_attempt;
    uint16_t index;
    ret_t ret;

    (void)argument;

    DBG_log("TOUCH task start\r\n");

    ret = dev_gt911_init();
    for (init_attempt = 0U;
         (init_attempt < LCD_PLAN_TOUCH_INIT_RETRY) &&
         (ret != RET_OK) && (ret != RET_ALREADY_INIT);
         init_attempt++)
    {
        DBG_log("TOUCH init retry %lu, ret=%ld\r\n",
                (unsigned long)init_attempt + 1UL,
                (long)ret);
        vTaskDelay(pdMS_TO_TICKS(LCD_PLAN_TOUCH_INIT_RETRY_DELAY_MS));
        ret = dev_gt911_init();
    }

    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        DBG_log("TOUCH init failed, ret=%ld\r\n", (long)ret);
        (void)memset(&message, 0, sizeof(message));
        message.field = APP_LCD_PLAN_FIELD_TOUCH_ID;
        message.value.touch_id[0] = 'F';
        message.value.touch_id[1] = 'A';
        message.value.touch_id[2] = 'I';
        message.value.touch_id[3] = 'L';
        (void)app_lcd_plan_page_post(&message);
        s_lcd_plan_touch_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ret = dev_gt911_product_id_get(product_id,
                                   (uint16_t)sizeof(product_id));
    if (ret != RET_OK)
    {
        DBG_log("TOUCH ID read failed, ret=%ld\r\n", (long)ret);
        (void)memset(&message, 0, sizeof(message));
        message.field = APP_LCD_PLAN_FIELD_TOUCH_ID;
        message.value.touch_id[0] = 'F';
        message.value.touch_id[1] = 'A';
        message.value.touch_id[2] = 'I';
        message.value.touch_id[3] = 'L';
        (void)app_lcd_plan_page_post(&message);
    }
    else
    {
        DBG_log("TOUCH ID read pass, id=%s\r\n", product_id);
        (void)memset(&message, 0, sizeof(message));
        message.field = APP_LCD_PLAN_FIELD_TOUCH_ID;
        for (index = 0U;
             index < DEV_GT911_PRODUCT_ID_LENGTH;
             index++)
        {
            message.value.touch_id[index] = product_id[index];
        }
        ret = app_lcd_plan_page_post(&message);
        if (ret != RET_OK)
        {
            DBG_log("TOUCH_ID display post failed, ret=%ld\r\n", (long)ret);
        }
    }

    ret = dev_gt911_register_notify_task(xTaskGetCurrentTaskHandle());
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        DBG_log("TOUCH notify register failed, ret=%ld\r\n", (long)ret);
        s_lcd_plan_touch_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ret = dev_gt911_touch_irq_init();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        DBG_log("TOUCH irq init failed, ret=%ld\r\n", (long)ret);
        s_lcd_plan_touch_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    DBG_log("TOUCH task ready, waiting INT/poll=%lums\r\n",
            (unsigned long)LCD_PLAN_TOUCH_SCAN_PERIOD_MS);

    /* 中断+轮询双保险：ulTaskNotifyTake 以 20ms 超时兜底——
     * INT 到了立即唤醒读坐标；INT 丢了（脉冲窄/极性错误）20ms 超时
     * 自动醒来轮询一次。notify_count 丢弃不判：无论被唤醒还是超时，
     * 都读一次状态寄存器，以寄存器为准。 */
    last_state = DEV_GT911_TOUCH_UP;
    touch_start_x = 0U;
    touch_start_y = 0U;
    touch_last_x = 0U;
    touch_last_y = 0U;
    press_latched = 0U;
    release_frames = 0U;
    for (;;)
    {
        notify_count = ulTaskNotifyTake(pdTRUE,
                                        pdMS_TO_TICKS(LCD_PLAN_TOUCH_SCAN_PERIOD_MS));
        (void)notify_count;

        ret = dev_gt911_touch_read(&touch_info);
        if (ret != RET_OK)
        {
            DBG_log("TOUCH read failed, ret=%ld\r\n", (long)ret);
            continue;
        }

        if (touch_info.state == DEV_GT911_TOUCH_UP)
        {
            /* 抬起沿：只上报一次 UP，坐标取最后一次按下/移动帧 */
            if (last_state != DEV_GT911_TOUCH_UP)
            {
                DBG_log("TOUCH event=UP x=%u y=%u\r\n",
                        (unsigned int)touch_last_x,
                        (unsigned int)touch_last_y);

                (void)memset(&message, 0, sizeof(message));
                message.field = APP_LCD_PLAN_FIELD_TOUCH_EVENT;
                message.value.touch.event = APP_LCD_PLAN_TOUCH_EVENT_UP;
                message.value.touch.x = touch_last_x;
                message.value.touch.y = touch_last_y;
                ret = app_lcd_plan_page_post(&message);
                if (ret != RET_OK)
                {
                    DBG_Error("TOUCH UP post failed, ret=%ld\r\n", (long)ret);
                }
            }

            /* 松开确认：只有连续 N 帧 UP 才重新武装按钮。
               GT911 手指静止时会停发新帧，驱动按 READY=0 报 UP；
               不确认就把一次按压当成多次按下沿，按钮会被重复触发。 */
            if (press_latched != 0U)
            {
                if (release_frames < 0xFFU)
                {
                    release_frames++;
                }
                if (release_frames >= LCD_PLAN_TOUCH_RELEASE_FRAMES)
                {
                    press_latched = 0U;
                    release_frames = 0U;
                }
            }

            last_state = DEV_GT911_TOUCH_UP;
            continue;
        }

        is_press_edge = ((last_state == DEV_GT911_TOUCH_UP) &&
                         (press_latched == 0U)) ? 1U : 0U;
        if (is_press_edge != 0U)
        {
            touch_start_x = touch_info.points[0U].x;
            touch_start_y = touch_info.points[0U].y;
        }
        touch_last_x = touch_info.points[0U].x;
        touch_last_y = touch_info.points[0U].y;

        DBG_log("TOUCH event=%s points=%u x=%u y=%u size=%u\r\n",
                (is_press_edge != 0U) ? "DOWN" : "MOVE",
                (unsigned int)touch_info.point_count,
                (unsigned int)touch_info.points[0U].x,
                (unsigned int)touch_info.points[0U].y,
                (unsigned int)touch_info.points[0U].size);

        /* 按下沿：只上报一次 DOWN，命中判定由 Display Task 执行 */
        if (is_press_edge != 0U)
        {
            (void)memset(&message, 0, sizeof(message));
            message.field = APP_LCD_PLAN_FIELD_TOUCH_EVENT;
            message.value.touch.event = APP_LCD_PLAN_TOUCH_EVENT_DOWN;
            message.value.touch.x = touch_start_x;
            message.value.touch.y = touch_start_y;
            ret = app_lcd_plan_page_post(&message);
            if (ret != RET_OK)
            {
                DBG_Error("TOUCH DOWN post failed, ret=%ld\r\n", (long)ret);
            }

            /* 本次按压已被消费：确认松开前不再响应新按下 */
            press_latched = 1U;
            release_frames = 0U;
        }

        last_state = DEV_GT911_TOUCH_DOWN;
    }
}

/**
 * @brief 注册设备�?LED2 操作回调
 * @param handler 回调函数；传�?NULL 表示取消注册
 * @param context 由回调拥有者解释的上下文指�?
 * @retval RET_OK 当前实现始终返回成功
 * @note 注册动作本身不操作硬件，触摸命中后才在 Display Task 中调用回调。
 */
ret_t app_lcd_plan_page_set_led_toggle_handler(
    app_lcd_plan_led_toggle_handler_t handler,
    void *context)
{
    s_lcd_plan_led_toggle_handler = handler;
    s_lcd_plan_led_toggle_context = context;
    return RET_OK;
}

/************************************************************
 * @brief   初始�?P0 计划页队列、Display Task �?Touch Task
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始�?
 *          RET_NO_MEMORY 内存不足; RET_TASK_CREATE_ERROR 任务创建失败
 ************************************************************/
/**
 * @brief 创建 LCD 页面队列、Display Task �?Touch Task
 * @note Display Task 独占 dev_lcd；Touch Task 只上报原始触摸事件，命中判定由 Display Task 完成。
 */
ret_t app_lcd_plan_page_init(void)
{
    BaseType_t task_result;

    if ((s_lcd_plan_queue != NULL) ||
        (s_lcd_plan_task != NULL) ||
        (s_lcd_plan_touch_task != NULL))
    {
        return RET_ALREADY_INIT;
    }

    s_lcd_plan_queue = xQueueCreate(LCD_PLAN_QUEUE_LENGTH,
                                    sizeof(app_lcd_plan_msg_t));
    if (s_lcd_plan_queue == NULL)
    {
        return RET_NO_MEMORY;
    }

    task_result = xTaskCreate(lcd_plan_task,
                              "lcd_plan",
                              LCD_PLAN_TASK_STACK_DEPTH,
                              NULL,
                              LCD_PLAN_TASK_PRIORITY,
                              &s_lcd_plan_task);
    if (task_result != pdPASS)
    {
        vQueueDelete(s_lcd_plan_queue);
        s_lcd_plan_queue = NULL;
        s_lcd_plan_task = NULL;
        return RET_TASK_CREATE_ERROR;
    }

    task_result = xTaskCreate(lcd_plan_touch_task,
                              "touch",
                              LCD_PLAN_TOUCH_TASK_STACK_DEPTH,
                              NULL,
                              LCD_PLAN_TOUCH_TASK_PRIORITY,
                              &s_lcd_plan_touch_task);
    if (task_result != pdPASS)
    {
        vTaskDelete(s_lcd_plan_task);
        s_lcd_plan_task = NULL;
        vQueueDelete(s_lcd_plan_queue);
        s_lcd_plan_queue = NULL;
        return RET_TASK_CREATE_ERROR;
    }

    DBG_log("DISPLAY queue init pass, length=%lu item=%lu\r\n",
            (unsigned long)LCD_PLAN_QUEUE_LENGTH,
            (unsigned long)sizeof(app_lcd_plan_msg_t));

    return RET_OK;
}

/************************************************************
 * @brief   向P0页面显示队列投递一条语义消�?
 * @param   message 显示消息指针；函数返回前会复制消息内�?
 * @note    普通状�?遥测快照立即返回；页面切换属于关键事件，队列满时最多等�?
 *          LCD_PLAN_PAGE_EVENT_WAIT_MS 后再返回 RET_QUEUE_FULL�?
 * @retval  RET_OK 成功; RET_NULL_PTR 空指�? RET_NOT_INIT 未初始化;
 *          RET_NOT_READY Display Task未就�? RET_QUEUE_FULL 队列已满
 ************************************************************/
ret_t app_lcd_plan_page_post(const app_lcd_plan_msg_t *message)
{
    TickType_t wait_ticks;

    if (message == NULL)
    {
        return RET_NULL_PTR;
    }

    if (s_lcd_plan_queue == NULL)
    {
        return RET_NOT_INIT;
    }

    if (s_lcd_plan_task == NULL)
    {
        return RET_NOT_READY;
    }

    /* 分级等待：周期状�?ID快照队列满直接返回，下一次会有更新；
     * 主页�?首页轮播切换及Flash测试结果是关键事件，允许�?100ms�?
     * �?Display Task 消费出空位�?*/
    wait_ticks = 0U;
    if ((message->field == APP_LCD_PLAN_FIELD_PAGE) ||
        (message->field == APP_LCD_PLAN_FIELD_HOME_SLIDE) ||
        (message->field == APP_LCD_PLAN_FIELD_TOUCH_EVENT) ||
        (message->field == APP_LCD_PLAN_FIELD_FLASH_TEST))
    {
        wait_ticks = pdMS_TO_TICKS(LCD_PLAN_PAGE_EVENT_WAIT_MS);
    }

    if (xQueueSend(s_lcd_plan_queue, message, wait_ticks) != pdPASS)
    {
        return RET_QUEUE_FULL;
    }

    return RET_OK;
}
