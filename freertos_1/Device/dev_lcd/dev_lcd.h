/**
 * @file    dev_lcd.h
 * @brief   外部并口 LCD 设备驱动接口
 *
 * @note    本头文件只提供 LCD 设备语义接口。
 *          EXMC 地址、命令/数据读写原语和控制器协议细节均为模块内部实现。
 */
#ifndef __DEV_LCD_H__
#define __DEV_LCD_H__

#include "config.h"

/** ILI9488 有效显示宽度（像素） */
#define DEV_LCD_WIDTH          (320U)
/** ILI9488 有效显示高度（像素） */
#define DEV_LCD_HEIGHT         (480U)
/** Device 层固定 ASCII 字符单元宽度（像素） */
#define DEV_LCD_FONT_WIDTH     (8U)
/** Device 层固定 ASCII 字符单元高度（像素） */
#define DEV_LCD_FONT_HEIGHT    (16U)

/************************************************************
 * @brief   初始化 LCD 外部总线
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始化;
 *          RET_NOT_SUPPORT BSP_LCD 未使能
 *
 * @note    该函数只初始化 EXMC 总线，不执行具体 LCD 控制器的上电命令。
 ************************************************************/
ret_t dev_lcd_init(void);

/************************************************************
 * @brief   读取 ILI9488 控制器 ID
 * @param   id  输出控制器 ID
 * @retval  RET_OK 成功; RET_NOT_INIT 总线未初始化; RET_NULL_PTR 空指针
 *
 * @note    该接口用于验证 LCD 命令地址、数据地址和 16 位 EXMC 读通道。
 *          命令 0xD3 及其多次读时序属于 Device 内部细节。
 ************************************************************/
ret_t dev_lcd_read_id(uint16_t *id);

/************************************************************
 * @brief   初始化 ILI9488 控制器并打开 LCD 背光
 * @retval  RET_OK 成功; RET_ALREADY_INIT 控制器已初始化;
 *          RET_NOT_INIT LCD 外部总线未初始化;
 *          RET_DEVICE_NOT_FOUND 控制器 ID 不匹配
 *
 * @note    该接口执行项目确认过的 ILI9488 初始化序列。
 *          LCD 复位脚未在麒麟 GD32 例程中确认，因此不在此接口中猜测添加。
 ************************************************************/
ret_t dev_lcd_init_controller(void);

/************************************************************
 * @brief   清空 LCD 整个显示区域
 * @param   color RGB565 颜色
 * @retval  RET_OK 成功; RET_NOT_READY 控制器未初始化
 ************************************************************/
ret_t dev_lcd_clear(uint16_t color);

/************************************************************
 * @brief   填充一个闭区间矩形
 * @param   x_start  起始 X 坐标
 * @param   y_start  起始 Y 坐标
 * @param   x_end    结束 X 坐标（包含）
 * @param   y_end    结束 Y 坐标（包含）
 * @param   color    RGB565 颜色
 * @retval  RET_OK 成功; RET_NOT_READY 控制器未初始化;
 *          RET_INVALID_PARAM 起止坐标非法; RET_OUT_OF_RANGE 坐标越界
 ************************************************************/
ret_t dev_lcd_fill_rect(uint16_t x_start,
                        uint16_t y_start,
                        uint16_t x_end,
                        uint16_t y_end,
                        uint16_t color);

/************************************************************
 * @brief   绘制一个像素点
 * @param   x      X 坐标
 * @param   y      Y 坐标
 * @param   color  RGB565 颜色
 * @retval  RET_OK 成功; RET_NOT_READY 控制器未初始化;
 *          RET_OUT_OF_RANGE 坐标越界
 ************************************************************/
ret_t dev_lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/************************************************************
 * @brief   绘制一个 ASCII 字符
 * @param   x         字符左上角 X 坐标
 * @param   y         字符左上角 Y 坐标
 * @param   ch        可打印 ASCII 字符（0x20..0x7E）
 * @param   color     前景色 RGB565
 * @param   bg_color  背景色 RGB565
 * @retval  RET_OK 成功; RET_NOT_READY 控制器未初始化;
 *          RET_INVALID_PARAM 字符非法; RET_OUT_OF_RANGE 坐标越界
 ************************************************************/
ret_t dev_lcd_draw_char(uint16_t x,
                        uint16_t y,
                        char ch,
                        uint16_t color,
                        uint16_t bg_color);

/************************************************************
 * @brief   绘制一行 ASCII 文本
 * @param   x         文本左上角 X 坐标
 * @param   y         文本左上角 Y 坐标
 * @param   text      以 '\0' 结尾的 ASCII 文本
 * @param   color     前景色 RGB565
 * @param   bg_color  背景色 RGB565
 * @retval  RET_OK 成功; RET_NOT_READY 控制器未初始化;
 *          RET_NULL_PTR 空指针; RET_INVALID_PARAM 字符非法;
 *          RET_OUT_OF_RANGE 文本区域越界
 ************************************************************/
ret_t dev_lcd_draw_text(uint16_t x,
                        uint16_t y,
                        const char *text,
                        uint16_t color,
                        uint16_t bg_color);

/************************************************************
 * @brief   按整数倍缩放绘制一行 ASCII 文本
 * @param   x         文本左上角 X 坐标
 * @param   y         文本左上角 Y 坐标
 * @param   text      以 '\0' 结尾的可打印 ASCII 文本
 * @param   scale     缩放倍数，范围 1..4
 * @param   color     前景色 RGB565
 * @param   bg_color  背景色 RGB565
 * @retval  RET_OK 成功；RET_INVALID_PARAM 字符或缩放倍数非法；
 *          RET_OUT_OF_RANGE 绘制区域越界；其他值同 dev_lcd_draw_text()
 ************************************************************/
ret_t dev_lcd_draw_text_scaled(uint16_t x,
                               uint16_t y,
                               const char *text,
                               uint8_t scale,
                               uint16_t color,
                               uint16_t bg_color);

#endif /* __DEV_LCD_H__ */
