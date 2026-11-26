/*****************************************************************************
 * RTT_Debug.h - 轻量级嵌入式调试输出封装
 *
 * 依赖:
 *   - SEGGER_RTT.c / SEGGER_RTT.h
 *   - SEGGER_RTT_printf.c (使用 RTT 模式时需要)
 *
 * 用法:
 *   包含此头文件后直接用 DBG_log / DBG_Warning / DBG_Error 即可。
 *
 *   默认使用 RTT 输出 (需 J-Link + RTT Viewer/Client)，
 *   也可切换到 USART 串口输出。
 *
 *   通过定义/取消定义 DBGLOG / DBGWARNING / DBGERROR 控制日志级别。
 *
 * 示例:
 *   #include "RTT_Debug.h"
 *
 *   void foo(void) {
 *       int x = 42;
 *       DBG_log("entry");
 *       DBG_log("x = %d", x);
 *       DBG_Warning("x is large: %d", x);
 *       DBG_Error("something failed, err=%d", err);
 *   }
 *****************************************************************************/

#ifndef RTT_DEBUG_H__
#define RTT_DEBUG_H__

#include "SEGGER_RTT.h"
#include <stdio.h>

/*============================================================================
 * 输出模式选择 (二选一)
 *============================================================================*/
#define STR_DEBUG            /* RTT 调试 (通过 J-Link RTT Viewer/Client)   */
//#define USART_DEBUG        /* 串口调试 (需 printf 已重定向到目标串口)    */

/*============================================================================
 * 日志级别控制 (注释掉对应行即关闭该级别)
 *============================================================================*/
#define DBGLOG               /* 普通信息 */
#define DBGWARNING           /* 警告     */
#define DBGERROR             /* 错误     */

/*============================================================================
 * 行号转字符串辅助宏
 *============================================================================*/
#define _RTT_SLINE(x)       #x
#define _RTT_SLINE2(x)      _RTT_SLINE(x)
#define _RTT_SLINE_STR      _RTT_SLINE2(__LINE__)

/*============================================================================
 * DBG_log / DBG_Warning / DBG_Error 宏定义
 *============================================================================*/
/*---- RTT 模式 ----*/
#if defined(STR_DEBUG)

    #if defined(DBGLOG)
        #define DBG_log(format, ...) \
            SEGGER_RTT_printf(0, \
                RTT_CTRL_TEXT_GREEN "[log]" RTT_CTRL_RESET \
                format "[" __FILE__ ":" _RTT_SLINE_STR "]\r\n", \
                ##__VA_ARGS__)
    #else
        #define DBG_log(format, ...)
    #endif

    #if defined(DBGWARNING)
        #define DBG_Warning(format, ...) \
            SEGGER_RTT_printf(0, \
                RTT_CTRL_TEXT_YELLOW "[Warning!]" RTT_CTRL_RESET \
                format "[" __FILE__ ":" _RTT_SLINE_STR "]\r\n", \
                ##__VA_ARGS__)
    #else
        #define DBG_Warning(format, ...)
    #endif

    #if defined(DBGERROR)
        #define DBG_Error(format, ...) \
            SEGGER_RTT_printf(0, \
                RTT_CTRL_TEXT_RED "[!Error!]" RTT_CTRL_RESET \
                format "[" __FILE__ ":" _RTT_SLINE_STR "]\r\n", \
                ##__VA_ARGS__)
    #else
        #define DBG_Error(format, ...)
    #endif

/*---- USART 串口模式 ----*/
#elif defined(USART_DEBUG)

    #if defined(DBGLOG)
        #define DBG_log(format, ...) \
            printf("[log]" format "[" __FILE__ ":" _RTT_SLINE_STR "]\r\n", \
                ##__VA_ARGS__)
    #else
        #define DBG_log(format, ...)
    #endif

    #if defined(DBGWARNING)
        #define DBG_Warning(format, ...) \
            printf("[Warning!]" format "[" __FILE__ ":" _RTT_SLINE_STR "]\r\n", \
                ##__VA_ARGS__)
    #else
        #define DBG_Warning(format, ...)
    #endif

    #if defined(DBGERROR)
        #define DBG_Error(format, ...) \
            printf("[!Error!]" format "[" __FILE__ ":" _RTT_SLINE_STR "]\r\n", \
                ##__VA_ARGS__)
    #else
        #define DBG_Error(format, ...)
    #endif

/*---- 无输出 (安全兜底) ----*/
#else
    #define DBG_log(format, ...)
    #define DBG_Warning(format, ...)
    #define DBG_Error(format, ...)
#endif

/*============================================================================
 * RTT_ASSERT — 带调试输出的断言
 *   条件为假时输出错误信息并进入死循环，方便调试器捕获。
 *   可通过定义 RTT_DEBUG_DISABLE_ASSERT 全局禁用。
 *============================================================================*/
#ifndef RTT_DEBUG_DISABLE_ASSERT
    #define RTT_ASSERT(cond) \
        do { \
            if (!(cond)) { \
                DBG_Error("ASSERT FAILED: %s", #cond); \
                while(1); \
            } \
        } while(0)
#else
    #define RTT_ASSERT(cond)  ((void)0)
#endif

#endif /* RTT_DEBUG_H__ */
