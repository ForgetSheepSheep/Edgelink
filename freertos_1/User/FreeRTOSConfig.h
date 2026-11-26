/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS 内核配置表（裁剪与移植参数的唯一入口）
 *
 * @note    这个文件决定任务调度方式、时钟节拍、堆大小、优先级位宽
 *          和中断优先级边界。分组说明：
 *          1) 调度与节拍：抢占式调度、1ms tick；
 *          2) 任务与堆：8 级优先级、36KB 堆（heap_4）；
 *          3) 同步与健壮性：互斥锁/计数信号量/任务通知/事件组、
 *             栈溢出检测 2 级、Malloc 失败钩子；
 *          4) 软件定时器：Timer 服务任务参数；
 *          5) 中断优先级边界：configMAX_SYSCALL_INTERRUPT_PRIORITY=5，
 *             优先级数值 0~4 的中断禁止调用 FromISR API。
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include "gd32f30x.h"

#ifndef __NVIC_PRIO_BITS
#define __NVIC_PRIO_BITS                         4
#endif

/* 调度与节拍：
 * - 抢占式调度打开后，高优先级任务就绪时会抢占低优先级任务。
 * - Tick 频率为 1000Hz，也就是内核每 1ms 处理一次系统节拍。
 * - Tick Hook 适合放非常轻量的周期逻辑，不适合做耗时工作。
 */
#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( ( uint32_t ) SystemCoreClock )
#define configTICK_RATE_HZ                       ( ( TickType_t ) 1000 )
/* 任务与堆：
 * - 优先级档位一共 8 档，数字越大优先级越高。
 * - 最小栈深只是起点，真实任务仍要结合局部变量和调用链估算。
 * - 36KB 堆会被任务栈、队列、信号量、软件定时器等内核对象共同使用。
 */
#define configMAX_PRIORITIES                     8
#define configMINIMAL_STACK_SIZE                 ( ( uint16_t ) 256 )
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 36 * 1024 ) )
#define configMAX_TASK_NAME_LEN                  16
#define configUSE_TRACE_FACILITY                 0
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1
/* 同步与健壮性：
 * - 这里启用了互斥锁、计数信号量、任务通知、事件组，业务层同步手段比较完整。
 * - 栈溢出检测级别 2 更偏调试友好，能更早暴露任务栈不足问题。
 * - Malloc 失败钩子打开后，堆耗尽时不会悄悄失效，而是进入明确故障路径。
 */
#define configUSE_MUTEXES                        1
/* 普通消息队列是FreeRTOS内核基础功能，没有configUSE_QUEUE总开关。
 * 当前工程已编译queue.c，并开启动态分配，因此可使用xQueueCreate()。
 * configUSE_QUEUE_SETS只控制“一个任务等待多个队列”的队列集功能；
 * 第28节的多目标分发不使用队列集，保持关闭。
 */
#define configUSE_QUEUE_SETS                     0
#define configQUEUE_REGISTRY_SIZE                1
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_MALLOC_FAILED_HOOK             1
#define configUSE_APPLICATION_TASK_TAG           0
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_TASK_NOTIFICATIONS             1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES    1
#define configUSE_EVENT_GROUPS                   1
#define configUSE_TIME_SLICING                   1
#define configUSE_NEWLIB_REENTRANT               0
#define configENABLE_BACKWARD_COMPATIBILITY      0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0
/* 软件定时器是由专门的 Timer Task 驱动的：
 * 它的优先级、命令队列长度和栈深都在这里集中配置。
 */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                2
#define configTIMER_QUEUE_LENGTH                 8
#define configTIMER_TASK_STACK_DEPTH             256

/* 当前工程只启用动态分配；如果未来切静态分配，还要补齐静态内存回调。 */
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1

/* 中断优先级边界是 FreeRTOS 移植里最容易踩坑的地方：
 * - 数值越小，表示硬件优先级越高。
 * - 高于 configMAX_SYSCALL_INTERRUPT_PRIORITY 的中断，不能调用任何 FromISR API。
 * - 配错时常见现象是断言、偶发死锁，或者系统在高负载下异常跑飞。
 */
#define configPRIO_BITS                          __NVIC_PRIO_BITS
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* 断言失败后记录跨复位故障标记（0x2000FF00），再关中断停机。 */
#define configASSERT( x )                        if( ( x ) == 0 ) { \
                                                     *(volatile unsigned long *)0x2000FF00UL = 0xF0F0F0F0UL; \
                                                     *(volatile unsigned long *)0x2000FF04UL = 7UL; \
                                                     *(volatile unsigned long *)0x2000FF08UL = (unsigned long)__LINE__; \
                                                     taskDISABLE_INTERRUPTS(); for( ;; ) { } }

/* INCLUDE_ 宏控制对应 API 是否参与编译。
 * 后续如果要压缩代码体积，可以优先从“确认没用到的 API”开始裁剪。
 */
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xResumeFromISR                   1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1

/* 将 FreeRTOS 端口层使用的异常入口映射到 CMSIS 启动文件里的标准名字。 */
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler
#define xPortSysTickHandler                      SysTick_Handler

#endif
