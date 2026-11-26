/**
 * @file    bsp_systick.c
 * @brief   SysTick 定时器 BSP 驱动实现（已弃用，仅保留说明）
 *
 * @note    本文件已不再编译进工程，仅保留弃用说明供历史参考。
 */

/*
 * 此模块已弃用。
 *
 * 原先通过 vApplicationTickHook 维护 g_sys_run_time 作为毫秒计数器，
 * 现已统一使用 FreeRTOS 的 xTaskGetTickCount() 获取系统 tick。
 *
 * 如需周期回调，请直接使用 FreeRTOS 软件定时器。
 */
