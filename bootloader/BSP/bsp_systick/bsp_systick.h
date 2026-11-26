/**
 * @file    bsp_systick.h
 * @brief   RTOS-free Cortex-M SysTick timebase.
 */
#ifndef BOOTLOADER_BSP_SYSTICK_H
#define BOOTLOADER_BSP_SYSTICK_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_SYSTICK_FREQUENCY_HZ (1000UL)

ret_t bsp_systick_init(void);
void bsp_systick_deinit(void);
uint32_t bsp_systick_get(void);
ret_t bsp_systick_delay_ms(uint32_t milliseconds);

/** The vector-table handler is intentionally part of the BSP, not the app. */
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_BSP_SYSTICK_H */
