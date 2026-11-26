/**
 * @file    boot_config.h
 * @brief   Bootloader hardware constants and hand-off policy.
 *
 * The A/B partition map is owned by Application/boot_protocol.h so that the
 * bootloader and the application cannot drift apart.  This header only keeps
 * the constants that belong to the bootloader's own execution environment.
 */
#ifndef BOOT_CONFIG_H
#define BOOT_CONFIG_H

#include <stdint.h>

/* Internal Flash geometry (GD32F303ZE: 512 KiB). */
#define BOOT_FLASH_BASE            (0x08000000UL)
#define BOOT_FLASH_SIZE            (0x00080000UL)

/* On-chip SRAM. */
#define BOOT_SRAM_BASE             (0x20000000UL)
#define BOOT_SRAM_SIZE             (0x00010000UL)
#define BOOT_SRAM_END              (BOOT_SRAM_BASE + BOOT_SRAM_SIZE)

/* How long a status screen stays visible before the bootloader jumps. */
#define BOOT_STATUS_HOLD_MS        (400UL)

/* VTOR is architecturally aligned to at least 128 bytes on Cortex-M4. */
#define BOOT_VECTOR_ALIGNMENT      (0x00000080UL)

#endif /* BOOT_CONFIG_H */
