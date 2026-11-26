/**
 * @file    bootloader.h
 * @brief   Slot validation and application hand-off primitives.
 *
 * The A/B decision logic lives in boot_ota.c; this module only knows how to
 * validate one application slot and how to transfer control to it.
 */
#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "config.h"

/**
 * @brief Validate one application slot's vector table without changing CPU state.
 *
 * The slot address must be Slot A or Slot B.  The reset handler must be Thumb
 * code inside the slot; the initial MSP must be 8-byte aligned and inside SRAM.
 *
 * @return RET_OK, RET_NULL_PTR, RET_ALIGN_ERROR, RET_PARTITION_ERROR, or
 *         RET_APP_INVALID.
 */
ret_t bootloader_validate_slot(uint32_t slot_address,
                               uint32_t *initial_msp,
                               uint32_t *reset_handler);

/** Tear down bootloader state and branch to a validated slot. */
void bootloader_jump(uint32_t slot_address,
                     uint32_t initial_msp,
                     uint32_t reset_handler);

#endif /* BOOTLOADER_H */
