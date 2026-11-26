/**
 * @file    boot_ota.h
 * @brief   A/B boot decision: install, trial, rollback, jump.
 */
#ifndef BOOT_OTA_H
#define BOOT_OTA_H

#include "config.h"

/**
 * @brief Run the complete boot policy; returns only if no slot can be entered.
 *
 * Sequence:
 * 1. Initialize time base, GPIO/EXMC, LCD status screen and GD25Q128.
 * 2. Load the dual-copy boot parameter record; blank media gets defaults.
 * 3. DOWNLOAD_READY/COPYING: verify the external image header against the
 *    parameter record, copy the image into the pending slot with CRC and
 *    vector checks, then enter TRIAL.
 * 4. TRIAL: count attempts; exceeding BOOT_MAX_TRIAL_COUNT or an invalid
 *    pending slot rolls back to the active slot.
 * 5. CONFIRMED: jump the active slot, falling back to the other slot.
 *
 * The independent watchdog is enabled only for trial jumps so that a hung
 * new firmware resets back into the bootloader and eventually rolls back.
 */
void boot_ota_run(void);

#endif /* BOOT_OTA_H */
