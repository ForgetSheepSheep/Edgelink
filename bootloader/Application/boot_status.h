/**
 * @file    boot_status.h
 * @brief   Bootloader status screen (ILI9488 via Device/dev_lcd).
 *
 * The status screen is best-effort: if the LCD bus or controller does not
 * answer, the boot flow continues without a display.  It never blocks a jump.
 */
#ifndef BOOT_STATUS_H
#define BOOT_STATUS_H

#include "config.h"

/** Initialize the LCD and clear it; failure only disables the screen. */
void boot_status_init(void);

/** True when the screen is usable. */
uint8_t boot_status_ready(void);

/** Redraw the whole screen with a title and up to two detail lines. */
void boot_status_show(const char *title,
                      const char *line1,
                      const char *line2);

/** Update the copy progress bar and its percentage label. */
void boot_status_progress(uint32_t percent, const char *label);

#endif /* BOOT_STATUS_H */
