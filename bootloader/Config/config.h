/**
 * @file    config.h
 * @brief   Common include boundary for the bare-metal bootloader.
 *
 * This header deliberately contains no scheduler/RTOS header.  The same
 * error type and RTT logging style used by freertos_1 are retained, while
 * the bootloader itself remains a single-threaded firmware image.
 */
#ifndef BOOTLOADER_CONFIG_H
#define BOOTLOADER_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "boot_config.h"
#include "ret_error.h"
#include "RTT_Debug.h"

#define OPEN    (1U)
#define CLOSE   (0U)

#endif /* BOOTLOADER_CONFIG_H */
