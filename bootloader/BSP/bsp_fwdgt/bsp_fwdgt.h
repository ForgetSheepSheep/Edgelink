/**
 * @file    bsp_fwdgt.h
 * @brief   Independent watchdog (FWDGT) board support.
 */
#ifndef BOOTLOADER_BSP_FWDGT_H
#define BOOTLOADER_BSP_FWDGT_H

#include "config.h"

ret_t bsp_fwdgt_init(void);
ret_t bsp_fwdgt_feed(void);

#endif /* BOOTLOADER_BSP_FWDGT_H */
