/**
 * @file    bsp_wwdgt.h
 * @brief   Window watchdog (WWDGT) board support.
 */
#ifndef BOOTLOADER_BSP_WWDGT_H
#define BOOTLOADER_BSP_WWDGT_H

#include "config.h"

ret_t bsp_wwdgt_init(void);
ret_t bsp_wwdgt_feed(void);

#endif /* BOOTLOADER_BSP_WWDGT_H */
