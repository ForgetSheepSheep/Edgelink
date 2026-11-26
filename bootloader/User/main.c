/**
 * @file    main.c
 * @brief   Composition root for the bare-metal bootloader.
 */
#include "boot_ota.h"

int main(void)
{
    /* A/B decision, install, trial and rollback all live in boot_ota.c. */
    boot_ota_run();

    /* boot_ota_run only returns if no slot can be entered. */
    for (;;)
    {
    }
}
