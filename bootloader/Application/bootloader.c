/**
 * @file    bootloader.c
 * @brief   Slot validation and vector-table hand-off.
 */
#include "bootloader.h"
#include "boot_protocol.h"
#include "gd32f30x.h"

typedef void (*boot_entry_t)(void);

static bool boot_address_in_range(uint32_t address,
                                  uint32_t begin,
                                  uint32_t end)
{
    return (address >= begin) && (address < end);
}

ret_t bootloader_validate_slot(uint32_t slot_address,
                               uint32_t *initial_msp,
                               uint32_t *reset_handler)
{
    const volatile uint32_t *vector;
    uint32_t msp;
    uint32_t reset;

    if ((initial_msp == NULL) || (reset_handler == NULL))
    {
        return RET_NULL_PTR;
    }

    if ((slot_address & (BOOT_VECTOR_ALIGNMENT - 1UL)) != 0UL)
    {
        return RET_ALIGN_ERROR;
    }

    if ((slot_address != BOOT_SLOT_A_ADDR) &&
        (slot_address != BOOT_SLOT_B_ADDR))
    {
        return RET_PARTITION_ERROR;
    }

    vector = (const volatile uint32_t *)(uintptr_t)slot_address;
    msp = vector[0];
    reset = vector[1];

    /* A reset vector must be Thumb code and remain inside its own slot. */
    if ((reset & 1UL) == 0UL)
    {
        return RET_APP_INVALID;
    }

    reset &= ~1UL;
    if (!boot_address_in_range(reset,
                               slot_address,
                               slot_address + BOOT_SLOT_SIZE))
    {
        return RET_APP_INVALID;
    }

    /* The GD32F303ZE exposes 64 KiB SRAM; the stack must point into it. */
    if (((msp & 7UL) != 0UL) ||
        !boot_address_in_range(msp, BOOT_SRAM_BASE + 8UL, BOOT_SRAM_END + 1UL))
    {
        return RET_APP_INVALID;
    }

    *initial_msp = msp;
    *reset_handler = reset | 1UL;
    return RET_OK;
}

void bootloader_jump(uint32_t slot_address,
                     uint32_t initial_msp,
                     uint32_t reset_handler)
{
    uint32_t i;
    const uint32_t nvic_register_count =
        (uint32_t)(sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]));
    boot_entry_t entry = (boot_entry_t)(uintptr_t)reset_handler;

    __disable_irq();

    /* Stop the bootloader time base and remove every pending interrupt. */
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    for (i = 0U; i < nvic_register_count; ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    /* The application owns the vector table and starts in privileged MSP mode. */
    SCB->VTOR = slot_address;
    __set_CONTROL(0U);
    __set_MSP(initial_msp);
    __DSB();
    __ISB();

    entry();

    /* A well-formed reset handler should never return to the bootloader. */
    for (;;)
    {
    }
}
