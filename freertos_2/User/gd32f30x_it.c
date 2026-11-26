/*
 * GD32 官方异常处理模板实现。
 *
 * 当前多数异常处理仍保持进入死循环等待调试的默认策略。
 */
/*!
    \file    gd32f30x_it.c
    \brief   interrupt service routines

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
    \version 2020-09-30, V2.1.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f30x_it.h"
#include "dev_gt911/dev_gt911.h"


/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
    \note       HardFault（非法访问/总线错误等）发生后原地死循环：
                不复位、不返回，让调试器连上后能看到精确的故障现场
                （LR、SP 和故障状态寄存器）。
*/
void HardFault_Handler(void)
{
    /* 记录跨复位故障现场：0x2000FF00 位于应用 .bss/栈之上，IWDG 复位不清除 */
    *(volatile uint32_t *)0x2000FF00UL = 0xF0F0F0F0UL;
    *(volatile uint32_t *)0x2000FF04UL = 3UL;
    *(volatile uint32_t *)0x2000FF08UL = SCB->CFSR;
    *(volatile uint32_t *)0x2000FF0CUL = SCB->HFSR;
    *(volatile uint32_t *)0x2000FF10UL = SCB->BFAR;
    *(volatile uint32_t *)0x2000FF14UL = SCB->MMFAR;
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* 故障码 4：记录 CFSR/MMFAR 供复位后定位 */
    *(volatile uint32_t *)0x2000FF00UL = 0xF0F0F0F0UL;
    *(volatile uint32_t *)0x2000FF04UL = 4UL;
    *(volatile uint32_t *)0x2000FF08UL = SCB->CFSR;
    *(volatile uint32_t *)0x2000FF14UL = SCB->MMFAR;
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* 故障码 5：记录 CFSR/BFAR 供复位后定位 */
    *(volatile uint32_t *)0x2000FF00UL = 0xF0F0F0F0UL;
    *(volatile uint32_t *)0x2000FF04UL = 5UL;
    *(volatile uint32_t *)0x2000FF08UL = SCB->CFSR;
    *(volatile uint32_t *)0x2000FF10UL = SCB->BFAR;
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* 故障码 6：记录 CFSR 供复位后定位 */
    *(volatile uint32_t *)0x2000FF00UL = 0xF0F0F0F0UL;
    *(volatile uint32_t *)0x2000FF04UL = 6UL;
    *(volatile uint32_t *)0x2000FF08UL = SCB->CFSR;
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/**
 * @brief   GT911 INT：PB12 对应 EXTI12，入口交给 Device 层处理
 * @note    EXTI10_15 是“共用入口”：PB10~PB15 六个引脚共享这一个中断向量，
 *          因此 Device 层第一件事是查 EXTI12 的 pending 位，确认真是 PB12。
 *          Device 层只清 pending 并通知 Touch Task；禁止在 ISR 中读 I²C
 *          （I2C 读写是毫秒级阻塞操作，会卡死所有同优先级中断）。
 */
void EXTI10_15_IRQHandler(void)
{
    dev_gt911_irq_handler();
}
