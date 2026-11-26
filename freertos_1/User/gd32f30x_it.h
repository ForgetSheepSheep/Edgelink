/*
 * GD32 官方异常/中断头文件模板。
 *
 * 当前工程主要保留 Cortex-M 内核异常入口，便于后续扩展外设中断。
 */
/*!
    \file    gd32f30x_it.h
    \brief   the header file of the ISR

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

#ifndef GD32F30X_IT_H
#define GD32F30X_IT_H

#include "gd32f30x.h"

/*
 * 异常入口说明（配合 FreeRTOSConfig.h 的宏映射）：
 * - PendSV_Handler：FreeRTOS 任务切换实际发生的地方（最低优先级异常）；
 * - SysTick_Handler：FreeRTOS 时钟节拍入口（xPortSysTickHandler 映射）；
 * - SVC_Handler：启动第一个任务时使用（xPortSVCHandler 映射）。
 * 这三个名字通过 FreeRTOSConfig.h 底部的 #define 映射到 port.c 的实现，
 * 启动文件向量表无需改动。
 */

/* function declarations */
/* NMI 异常入口（不可屏蔽中断） */
void NMI_Handler(void);
/* HardFault 异常入口（硬件错误，通常意味着严重程序故障） */
void HardFault_Handler(void);
/* MemManage 异常入口（存储器管理错误） */
void MemManage_Handler(void);
/* BusFault 异常入口（总线错误，如非法地址访问） */
void BusFault_Handler(void);
/* UsageFault 异常入口（用法错误，如未定义指令、除零） */
void UsageFault_Handler(void);
/* SVC 异常入口（被 FreeRTOS 映射为 xPortSVCHandler） */
void SVC_Handler(void);
/* DebugMon 异常入口（调试监视器） */
void DebugMon_Handler(void);
/* PendSV 异常入口（被 FreeRTOS 映射为 xPortPendSVHandler） */
void PendSV_Handler(void);

#endif /* GD32F30X_IT_H */
