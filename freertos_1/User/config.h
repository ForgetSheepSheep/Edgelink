/**
 * @file    config.h
 * @brief   工程公共配置头文件（全工程统一的包含入口）
 *
 * @note    各模块头文件统一 include 本文件，从而只写一行即可拿到：
 *          - C 标准库：stdio/stdlib/stdint/string/stdbool；
 *          - RTT 日志接口（DBG_log / DBG_Error，经 SEGGER RTT 输出）；
 *          - 统一错误码 ret_t（见 ret_error.h）；
 *          - 常用开关量宏 OPEN/CLOSE。
 */
#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "RTT_Debug.h"
#include "ret_error.h"

/** 通用开关量：OPEN=1（开），CLOSE=0（关） */
#define    OPEN     1
#define    CLOSE    0


#endif /* __CONFIG_H */
