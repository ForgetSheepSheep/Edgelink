#ifndef __APP_BOOT_AB_H__
#define __APP_BOOT_AB_H__

 /* 1. boot_protocol.h：Flash 分区、状态、协议结构体等公共定义； */
#include "boot_protocol.h"
 /* 2. boot_param.h：启动参数读取、保存和默认值接口。 */
#include "boot_param.h"

/*
 * App 侧启动服务：连接 BootLoader 与业务侧的启动事件。
 *
 * 职责：
 * - 把 BootLoader 留下的瞬时回滚标志（boot_param.rollback_flag）
 *   转成 fw_nvs 里的持久回滚事件记录，并消费该瞬时标志；
 * - 试运行（TRIAL）确认成功后，把当前已确认固件版本写入 fw_nvs。
 *
 * 回滚事件是"留给云端查询"的锁存：确认新固件正常不会清除它，
 * 只由未来的显式清除接口（云端已上报）清除。
 */

/*
 * @brief  启动服务初始化。
 * @note   可在调度器启动前调用（只访问内部 Flash，不创建 RTOS 对象）。
 * @return RET_OK 或具体 Flash/NVS 错误。
 */
ret_t app_boot_service_init(void);

/*
 * @brief  试运行确认并记录已确认版本。
 * @note   由健康任务在上电稳定后调用；非 TRIAL 状态下 app_boot_confirm_running
 *         直接返回 RET_OK，本函数仍会把当前版本同步到 fw_nvs。
 */
ret_t app_boot_service_confirm(void);

#endif /* __APP_BOOT_AB_H__ */




