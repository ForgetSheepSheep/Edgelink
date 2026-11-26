#include "./app_boot_ab/app_boot_ab.h"
#include "app_fw_nvs/fw_nvs.h"
#include "app_ota/app_ota.h"
#include "app_version.h"

ret_t app_boot_service_init(void)
{
    boot_param_t param;
    ret_t ret;

    DBG_log("running firmware version=%u.%u.%u\r\n",
            (unsigned int)APP_FW_VERSION_MAJOR,
            (unsigned int)APP_FW_VERSION_MINOR,
            (unsigned int)APP_FW_VERSION_PATCH);

    ret = fw_nvs_init();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        return ret;
    }

    ret = boot_param_load(&param);
    if (ret == RET_MAGIC_ERROR)
    {
        /* 首次上电（参数区空白）：没有历史回滚可记录。 */
        return RET_OK;
    }
    if (ret != RET_OK)
    {
        return ret;
    }

    if (param.rollback_flag == 0U)
    {
        return RET_OK;
    }

    /* 先把事件落成持久记录，再消费瞬时标志；中途掉电最多重记一次。 */
    ret = fw_nvs_mark_rollback();
    if (ret != RET_OK)
    {
        DBG_Error("record rollback event failed ret=%ld\r\n", (long)ret);
        return ret;
    }

    param.rollback_flag = 0U;
    ret = boot_param_save(&param);
    if (ret != RET_OK)
    {
        DBG_Error("consume rollback flag failed ret=%ld\r\n", (long)ret);
    }
    return ret;
}

ret_t app_boot_service_confirm(void)
{
    boot_param_t param;
    fw_nvs_params_t nvs;
    ret_t ret;

    ret = app_boot_confirm_running();
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = boot_param_load(&param);
    if (ret == RET_MAGIC_ERROR)
    {
        /* 参数区空白时没有版本号可记录，不视为失败。 */
        return RET_OK;
    }
    if (ret != RET_OK)
    {
        return ret;
    }

    /*
     * 读-改-写：版本号取【本镜像自证常量】而不是 boot_param.fw_version——
     * 后者是"待升级包"的身份，回滚后不会随旧固件复位，直接抄会把失败
     * 版本的号写进 NVS。活动槽与回滚事件标志沿用 NVS 现有值。
     */
    (void)memset(&nvs, 0, sizeof(nvs));
    ret = fw_nvs_get(&nvs);
    if ((ret != RET_OK) &&
        (ret != RET_NVS_NOT_FOUND) &&
        (ret != RET_CRC_ERROR))
    {
        return ret;
    }

    nvs.fw_version = (uint32_t)APP_FW_VERSION;
    nvs.active_slot = param.active_slot;
    return fw_nvs_set(&nvs);
}
