#include "boot_ota.h"
#include "bootloader.h"
#include "boot_protocol.h"
#include "boot_param.h"
#include "boot_status.h"
#include "ota_crc/ota_crc.h"
#include "dev_flash/dev_flash.h"
#include "dev_gd25q128/dev_gd25q128.h"
#include "bsp_delay/bsp_delay.h"
#include "bsp_fwdgt/bsp_fwdgt.h"
#include "bsp_gpio/bsp_gpio.h"
#include "bsp_systick/bsp_systick.h"
#include <stddef.h>

#define BOOT_COPY_BUFFER_SIZE (512U)
#define BOOT_PROGRESS_STEP (5U)
#define BOOT_ERASE_CHUNK_SIZE (8192U)

static uint8_t boot_slot_check(uint32_t slot,
                               uint32_t *initial_msp,
                               uint32_t *reset_handler)
{
    uint32_t address = boot_slot_addr(slot);

    if (address == 0U)
    {
        return 0U;
    }
    return (bootloader_validate_slot(address,
                                     initial_msp,
                                     reset_handler) == RET_OK) ? 1U : 0U;
}

/*
 * 外部信息头与启动参数必须逐字段一致；头存在即代表正文已通过 App �? * 三重校验，BootLoader 仍然会自己再算一遍，不信任任何单一来源�? */
static ret_t boot_verify_info(const boot_param_t *param)
{
    boot_fw_info_t info;
    uint32_t header_crc;
    ret_t ret;

    (void)memset(&info, 0, sizeof(info));
    ret = dev_gd25q128_read(BOOT_FW_INFO_ADDR, (uint8_t *)&info, sizeof(info));
    if (ret != RET_OK)
    {
        return ret;
    }

    if (info.magic != BOOT_FW_INFO_MAGIC)
    {
        return RET_DATA_ERROR;
    }

    header_crc = ota_crc32_calc((const uint8_t *)&info,
                                (uint32_t)offsetof(boot_fw_info_t,
                                                   header_crc32));
    if (info.header_crc32 != header_crc)
    {
        return RET_DATA_ERROR;
    }

    if ((info.fw_addr != param->fw_addr) ||
        (info.fw_size != param->fw_size) ||
        (info.fw_crc32 != param->fw_crc32) ||
        (info.fw_version != param->fw_version) ||
        (info.target_slot != param->pending_slot))
    {
        return RET_DATA_ERROR;
    }
    return RET_OK;
}

/*
 * 擦除目标�?-> 分块搬运（边搬边�?CRC�?> 内部 Flash 全量回读 CRC ->
 * 向量表校验。任一环节失败都返回错误，由调用方走回滚�? */
#define BOOT_READBACK_CHUNK_SIZE (4096U)

typedef enum
{
    BOOT_INSTALL_IDLE = 0,
    BOOT_INSTALL_ERASE,
    BOOT_INSTALL_COPY,
    BOOT_INSTALL_READBACK,
    BOOT_INSTALL_VECTOR,
    BOOT_INSTALL_DONE
} boot_install_phase_t;

typedef struct
{
    boot_install_phase_t phase;
    const boot_param_t *param;
    uint32_t slot_address;
    uint32_t offset;
    uint32_t crc;
    uint32_t shown_percent;
    ret_t result;
} boot_install_ctx_t;

static boot_install_ctx_t s_boot_install;

static void boot_install_begin(const boot_param_t *param)
{
    (void)memset(&s_boot_install, 0, sizeof(s_boot_install));
    s_boot_install.phase = BOOT_INSTALL_ERASE;
    s_boot_install.param = param;
    s_boot_install.slot_address = boot_slot_addr(param->pending_slot);
    s_boot_install.crc = CRC32_INIT_VALUE;
    s_boot_install.shown_percent = 0xFFFFFFFFUL;
    s_boot_install.result = RET_OK;
}

static void boot_install_show(uint32_t percent, const char *label)
{
    if ((percent / BOOT_PROGRESS_STEP) !=
        (s_boot_install.shown_percent / BOOT_PROGRESS_STEP) ||
        (percent == 100U))
    {
        s_boot_install.shown_percent = percent;
        boot_status_progress(percent, label);
    }
}

/*
 * 安装状态机单步：每次只做一小块有界工作，
 * 返回 0 继续、1 完成、2 失败（错误码在 s_boot_install.result）。
 */
static uint8_t boot_install_step(void)
{
    boot_install_ctx_t *ctx = &s_boot_install;
    uint32_t length;
    uint32_t percent;
    uint32_t msp;
    uint32_t reset;
    ret_t ret;

    switch (ctx->phase)
    {
    case BOOT_INSTALL_ERASE:
        if (ctx->offset >= ctx->param->fw_size)
        {
            ctx->phase = BOOT_INSTALL_COPY;
            ctx->offset = 0U;
            ctx->shown_percent = 0xFFFFFFFFUL;
            return 0U;
        }
        length = ctx->param->fw_size - ctx->offset;
        if (length > BOOT_ERASE_CHUNK_SIZE)
        {
            length = BOOT_ERASE_CHUNK_SIZE;
        }
        ret = dev_flash_erase(ctx->slot_address + ctx->offset, length);
        if (ret != RET_OK)
        {
            ctx->result = ret;
            return 2U;
        }
        ctx->offset += length;
        percent = (uint32_t)(((uint64_t)ctx->offset * 100ULL) /
                             ctx->param->fw_size);
        boot_install_show(percent, "ERASING SLOT");
        return 0U;

    case BOOT_INSTALL_COPY:
        if (ctx->offset >= ctx->param->fw_size)
        {
            ctx->crc ^= CRC32_XOR_VALUE;
            if (ctx->crc != ctx->param->fw_crc32)
            {
                ctx->result = RET_CRC_ERROR;
                return 2U;
            }
            ctx->phase = BOOT_INSTALL_READBACK;
            ctx->offset = 0U;
            ctx->crc = CRC32_INIT_VALUE;
            ctx->shown_percent = 0xFFFFFFFFUL;
            return 0U;
        }
        {
            uint8_t buffer[BOOT_COPY_BUFFER_SIZE];

            length = ctx->param->fw_size - ctx->offset;
            if (length > sizeof(buffer))
            {
                length = sizeof(buffer);
            }
            ret = dev_gd25q128_read(ctx->param->fw_addr + ctx->offset,
                                    buffer,
                                    (uint16_t)length);
            if (ret != RET_OK)
            {
                ctx->result = ret;
                return 2U;
            }
            ctx->crc = ota_crc32_update(ctx->crc, buffer, length);
            ret = dev_flash_write(ctx->slot_address + ctx->offset,
                                  buffer,
                                  length);
            if (ret != RET_OK)
            {
                ctx->result = ret;
                return 2U;
            }
            ctx->offset += length;
            percent = (uint32_t)(((uint64_t)ctx->offset * 100ULL) /
                                 ctx->param->fw_size);
            boot_install_show(percent, "WRITING SLOT");
        }
        return 0U;

    case BOOT_INSTALL_READBACK:
        if (ctx->offset >= ctx->param->fw_size)
        {
            ctx->crc ^= CRC32_XOR_VALUE;
            if (ctx->crc != ctx->param->fw_crc32)
            {
                ctx->result = RET_CRC_ERROR;
                return 2U;
            }
            ctx->phase = BOOT_INSTALL_VECTOR;
            return 0U;
        }
        length = ctx->param->fw_size - ctx->offset;
        if (length > BOOT_READBACK_CHUNK_SIZE)
        {
            length = BOOT_READBACK_CHUNK_SIZE;
        }
        ctx->crc = ota_crc32_update(
            ctx->crc,
            (const uint8_t *)(uintptr_t)(ctx->slot_address + ctx->offset),
            length);
        ctx->offset += length;
        percent = (uint32_t)(((uint64_t)ctx->offset * 100ULL) /
                             ctx->param->fw_size);
        boot_install_show(percent, "VERIFYING");
        return 0U;

    case BOOT_INSTALL_VECTOR:
        if (boot_slot_check(ctx->param->pending_slot, &msp, &reset) == 0U)
        {
            ctx->result = RET_APP_INVALID;
            return 2U;
        }
        ctx->phase = BOOT_INSTALL_DONE;
        return 1U;

    default:
        return 1U;
    }
}

/*
 * 非阻塞安装：每步之间有界让出（喂狗、刷进度），
 * 擦除/搬运/回读全部切片执行，长镜像也不会拖死看门狗。
 */
static ret_t boot_install_run(const boot_param_t *param)
{
    if ((param->fw_size < 8U) || (param->fw_size > BOOT_SLOT_SIZE))
    {
        return RET_LENGTH_ERROR;
    }
    if (boot_slot_addr(param->pending_slot) == 0U)
    {
        return RET_PARTITION_ERROR;
    }

    boot_install_begin(param);
    for (;;)
    {
        uint8_t step_result;

        (void)bsp_fwdgt_feed();
        step_result = boot_install_step();
        if (step_result == 1U)
        {
            return RET_OK;
        }
        if (step_result == 2U)
        {
            return s_boot_install.result;
        }
    }
}

static void boot_rollback(boot_param_t *param)
{
    DBG_Warning("rollback to slot %lu",
                (unsigned long)param->active_slot);

    param->state = BOOT_STATE_CONFIRMED;
    param->pending_slot = BOOT_SLOT_NONE;
    param->trial_count = 0U;
    param->rollback_flag = 1U;
    DBG_log("param rollback: active=%lu gen=%lu",
            (unsigned long)param->active_slot,
            (unsigned long)param->generation);
    if (boot_param_save(param) != RET_OK)
    {
        DBG_Error("save rollback failed");
    }
}

static void boot_halt(const char *title, const char *line1)
{
    boot_status_show(title, line1, "HALT FOR INSPECTION");
    DBG_Error("boot halted: %s", title);
    for (;;)
    {
    }
}

void boot_ota_run(void)
{
    boot_param_t param;
    dev_gd25q128_jedec_id_t jedec;
    uint32_t msp = 0U;
    uint32_t reset = 0U;
    uint32_t jump_slot;
    uint8_t ext_ok;
    ret_t ret;

    /* 时间基准与引脚必须先�?LCD / SPI / Flash 操作�?*/
    (void)bsp_delay_init();
    (void)bsp_systick_init();
    (void)bsp_gpio_init();
    boot_status_init();
    boot_status_show("EDGELINK BOOT", "STARTING", NULL);

    (void)dev_flash_init();

    ext_ok = 0U;
    ret = dev_gd25q128_init();
    if ((ret == RET_OK) || (ret == RET_ALREADY_INIT))
    {
        (void)memset(&jedec, 0, sizeof(jedec));
        if (dev_gd25q128_jedec_id_get(&jedec) == RET_OK)
        {
            ext_ok = 1U;
        }
    }
    if (ext_ok == 0U)
    {
        DBG_Warning("external flash unavailable; install path disabled");
    }

    ret = boot_param_load(&param);
    if (ret == RET_MAGIC_ERROR)
    {
        /* 首次上电：参数区空白，建立默认记录�?*/
        boot_param_make_default(&param);
        (void)boot_param_save(&param);
        DBG_log("param default created: active=%lu gen=%lu",
                (unsigned long)param.active_slot,
                (unsigned long)param.generation);
    }
    else if (ret != RET_OK)
    {
        boot_halt("PARAM ERROR", "BOOT PARAMETER CORRUPTED");
    }

    /* 升级安装阶段：外部镜像就绪则搬运�?pending 槽�?*/
    if ((param.state == BOOT_STATE_DOWNLOAD_READY) ||
        (param.state == BOOT_STATE_COPYING))
    {
        if (ext_ok == 0U)
        {
            boot_status_show("INSTALL ABORT", "EXTERNAL FLASH MISSING", NULL);
            boot_rollback(&param);
        }
        else
        {
            ret = boot_verify_info(&param);
            if (ret != RET_OK)
            {
                DBG_Error("image header mismatch ret=%ld", (long)ret);
                boot_status_show("INSTALL ABORT", "IMAGE HEADER INVALID", NULL);
                boot_rollback(&param);
            }
            else
            {
                /* 先落 COPYING：中途掉电后下次上电从头重搬�?*/
                param.state = BOOT_STATE_COPYING;
                (void)boot_param_save(&param);
                DBG_log("param state=COPYING gen=%lu",
                        (unsigned long)param.generation);
                boot_status_show("INSTALLING", "COPYING IMAGE",
                                 "DO NOT POWER OFF");

                ret = boot_install_run(&param);
                if (ret != RET_OK)
                {
                    DBG_Error("image copy failed ret=%ld", (long)ret);
                    boot_status_show("INSTALL FAILED", "ROLLING BACK", NULL);
                    boot_rollback(&param);
                }
                else
                {
                    param.state = BOOT_STATE_TRIAL;
                    param.trial_count = 0U;
                    DBG_log("param state=TRIAL pending=%lu gen=%lu",
                            (unsigned long)param.pending_slot,
                            (unsigned long)param.generation);
                    if (boot_param_save(&param) != RET_OK)
                    {
                        boot_halt("PARAM ERROR", "SAVE TRIAL STATE FAILED");
                    }
                    DBG_log("image installed to slot %lu; trial begins",
                            (unsigned long)param.pending_slot);
                }
            }
        }
    }

    /* 试运行阶段：次数用尽或槽无效即回滚，否则计数后带狗跳转�?*/
    if (param.state == BOOT_STATE_TRIAL)
    {
        if (param.trial_count >= BOOT_MAX_TRIAL_COUNT)
        {
            boot_status_show("TRIAL LIMIT", "ROLLING BACK", NULL);
            boot_rollback(&param);
        }
        else if (boot_slot_check(param.pending_slot, &msp, &reset) == 0U)
        {
            boot_status_show("TRIAL INVALID", "ROLLING BACK", NULL);
            boot_rollback(&param);
        }
        else
        {
            param.trial_count++;
            if (boot_param_save(&param) != RET_OK)
            {
                boot_halt("PARAM ERROR", "SAVE TRIAL COUNT FAILED");
            }

            /* 独立看门�?~10s：新固件必须启动健康任务持续喂狗�?*/
            (void)bsp_fwdgt_init();
            boot_status_show((param.pending_slot == BOOT_SLOT_A) ?
                             "TRIAL RUN A" : "TRIAL RUN B",
                             "NEW FIRMWARE MUST CONFIRM",
                             "WATCHDOG ARMED");
            (void)bsp_delay_ms(BOOT_STATUS_HOLD_MS);
            bootloader_jump(boot_slot_addr(param.pending_slot), msp, reset);
        }
    }

    /* 正常启动：active 槽优先，无效时试另一个槽�?*/
    jump_slot = param.active_slot;
    if (boot_slot_check(jump_slot, &msp, &reset) == 0U)
    {
        jump_slot ^= 1UL;
        if (boot_slot_check(jump_slot, &msp, &reset) == 0U)
        {
            boot_halt("NO VALID APP", "BOTH SLOTS INVALID");
        }
        DBG_Warning("active slot invalid; booting slot %lu",
                    (unsigned long)jump_slot);
    }

    DBG_log("booting slot %lu state=%lu gen=%lu",
            (unsigned long)jump_slot,
            (unsigned long)param.state,
            (unsigned long)param.generation);
    boot_status_show("BOOTING",
                     (jump_slot == BOOT_SLOT_A) ? "SLOT A" : "SLOT B",
                     (param.rollback_flag != 0U) ?
                     "LAST UPGRADE ROLLED BACK" : NULL);
    (void)bsp_delay_ms(BOOT_STATUS_HOLD_MS);
    bootloader_jump(boot_slot_addr(jump_slot), msp, reset);
}
