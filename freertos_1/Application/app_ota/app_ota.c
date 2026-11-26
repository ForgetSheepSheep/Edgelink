#include "app_ota.h"
#include "../app_boot_ab/boot_protocol.h"
#include "../app_boot_ab/boot_param.h"
#include "ota_crc/ota_crc.h"
#include "dev_flash/dev_flash.h"
#include "dev_gd25q128/dev_gd25q128.h"
#include "gd32f30x.h"
#include <stddef.h>

#ifndef APP_CURRENT_SLOT
#error "App A/B target must define APP_CURRENT_SLOT (via COMPONENT_BOOT_AB_CURRENT_SLOT)"
#endif

#if (APP_CURRENT_SLOT != BOOT_SLOT_A) && (APP_CURRENT_SLOT != BOOT_SLOT_B)
#error "APP_CURRENT_SLOT must be BOOT_SLOT_A or BOOT_SLOT_B"
#endif

/*
 * 单片写入上限：传输层分片通常 256B，这里放宽到 1KB，
 * 核心只做上限拦截，具体分片大小由调用方决定。
 */
#define OTA_CORE_CHUNK_MAX 1024U

/* 全量复验时的暂存缓冲：512B，远小于驱动单次读取上限 0xFFFF。 */
#define OTA_VERIFY_BUF_SIZE 512U

/* GD32F303ZE 片上资源边界（向量头合法性用，不依赖 Bootloader 头文件）。 */
#define OTA_SRAM_BASE  0x20000000UL
#define OTA_SRAM_SIZE  0x00010000UL
#define OTA_FLASH_BASE 0x08000000UL
#define OTA_FLASH_END  0x08080000UL

/* GD25Q128 出厂 JEDEC：C8=兆易，40=SPI NOR，18=16MB。 */
#define OTA_JEDEC_MANUFACTURER 0xC8U
#define OTA_JEDEC_MEMORY_TYPE  0x40U
#define OTA_JEDEC_CAPACITY     0x18U

typedef struct
{
    app_ota_state_t state;
    app_ota_meta_t meta;
    uint32_t received;
    uint32_t stream_crc; /* 写分片时累积的中间值，finish 时才做最终异或 */
} ota_context_t;

static ota_context_t s_ctx;
/** 复位前钩子：组合根用它把"即将重启"显示到 LCD。 */
static app_ota_reboot_hook_t s_reboot_hook = NULL;

/* 信息头 CRC 只覆盖 header_crc32 之前的字段，自身不参与。 */
static uint32_t ota_info_crc(const boot_fw_info_t *info)
{
    return ota_crc32_calc((const uint8_t *)info,
                          (uint32_t)offsetof(boot_fw_info_t, header_crc32));
}

/*
 * 按 4KB 扇区逐个擦除 [addr, addr + len)。
 * 调用方保证 addr 扇区对齐（下载区 0x000000 与信息头 0x0F0000 天然对齐）。
 */
static ret_t ota_erase_range(uint32_t addr, uint32_t len)
{
    uint32_t sector;
    uint32_t end;
    ret_t ret;

    if (len == 0U)
    {
        return RET_OK;
    }

    end = addr + len;
    for (sector = addr; sector < end; sector += DEV_GD25Q128_SECTOR_SIZE)
    {
        ret = dev_gd25q128_sector_erase(sector);
        if (ret != RET_OK)
        {
            return ret;
        }
    }
    return RET_OK;
}

/*
 * 驱动单次读入参是 uint16_t，这里统一拆成小段循环，
 * 上层传任意长度都不会截断。
 */
static ret_t ota_ext_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t step;
    ret_t ret;

    while (len > 0U)
    {
        step = (len > 0xFFFFU) ? 0xFFFFU : len;
        ret = dev_gd25q128_read(addr, buf, (uint16_t)step);
        if (ret != RET_OK)
        {
            return ret;
        }
        addr += step;
        buf += step;
        len -= step;
    }
    return RET_OK;
}

/* 把外部暂存区从头到尾重读一遍，与期望 CRC 比对。 */
static ret_t ota_verify_staged(uint32_t size, uint32_t expected_crc)
{
    uint8_t buf[OTA_VERIFY_BUF_SIZE];
    uint32_t offset = 0U;
    uint32_t step;
    uint32_t crc = CRC32_INIT_VALUE;
    ret_t ret;

    while (offset < size)
    {
        step = size - offset;
        if (step > sizeof(buf))
        {
            step = sizeof(buf);
        }
        ret = ota_ext_read(BOOT_FW_DOWNLOAD_ADDR + offset, buf, step);
        if (ret != RET_OK)
        {
            return ret;
        }
        crc = ota_crc32_update(crc, buf, step);
        offset += step;
    }

    crc ^= CRC32_XOR_VALUE;
    if (crc != expected_crc)
    {
        DBG_Error("OTA finish CRC mismatch (flash readback): got=0x%08lX exp=0x%08lX\r\n",
                  (unsigned long)crc,
                  (unsigned long)expected_crc);
        return RET_CRC_ERROR;
    }
    return RET_OK;
}

/*
 * 向量头检查：Cortex-M 固件前 8 字节是初始 SP 与 Reset_Handler。
 * SP 必须落在片内 SRAM 且 4 字节对齐；Reset 必须是奇地址（Thumb）
 * 且偶地址部分落在片内 Flash。烂包在这里拦截，不占用提交流程。
 */
static ret_t ota_check_vector(void)
{
    uint8_t head[8];
    uint32_t sp;
    uint32_t reset;
    uint32_t reset_even;
    ret_t ret;

    ret = ota_ext_read(BOOT_FW_DOWNLOAD_ADDR, head, sizeof(head));
    if (ret != RET_OK)
    {
        return ret;
    }

    (void)memcpy(&sp, &head[0], sizeof(sp));
    (void)memcpy(&reset, &head[4], sizeof(reset));

    if ((sp < OTA_SRAM_BASE) ||
        (sp >= (OTA_SRAM_BASE + OTA_SRAM_SIZE)) ||
        ((sp & 3U) != 0U))
    {
        return RET_APP_INVALID;
    }
    if ((reset & 1U) == 0U)
    {
        return RET_APP_INVALID;
    }
    reset_even = reset & ~1U;
    if ((reset_even < OTA_FLASH_BASE) || (reset_even >= OTA_FLASH_END))
    {
        return RET_APP_INVALID;
    }
    return RET_OK;
}

ret_t app_ota_init(void)
{
    dev_gd25q128_jedec_id_t jedec;
    ret_t ret;

    (void)memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = APP_OTA_STATE_IDLE;

    ret = dev_flash_init();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        return ret;
    }

    ret = dev_gd25q128_init();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        return ret;
    }

    ret = dev_gd25q128_jedec_id_get(&jedec);
    if (ret != RET_OK)
    {
        return ret;
    }

    if ((jedec.manufacturer != OTA_JEDEC_MANUFACTURER) ||
        (jedec.memory_type != OTA_JEDEC_MEMORY_TYPE) ||
        (jedec.capacity != OTA_JEDEC_CAPACITY))
    {
        return RET_DEVICE_NOT_FOUND;
    }
    return RET_OK;
}

ret_t app_ota_start(const app_ota_meta_t *meta)
{
    boot_param_t param;
    uint32_t max_size;
    uint32_t peer;
    ret_t ret;

    if (meta == NULL)
    {
        return RET_NULL_PTR;
    }
    if ((s_ctx.state != APP_OTA_STATE_IDLE) &&
        (s_ctx.state != APP_OTA_STATE_ERROR))
    {
        return RET_BUSY;
    }

    max_size = BOOT_SLOT_SIZE;
    if (BOOT_FW_DOWNLOAD_MAX_SIZE < max_size)
    {
        max_size = (uint32_t)BOOT_FW_DOWNLOAD_MAX_SIZE;
    }
    if ((meta->fw_size < 8U) || (meta->fw_size > max_size))
    {
        return RET_LENGTH_ERROR;
    }

    /* A/B 互备：本槽运行就只能写对端槽。 */
    peer = ((uint32_t)APP_CURRENT_SLOT) ^ 1UL;
    if ((meta->target_slot != peer) || !boot_slot_is_valid(meta->target_slot))
    {
        return RET_PARTITION_ERROR;
    }

    /*
     * 叠轮保护：上一轮没确认（TRIAL/DOWNLOAD_READY/COPYING）时拒绝开新轮，
     * 否则回滚链会被覆盖。 virgin 机（参数区全 0xFF）放行。
     */
    ret = boot_param_load(&param);
    if (ret == RET_OK)
    {
        if ((param.state != BOOT_STATE_CONFIRMED) ||
            (param.active_slot != (uint32_t)APP_CURRENT_SLOT))
        {
            return RET_INVALID_STATE;
        }
    }
    else if (ret != RET_MAGIC_ERROR)
    {
        return ret;
    }

    s_ctx.state = APP_OTA_STATE_PREPARING;

    /* 先毒死旧信息头：中途掉电时 Bootloader 看不到有效头，不会误搬运。 */
    ret = ota_erase_range(BOOT_FW_INFO_ADDR, DEV_GD25Q128_SECTOR_SIZE);
    if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }

    ret = ota_erase_range(BOOT_FW_DOWNLOAD_ADDR, meta->fw_size);
    if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }

    (void)memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = APP_OTA_STATE_DOWNLOADING;
    s_ctx.meta = *meta;
    s_ctx.stream_crc = CRC32_INIT_VALUE;
    return RET_OK;
}

ret_t app_ota_write_chunk(uint32_t offset, const uint8_t *data, uint32_t len)
{
    ret_t ret;

    if (s_ctx.state != APP_OTA_STATE_DOWNLOADING)
    {
        return RET_INVALID_STATE;
    }
    if ((data == NULL) || (len == 0U) || (len > OTA_CORE_CHUNK_MAX))
    {
        return RET_INVALID_PARAM;
    }
    /* 严格顺序：不接受重传覆盖，也不接受跳号，保证流式 CRC 与外存一致。 */
    if (offset != s_ctx.received)
    {
        return RET_SEQUENCE_ERROR;
    }
    if (len > (s_ctx.meta.fw_size - s_ctx.received))
    {
        return RET_SEQUENCE_ERROR;
    }

    /*
     * 逐片写后不做读回：最终 finish 会全量重读复验，
     * 逐片回读会让 SPI 流量翻倍，224KB 包多花一倍时间却只提前发现错误。
     */
    ret = dev_gd25q128_write(BOOT_FW_DOWNLOAD_ADDR + offset, data, len);
    if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }

    s_ctx.stream_crc = ota_crc32_update(s_ctx.stream_crc, data, len);
    s_ctx.received += len;
    return RET_OK;
}

ret_t app_ota_finish(void)
{
    boot_fw_info_t info;
    boot_fw_info_t check;
    boot_param_t param;
    uint32_t stream_final;
    ret_t ret;

    if ((s_ctx.state != APP_OTA_STATE_DOWNLOADING) ||
        (s_ctx.received != s_ctx.meta.fw_size))
    {
        return RET_INVALID_STATE;
    }

    s_ctx.state = APP_OTA_STATE_VERIFYING;

    /* 第一闸：内存流 CRC 必须与云端声明一致（fw_crc32 已由传输层归一化字节序）。 */
    stream_final = s_ctx.stream_crc ^ CRC32_XOR_VALUE;
    if (stream_final != s_ctx.meta.fw_crc32)
    {
        DBG_Error("OTA finish CRC mismatch (stream): got=0x%08lX exp=0x%08lX rx=%lu\r\n",
                  (unsigned long)stream_final,
                  (unsigned long)s_ctx.meta.fw_crc32,
                  (unsigned long)s_ctx.received);
        s_ctx.state = APP_OTA_STATE_ERROR;
        return RET_CRC_ERROR;
    }

    /* 第二闸：外存重读 CRC，防止 SPI 写入丢位或掉电残留。 */
    ret = ota_verify_staged(s_ctx.meta.fw_size, s_ctx.meta.fw_crc32);
    if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }

    /* 第三闸：向量头合法性，烂包到此为止。 */
    ret = ota_check_vector();
    if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }

    s_ctx.state = APP_OTA_STATE_COMMITTING;

    (void)memset(&info, 0, sizeof(info));
    info.magic = BOOT_FW_INFO_MAGIC;
    info.fw_addr = BOOT_FW_DOWNLOAD_ADDR;
    info.fw_size = s_ctx.meta.fw_size;
    info.fw_crc32 = s_ctx.meta.fw_crc32;
    info.fw_version = s_ctx.meta.fw_version;
    info.target_slot = s_ctx.meta.target_slot;
    info.header_crc32 = ota_info_crc(&info);

    ret = dev_gd25q128_write(BOOT_FW_INFO_ADDR, (const uint8_t *)&info, sizeof(info));
    if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }

    /* 信息头写后必须回读比对：它是 Bootloader 搬运的唯一依据。 */
    (void)memset(&check, 0, sizeof(check));
    ret = ota_ext_read(BOOT_FW_INFO_ADDR, (uint8_t *)&check, sizeof(check));
    if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }
    if (memcmp(&info, &check, sizeof(info)) != 0)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return RET_FLASH_VERIFY_ERROR;
    }

    ret = boot_param_load(&param);
    if (ret == RET_MAGIC_ERROR)
    {
        boot_param_make_default(&param);
        param.active_slot = (uint32_t)APP_CURRENT_SLOT;
    }
    else if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }

    param.state = BOOT_STATE_DOWNLOAD_READY;
    param.pending_slot = s_ctx.meta.target_slot;
    param.trial_count = 0U;
    param.rollback_flag = 0U;
    param.fw_addr = BOOT_FW_DOWNLOAD_ADDR;
    param.fw_size = s_ctx.meta.fw_size;
    param.fw_crc32 = s_ctx.meta.fw_crc32;
    param.fw_version = s_ctx.meta.fw_version;

    ret = boot_param_save(&param);
    if (ret != RET_OK)
    {
        s_ctx.state = APP_OTA_STATE_ERROR;
        return ret;
    }

    /* 提交成功后回 IDLE：参数区已记录一切，RAM 状态不再重要，下一轮可直接 start。 */
    (void)memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = APP_OTA_STATE_IDLE;
    return RET_OK;
}

void app_ota_abort(void)
{
    (void)memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = APP_OTA_STATE_IDLE;
}

void app_ota_reboot(void)
{
    if (s_reboot_hook != NULL)
    {
        s_reboot_hook();
    }
    NVIC_SystemReset();
}

void app_ota_set_reboot_hook(app_ota_reboot_hook_t hook)
{
    s_reboot_hook = hook;
}

ret_t app_boot_confirm_running(void)
{
    boot_param_t param;
    ret_t ret;

    ret = boot_param_load(&param);
    if (ret == RET_MAGIC_ERROR)
    {
        return RET_OK;
    }
    if (ret != RET_OK)
    {
        return ret;
    }
    if (param.state != BOOT_STATE_TRIAL)
    {
        return RET_OK;
    }
    if (param.pending_slot != (uint32_t)APP_CURRENT_SLOT)
    {
        return RET_INVALID_STATE;
    }

    param.active_slot = param.pending_slot;
    param.pending_slot = BOOT_SLOT_NONE;
    param.state = BOOT_STATE_CONFIRMED;
    param.trial_count = 0U;
    param.rollback_flag = 0U;
    return boot_param_save(&param);
}

app_ota_state_t app_ota_get_state(void)
{
    return s_ctx.state;
}

void app_ota_get_progress(app_ota_progress_t *progress)
{
    if (progress == NULL)
    {
        return;
    }
    progress->received = s_ctx.received;
    progress->total = s_ctx.meta.fw_size;
}
