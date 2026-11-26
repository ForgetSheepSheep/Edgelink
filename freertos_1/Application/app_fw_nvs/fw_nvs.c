#include "app_fw_nvs/fw_nvs.h"
#include "../app_boot_ab/boot_protocol.h"
#include "ota_crc/ota_crc.h"
#include "dev_flash/dev_flash.h"
#include <stddef.h>

/* 页内记录：28 字节载荷 + 4 字节 CRC，必须恰好 32 字节以对齐槽位。 */
typedef struct
{
    uint32_t magic;
    uint32_t format_version;
    uint32_t seq;
    uint32_t fw_version;
    uint32_t rollback_flag;
    uint32_t active_slot;
    uint32_t reserved;
    uint32_t crc32; /* 以上 7 个字段的 CRC32，本字段不参与 */
} fw_nvs_record_t;

typedef char fw_nvs_record_size_check
    [(sizeof(fw_nvs_record_t) == FW_NVS_RECORD_SIZE) ? 1 : -1];

typedef struct
{
    bool has_valid;
    uint32_t best_seq;
    uint32_t best_index;
    fw_nvs_record_t best;
    bool has_empty;
    uint32_t first_empty;
    uint32_t non_empty_count;
} fw_nvs_scan_t;

static uint8_t s_fw_nvs_inited = 0U;

static uint32_t fw_nvs_record_crc(const fw_nvs_record_t *rec)
{
    return ota_crc32_calc((const uint8_t *)rec,
                          (uint32_t)offsetof(fw_nvs_record_t, crc32));
}

static bool fw_nvs_slot_erased(const fw_nvs_record_t *rec)
{
    const uint8_t *bytes = (const uint8_t *)rec;
    uint32_t i;

    for (i = 0U; i < sizeof(*rec); i++)
    {
        if (bytes[i] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

/* seq 回绕安全比较：差值转有符号，64 槽跨度下恒正确（同 boot_param 的 generation）。 */
static bool fw_nvs_seq_newer(uint32_t a, uint32_t b)
{
    return ((int32_t)(a - b) > 0);
}

static bool fw_nvs_record_valid(const fw_nvs_record_t *rec)
{
    if ((rec->magic != FW_NVS_MAGIC) ||
        (rec->format_version != FW_NVS_FORMAT_VERSION))
    {
        return false;
    }
    if ((rec->rollback_flag > 1U) ||
        ((rec->active_slot != BOOT_SLOT_A) &&
         (rec->active_slot != BOOT_SLOT_B) &&
         (rec->active_slot != BOOT_SLOT_NONE)))
    {
        return false;
    }
    return rec->crc32 == fw_nvs_record_crc(rec);
}

static ret_t fw_nvs_scan(fw_nvs_scan_t *scan)
{
    fw_nvs_record_t rec;
    uint32_t i;
    ret_t ret;

    (void)memset(scan, 0, sizeof(*scan));

    for (i = 0U; i < FW_NVS_SLOT_COUNT; i++)
    {
        ret = dev_flash_read(FW_NVS_PAGE_ADDR + i * FW_NVS_RECORD_SIZE,
                             (uint8_t *)&rec, sizeof(rec));
        if (ret != RET_OK)
        {
            return ret;
        }

        if (fw_nvs_slot_erased(&rec))
        {
            if (!scan->has_empty)
            {
                scan->has_empty = true;
                scan->first_empty = i;
            }
            continue;
        }

        scan->non_empty_count++;

        if (!fw_nvs_record_valid(&rec))
        {
            continue; /* 被打断的半条记录：跳过，旧有效记录仍在 */
        }

        if (!scan->has_valid || fw_nvs_seq_newer(rec.seq, scan->best_seq))
        {
            scan->has_valid = true;
            scan->best_seq = rec.seq;
            scan->best_index = i;
            scan->best = rec;
        }
    }
    return RET_OK;
}

static ret_t fw_nvs_write_slot(uint32_t index, const fw_nvs_record_t *rec)
{
    fw_nvs_record_t check;
    ret_t ret;

    ret = dev_flash_write(FW_NVS_PAGE_ADDR + index * FW_NVS_RECORD_SIZE,
                          (uint8_t *)rec, sizeof(*rec));
    if (ret != RET_OK)
    {
        return ret;
    }

    (void)memset(&check, 0, sizeof(check));
    ret = dev_flash_read(FW_NVS_PAGE_ADDR + index * FW_NVS_RECORD_SIZE,
                         (uint8_t *)&check, sizeof(check));
    if (ret != RET_OK)
    {
        return ret;
    }

    return (memcmp(rec, &check, sizeof(*rec)) == 0)
         ? RET_OK
         : RET_FLASH_VERIFY_ERROR;
}

ret_t fw_nvs_init(void)
{
    ret_t ret;

    if (s_fw_nvs_inited != 0U)
    {
        return RET_OK;
    }

    ret = dev_flash_init();
    if ((ret != RET_OK) && (ret != RET_ALREADY_INIT))
    {
        return ret;
    }

    s_fw_nvs_inited = 1U;
    return RET_OK;
}

ret_t fw_nvs_format(void)
{
    fw_nvs_record_t slot0;
    uint32_t i;
    ret_t ret;

    if (s_fw_nvs_inited == 0U)
    {
        return RET_NOT_INIT;
    }

    ret = dev_flash_erase(FW_NVS_PAGE_ADDR, FW_NVS_PAGE_SIZE);
    if (ret != RET_OK)
    {
        return ret;
    }

    /* 抽查首槽全 FF，确认擦除真正生效。 */
    (void)memset(&slot0, 0, sizeof(slot0));
    ret = dev_flash_read(FW_NVS_PAGE_ADDR, (uint8_t *)&slot0, sizeof(slot0));
    if (ret != RET_OK)
    {
        return ret;
    }
    for (i = 0U; i < sizeof(slot0); i++)
    {
        if (((const uint8_t *)&slot0)[i] != 0xFFU)
        {
            return RET_FLASH_VERIFY_ERROR;
        }
    }
    return RET_OK;
}

ret_t fw_nvs_set(const fw_nvs_params_t *params)
{
    fw_nvs_scan_t scan;
    fw_nvs_record_t rec;
    uint32_t target;
    ret_t ret;

    if (s_fw_nvs_inited == 0U)
    {
        return RET_NOT_INIT;
    }
    if (params == NULL)
    {
        return RET_NULL_PTR;
    }
    if ((params->rollback_flag > 1U) ||
        ((params->active_slot != BOOT_SLOT_A) &&
         (params->active_slot != BOOT_SLOT_B) &&
         (params->active_slot != BOOT_SLOT_NONE)))
    {
        return RET_NVS_INVALID_DATA;
    }

    ret = fw_nvs_scan(&scan);
    if (ret != RET_OK)
    {
        return ret;
    }

    (void)memset(&rec, 0, sizeof(rec));
    rec.magic = FW_NVS_MAGIC;
    rec.format_version = FW_NVS_FORMAT_VERSION;
    rec.seq = scan.has_valid ? (scan.best_seq + 1U) : 1U;
    rec.fw_version = params->fw_version;
    rec.rollback_flag = params->rollback_flag;
    rec.active_slot = params->active_slot;
    rec.reserved = 0U;
    rec.crc32 = fw_nvs_record_crc(&rec);

    if (scan.has_empty)
    {
        target = scan.first_empty;
    }
    else
    {
        /*
         * 页满整理：整页擦后把本条写回 0 号槽。
         * 擦除瞬间掉电则整页变空，下次 get 返回 NOT_FOUND（见头文件掉电语义）。
         */
        ret = dev_flash_erase(FW_NVS_PAGE_ADDR, FW_NVS_PAGE_SIZE);
        if (ret != RET_OK)
        {
            return ret;
        }
        target = 0U;
    }

    return fw_nvs_write_slot(target, &rec);
}

ret_t fw_nvs_get(fw_nvs_params_t *params)
{
    fw_nvs_scan_t scan;
    ret_t ret;

    if (s_fw_nvs_inited == 0U)
    {
        return RET_NOT_INIT;
    }
    if (params == NULL)
    {
        return RET_NULL_PTR;
    }

    ret = fw_nvs_scan(&scan);
    if (ret != RET_OK)
    {
        return ret;
    }

    if (scan.has_valid)
    {
        params->fw_version = scan.best.fw_version;
        params->rollback_flag = scan.best.rollback_flag;
        params->active_slot = scan.best.active_slot;
        return RET_OK;
    }

    return (scan.non_empty_count == 0U) ? RET_NVS_NOT_FOUND : RET_CRC_ERROR;
}

static ret_t fw_nvs_remake_flag(uint32_t flag)
{
    fw_nvs_params_t params;
    ret_t ret;

    ret = fw_nvs_get(&params);
    if (ret == RET_NVS_NOT_FOUND)
    {
        /* 从未写入：以空基线建档，版本号未知填 0。 */
        params.fw_version = 0U;
        params.active_slot = BOOT_SLOT_NONE;
    }
    else if (ret != RET_OK)
    {
        return ret;
    }

    params.rollback_flag = flag;
    return fw_nvs_set(&params);
}

ret_t fw_nvs_mark_rollback(void)
{
    if (s_fw_nvs_inited == 0U)
    {
        return RET_NOT_INIT;
    }
    return fw_nvs_remake_flag(1U);
}

ret_t fw_nvs_clear_rollback(void)
{
    if (s_fw_nvs_inited == 0U)
    {
        return RET_NOT_INIT;
    }
    return fw_nvs_remake_flag(0U);
}
