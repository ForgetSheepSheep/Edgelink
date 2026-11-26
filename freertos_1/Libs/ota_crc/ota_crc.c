#include "ota_crc/ota_crc.h"

/*
 * 设计说明：逐字节查表法。
 * 老写法是每字节内层再循环 8 次逐位移位，224KB 固件要跑约 180 万次分支，
 * 在 M4 上做 finish 全量复验时明显变慢。查表法把 8 次位运算压缩成一次查表，
 * 每个字节只做一次异或和一次移位，结果与逐位法完全一致（同一多项式）。
 */

#define OTA_CRC_TABLE_SIZE 256U

static uint32_t s_crc_table[OTA_CRC_TABLE_SIZE];
static bool s_crc_table_ready = false;

/* 按反射多项式生成 256 项查找表，首字节任意取值 0x00~0xFF 的 8 步演化结果。 */
static void ota_crc_table_ensure(void)
{
    uint32_t value;
    uint32_t index;
    uint32_t bit;

    if (s_crc_table_ready)
    {
        return;
    }

    for (index = 0U; index < OTA_CRC_TABLE_SIZE; index++)
    {
        value = index;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((value & 1U) != 0U)
            {
                value = (value >> 1) ^ CRC32_POLY_VALUE;
            }
            else
            {
                value >>= 1;
            }
        }
        s_crc_table[index] = value;
    }

    s_crc_table_ready = true;
}

uint32_t ota_crc32_update(uint32_t crc,
                          const uint8_t *buf,
                          uint32_t len)
{
    ota_crc_table_ensure();

    if (buf == NULL)
    {
        return crc;
    }

    while (len > 0U)
    {
        /* 低 8 位与新字节混合后查表，高 24 位右移保留。 */
        crc = s_crc_table[(uint8_t)(crc ^ *buf)] ^ (crc >> 8);
        buf++;
        len--;
    }

    return crc;
}

uint32_t ota_crc32_calc(const uint8_t *buf, uint32_t len)
{
    /* update 已处理 buf==NULL（原值返回），空输入的 CRC32 按标准为 0。 */
    return ota_crc32_update(CRC32_INIT_VALUE, buf, len) ^ CRC32_XOR_VALUE;
}
