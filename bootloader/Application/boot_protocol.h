#ifndef __BOOT_PROTOCOL_H__
#define __BOOT_PROTOCOL_H__

#include "config.h"



/* ------------------------- 内部 Flash 分区 ------------------------- */




/* BootLoader 占用 32KB：0x08000000～0x08007FFF。 */
#define BOOT_LOADER_SIZE           0x00008000UL


#define BOOT_SLOT_A_ADDR           0x08008000UL


#define BOOT_SLOT_B_ADDR           0x08040000UL

/* 每个固件槽大小为 0x38000，即 224KB。 Slot A：0x08008000～0x0803FFF   Slot B：0x08040000～0x08077FFF*/
#define BOOT_SLOT_SIZE             0x00038000UL   









/* ------------------------- 内部 Flash 分区 ------------------------- */
/* 芯片内部 Flash 起始地址。 */
#define BOOT_FLASH_BASE_ADDR       0x08000000UL

/* 512KB Flash 的末地址后一字节；最后一个可写地址是 0x0807FFFF。 */
#define BOOT_FLASH_END_ADDR        0x08080000UL

/* BootLoader 固定占前 32KB，因此 Slot A 从 base + 0x8000 开始。 */
#define BOOT_LOADER_SIZE           0x00008000UL


/* A 槽固件起始地址，紧跟在 32KB BootLoader 后。 */
#define BOOT_SLOT_A_ADDR           0x08008000UL

/* B 槽固件起始地址: Slot A 起点加 224KB 等于 0x08040000，正好是 Slot B 起点。 */
#define BOOT_SLOT_B_ADDR           0x08040000UL
/* 
每个固件槽大小为 0x38000，即 224KB。
Slot A：0x08008000～0x0803FFF  
Slot B：0x08040000～0x08077FFF
*/
#define BOOT_SLOT_SIZE             0x00038000UL /* 每槽 224KB */

/* GD32F30x 每页 2KB；最后两页分别保存主、备启动参数记录。 */
#define BOOT_PARAM_PRIMARY_ADDR    0x0807F000UL /*  主参数区  */
#define BOOT_PARAM_BACKUP_ADDR     0x0807F800UL /* 备份参数区 */

/* ------------------------------------------------------------------- */





/* ------------------------- 外部 Flash 分区 ------------------------- */
/* OTA 二进制从 GD25Q128 的第 0 字节连续写入。 */
#define BOOT_FW_DOWNLOAD_ADDR      0x000000UL


/* 外部区域预留 256KB，但 app_ota 还会用 224KB 的 BOOT_SLOT_SIZE 二次限制。
 * 外部 Flash 下载区最大容量为 256KB。
 *
 * 实际固件还必须满足 fw_size <= BOOT_SLOT_SIZE，
 * 因此当前可升级固件的有效最大值仍然是 224KB。
 */
#define BOOT_FW_DOWNLOAD_MAX_SIZE  (256UL * 1024UL)

/* 固件信息头独占 0x0F0000 所在的 4KB 扇区，在正文 CRC 成功后才写。 */
#define BOOT_FW_INFO_ADDR          0x0F0000UL

/*
 * GD32F303 小端：0x42415442 写入内存后字节为 42 54 41 42，即 "BTAB"。
 * 旧值 0x42544142 字节序为 "BATB"，与注释不一致，本章纠正。
 */
#define BOOT_PARAM_MAGIC           0x42415442UL

/*
 * 启动参数结构格式版本。
 *
 * 当 boot_param_t 的字段含义或布局发生变化时，
 * 应同步修改该版本号，防止新程序误解析旧参数。
 */
#define BOOT_PARAM_FORMAT_VERSION  1UL

/* 0x474D5746 小端字节为 46 57 4D 47，即 "FWMG"。 */
#define BOOT_FW_INFO_MAGIC         0x474D5746UL

/* A 槽编号。 */
#define BOOT_SLOT_A                0UL

/* B 槽编号。 */
#define BOOT_SLOT_B                1UL

/* 表示当前不存在待升级槽。 */
#define BOOT_SLOT_NONE             0xFFFFFFFFUL

/*
 * 新固件允许试运行的最大次数。
 *
 * BootLoader 每次进入未确认的新固件时增加 trial_count。
 * 超过该次数后，应回滚到 active_slot。
 */
#define BOOT_MAX_TRIAL_COUNT       5UL

/*
 * 兼容当前 Keil Target 已使用的宏名。
 *
 * 原工程如果仍使用 COMPONENT_BOOT_AB_SLOT_A/B，
 * 可以继续编译，不需要一次性修改所有旧代码。
 */
#define COMPONENT_BOOT_AB_SLOT_A   BOOT_SLOT_A
#define COMPONENT_BOOT_AB_SLOT_B   BOOT_SLOT_B

/*
 * 兼容旧的当前槽配置宏。
 *
 * 当工程没有直接定义 APP_CURRENT_SLOT，
 * 但已经定义 COMPONENT_BOOT_AB_CURRENT_SLOT 时，
 * 自动将旧宏映射为新的 APP_CURRENT_SLOT。
 */
#if !defined(APP_CURRENT_SLOT) && defined(COMPONENT_BOOT_AB_CURRENT_SLOT)
#define APP_CURRENT_SLOT COMPONENT_BOOT_AB_CURRENT_SLOT
#endif


/*
 * BootLoader A/B 升级状态。
 *
 * 状态变化通常为：
 *
 * CONFIRMED
 *     ↓ App 完成固件下载和校验
 * DOWNLOAD_READY
 *     ↓ BootLoader 准备复制固件
 * COPYING
 *     ↓ 固件复制完成
 * TRIAL
 *     ↓ 新 App 自检成功并主动确认
 * CONFIRMED
 */
typedef enum
{
   
    BOOT_STATE_CONFIRMED = 0, /* 当前 active_slot 中运行的是已经确认的稳定固件。 */

    BOOT_STATE_DOWNLOAD_READY,/* 外部 Flash 中的新固件已经下载完成，等待 BootLoader 处理。 */

    BOOT_STATE_COPYING,/* BootLoader 正在将外部 Flash 固件复制到 pending_slot。 */

    BOOT_STATE_TRIAL /* 新固件已经写入目标槽，当前处于试运行阶段。 */
} boot_state_t;

/* 13 个 uint32_t，共 52 字节。每次只更新主备页中较旧的一页。 */
typedef struct
{
    uint32_t magic;
    uint32_t format_version;
    uint32_t generation;     /* 每次保存加 1，选最新有效副本 */
    uint32_t state;          /* boot_state_t */
    uint32_t active_slot;    /* 已确认的旧槽，回滚目标 */
    uint32_t pending_slot;   /* 正在升级的新槽；无则为 NONE */
    uint32_t trial_count;
    uint32_t rollback_flag;
    uint32_t fw_addr;
    uint32_t fw_size;
    uint32_t fw_crc32;
    uint32_t fw_version;
    uint32_t param_crc32;    /* 本字段之前所有字节的 CRC32 */
} boot_param_t;
/* 7 个 uint32_t，共 28 字节。App 在固件校验完成后最后写入。 */
typedef struct
{
    uint32_t magic;
    uint32_t fw_addr;
    uint32_t fw_size;
    uint32_t fw_crc32;
    uint32_t fw_version;
    uint32_t target_slot;
    uint32_t header_crc32;
} boot_fw_info_t;

static __inline bool boot_slot_is_valid(uint32_t slot)
{
    return (slot == BOOT_SLOT_A) || (slot == BOOT_SLOT_B);
}

static __inline uint32_t boot_slot_addr(uint32_t slot)
{
    if (slot == BOOT_SLOT_A) return BOOT_SLOT_A_ADDR;
    if (slot == BOOT_SLOT_B) return BOOT_SLOT_B_ADDR;
    return 0UL;
}


#endif /* __BOOT_PROTOCOL_H__ */




