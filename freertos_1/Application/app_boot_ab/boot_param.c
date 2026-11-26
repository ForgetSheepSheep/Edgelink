#include "boot_param.h"
#include "ota_crc/ota_crc.h"
#include "dev_flash/dev_flash.h"

#include <stddef.h>     /* offsetof() */
#include <string.h>     /* memset()、memcmp() */


/**
 * @brief 计算启动参数结构体的 CRC32。
 *
 * @param param 指向启动参数结构体的指针。
 *
 * @return 从结构体起始位置到 param_crc32 字段之前所有字节的 CRC32。
 *
 * @note
 * CRC 计算不能包含 param_crc32 字段本身，否则会出现：
 *
 *     CRC 的计算结果又依赖 CRC 字段自己的值
 *
 * 当前计算范围为：
 *
 *     boot_param_t 起始地址
 *              ↓
 *     +-------------------------+
 *     | magic                   | 参与 CRC
 *     | format_version          | 参与 CRC
 *     | state                   | 参与 CRC
 *     | active_slot             | 参与 CRC
 *     | pending_slot            | 参与 CRC
 *     | generation              | 参与 CRC
 *     | fw_addr                 | 参与 CRC
 *     | fw_size                 | 参与 CRC
 *     | ...                     | 参与 CRC
 *     +-------------------------+
 *     | param_crc32             | 不参与 CRC
 *     +-------------------------+
 *
 * offsetof(boot_param_t, param_crc32) 返回：
 *
 *     param_crc32 字段相对于 boot_param_t 起始地址的字节偏移量
 *
 * 这个偏移量正好可以作为 CRC 计算长度。
 *
 * 注意：
 * 这种写法要求 param_crc32 是结构体最后一个有效字段。
 * 如果 param_crc32 后面还有其他字段，那么后面的字段也不会参与 CRC。
 */
static uint32_t boot_param_crc(const boot_param_t *param)
{
    /*
     * 不能直接使用 sizeof(boot_param_t)：
     *
     * sizeof(boot_param_t) 会包含 param_crc32 字段，
     * 而 CRC 字段本身不能参与 CRC 计算。
     *
     * offsetof(boot_param_t, param_crc32) 表示：
     *
     * 从 boot_param_t 起始地址到 param_crc32 字段之前，
     * 一共有多少个字节。
     */
    return ota_crc32_calc(
        (const uint8_t *)param,
        (uint32_t)offsetof(boot_param_t, param_crc32)
    );
}


/**
 * @brief 判断一条启动参数记录是否处于 Flash 擦除状态。
 *
 * @param param 指向待检查的启动参数结构体。
 *
 * @return true  结构体对应的所有字节全部为 0xFF。
 * @return false 至少有一个字节不是 0xFF。
 *
 * @note
 * Flash 擦除以后，每一位通常都会恢复为 1，因此读取结果为 0xFF。
 *
 * 这个函数用于区分：
 *
 * 1. 参数页从未写入，属于真正的首次启动；
 * 2. 参数页已经写入，但是数据损坏、CRC 错误或内容非法。
 *
 * 不能只检查 magic 是否为 0xFFFFFFFF，因为损坏的数据也可能刚好
 * 导致 magic 无效。必须检查整个结构体是否全部为 0xFF。
 * 因为flash位空的值为 0xFFFFFFFF
 */
static bool boot_param_is_erased(const boot_param_t *param)
{
    /*
     * 将结构体指针转换成字节指针，随后逐字节检查。
     */
    const uint8_t *bytes = (const uint8_t *)param;
    size_t index;

    /*
     * sizeof(*param) 等价于 sizeof(boot_param_t)。
     *
     * 使用 sizeof(*param) 的优点是：
     * 即使以后 param 的类型发生变化，这里的代码也不需要同步修改。
     */
    for (index = 0U; index < sizeof(*param); index++)
    {
        /*
         * 只要发现一个字节不是 Flash 擦除值 0xFF，
         * 就说明这个参数记录曾经被写入过。
         */
        if (bytes[index] != 0xFFU)
        {
            return false;
        }
    }

    /*
     * 全部字节均为 0xFF，说明该记录处于擦除状态。
     */
    return true;
}


/**
 * @brief 完整检查一条启动参数记录是否合法。
 *
 * @param param 指向待校验的启动参数结构体。
 *
 * @return true  标识、版本、字段组合和 CRC 均有效。
 * @return false 参数记录无效。
 *
 * @note
 * 本函数不仅检查 CRC，还检查字段之间的逻辑关系。
 *
 * 仅仅 CRC 正确并不代表参数一定合理。例如：
 *
 * - active_slot 既不是 A 也不是 B；
 * - pending_slot 与 active_slot 相同；
 * - 固件长度超过槽位容量；
 * - 已确认状态下仍然存在待启动槽；
 * - 非试运行状态下 trial_count 不为 0。
 *
 * 这些数据即使 CRC 正确，也不能被 Bootloader 接受。
 */
static bool boot_param_valid(const boot_param_t *param)
{
    /*
     * 第一层校验：检查固定标识和结构体格式版本。
     *
     * magic 用于判断这块数据是不是 boot_param_t 参数记录。
     * format_version 用于判断 Flash 中的数据格式是否与当前程序兼容。
     */
    if ((param->magic != BOOT_PARAM_MAGIC) ||
        (param->format_version != BOOT_PARAM_FORMAT_VERSION))
    {
        return false;
    }

    /*
     * 第二层校验：检查所有独立字段的基本取值范围。
     *
     * active_slot：
     * 当前正在运行或者已经确认的固件槽，必须是 A 或 B。
     *
     * state：
     * 不能超过系统定义的最大状态 BOOT_STATE_TRIAL。
     *
     * rollback_flag：
     * 作为布尔标志，只允许为 0 或 1。
     *
     * trial_count：
     * 试启动次数不能超过系统允许的最大次数。
     */
    if (!boot_slot_is_valid(param->active_slot) ||
        (param->state > BOOT_STATE_TRIAL) ||
        (param->rollback_flag > 1U) ||
        (param->trial_count > BOOT_MAX_TRIAL_COUNT))
    {
        return false;
    }

    /*
     * 第三层校验：根据当前升级状态检查字段组合。
     */
    if (param->state == BOOT_STATE_CONFIRMED)
    {
        /*
         * BOOT_STATE_CONFIRMED 表示当前固件已经稳定运行并得到确认。
         *
         * 在该状态下：
         *
         * 1. 不应该存在待启动槽；
         * 2. 不应该保留试启动次数。
         */
        if ((param->pending_slot != BOOT_SLOT_NONE) ||
            (param->trial_count != 0U))
        {
            return false;
        }
    }
    else
    {
        /*
         * 除 CONFIRMED 以外的状态表示系统正处于升级流程中。
         *
         * 此时参数必须描述一份已经下载或正在处理的待升级固件。
         */

        /*
         * pending_slot 必须是有效的 A/B 槽。
         *
         * pending_slot 不能与 active_slot 相同：
         * A/B OTA 必须将新固件写入当前非活动槽。
         *
         * fw_addr 必须等于固定下载区地址：
         * 表示待处理固件存放于指定的固件下载区。
         *
         * fw_size 至少为 8 字节：
         * 对 Cortex-M 固件来说，前 8 字节通常至少包含：
         *
         *     初始 MSP
         *     Reset_Handler 地址
         *
         * fw_size 不能超过：
         *
         *     目标固件槽容量 BOOT_SLOT_SIZE
         *
         * 也不能超过：
         *
         *     固件下载区最大容量 BOOT_FW_DOWNLOAD_MAX_SIZE
         */
        if (!boot_slot_is_valid(param->pending_slot) ||
            (param->pending_slot == param->active_slot) ||
            (param->fw_addr != BOOT_FW_DOWNLOAD_ADDR) ||
            (param->fw_size < 8U) ||
            (param->fw_size > BOOT_SLOT_SIZE) ||
            (param->fw_size > BOOT_FW_DOWNLOAD_MAX_SIZE))
        {
            return false;
        }

        /*
         * 只有进入 BOOT_STATE_TRIAL 状态以后，
         * 才允许 trial_count 大于 0。
         *
         * 下载完成、等待搬运或固件搬运阶段尚未真正启动新固件，
         * 因此试启动次数必须保持为 0。
         */
        if ((param->state != BOOT_STATE_TRIAL) &&
            (param->trial_count != 0U))
        {
            return false;
        }
    }

    /*
     * 最后一层校验：重新计算 CRC，并与 Flash 中保存的 CRC 比较。
     *
     * 只有前面的字段逻辑检查和这里的 CRC 检查都通过，
     * 参数记录才被认为有效。
     */
    return param->param_crc32 == boot_param_crc(param);
}


/**
 * @brief 在 RAM 中建立首次启动时使用的默认参数。
 *
 * @param param 指向需要初始化的启动参数结构体。
 *
 * @note
 * 本函数只修改 RAM 中的结构体，不会擦除或写入 Flash。
 *
 * 如果需要把默认参数保存到 Flash，调用者还需要执行：
 *
 *     boot_param_save(param);
 */
void boot_param_make_default(boot_param_t *param)
{
    /*
     * 防止空指针访问。
     */
    if (param == NULL)
    {
        return;
    }

    /*
     * 先将整个结构体清零。
     *
     * 这样不仅会初始化所有显式字段，还可以把结构体中的填充字节
     * 一并清零，避免未初始化字节影响 CRC。
     */
    (void)memset(param, 0, sizeof(*param));

    /*
     * 设置参数记录固定标识。
     */
    param->magic = BOOT_PARAM_MAGIC;

    /*
     * 设置当前参数结构体格式版本。
     */
    param->format_version = BOOT_PARAM_FORMAT_VERSION;

    /*
     * 首次启动时默认认为槽 A 中的出厂固件已经确认。
     */
    param->state = BOOT_STATE_CONFIRMED;
    param->active_slot = BOOT_SLOT_A;

    /*
     * 当前没有等待启动的新固件。
     */
    param->pending_slot = BOOT_SLOT_NONE;

    /*
     * 此处没有主动填写 param_crc32。
     *
     * 调用 boot_param_save() 时会：
     *
     * 1. generation 加一；
     * 2. 重新计算 param_crc32；
     * 3. 写入 Flash。
     */
}


/**
 * @brief 从主参数页和备份参数页中加载最新的有效参数。
 *
 * @param param 用于接收最终加载结果的结构体指针。
 *
 * @return RET_OK               成功加载有效参数。
 * @return RET_NULL_PTR         param 为空指针。
 * @return RET_FLASH_READ_ERROR 至少有一个参数页读取失败。
 * @return RET_MAGIC_ERROR      两个参数页都为全 0xFF，表示首次启动。
 * @return RET_CRC_ERROR        两个参数页都非空，但都不是有效记录。
 *
 * @note
 * 系统维护主、备两份参数记录：
 *
 *     主参数页 primary
 *     备参数页 backup
 *
 * 如果两份记录都有效，则比较 generation，选择更新的一份。
 *
 * 如果只有一份有效，则使用有效的那一份。
 *
 * 如果两份都无效，则进一步判断：
 *
 * - 两页都是 0xFF：首次启动；
 * - 两页不是全 0xFF：参数曾经写入，但已经损坏。
 */
ret_t boot_param_load(boot_param_t *param)
{
    boot_param_t primary;
    boot_param_t backup;

    ret_t primary_ret;
    ret_t backup_ret;

    bool primary_ok;
    bool backup_ok;

    /*
     * 检查输出参数指针。
     */
    if (param == NULL)
    {
        return RET_NULL_PTR;
    }

    /*
     * 在读取 Flash 前，先把两个局部结构体全部初始化为 0xFF。
     *
     * 这样即使底层读取函数失败或只读取了部分数据，
     * 后续也不会对未初始化的栈内存进行校验。
     */
    (void)memset(&primary, 0xFF, sizeof(primary));
    (void)memset(&backup, 0xFF, sizeof(backup));

    /*
     * 读取主参数页。
     */
    primary_ret = dev_flash_read(
        BOOT_PARAM_PRIMARY_ADDR,
        (uint8_t *)&primary,
        sizeof(primary)
    );

    /*
     * 读取备份参数页。
     */
    backup_ret = dev_flash_read(
        BOOT_PARAM_BACKUP_ADDR,
        (uint8_t *)&backup,
        sizeof(backup)
    );

    /*
     * 参数页只有在以下两个条件同时满足时才有效：
     *
     * 1. Flash 读取成功；
     * 2. 参数内容通过 boot_param_valid() 完整校验。
     */
    primary_ok = (primary_ret == RET_OK) &&
                 boot_param_valid(&primary);

    backup_ok = (backup_ret == RET_OK) &&
                boot_param_valid(&backup);

    /*
     * 两份参数都无效。
     */
    if (!primary_ok && !backup_ok)
    {
        /*
         * 只要有一页发生底层读取错误，就不能把问题解释成
         * 首次启动或者普通 CRC 损坏。
         */
        if ((primary_ret != RET_OK) ||
            (backup_ret != RET_OK))
        {
            return RET_FLASH_READ_ERROR;
        }

        /*
         * 两页均读取成功，但内容校验失败。
         *
         * 只有两页的所有字节都为 0xFF 时，才认为系统从未保存过参数，
         * 即真正的首次启动。
         *
         * 如果不是全 0xFF，说明参数页曾经写入过，只是当前内容损坏、
         * CRC 错误、格式版本不匹配或字段组合非法。
         */
        return boot_param_is_erased(&primary) &&
               boot_param_is_erased(&backup)
             ? RET_MAGIC_ERROR
             : RET_CRC_ERROR;
    }

    /*
     * 主、备两份参数都有效，需要根据 generation 选择较新的记录。
     */
    if (primary_ok && backup_ok)
    {
        /*
         * generation 是 uint32_t，会在达到 0xFFFFFFFF 后自然回绕到 0。
         *
         * 不能简单写成：
         *
         *     primary.generation > backup.generation
         *
         * 例如：
         *
         *     primary.generation = 0
         *     backup.generation  = 0xFFFFFFFF
         *
         * 实际上 primary 是回绕之后的新记录，但普通无符号比较会认为
         * backup 更大。
         *
         * 使用有符号差值：
         *
         *     (int32_t)(primary - backup) > 0
         *
         * 可以在两次记录差值不超过 INT32_MAX 的前提下正确处理自然回绕。
         */
        *param = ((int32_t)(primary.generation -
                            backup.generation) > 0)
               ? primary
               : backup;
    }
    else
    {
        /*
         * 只有一份参数有效，直接使用有效副本。
         */
        *param = primary_ok ? primary : backup;
    }

    return RET_OK;
}


/**
 * @brief 将 RAM 中的启动参数保存到主参数页或备份参数页。
 *
 * @param param 指向准备保存的启动参数结构体。
 *
 * @return RET_OK                 保存并回读校验成功。
 * @return RET_NULL_PTR           param 为空指针。
 * @return RET_INVALID_PARAM      参数字段组合非法。
 * @return RET_FLASH_READ_ERROR   读取原参数页或回读验证失败。
 * @return RET_FLASH_VERIFY_ERROR 写入后的数据与 RAM 数据不一致。
 * @return 其他错误码             Flash 擦除或写入操作失败。
 *
 * @note
 * 保存策略不是同时覆盖主、备两个参数页，而是：
 *
 * 1. 读取主参数页和备份参数页；
 * 2. 找到无效页或较旧页；
 * 3. 只擦除并更新这一页；
 * 4. 保留另一页作为掉电恢复副本；
 * 5. 写入完成后重新读取并校验。
 *
 * 因此，即使系统在擦除或写入过程中突然掉电，
 * 另一页通常仍然保留一份完整有效的旧参数。
 */
ret_t boot_param_save(boot_param_t *param)
{
    boot_param_t primary;
    boot_param_t backup;
    boot_param_t verify;

    uint32_t target_addr;
    ret_t ret;

    /*
     * 检查输入参数指针。
     */
    if (param == NULL)
    {
        return RET_NULL_PTR;
    }

    /*
     * 先将局部结构体初始化为 Flash 擦除值。
     *
     * 防止底层读取异常时留下未初始化数据。
     */
    (void)memset(&primary, 0xFF, sizeof(primary));
    (void)memset(&backup, 0xFF, sizeof(backup));

    /*
     * 保存前必须读取主、备两个参数页。
     *
     * 只有知道当前两页的有效性和新旧关系，
     * 才能安全地决定擦除哪一个参数页。
     */
    ret = dev_flash_read(
        BOOT_PARAM_PRIMARY_ADDR,
        (uint8_t *)&primary,
        sizeof(primary)
    );

    if (ret != RET_OK)
    {
        /*
         * 无法读取主参数页时，不能判断主参数页是否仍是唯一有效副本，
         * 因此禁止继续擦除。
         */
        return RET_FLASH_READ_ERROR;
    }

    ret = dev_flash_read(
        BOOT_PARAM_BACKUP_ADDR,
        (uint8_t *)&backup,
        sizeof(backup)
    );

    if (ret != RET_OK)
    {
        /*
         * 无法读取备份页时同样不能安全选择目标页。
         */
        return RET_FLASH_READ_ERROR;
    }

    /*
     * 选择本次写入的目标参数页。
     *
     * 优先级：
     *
     * 1. 主参数页无效：写主参数页；
     * 2. 主页有效、备份页无效：写备份页；
     * 3. 两页都有效：覆盖 generation 较旧的一页。
     */
    if (!boot_param_valid(&primary))
    {
        target_addr = BOOT_PARAM_PRIMARY_ADDR;
    }
    else if (!boot_param_valid(&backup))
    {
        target_addr = BOOT_PARAM_BACKUP_ADDR;
    }
    else
    {
        /*
         * 两份参数均有效时，保留 generation 更新的一页，
         * 擦除并覆盖 generation 较旧的一页。
         */
        target_addr = ((int32_t)(primary.generation -
                                 backup.generation) > 0)
                    ? BOOT_PARAM_BACKUP_ADDR
                    : BOOT_PARAM_PRIMARY_ADDR;
    }

    /*
     * 保存前强制更新固定标识和结构体格式版本。
     *
     * 避免调用者忘记设置这些字段。
     */
    param->magic = BOOT_PARAM_MAGIC;
    param->format_version = BOOT_PARAM_FORMAT_VERSION;

    /*
     * 每成功准备一次新记录，generation 增加一次。
     *
     * 加载时通过 generation 判断主、备两份记录谁更新。
     */
    param->generation++;

    /*
     * CRC 必须在其他字段全部确定之后最后计算。
     */
    param->param_crc32 = boot_param_crc(param);

    /*
     * 计算 CRC 后再次执行完整合法性检查。
     *
     * 这样可以阻止以下情况被写入 Flash：
     *
     * - CRC 正确，但槽位组合错误；
     * - 固件大小越界；
     * - 状态与 trial_count 不匹配；
     * - pending_slot 与 active_slot 相同。
     */
    if (!boot_param_valid(param))
    {
        return RET_INVALID_PARAM;
    }

    /*
     * 擦除选中的目标参数页。
     *
     * 这里只擦除一页，另一页仍然保留旧的有效参数，
     * 用于处理擦除过程中突然掉电的情况。
     */
    ret = dev_flash_erase(
        target_addr,
        DEV_FLASH_PAGE_SIZE
    );

    if (ret != RET_OK)
    {
        return ret;
    }

    /*
     * 将完整启动参数写入目标页。
     */
    ret = dev_flash_write(
        target_addr,
        (uint8_t *)param,
        sizeof(*param)
    );

    if (ret != RET_OK)
    {
        return ret;
    }

    /*
     * 写入完成后，先把回读结构体初始化为 0xFF。
     */
    (void)memset(&verify, 0xFF, sizeof(verify));

    /*
     * 从目标参数页重新读取刚刚写入的数据。
     */
    ret = dev_flash_read(
        target_addr,
        (uint8_t *)&verify,
        sizeof(verify)
    );

    if (ret != RET_OK)
    {
        return RET_FLASH_READ_ERROR;
    }

    /*
     * 写后校验包含两部分：
     *
     * 1. boot_param_valid(&verify)
     *
     *    检查回读数据的 magic、版本、字段组合和 CRC。
     *
     * 2. memcmp(&verify, param, sizeof(verify))
     *
     *    检查 Flash 回读的每一个字节是否都与 RAM 中准备写入的数据一致。
     *
     * 两项都通过才认为本次保存成功。
     */
    return (boot_param_valid(&verify) &&
            (memcmp(&verify, param, sizeof(verify)) == 0))
         ? RET_OK
         : RET_FLASH_VERIFY_ERROR;
}


