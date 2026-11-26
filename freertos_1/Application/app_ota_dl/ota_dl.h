#ifndef __OTA_DL_H__
#define __OTA_DL_H__

#include "config.h"
#include "app_ota/app_ota.h"

/*
 * OTA 下载会话：云端 MQTT → GD25Q128 落盘 的最后一公里。
 *
 * 协议（与云端/测试服务器的约定，改动需同步云端）：
 * - 固件通知：订阅 v1/devices/me/attributes，JSON {fw_title,fw_version,fw_size,fw_checksum}
 * - 分片请求：发布 v2/fw/request/{rid}/chunk/{cid}，载荷为十进制分片长度（固定 256）
 * - 分片响应：订阅 v2/fw/response/+/chunk/#，
 *   正文为文本头 + 二进制体：v2/fw/response/{rid}/chunk/{cid},{bytes},<bytes个原始字节>
 * - 版本号：V1 / V3 / 1.2.3 / V1.2.3，编码 (major<<16)|(minor<<8)|patch
 * - fw_checksum：十六进制字符串，可带 0x 前缀；云端（ThingsBoard）按小端字节序下发 CRC-32/ISO-HDLC，解析时归一化为标准数值
 *
 * 节流（保护模组发送队列，沿用实测结论）：
 * - 分片 256B，单片超时 15s、最多重试 3 次、分片间隔 50ms；
 * - 下载期间上层应暂停周期遥测（见 ota_dl_active）。
 *
 * 线程模型：
 * - ota_dl_feed / ota_dl_in_binary 运行在 dev_net net_rx 任务上下文：
 *   只做逐字节分类与入队，不解析 JSON、不碰 Flash、不阻塞；
 * - ota_dl_tick 运行在 app_net 任务上下文（建议 100ms 周期）：
 *   消费队列、发分片请求、做超时重试；
 * - Flash 操作全部在 ota_worker 任务内（经 app_ota_post_* 投递）。
 *
 * 可靠性（关键事件不丢、不永久挂起）：
 * - worker 结果事件不允许静默丢弃：入队带等待，仍失败时转入兜底槽，
 *   由 ota_dl_tick 保证最终被处理；
 * - FINISH/ABORT 投递失败进入重试状态，ABORT 幂等补投；
 * - finish 阶段（校验+提交）有独立超时，超时后放弃本轮；
 * - 会话结束后的在途 worker 结果按陈旧事件忽略，不会重新推进旧会话。
 */
#define OTA_DL_CHUNK_SIZE 256U

/* @brief  创建会话队列并初始化 OTA worker；重复调用返回 RET_ALREADY_INIT。 */
ret_t ota_dl_init(void);

/* @brief  订阅 attributes + 分片响应两个主题（在 MQTT 已连接后调用）。 */
ret_t ota_dl_subscribe(void);

/*
 * @brief  向会话投喂一个下行字节（net_rx 上下文）。
 * @note   二进制体按头中 bytes 精确计数，与内容无关（含 0x00 安全）。
 */
void ota_dl_feed(uint8_t byte);

/* @brief  当前是否正处二进制体接收中（供 RX 分流跳过 JSON 解析）。 */
uint8_t ota_dl_in_binary(void);

/* @brief  当前是否有下载会话在进行（供上层暂停周期遥测）。 */
uint8_t ota_dl_active(void);

/* @brief  会话推进：消费队列、发请求、超时重试（app_net 任务周期调用）。 */
void ota_dl_tick(void);

/*
 * @brief  会话状态快照（供 LCD/上报等消费者轮询）。
 * @note   state 直接取 app_ota 状态机；target_version 是当前会话云端声明的
 *         目标版本（无会话时为 0）。消费者按需映射到自己的显示语义。
 */
typedef struct
{
    app_ota_state_t state;
    uint32_t received;
    uint32_t total;
    uint32_t target_version;
} ota_dl_status_t;

void ota_dl_get_status(ota_dl_status_t *status);

#endif /* __OTA_DL_H__ */
