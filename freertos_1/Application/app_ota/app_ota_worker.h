#ifndef __APP_OTA_WORKER_H__
#define __APP_OTA_WORKER_H__

#include "app_ota.h"

/* 单分片上限：队列每个条目内嵌该数组，256B × 4 深 ≈ 1KB 堆占用。 */
#define APP_OTA_CHUNK_MAX 256U

typedef enum
{
    APP_OTA_EVENT_STARTED = 0, /* 建档完成（含失败，看 result） */
    APP_OTA_EVENT_CHUNK_WRITTEN, /* 一片落盘（含失败） */
    APP_OTA_EVENT_FINISHED,    /* 提交完成（含失败）；成功后需调用方 reboot */
    APP_OTA_EVENT_ABORTED      /* 已放弃并回 IDLE */
} app_ota_event_t;

/*
 * 进度回调：received/total 为事件完成后的快照。
 * STARTED 成功时 received=0；CHUNK 成功时 received 递增；
 * FINISHED 成功时 received==total；失败时 received 保持出错前的值。
 */
typedef void (*app_ota_result_cb_t)(app_ota_event_t event,
                                    ret_t result,
                                    uint32_t received,
                                    uint32_t total);

/* @brief  初始化 OTA（含底层 init）并启动 worker 任务；重复调用返回 RET_ALREADY_INIT。 */
ret_t app_ota_worker_init(app_ota_result_cb_t result_cb);

/* 以下 post 均为非阻塞：队列满立即返回 RET_QUEUE_FULL，调用方退避后重发。 */
ret_t app_ota_post_start(const app_ota_meta_t *meta);
ret_t app_ota_post_chunk(uint32_t offset, const uint8_t *data, uint32_t len);
ret_t app_ota_post_finish(void);
ret_t app_ota_post_abort(void);

/*
 * @brief  带等待的入队（只能用于任务上下文，不可用于 ISR）。
 * @param  timeout_ms 队列满时的最长等待毫秒数。
 * @note   FINISH/ABORT 是会话收尾命令，投递失败必须由调用方按
 *         "重试 + 超时"处理；带等待版本用于让 worker 有机会消费出空位。
 */
ret_t app_ota_post_chunk_wait(uint32_t offset,
                              const uint8_t *data,
                              uint32_t len,
                              uint32_t timeout_ms);
ret_t app_ota_post_finish_wait(uint32_t timeout_ms);
ret_t app_ota_post_abort_wait(uint32_t timeout_ms);

#endif /* __APP_OTA_WORKER_H__ */
