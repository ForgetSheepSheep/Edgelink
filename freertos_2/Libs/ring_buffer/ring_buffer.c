/**
 * @file    ring_buffer.c
 * @brief   环形缓冲区（Ring Buffer）通用库实现
 *
 * @note    读写指针推进遵循"先判断后写入"策略：
 *          write+1==read 判满，read==write 判空。
 *          核心约定：永远空出 1 个槽位——否则“全满”和“全空”
 *          都是 read==write，无法区分。代价是 size=N 时只能存 N-1 字节。
 *          单生产者（ISR 写）+ 单消费者（任务读）场景下，
 *          写只动 write、读只动 read，天然无锁，不需要关中断。 */

#include "ring_buffer/ring_buffer.h"

/************************************************************
 * @brief  初始化环形缓冲区
 * @param  rb    环形缓冲区控制块
 * @param  buf   数据缓冲区
 * @param  size  缓冲区大小（>=2）
 * @retval RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t ring_buffer_init(ring_buffer_t *rb, uint8_t *buf, uint16_t size)
{
    if ((rb == NULL) || (buf == NULL))
    {
        DBG_Error("RET_NULL_PTR");
        return RET_NULL_PTR;
    }

    if (size < 2U)
    {
        DBG_Error("RET_INVALID_PARAM");
        return RET_INVALID_PARAM;
    }

    rb->buffer = buf;
    rb->size = size;
    rb->read = 0U;
    rb->write = 0U;

    return RET_OK;
}

/************************************************************
 * @brief  向环形缓冲区写入一个字节
 * @param  rb   环形缓冲区控制块
 * @param  buf  待写入字节
 * @retval RET_OK 成功; RET_NULL_PTR 空指针; RET_FULL 缓冲区已满
 ************************************************************/
ret_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *buf)
{
    uint16_t next_pos;

    if ((rb == NULL) || (buf == NULL))
    {
        DBG_Error("RET_NULL_PTR");
        return RET_NULL_PTR;
    }

    /* 先算“写完这一字节后 write 会到哪”：+1 后对 size 取模，
     * 到末尾自动卷绕回 0，这就是“环形”的全部实现。 */
    next_pos = (uint16_t)((rb->write + 1U) % rb->size);

    /* 预判满：下一个位置追上 read 说明缓冲已满。
     * 先判再写，保证任何时刻 write 都不会覆盖未读数据。 */
    if (next_pos == rb->read)
    {
        DBG_Warning("RET_FULL");
        return RET_FULL;
    }

    /* 两步提交：先放数据，再挪指针。顺序不能反——
     * 若先挪指针，消费者可能在你放数据前读到旧垃圾。 */
    rb->buffer[rb->write] = *buf;
    rb->write = next_pos;

    return RET_OK;
}

/************************************************************
 * @brief  从环形缓冲区读取一个字节
 * @param  rb   环形缓冲区控制块
 * @param  buf  读出的字节
 * @retval RET_OK 成功; RET_NULL_PTR 空指针; RET_EMPTY 缓冲区为空
 ************************************************************/
ret_t ring_buffer_read(ring_buffer_t *rb, uint8_t *buf)
{
    uint16_t next_pos;

    if ((rb == NULL) || (buf == NULL))
    {
        DBG_Error("RET_NULL_PTR");
        return RET_NULL_PTR;
    }

    if (rb->read == rb->write)
    {
        return RET_EMPTY;
    }

    *buf = rb->buffer[rb->read];
    next_pos = (uint16_t)((rb->read + 1U) % rb->size);
    rb->read = next_pos;

    return RET_OK;
}
