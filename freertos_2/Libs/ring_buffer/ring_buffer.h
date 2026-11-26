/**
 * @file    ring_buffer.h
 * @brief   环形缓冲区（Ring Buffer）通用库接口
 *
 * @note    字节型环形缓冲，有效容量为 size-1（留一个空位区分空/满），
 *          单生产者/单消费者场景下读写无需加锁。
 */
#ifndef  __RING_BUFFER_H__
#define  __RING_BUFFER_H__
#include "config.h"

/************************************************************
 * @brief  环形缓冲区控制块
 ************************************************************/
typedef struct{

    uint8_t *buffer;                /*!< 数据缓冲区指针（调用者提供内存） */
    uint16_t size;                  /*!< 缓冲区总大小（有效容量=size-1） */
    volatile uint16_t read;         /*!< 读指针（消费者推进；volatile 防编译器缓存，中断与任务共享） */
    volatile uint16_t write;        /*!< 写指针（生产者推进；DMA+IDLE 模式下由 ISR 直接同步硬件位置） */

}ring_buffer_t;

/************************************************************
 * @brief  初始化环形缓冲区
 * @param  rb    环形缓冲区控制块
 * @param  buf   数据缓冲区
 * @param  size  缓冲区大小（>=2）
 * @retval RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法
 ************************************************************/
ret_t ring_buffer_init(ring_buffer_t *rb, uint8_t *buf, uint16_t size);

/************************************************************
 * @brief  向环形缓冲区写入一个字节
 * @param  rb   环形缓冲区控制块
 * @param  buf  待写入字节
 * @retval RET_OK 成功; RET_NULL_PTR 空指针; RET_FULL 缓冲区已满
 ************************************************************/
ret_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *buf);

/************************************************************
 * @brief  从环形缓冲区读取一个字节
 * @param  rb   环形缓冲区控制块
 * @param  buf  读出的字节
 * @retval RET_OK 成功; RET_NULL_PTR 空指针; RET_EMPTY 缓冲区为空
 ************************************************************/
ret_t ring_buffer_read(ring_buffer_t *rb, uint8_t *buf);

#endif /*__RING_BUFFER_H__*/
