/**
 * @file    dev_gd25q128.h
 * @brief   GD25Q128 外部 SPI NOR Flash Device 层接口框架
 *
 * @note    本层负责 GD25Q128 的命令协议、地址范围和忙状态；
 *          SPI 电气配置与 GPIO 电平只通过 BSP 接口完成。
 *          当前已接入读、页写、跨页写、扇区擦除、块擦除和整片擦除。
 */
#ifndef __DEV_GD25Q128_H__
#define __DEV_GD25Q128_H__

#include "config.h"

/** GD25Q128 JEDEC ID 长度。 */
#define DEV_GD25Q128_JEDEC_ID_LENGTH   (3U)
/** GD25Q128 页大小。 */
#define DEV_GD25Q128_PAGE_SIZE         (256UL)
/** GD25Q128 扇区大小。 */
#define DEV_GD25Q128_SECTOR_SIZE       (4096UL)
/** GD25Q128 块大小。 */
#define DEV_GD25Q128_BLOCK_SIZE        (65536UL)
/** GD25Q128 总容量：128 Mbit = 16 MByte。 */
#define DEV_GD25Q128_TOTAL_SIZE        (16UL * 1024UL * 1024UL)

/** JEDEC ID 的三个字段。 */
typedef struct
{
    uint8_t manufacturer;
    uint8_t memory_type;
    uint8_t capacity;
} dev_gd25q128_jedec_id_t;

/************************************************************
 * @brief   初始化 GD25Q128 Device
 * @retval  RET_OK 成功; RET_ALREADY_INIT 重复初始化;
 *          其他为 BSP 初始化错误
 ************************************************************/
ret_t dev_gd25q128_init(void);

/************************************************************
 * @brief   读取 JEDEC ID
 * @param   id  JEDEC ID 输出结构体
 * @retval  RET_OK 成功; RET_NOT_INIT 尚未初始化; RET_NULL_PTR 空指针;
 *          其他为 SPI/GPIO 错误
 ************************************************************/
ret_t dev_gd25q128_jedec_id_get(dev_gd25q128_jedec_id_t *id);

/************************************************************
 * @brief   在当前 256 字节页内执行一次 Page Program
 * @param   address Flash 起始地址（0 <= address < 16MB）
 * @param   buffer  待写入数据缓冲区
 * @param   len     写入长度；不能为 0，且不能跨当前页
 * @retval  RET_OK 成功且 BUSY 已清零;
 *          RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_INIT 尚未初始化; RET_BUSY 芯片仍忙;
 *          RET_TIMEOUT 内部编程或 SPI 等待超时;
 *          其他为 GPIO/SPI 错误
 * @note    本基础接口只负责单页写入和 BUSY 轮询，跨页由上层拆分；
 *          所有等待均为有界的裸机轮询。
 ************************************************************/
ret_t dev_gd25q128_page_program(uint32_t address,
                                 const uint8_t *buffer,
                                 uint16_t len);

/************************************************************
 * @brief   连续读取 GD25Q128 数据
 * @param   address Flash 起始地址（0 <= address < 16MB）
 * @param   buffer  读数据输出缓冲区
 * @param   len     读取长度；不能为 0
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_INIT 尚未初始化; RET_TIMEOUT 等待超时;
 *          其他为 GPIO/SPI 错误
 ************************************************************/
ret_t dev_gd25q128_read(uint32_t address,
                         uint8_t *buffer,
                         uint16_t len);

/************************************************************
 * @brief   写入任意长度数据，并自动按页边界拆分
 * @param   address Flash 起始地址（0 <= address < 16MB）
 * @param   buffer  待写入数据缓冲区
 * @param   len     写入长度；不能为 0
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_INIT 尚未初始化; RET_TIMEOUT 等待超时;
 *          其他为 Flash/SPI 错误
 * @note    本接口不会自动擦除；Flash 只能把 bit 从 1 写成 0。
 *          每跨过一页就开始新的 CS/WREN/Page Program 事务。
 ************************************************************/
ret_t dev_gd25q128_write(uint32_t address,
                          const uint8_t *buffer,
                          uint32_t len);

/************************************************************
 * @brief   擦除一个 4KB 扇区
 * @param   address 扇区起始地址，必须 4KB 对齐
 * @retval  RET_OK 成功; RET_INVALID_PARAM 地址非法或未对齐;
 *          RET_NOT_INIT 尚未初始化; RET_TIMEOUT 擦除超时;
 *          其他为 Flash/SPI 错误
 ************************************************************/
ret_t dev_gd25q128_sector_erase(uint32_t address);

/************************************************************
 * @brief   擦除一个 64KB 块
 * @param   address 块起始地址，必须 64KB 对齐
 * @retval  RET_OK 成功; RET_INVALID_PARAM 地址非法或未对齐;
 *          RET_NOT_INIT 尚未初始化; RET_TIMEOUT 擦除超时;
 *          其他为 Flash/SPI 错误
 ************************************************************/
ret_t dev_gd25q128_block_erase(uint32_t address);

/************************************************************
 * @brief   擦除整片 GD25Q128
 * @retval  RET_OK 成功; RET_NOT_INIT 尚未初始化; RET_TIMEOUT 擦除超时;
 *          其他为 Flash/SPI 错误
 * @warning 该操作会清除整片 16MB 内容，禁止在普通业务流程中调用。
 ************************************************************/
ret_t dev_gd25q128_chip_erase(void);

#endif /* __DEV_GD25Q128_H__ */
