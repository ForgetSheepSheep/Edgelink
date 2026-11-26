/**
 * @file    bsp_iic.h
 * @brief   I2C BSP 驱动接口（多总线、软件/硬件两种模式）
 *
 * @note    总线参数由 bsp_iic.c 内部结构体数组维护：
 *          - AT24C02：PB6=SCL、PB7=SDA，默认使用 BSP_IIC_MODE；
 *          - GT911：PF10=SCL、PF9=SDA，固定使用软件 I2C。
 *          上层通过 bus_id 选择总线，不在 BSP 中放置设备专用寄存器接口。
 *          需要设备复位/地址选择时，可先单独初始化 I²C GPIO，
 *          再调用完整总线初始化接口检查空闲状态。
 */
#ifndef __BSP_IIC_H__
#define __BSP_IIC_H__

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 临时对照：仅限调度器启动前、所有 I2C 设备初始化前调用。
 * 硬件发送 AT24C02 写方向地址后立即 STOP，不发送存储地址或数据。
 * 返回后关闭硬件、释放 PB6/PB7，保留原软件 I2C 配置。
 */
ret_t bsp_iic_boot_address_probe(void);
/** 任务中的同流程硬件对照；内部持总线锁，仅限 AT24C02 初始化前。 */
ret_t bsp_iic_task_address_probe(void);
/** 临时对照：持锁后挂起调度，中断保留；仍仅发送设备地址。 */
ret_t bsp_iic_task_address_probe_noswitch(void);

/** I2C 实现模式：软件模拟 */
#define BSP_IIC_MODE_SOFT    0U
/** I2C 实现模式：硬件外设 */
#define BSP_IIC_MODE_HARD    1U

/** 默认模式：硬件 I2C0（2026-09-13 恢复决定）。
 * @note  2026-09-06 曾因"任务态地址探测稳定 NACK"回退软件 I2C。
 *        根因现已定位：PB7 与 EXMC_NADV 复用，且 AFIO_PCF1.EXMC_NADV
 *        复位默认值为 0（NADV 连接到 PB7）。LCD 走 EXMC 后 NADV 占住
 *        PB7，I2C0_SDA 被夺走——地址阶段永远收不到 ACK，表现为
 *        stat0=AERR、SCL 被拉低、SDA 无跳变；boot 探测（EXMC 未使能）
 *        通过、任务探测（EXMC 已使能）失败，之前被误判为 AERR 残留。
 *        hard_i2c_periph_init() 现在显式断开 NADV（AFIO_PCF1 bit10=1），
 *        boot 与任务态全链板测均通过：probe + 单字节 + 跨页(3+3) + 边界。
 *        证据与结论见 note/I2C总线卡死排查报告.md。
 *        GT911 恒为软件 I2C。 */
#ifndef BSP_IIC_MODE
#define BSP_IIC_MODE         BSP_IIC_MODE_HARD
#endif

/** BSP 内部总线实例编号。结构体具体引脚配置不对外暴露。 */
typedef enum
{
    BSP_IIC_BUS_AT24C02 = 0U,
    BSP_IIC_BUS_GT911,
    BSP_IIC_BUS_COUNT
} bsp_iic_bus_id_t;

/** 从机内部地址宽度。 */
#define BSP_IIC_MEM_ADDR_8BIT     1U
#define BSP_IIC_MEM_ADDR_16BIT    2U

/************************************************************
 * @brief   I2C BSP 全局初始化：创建全局事务互斥量
 *
 * @note    必须在调度器启动前调用（参照 dev_gd25q128 的
 *          "init 里建锁"模式）；未调用时所有读写事务报 RET_NOT_INIT。
 * @retval  RET_OK 成功; RET_ALREADY_INIT 已初始化; RET_NO_MEMORY 堆不足
 ************************************************************/
ret_t bsp_iic_init(void);

/************************************************************
 * @brief   仅初始化指定软件 I²C 总线的 GPIO 引脚
 *
 * @note    用于需要先完成设备复位/地址选择、再检查总线空闲状态的器件。
 *          该接口不会把总线标记为已初始化；后续仍需调用
 *          bsp_iic_bus_init() 完成总线初始化和空闲电平检查。
 * @param   bus_id  总线实例编号
 * @retval  RET_OK 成功; RET_INVALID_PARAM 总线编号或模式非法;
 *          其他为 GPIO/延时初始化错误
 ************************************************************/
ret_t bsp_iic_bus_gpio_init(bsp_iic_bus_id_t bus_id);

/************************************************************
 * @brief   初始化指定 I2C 总线
 * @param   bus_id 总线实例编号
 * @retval  RET_OK 成功; RET_ALREADY_INIT 已初始化;
 *          RET_INVALID_PARAM 总线编号非法; 其他为硬件初始化错误
 ************************************************************/
ret_t bsp_iic_bus_init(bsp_iic_bus_id_t bus_id);

/************************************************************
 * @brief   使用指定总线向从机内部地址连续写入数据
 * @param   bus_id         总线实例编号
 * @param   slave_addr     从机地址（8 位格式，7bit<<1）
 * @param   mem_addr       从机内部地址
 * @param   mem_addr_size  内部地址宽度（8 位或 16 位）
 * @param   buffer         待写数据缓冲区
 * @param   len            数据长度（字节）
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_INIT 未初始化; RET_TIMEOUT 超时;
 *          RET_NACK_ERROR 无应答; RET_BUS_ERROR 总线错误
 ************************************************************/
ret_t bsp_iic_bus_write(bsp_iic_bus_id_t bus_id,
                        uint16_t slave_addr,
                        uint16_t mem_addr,
                        uint8_t mem_addr_size,
                        const uint8_t *buffer,
                        uint16_t len);

/************************************************************
 * @brief   使用指定总线从从机内部地址连续读取数据
 * @param   bus_id         总线实例编号
 * @param   slave_addr     从机地址（8 位格式，7bit<<1）
 * @param   mem_addr       从机内部地址
 * @param   mem_addr_size  内部地址宽度（8 位或 16 位）
 * @param   buffer         读数据缓冲区
 * @param   len            数据长度（字节）
 * @retval  RET_OK 成功; RET_NULL_PTR 空指针; RET_INVALID_PARAM 参数非法;
 *          RET_NOT_INIT 未初始化; RET_TIMEOUT 超时;
 *          RET_NACK_ERROR 无应答; RET_BUS_ERROR 总线错误
 ************************************************************/
ret_t bsp_iic_bus_read(bsp_iic_bus_id_t bus_id,
                       uint16_t slave_addr,
                       uint16_t mem_addr,
                       uint8_t mem_addr_size,
                       uint8_t *buffer,
                       uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IIC_H__ */
