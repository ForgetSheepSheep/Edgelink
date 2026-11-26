/**
 * @file    app_net.h
 * @brief   云端 MQTT 应用接口
 */
#ifndef __APP_NET_H__
#define __APP_NET_H__

#include "config.h"

/** 控制从站一次轮询的保持寄存器数量，地址范围为 0~6。 */
#define APP_NET_CONTROL_HOLDING_REGISTER_COUNT  7U
/** 控制从站一次轮询的状态寄存器数量，起始地址为 10。 */
#define APP_NET_CONTROL_STATUS_REGISTER_COUNT   6U
/** 模拟量从站一次轮询的输入寄存器数量，地址范围为 0~10。 */
#define APP_NET_ANALOG_INPUT_REGISTER_COUNT     11U
/** IO 从站一次读取的线圈和离散输入点数。 */
#define APP_NET_IO_POINT_COUNT                  8U
/** 三个业务从站全部有效时的掩码：bit0/1/2 分别对应从站 1/2/3。 */
#define APP_NET_ALL_SLAVES_VALID_MASK           0x07U

/**
 * @brief   网络业务快照
 *
 * @note    快照只描述业务数据，不包含 LCD 消息、Modbus 帧或硬件句柄。
 *          生产者和显示/记录等消费者通过值传递交换它，避免跨层暴露
 *          具体实现对象。
 */
typedef struct
{
    uint16_t control_holding_registers[
        APP_NET_CONTROL_HOLDING_REGISTER_COUNT];
    uint16_t control_status_registers[
        APP_NET_CONTROL_STATUS_REGISTER_COUNT];
    uint16_t analog_input_registers[
        APP_NET_ANALOG_INPUT_REGISTER_COUNT];
    uint8_t io_coil_states;              /**< 8 个线圈状态，bit0 对应点 0。 */
    uint8_t io_discrete_input_states;    /**< 8 个离散输入状态。 */
    uint8_t valid_slave_mask;            /**< 本次有效的从站位图。 */
    uint8_t poll_was_attempted;          /**< 是否至少执行过一次真实轮询。 */
    uint8_t led2_is_on;                  /**< 当前板载 LED2 电平。 */
} app_net_snapshot_t;

/**
 * @brief 网络业务读取的寄存器区
 * @note 这是网络业务层的语义类型。组合层负责把它映射成具体协议的
 *       功能码，因此 app_net 不需要知道 Modbus 03H/04H 的编码。
 */
typedef enum
{
    APP_NET_REGISTER_AREA_HOLDING = 0U, /**< 保持寄存器。 */
    APP_NET_REGISTER_AREA_INPUT          /**< 输入寄存器。 */
} app_net_register_area_t;

/**
 * @brief 网络业务读取的位区
 * @note 组合层负责把该语义类型映射成线圈或离散输入协议操作。
 */
typedef enum
{
    APP_NET_BIT_AREA_COIL = 0U,     /**< 线圈输出状态。 */
    APP_NET_BIT_AREA_DISCRETE_INPUT /**< 离散输入状态。 */
} app_net_bit_area_t;

/**
 * @brief 读取寄存器的业务端口
 * @param context         组合层上下文，由注册者解释
 * @param slave_address   目标设备地址
 * @param area            需要读取的寄存器区
 * @param start_address   起始地址
 * @param quantity        读取数量
 * @param register_values 调用者提供的输出数组
 * @retval RET_OK 成功；其他值由底层适配器返回
 * @note 输出数组至少容纳 quantity 个 uint16_t。
 */
typedef ret_t (*app_net_read_registers_fn)(void *context,
                                           uint8_t slave_address,
                                           app_net_register_area_t area,
                                           uint16_t start_address,
                                           uint16_t quantity,
                                           uint16_t *register_values);

/**
 * @brief 读取线圈或离散输入的业务端口
 * @note 真实轮询时需要，假数据模式可不提供；bit_values 至少容纳
 *       (quantity + 7) / 8 字节，低位对应低地址点。
 */
typedef ret_t (*app_net_read_bits_fn)(void *context,
                                      uint8_t slave_address,
                                      app_net_bit_area_t area,
                                      uint16_t start_address,
                                      uint16_t quantity,
                                      uint8_t *bit_values);

/** 写入保持寄存器的业务端口；回调只负责执行，不负责生成网络响应。 */
typedef ret_t (*app_net_write_register_fn)(void *context,
                                           uint8_t slave_address,
                                           uint16_t register_address,
                                           uint16_t register_value);

/** 写入线圈的业务端口；is_on 表示目标电平。 */
typedef ret_t (*app_net_write_coil_fn)(void *context,
                                       uint8_t slave_address,
                                       uint16_t coil_address,
                                       uint8_t is_on);

/** 控制板载 LED2 的输出端口；成功后 app_net 才更新快照中的 LED 状态。 */
typedef ret_t (*app_net_led2_write_fn)(void *context, uint8_t is_on);

/** 网络在线状态变化通知；回调运行在 app_net 上下文中，不得阻塞。 */
typedef void (*app_net_online_changed_fn)(void *context, uint8_t is_online);

/**
 * 网络快照更新通知；快照是 app_net 的临时副本，仅在回调执行期间有效。
 * 如果消费者要异步处理，必须在回调内复制数据并自行投递队列。
 */
typedef void (*app_net_snapshot_updated_fn)(void *context,
                                             const app_net_snapshot_t *snapshot);

/**
 * @brief   app_net 的外部依赖
 *
 * @note    由系统组合根注册。app_net 不直接包含 Modbus、LCD 或 BSP
 *          头文件，因此可以在不同硬件或测试环境中替换这些实现。
 *          前五个业务/硬件回调是必填项，两个通知回调可以为 NULL。
 */
typedef struct
{
    void *context;                                   /**< 适配器私有上下文。 */
    app_net_read_registers_fn read_registers;        /**< 读取寄存器。 */
    app_net_read_bits_fn read_bits;                  /**< 读取位区。 */
    app_net_write_register_fn write_register;        /**< 写保持寄存器。 */
    app_net_write_coil_fn write_coil;                /**< 写线圈。 */
    app_net_led2_write_fn write_led2;                /**< 写 LED2。 */
    app_net_online_changed_fn on_online_changed;     /**< 网络状态通知，可选。 */
    app_net_snapshot_updated_fn on_snapshot_updated; /**< 快照通知，可选。 */
} app_net_dependencies_t;

/**
 * @brief   初始化云端 MQTT 应用
 * @param   dependencies 业务数据、硬件输出和状态通知接口；初始化时复制
 *                       回调表，context 指向的对象需持续有效
 * @retval  RET_OK 初始化成功；RET_INVALID_PARAM 依赖不完整；
 *          RET_NO_MEMORY 内核对象创建失败；其他值表示联网失败
 * @note    应在 scheduler 已启动的初始化任务中调用，因为联网流程包含阻塞等待。
 */
ret_t app_net_init(const app_net_dependencies_t *dependencies);

/** @brief 设备页本地 LED2 按钮；LED 电平由云端 App 统一管理。 */
ret_t app_net_led_toggle(void);

#endif /* __APP_NET_H__ */
