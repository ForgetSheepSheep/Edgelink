/**
 * @file    dev_net.h
 * @brief   ML307R 网络模组 Device 层公共接口
 *
 * @note    本头文件只暴露 App 层需要的能力。任务、事件组、互斥锁、
 *          AT 缓冲区和解析函数全部属于 dev_net.c 私有实现。
 */

#ifndef __DEV_NET_H__
#define __DEV_NET_H__

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  异步云端下行回调类型
 * @param  data 接收数据，只在回调执行期间有效
 * @param  len  数据长度，不包含结尾 '\0'
 * @note   回调运行在 net_rx 任务上下文；只能快速复制或投递数据。
 */
typedef void (*dev_net_recv_cb_t)(const char *data, uint16_t len);

/** 网络链路生命周期事件，由 dev_net RX 任务顺序通知上层状态机。 */
typedef enum
{
    DEV_NET_LINK_CONNECTED = 0U,
    DEV_NET_LINK_DISCONNECTED
} dev_net_link_event_t;

/** MQTT 链路状态变化回调（运行在 dev_net RX 任务上下文，只能快速入队）。 */
typedef void (*dev_net_link_cb_t)(dev_net_link_event_t event);

/** 标准时间文本长度："YYYY-MM-DD HH:MM:SS" + '\0'。 */
#define DEV_NET_TIME_TEXT_SIZE  20U

/**
 * @brief  初始化网络 Device 层并执行 AT 在线检测
 * @return RET_OK 成功；其他值见 ret_error.h
 * @note   必须从 FreeRTOS 调度器启动后的任务上下文调用。
 */
ret_t dev_net_init(void);

/**
 * @brief  配置模组 MQTT 多主题和过滤参数
 * @return RET_OK 成功；其他值表示某个 AT 步骤失败
 */
ret_t dev_net_mqtt_connect(void);

/**
 * @brief  复位模组并等待 AT+REST 应答
 * @return RET_OK 表示复位命令已确认；其他值表示 AT 事务失败
 * @note   MQTT_CONNECT/MQTT_DISCONNECT 通过 dev_net_link_cb_t 异步通知，
 *         连接等待与重连策略由上层状态机负责。
 */
ret_t dev_net_reset(void);

/**
 * @brief  查询驻网后的模块时间
 * @param  time_text  输出 "YYYY-MM-DD HH:MM:SS"
 * @param  buffer_size 输出缓冲区长度，至少 DEV_NET_TIME_TEXT_SIZE
 * @return RET_OK 成功；RET_LENGTH_ERROR 响应格式或缓冲区长度错误；
 *         其他值表示 AT 事务失败
 * @note   使用 AT+TIME=1；模块未驻网时返回的时间不可信，应在联网后调用。
 */
ret_t dev_net_get_time(char *time_text, uint16_t buffer_size);

/**
 * @brief  订阅 MQTT 主题
 * @param  topic 主题字符串
 * @return RET_OK 成功；RET_FULL 已达订阅上限；其他值表示失败
 */
ret_t dev_net_mqtt_subscribe(const char *topic);

/**
 * @brief  发布 MQTT 消息
 * @param  topic   目标主题
 * @param  payload 消息正文
 * @return RET_OK 已完成发送；其他值表示参数、长度或 UART 错误
 * @note   模组固件的发布命令不带 QoS 字段（见 dev_net.c 中 MQPUB 拼装），
 *         所以本接口不提供 qos 形参——传了也无处可去，只会误导调用方。
 */
ret_t dev_net_mqtt_publish(const char *topic,
                           const char *payload);

/**
 * @brief  注册异步云端下行回调
 * @param  cb 回调函数；NULL 表示取消注册
 */
void dev_net_register_recv_cb(dev_net_recv_cb_t cb);

/** 注册 MQTT 链路状态回调；传 NULL 可取消注册。 */
void dev_net_register_link_cb(dev_net_link_cb_t cb);

/**
 * @brief  查询模组 MQTT 链路是否在线
 * @return 1 在线；0 离线或尚未收到任何链路 URC
 * @note   只反映最近一次 MQTT_CONNECT/MQTT_DISCONNECT URC。MCU 单独复位后
 *         模组会话仍在、不会再发 URC，此时该值为 0，调用方应直接用
 *         订阅尝试做乐观探测。
 */
uint8_t dev_net_link_is_online(void);

/**
 * @brief  复位订阅编号计数
 * @note   模组 MQTT 会话重建后（断线自愈或模组复位），旧订阅槽已失效；
 *         重新订阅前调用，让订阅从 0 号槽重新开始，避免撞上模组
 *         订阅上限（NET_MAX_SUBSCRIPTIONS）。
 */
void dev_net_mqtt_subscribe_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_NET_H__ */
