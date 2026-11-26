# freertos_1 应用层架构说明

## 1. 适用范围

本文档说明当前工程 `freertos_1` 中应用层的职责、依赖方向、任务边界和同步规则。工程入口是 `User/main.c`；业务模块不应通过全局变量直接访问其他业务模块的内部状态。

## 2. 模块依赖方向

```text
User/main.c（系统组合根）
  ├── app_net       MQTT 业务、RPC、遥测调度
  │     ├── dev_net（4G/MQTT 设备驱动）
  │     └── 回调接口 ──> app_modbus / BSP GPIO / LCD 消息队列
  ├── app_modbus    Modbus 事务、帧、CRC、RS485 互斥
  │     └── modbus_master / port_serial
  └── app_lcd_plan_page
        ├── dev_lcd / dev_gt911 / dev_gd25q128
        └── Display Task / Touch Task
```

`app_net` 不包含 Modbus、LCD 或 BSP 头文件。它只依赖 `app_net_dependencies_t` 中的业务端口；具体实现由 `main.c` 装配。这样更换 Modbus 从站、显示设备或测试替身时，不需要修改网络业务代码。

## 3. 运行数据流

### 下行 RPC

```text
dev_net 接收回调
  -> 增量 JSON 解析器
  -> app_net RPC 队列（值拷贝）
  -> app_net_task
  -> app_net_process_rpc
  -> Modbus/GPIO 回调
  -> MQTT response 主题
```

接收回调不能执行阻塞操作，也不能把临时缓冲区指针放入队列。所有 JSON 解析和设备访问都在 `app_net_task` 中串行完成。

### 周期遥测

```text
app_net_task
  -> 读取寄存器/位区语义回调
  -> app_modbus 生成并执行一笔完整事务
  -> app_net_snapshot_t
  -> 快照通知回调
  -> app_lcd_plan_page_post()
  -> Display Task 绘制
```

每个从站只有在整组数据读取成功后才置位 `valid_slave_mask`，避免把半组数据误认为有效数据。三组遥测之间保留发送间隔，防止连续写入 4G 模组发送队列。

## 4. 同步对象和所有权

| 模块 | 同步对象 | 保护内容 | 所有者 |
| --- | --- | --- | --- |
| `app_net` | `s_app_net_state_mutex` | 网络快照、LED2 状态 | `app_net` |
| `app_net` | `s_app_net_rpc_queue` | RPC 消息值拷贝 | `app_net_task` 消费 |
| `app_modbus` | `s_app_modbus_mutex` | 一次完整 RS485 事务 | `app_modbus` |
| LCD | `s_lcd_plan_queue` | 页面消息值拷贝 | Display Task 消费 |

## 5. 启动顺序

1. 初始化 LCD 页面队列和两个 LCD 任务。
2. 注册 LCD 的 LED2 操作回调。
3. 创建 Modbus 初始化任务；主站在后台完成串口注册和初始化。
4. 创建网络初始化任务；联网流程在任务上下文中等待 AT/MQTT 响应。
5. 创建 LED1 心跳任务。
6. Display Task 执行一次约 3 秒的启动动画，然后进入页面刷新循环。

启动动画由 `30 × 100ms` 组成，配置在 `app_lcd_plan_page.c` 的 `LCD_PLAN_BOOT_*` 宏中。

## 6. 注释和命名约定

- 任务句柄使用 `*_task_handle`，布尔状态使用 `is_*`、`has_*`、`*_was_*`。
- 地址、数量、长度分别使用 `*_address`、`quantity`、`*_length`，不要使用无上下文的 `n`、`len` 或 `value`。
- 业务层使用语义名，例如 `APP_NET_REGISTER_AREA_HOLDING`；协议层才使用 Modbus 功能码。
- 公共接口注释说明参数、返回值、线程上下文和缓冲区生命周期。
- 外部回调必须说明调用上下文；快照回调收到的数据只在回调期间有效，异步消费者必须自行复制。

## 7. 构建验证

使用 Keil ARMCC/UV4 从无旧构建产物的临时副本重新编译并链接，当前结果为 `0 Error(s), 0 Warning(s)`。
