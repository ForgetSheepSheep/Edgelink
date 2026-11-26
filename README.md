# EdgeLink

EdgeLink 是运行在 **GD32F303ZE** 上的嵌入式边缘网关固件。它通过 RS-485 轮询 Modbus RTU 从站，通过 4G 模组接入 MQTT 上报遥测并处理云端 RPC，同时在本地触摸屏显示设备状态。裸机 Bootloader 与两套 FreeRTOS 应用实现 A/B 固件升级、试运行确认和回滚。

## 工程

| 工程 | 职责 | 链接起始地址 |
| --- | --- | --- |
| [bootloader](bootloader/Project/bootloader.uvprojx) | 启动槽选择、镜像安装和失败回滚 | `0x08000000` |
| [freertos_1](freertos_1/Project/freertos_1.uvprojx) | FreeRTOS 应用，面向 Slot A | `0x08008000` |
| [freertos_2](freertos_2/Project/freertos_2.uvprojx) | FreeRTOS 应用，面向 Slot B | `0x08040000` |

两个应用工程使用同一套业务架构，分别按各自的 Flash 起始地址链接。应用入口位于各工程的 `User/main.c`。业务逻辑位于 `Application/`，设备驱动位于 `Device/`，板级外设配置位于 `BSP/`。

## 功能

- **现场采集与控制**：通过 Modbus RTU 主站读写 RS-485 从站，使用 FreeRTOS 互斥量保护整笔总线事务。
- **4G 与 MQTT**：通过 UART 与 4G 模组交互，周期上报遥测，并通过 RPC 处理寄存器读写和设备控制。
- **本地交互**：ILI9488 LCD 显示现场数据和升级状态，GT911 触摸控制器接收本地操作。
- **分层存储**：GD25Q128 SPI NOR 暂存 OTA 镜像，AT24C02 EEPROM 保存普通设备参数，片内 Flash 保存 Bootloader 与应用共享的启动状态。

## A/B 升级流程

1. 应用通过 MQTT 接收升级通知与固件分片，把镜像写入 GD25Q128。完整镜像通过 CRC32 校验后，应用记录待安装槽并重启。
2. Bootloader 将镜像写入非活动槽，完成回读 CRC 与向量表检查后，开启独立看门狗并进入试运行。
3. 新应用稳定运行后确认启动，待安装槽成为活动槽。如果试运行超过尝试上限或待安装镜像无效，Bootloader 回到原活动槽。

| 片内 Flash 区域 | 起始地址 | 容量 |
| --- | --- | --- |
| Bootloader | `0x08000000` | 32 KiB |
| Slot A | `0x08008000` | 224 KiB |
| Slot B | `0x08040000` | 224 KiB |
| 固件状态 NVS 页 | `0x0807E000` | 2 KiB |
| 启动参数主副本 | `0x0807F000` | 2 KiB |
| 启动参数备份 | `0x0807F800` | 2 KiB |

升级时按目标槽选择对应的 `.bin`。应用镜像的向量表和入口地址在链接时已绑定到各自的槽位。

## 构建与首次启动

1. 在 Keil MDK 中安装 **Arm Compiler 5.06 update 6（build 750）** 和 **GigaDevice GD32F30x DFP 2.2.3**。
2. 打开上表的 `.uvprojx` 文件并构建对应目标。每个工程在 `Project/Objects/` 生成 `.axf` 与 `.bin`，Bootloader 另外生成 `.hex`。
3. 首次部署时，通过 SWD 将 Bootloader 写入 `0x08000000`，再将 `freertos_1` 写入 `0x08008000`。保留两个分区的镜像，写入后执行校验。复位后可通过 LCD 和 SEGGER RTT 查看启动过程。

联调 4G 和 Modbus 功能需要已配置 MQTT 连接的 4G 模组及 RS-485 从站。应用默认轮询从站地址 `1`、`2`、`3`，串口速率为 `9600 baud`。

## 设计文档

- [FreeRTOS 应用架构](freertos_2/Application/APP_ARCHITECTURE.md)描述任务、消息队列和模块依赖。
- [Bootloader](bootloader/README.md)说明启动、镜像验证与调试。
- [启动状态与 NVS 设计](bootloader/Docs/BOOT_NVS_DESIGN.md)说明存储分工与状态持久化。

仓库内包含 FreeRTOS V11.1.0、GigaDevice 外设库与 SEGGER RTT 等第三方代码，许可和版权声明见相应源码。
