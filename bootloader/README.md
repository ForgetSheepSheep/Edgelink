# EdgeLink 裸机 Bootloader

Bootloader 在 GD32F303ZE 上执行应用槽选择、OTA 镜像安装、试运行与回滚。启动状态与升级进度显示在 LCD 上，诊断日志通过 SEGGER RTT 输出。

## 启动与升级

1. 初始化延时、SysTick、GPIO、LCD 和 Flash，并读取片内 Flash 中的主备启动参数。参数区空白时创建默认记录，默认使用 A 槽。参数读取或校验失败时显示故障并停机。
2. 在 `DOWNLOAD_READY` 或 `COPYING` 状态下，将 GD25Q128 中的镜像信息头与启动参数比对，再把镜像复制到待安装槽。复制过程中计算 CRC32，写入后执行完整回读 CRC32 和向量表校验。验证通过后记录 `TRIAL` 状态。
3. 试运行时，Bootloader 更新尝试次数并启用独立看门狗。新应用负责喂狗，并在稳定运行后调用 `app_boot_service_confirm()` 确认。默认最多尝试 5 次。次数用尽或待安装槽无效时回滚到原活动槽。
4. 正常启动优先验证活动槽，该槽无效时检查另一槽。两个槽均无效时显示故障并停机。

跳转前会关闭中断和 SysTick，清除 NVIC 的中断使能和挂起状态，将 `SCB->VTOR` 指向目标槽，并使用该槽向量表提供的初始 MSP 与复位入口。槽校验要求 MSP 在 SRAM 范围内且按 8 字节对齐，复位入口的 Thumb 位有效且地址落在同一槽内。

## 存储布局

| 区域 | 起始地址 | 容量 |
| --- | --- | --- |
| 片内 Flash · Bootloader | `0x08000000` | 32 KiB |
| 片内 Flash · A 槽 | `0x08008000` | 224 KiB |
| 片内 Flash · B 槽 | `0x08040000` | 224 KiB |
| 片内 Flash · 应用固件状态 NVS 页 | `0x0807E000` | 2 KiB |
| 片内 Flash · 启动参数主副本 | `0x0807F000` | 2 KiB |
| 片内 Flash · 启动参数备份 | `0x0807F800` | 2 KiB |
| GD25Q128 · OTA 镜像暂存区 | `0x000000` | 256 KiB |
| GD25Q128 · OTA 镜像信息头所在扇区 | `0x0F0000` | 4 KiB |
| SRAM | `0x20000000` | 64 KiB |

Bootloader 和两个应用工程使用相同的 [分区及数据结构定义](Application/boot_protocol.h)。存储介质的分工与启动参数写入规则见 [启动状态与 NVS 设计](Docs/BOOT_NVS_DESIGN.md)。

## 工程结构与构建

- `Application/` 负责启动、槽校验、升级状态机和 LCD 状态显示。
- `Device/` 提供同步设备接口，`BSP/` 管理引脚、时钟和外设寄存器。
- `Project/` 包含 Keil 工程与链接布局。通过 [bootloader.uvprojx](Project/bootloader.uvprojx) 构建 `bootloader` 目标，产物为 `Project/Objects/bootloader.axf`、`bootloader.hex` 和 `bootloader.bin`。工具链与器件包版本见 [项目说明](../README.md)。
