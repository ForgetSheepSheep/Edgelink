# Bootloader NVS 存储方案

## 方案结论

Bootloader 的 OTA 启动状态使用 GD32F303ZE 内部 Flash 最后两页实现
NVS。AT24C02 只保存普通设备参数和用户配置，GD25Q128 保存 OTA 固件
正文和固件信息头。

应用侧另使用片内 Flash 的独立 NVS 页记录已确认的固件版本、回滚标志与槽位。

## 数据归属

| 数据 | 所属模块 | 是否掉电保存 | 存储位置 |
| --- | --- | --- | --- |
| `app_ota_meta_t` | App OTA 会话 | 否，只在 RAM 中存在 | App 运行上下文 |
| OTA 固件正文 | GD25Q128 Device | 是 | 外部 Flash `0x000000` |
| `boot_fw_info_t` | GD25Q128 Device | 是 | 外部 Flash `0x0F0000` |
| 应用固件状态 | App 固件 NVS | 是 | 内部 Flash `0x0807E000` |
| Bootloader 启动状态 | Bootloader NVS | 是 | 内部 Flash `0x0807F000` / `0x0807F800` |
| 设备和用户参数 | AT24C02 Device | 是 | AT24C02 EEPROM |

## 内部 Flash NVS 区域

GD32F303ZE 的内部 Flash 页大小为 2 KiB，预留最后两页作为主、备 NVS
记录：

```text
0x0807F000  NVS 主记录页（2 KiB）
0x0807F800  NVS 备记录页（2 KiB）
0x08080000  Flash 结束地址
```

`boot_param_t` 由 13 个 32 位字段组成，共 52 字节。

```text
magic             固定标识
format_version    记录格式版本
generation        记录序号
state             升级状态
active_slot       当前活动槽
pending_slot      待升级槽
trial_count       试运行次数
rollback_flag     回滚标志
fw_addr           外部 Flash 镜像起始地址
fw_size           固件大小
fw_crc32          固件 CRC32
fw_version        固件版本
param_crc32       前 12 个字段的 CRC32
```

只有同时满足以下条件，记录才算有效：

- `magic` 正确；
- `format_version` 兼容；
- 槽位、状态、长度等字段范围正确；
- `param_crc32` 校验通过。

两份记录都有效时，使用支持回绕比较的最新 `generation`。保存时更新
较旧或无效的一份，写入后回读校验，另一份作为掉电恢复副本。

## 三种存储介质的职责

- Bootloader 的启动判断不依赖外部 I2C 芯片。
- OTA 状态更新频率很低，内部 Flash 页擦除可以接受。
- 两页主备记录可以防止擦除或写入过程中掉电导致状态全部丢失。
- AT24C02 继续用于设备配置，不和 Bootloader 启动协议混用。
- 固件正文和 `boot_fw_info_t` 保持在 GD25Q128，EEPROM 不保存固件镜像。
- Bootloader 通过同步接口访问存储设备。

## 异常恢复约定

主备记录均可读取且内容为空时，Bootloader 创建以 A 槽为活动槽的默认参数，并按正常启动流程验证和选择应用槽。
如果仅一份记录有效，Bootloader 使用有效副本。如果两份记录均无效且并非空白，或出现参数读取故障导致无法选定有效副本，Bootloader 显示错误并停机。

