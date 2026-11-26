#ifndef __RET_ERROR_H__
#define __RET_ERROR_H__

#include <stdint.h>                         // 使用 int32_t 类型

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************
 * @brief 统一返回值类型
 *
 * 说明：
 * 1. RET_OK = 0 表示成功
 * 2. ret_t < 0 表示失败
 * 3. 错误码按 100 为一个区间分组
 * 4. 不建议随便修改已经定义好的错误码数值
 ************************************************************/
typedef int32_t ret_t;


/* ============================================================
 * 0：成功
 * ============================================================ */

#define RET_OK                         (0)          // 执行成功


/* ============================================================
 * -1 ~ -99：通用错误
 *
 * 适合所有模块都可能出现的基础错误。
 * 例如：空指针、参数错误、状态错误、未初始化等。
 * ============================================================ */

#define RET_ERROR                      (-1)         // 通用失败，无法归类时使用
#define RET_NULL_PTR                   (-2)         // 空指针错误
#define RET_INVALID_PARAM              (-3)         // 参数非法
#define RET_INVALID_STATE              (-4)         // 当前状态不允许执行该操作
#define RET_NOT_SUPPORT                (-5)         // 当前功能不支持
#define RET_NOT_INIT                   (-6)         // 模块未初始化
#define RET_ALREADY_INIT               (-7)         // 模块已经初始化
#define RET_NOT_READY                  (-8)         // 模块未准备好
#define RET_PERMISSION_DENIED          (-9)         // 没有权限执行该操作

#define RET_OUT_OF_RANGE               (-10)        // 参数超出允许范围
#define RET_LENGTH_ERROR               (-11)        // 长度错误
#define RET_ALIGN_ERROR                (-12)        // 地址或长度未对齐
#define RET_UNKNOWN                    (-13)        // 未知错误


/* ============================================================
 * -100 ~ -199：资源类错误
 *
 * 适合描述系统资源不足、资源被占用、等待超时等问题。
 * 例如：内存不足、队列满、缓冲区空、互斥锁超时。
 * ============================================================ */

#define RET_BUSY                       (-100)       // 资源正忙
#define RET_TIMEOUT                    (-101)       // 操作超时
#define RET_NO_MEMORY                  (-102)       // 内存不足
#define RET_NO_RESOURCE                (-103)       // 资源不足
#define RET_LOCKED                     (-104)       // 资源已被锁定
#define RET_UNLOCKED                   (-105)       // 资源未锁定
#define RET_OVERFLOW                   (-106)       // 数据溢出
#define RET_UNDERFLOW                  (-107)       // 数据下溢
#define RET_FULL                       (-108)       // 队列、缓冲区或存储空间已满
#define RET_EMPTY                      (-109)       // 队列、缓冲区或数据区为空


/* ============================================================
 * -200 ~ -299：数据类错误
 *
 * 适合描述数据本身不正确。
 * 例如：CRC 错误、帧格式错误、长度错误、版本不匹配。
 * ============================================================ */

#define RET_DATA_ERROR                 (-200)       // 数据错误
#define RET_DATA_LOST                  (-201)       // 数据丢失
#define RET_DATA_FORMAT_ERROR          (-202)       // 数据格式错误
#define RET_DATA_LENGTH_ERROR          (-203)       // 数据长度错误
#define RET_CHECKSUM_ERROR             (-204)       // 校验和错误
#define RET_CRC_ERROR                  (-205)       // CRC 校验错误
#define RET_PARSE_ERROR                (-206)       // 数据解析失败
#define RET_MAGIC_ERROR                (-207)       // 魔数错误
#define RET_VERSION_ERROR              (-208)       // 数据版本不匹配
#define RET_SEQUENCE_ERROR             (-209)       // 序号错误或乱序


/* ============================================================
 * -300 ~ -399：IO 操作错误
 *
 * 适合抽象的读写操作。
 * 比如 I2C、SPI、UART、文件、Flash 都可能有 read/write 错误。
 * 如果不想暴露具体介质，可以返回这一组。
 * ============================================================ */

#define RET_IO_ERROR                   (-300)       // IO 通用错误
#define RET_OPEN_ERROR                 (-301)       // 打开失败
#define RET_CLOSE_ERROR                (-302)       // 关闭失败
#define RET_READ_ERROR                 (-303)       // 读取失败
#define RET_WRITE_ERROR                (-304)       // 写入失败
#define RET_SEEK_ERROR                 (-305)       // 地址跳转失败
#define RET_IOCTL_ERROR                (-306)       // 控制命令执行失败
#define RET_FLUSH_ERROR                (-307)       // 刷新缓存失败


/* ============================================================
 * -400 ~ -499：通信类错误
 *
 * 适合串口、I2C、SPI、CAN、RS485、4G、WiFi、BLE、Modbus 等。
 * 这一组重点描述“通信链路或协议交互失败”。
 * ============================================================ */

#define RET_COMM_ERROR                 (-400)       // 通信通用错误
#define RET_SEND_ERROR                 (-401)       // 发送失败
#define RET_RECV_ERROR                 (-402)       // 接收失败
#define RET_NO_RESPONSE                (-403)       // 对方无响应
#define RET_DISCONNECTED              (-404)       // 连接断开
#define RET_FRAME_ERROR                (-405)       // 通信帧错误
#define RET_ACK_ERROR                  (-406)       // ACK 应答错误
#define RET_NACK_ERROR                 (-407)       // 收到 NACK
#define RET_ADDR_ERROR                 (-408)       // 地址错误
#define RET_BUS_ERROR                  (-409)       // 总线错误

#define RET_UART_ERROR                 (-420)       // UART 通信错误
#define RET_I2C_ERROR                  (-421)       // I2C 通信错误
#define RET_SPI_ERROR                  (-422)       // SPI 通信错误
#define RET_CAN_ERROR                  (-423)       // CAN 通信错误
#define RET_RS485_ERROR                (-424)       // RS485 通信错误
#define RET_MODBUS_ERROR              (-425)       // Modbus 协议错误
#define RET_WIFI_ERROR                 (-426)       // WiFi 通信错误
#define RET_BLE_ERROR                  (-427)       // BLE 通信错误
#define RET_4G_ERROR                   (-428)       // 4G 通信错误
#define RET_MQTT_ERROR                 (-429)       // MQTT 通信错误


/* ============================================================
 * -500 ~ -599：存储介质错误
 *
 * 适合 Flash、EEPROM、SD 卡、NVS、外部 SPI Flash 等。
 * 这一组重点描述“存储介质访问失败”。
 * ============================================================ */

#define RET_STORAGE_ERROR              (-500)       // 存储通用错误
#define RET_FLASH_ERROR                (-501)       // Flash 通用错误
#define RET_FLASH_ERASE_ERROR          (-502)       // Flash 擦除失败
#define RET_FLASH_WRITE_ERROR          (-503)       // Flash 写入失败
#define RET_FLASH_READ_ERROR           (-504)       // Flash 读取失败
#define RET_FLASH_VERIFY_ERROR         (-505)       // Flash 写入后校验失败
#define RET_FLASH_BAD_BLOCK            (-506)       // Flash 坏块或坏扇区
#define RET_FLASH_NO_SPACE             (-507)       // Flash 空间不足

#define RET_EEPROM_ERROR               (-520)       // EEPROM 通用错误
#define RET_EEPROM_WRITE_ERROR         (-521)       // EEPROM 写入失败
#define RET_EEPROM_READ_ERROR          (-522)       // EEPROM 读取失败
#define RET_EEPROM_VERIFY_ERROR        (-523)       // EEPROM 校验失败

#define RET_NVS_ERROR                  (-540)       // NVS 通用错误
#define RET_NVS_NOT_FOUND              (-541)       // NVS 键不存在
#define RET_NVS_NO_SPACE               (-542)       // NVS 空间不足
#define RET_NVS_INVALID_DATA           (-543)       // NVS 数据非法

#define RET_SD_ERROR                   (-560)       // SD 卡通用错误
#define RET_SD_NOT_INSERTED            (-561)       // SD 卡未插入
#define RET_SD_MOUNT_ERROR             (-562)       // SD 卡挂载失败
#define RET_SD_READ_ERROR              (-563)       // SD 卡读取失败
#define RET_SD_WRITE_ERROR             (-564)       // SD 卡写入失败


/* ============================================================
 * -600 ~ -699：文件系统错误
 *
 * 适合 FATFS、LittleFS、SPIFFS 等文件系统。
 * 注意：存储介质正常，不代表文件系统一定正常。
 * ============================================================ */

#define RET_FS_ERROR                   (-600)       // 文件系统通用错误
#define RET_FS_MOUNT_ERROR             (-601)       // 文件系统挂载失败
#define RET_FS_UNMOUNT_ERROR           (-602)       // 文件系统卸载失败
#define RET_FS_FORMAT_ERROR            (-603)       // 文件系统格式化失败
#define RET_FS_NOT_FOUND               (-604)       // 文件或目录不存在
#define RET_FS_EXIST                   (-605)       // 文件或目录已经存在
#define RET_FS_OPEN_ERROR              (-606)       // 文件打开失败
#define RET_FS_CLOSE_ERROR             (-607)       // 文件关闭失败
#define RET_FS_READ_ERROR              (-608)       // 文件读取失败
#define RET_FS_WRITE_ERROR             (-609)       // 文件写入失败
#define RET_FS_REMOVE_ERROR            (-610)       // 文件删除失败
#define RET_FS_RENAME_ERROR            (-611)       // 文件重命名失败


/* ============================================================
 * -700 ~ -799：Bootloader / OTA / 固件错误
 *
 * 适合 Bootloader、APP 跳转、A/B 分区、OTA 升级、固件校验。
 * ============================================================ */

#define RET_BOOT_ERROR                 (-700)       // Bootloader 通用错误
#define RET_APP_INVALID                (-701)       // APP 镜像非法
#define RET_APP_NOT_EXIST              (-702)       // APP 不存在
#define RET_APP_VERIFY_ERROR           (-703)       // APP 校验失败
#define RET_JUMP_ERROR                 (-704)       // APP 跳转失败
#define RET_UPGRADE_ERROR              (-705)       // 升级通用错误
#define RET_OTA_ERROR                  (-706)       // OTA 通用错误
#define RET_OTA_DOWNLOAD_ERROR         (-707)       // OTA 下载失败
#define RET_OTA_WRITE_ERROR            (-708)       // OTA 写入失败
#define RET_OTA_VERIFY_ERROR           (-709)       // OTA 校验失败
#define RET_OTA_ROLLBACK_ERROR         (-710)       // OTA 回滚失败
#define RET_PARTITION_ERROR            (-711)       // 分区表或分区状态错误
#define RET_FIRMWARE_VERSION_ERROR     (-712)       // 固件版本错误


/* ============================================================
 * -800 ~ -899：安全类错误
 *
 * 适合加密、签名、密钥、权限、认证等。
 * 如果项目暂时不用安全功能，也可以先保留。
 * ============================================================ */

#define RET_SECURITY_ERROR             (-800)       // 安全通用错误
#define RET_AUTH_ERROR                 (-801)       // 认证失败
#define RET_ENCRYPT_ERROR              (-802)       // 加密失败
#define RET_DECRYPT_ERROR              (-803)       // 解密失败
#define RET_SIGN_ERROR                 (-804)       // 签名失败
#define RET_VERIFY_ERROR               (-805)       // 验签失败
#define RET_KEY_ERROR                  (-806)       // 密钥错误
#define RET_CERT_ERROR                 (-807)       // 证书错误


/* ============================================================
 * -900 ~ -999：硬件 / 驱动错误
 *
 * 适合外设驱动层，比如传感器、屏幕、电机、PWM、ADC、GPIO。
 * ============================================================ */

#define RET_HW_ERROR                   (-900)       // 硬件通用错误
#define RET_DEVICE_ERROR               (-901)       // 设备通用错误
#define RET_DEVICE_NOT_FOUND           (-902)       // 设备未找到
#define RET_DEVICE_OFFLINE             (-903)       // 设备离线
#define RET_DEVICE_BUSY                (-904)       // 设备忙
#define RET_DEVICE_INIT_ERROR          (-905)       // 设备初始化失败
#define RET_SENSOR_ERROR               (-920)       // 传感器通用错误
#define RET_SENSOR_DATA_ERROR          (-921)       // 传感器数据错误
#define RET_DISPLAY_ERROR              (-930)       // 显示设备错误
#define RET_TOUCH_ERROR                (-931)       // 触摸设备错误
#define RET_PWM_ERROR                  (-940)       // PWM 输出错误
#define RET_ADC_ERROR                  (-941)       // ADC 采样错误
#define RET_GPIO_ERROR                 (-942)       // GPIO 操作错误


/* ============================================================
 * -1000 ~ -1099：OS / RTOS 错误
 *
 * 适合 FreeRTOS、任务、队列、信号量、互斥锁、事件组等。
 * 裸机协程式任务框架也可以使用这一组。
 * ============================================================ */

#define RET_OS_ERROR                   (-1000)      // OS 通用错误
#define RET_TASK_ERROR                 (-1001)      // 任务错误
#define RET_TASK_CREATE_ERROR          (-1002)      // 任务创建失败
#define RET_TASK_DELETE_ERROR          (-1003)      // 任务删除失败
#define RET_QUEUE_ERROR                (-1010)      // 队列错误
#define RET_QUEUE_FULL                 (-1011)      // 队列满
#define RET_QUEUE_EMPTY                (-1012)      // 队列空
#define RET_SEM_ERROR                  (-1020)      // 信号量错误
#define RET_MUTEX_ERROR                (-1021)      // 互斥锁错误
#define RET_EVENT_ERROR                (-1030)      // 事件组错误
#define RET_TIMER_ERROR                (-1040)      // 软件定时器错误


/* ============================================================
 * -1100 ~ -1199：应用层业务错误
 *
 * 适合项目业务逻辑。
 * 例如门禁、日志、参数配置、网页、屏幕 UI、设备联动等。
 * ============================================================ */

#define RET_APP_ERROR                  (-1100)      // 应用层通用错误
#define RET_CONFIG_ERROR               (-1101)      // 配置参数错误
#define RET_CONFIG_NOT_FOUND           (-1102)      // 配置不存在
#define RET_CONFIG_SAVE_ERROR          (-1103)      // 配置保存失败
#define RET_CONFIG_LOAD_ERROR          (-1104)      // 配置加载失败
#define RET_LOG_ERROR                  (-1110)      // 日志系统错误
#define RET_LOG_FULL                   (-1111)      // 日志空间已满
#define RET_UI_ERROR                   (-1120)      // UI 逻辑错误
#define RET_CONTROL_ERROR              (-1130)      // 控制逻辑错误


/* ============================================================
 * -2000 以下：用户项目自定义扩展区
 *
 * 如果某个项目特别复杂，可以从这里继续扩展。
 * 例如：
 * - 门禁项目：-2000 ~ -2099
 * - OTA 项目：-2100 ~ -2199
 * - 传感器项目：-2200 ~ -2299
 * ============================================================ */

#define RET_USER_BASE                  (-2000)      // 用户自定义错误码起始值




#ifdef __cplusplus
}
#endif

#endif /* __RET_ERROR_H__ */
