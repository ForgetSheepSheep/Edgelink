#ifndef __APP_VERSION_H__
#define __APP_VERSION_H__

/*
 * 固件自证版本：编译期常量，随镜像一起烧录，App 永远知道自己是谁�? *
 * 与云端下发包里的 fw_version 是两码事，不要混用：
 * - 本宏：当前【运行镜像】的身份，用于上报云端、写�?fw_nvs�? * - 云端 fw_version：待升级【安装包】的身份，下载时写入 boot_param �? *   boot_fw_info，供 BootLoader 搬运校验；它不代表当前运行的是什么�? *
 * 编码规则与云端一致：(major << 16) | (minor << 8) | patch，各�?0~255�? * 发版时改下面三个数字；A/B 两个槽各自编译，同一版本应保持一致�? */
#define APP_FW_VERSION_MAJOR 1U
#define APP_FW_VERSION_MINOR 0U
#define APP_FW_VERSION_PATCH 0U

#define APP_FW_VERSION ((APP_FW_VERSION_MAJOR << 16) | \
                        (APP_FW_VERSION_MINOR << 8)  | \
                        (APP_FW_VERSION_PATCH))

#endif /* __APP_VERSION_H__ */
