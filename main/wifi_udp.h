#ifndef WIFI_UDP_H_
#define WIFI_UDP_H_

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 STA Wi-Fi 并建立 UDP 目标
 * @param ssid       Wi-Fi SSID
 * @param pass       Wi-Fi 密码
 * @param dest_ip    目标主机（点分十进制）
 * @param dest_port  目标 UDP 端口
 * @return ESP_OK 成功，其它为 ESP_ERR_*
 */
esp_err_t wifi_udp_init(const char *ssid,
                        const char *pass,
                        const char *dest_ip,
                        uint16_t    dest_port);

/**
 * @brief 通过已建立的 UDP socket 发送数据
 * @note   若未成功初始化或 socket 不可用，此函数自动忽略
 */
void wifi_udp_send(const void *data, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* WIFI_UDP_H_ */
