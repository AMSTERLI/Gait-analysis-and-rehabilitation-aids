#include "wifi_udp.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_check.h"  
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "wifi_udp";

static EventGroupHandle_t s_evt_grp;
#define WIFI_CONNECTED_BIT BIT0

static int udp_sock = -1;
static struct sockaddr_in dest_addr   = {};

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id,
                               void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGW(TAG, "Reconnecting Wi-Fi…");
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_evt_grp, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
    }
}

esp_err_t wifi_udp_init(const char *ssid,
                        const char *pass,
                        const char *dest_ip,
                        uint16_t    dest_port)
{
    ESP_RETURN_ON_FALSE(ssid && pass && dest_ip, ESP_ERR_INVALID_ARG, TAG,
                        "null arg");

    /* ---------- NVS ---------- */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* ---------- 网络栈 ---------- */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    /* ---------- Wi-Fi ---------- */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    s_evt_grp = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid, ssid, sizeof(wcfg.sta.ssid) - 1);
    strncpy((char *)wcfg.sta.password, pass, sizeof(wcfg.sta.password) - 1);
    wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* ---------- 等待连网 ---------- */
    xEventGroupWaitBits(s_evt_grp, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                        portMAX_DELAY);

    /* ---------- UDP ---------- */
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        return ESP_FAIL;
    }
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(dest_port);
    dest_addr.sin_addr.s_addr = inet_addr(dest_ip);
    ESP_LOGI(TAG, "UDP dest %s:%u ready", dest_ip, dest_port);
    return ESP_OK;
}

void wifi_udp_send(const void *data, size_t len)
{
    if (udp_sock < 0 || !data || len == 0) return;
    sendto(udp_sock, data, len, 0,
           (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}
