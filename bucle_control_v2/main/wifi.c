#include "wifi.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "lwip/ip4_addr.h"

#include <string.h>

static const char *TAG = "wifi";

static EventGroupHandle_t s_eg;
static volatile bool      s_connected = false;

#define CONNECTED_BIT  BIT0
#define FAIL_BIT       BIT1

static void wifi_event_handler(void *arg,
                               esp_event_base_t base,
                               int32_t          id,
                               void            *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        esp_wifi_connect();
        xEventGroupSetBits(s_eg, FAIL_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_connected = true;
        xEventGroupSetBits(s_eg, CONNECTED_BIT);
    }
}

esp_err_t wifi_connect(void)
{
    s_eg = xEventGroupCreate();

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    /* IP estática para que solar_monitor siempre encuentre el Motor */
    esp_netif_ip_info_t ip_info = {};
    ip4addr_aton(CFG_MOTOR_STATIC_IP, (ip4_addr_t *)&ip_info.ip);
    ip4addr_aton("192.168.4.1",       (ip4_addr_t *)&ip_info.gw);
    ip4addr_aton("255.255.255.0",     (ip4_addr_t *)&ip_info.netmask);
    esp_netif_dhcpc_stop(sta_netif);
    esp_netif_set_ip_info(sta_netif, &ip_info);

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg;
    memset(&wifi_cfg, 0, sizeof(wifi_cfg));
    strncpy((char *)wifi_cfg.sta.ssid,     CFG_WIFI_SSID,
            sizeof(wifi_cfg.sta.ssid)     - 1);
    strncpy((char *)wifi_cfg.sta.password, CFG_WIFI_PASS,
            sizeof(wifi_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    EventBits_t bits = xEventGroupWaitBits(
        s_eg,
        CONNECTED_BIT | FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(CFG_WIFI_CONNECT_TIMEOUT_MS));

    if (bits & CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi conectado al ESP32-CAM");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "WiFi no disponible (timeout o credenciales incorrectas)");
    return ESP_FAIL;
}

bool wifi_is_connected(void)
{
    return s_connected;
}
