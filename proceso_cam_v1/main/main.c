#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "vision.h"
#include "server.h"
#include <string.h>

static const char *TAG = "MAIN";

/* ── Configuracion WiFi (modo AP) ──────────────────────────────── *
 *
 *  El ESP32-CAM actua como punto de acceso. Conectar el PC o telefono
 *  a esta red y abrir http://192.168.4.1/ para ver el stream y el
 *  centroide en tiempo real.
 *
 *  Para usar modo STA (conectarse a un router existente) sustituir
 *  wifi_init_ap() por el patron estandar de esp_wifi en modo STATION.
 */
#define WIFI_SSID       "ESP32-CAM-Vision"
#define WIFI_PASS       "12345678"        /* minimo 8 caracteres para WPA2 */
#define WIFI_CHANNEL    6
#define WIFI_MAX_STA    2

static void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&icfg));

    wifi_config_t wcfg = {
        .ap = {
            .channel        = WIFI_CHANNEL,
            .max_connection = WIFI_MAX_STA,
            .authmode       = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    strlcpy((char *)wcfg.ap.ssid,     WIFI_SSID, sizeof(wcfg.ap.ssid));
    strlcpy((char *)wcfg.ap.password, WIFI_PASS,  sizeof(wcfg.ap.password));
    wcfg.ap.ssid_len = (uint8_t)strlen(WIFI_SSID);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP listo  SSID: \"%s\"  IP: 192.168.4.1", WIFI_SSID);
}

/* ── Punto de entrada ──────────────────────────────────────────── */
void app_main(void)
{
    /*
     * Orden de arranque:
     *   1. NVS  (requerido por WiFi)
     *   2. WiFi AP
     *   3. Camara
     *   4. Servidor HTTP  (inicializa mutex interno)
     *   5. Tarea de captura
     */
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_ap();

    ESP_ERROR_CHECK(vision_init());

    ESP_ERROR_CHECK(server_start());

    /* Prioridad 4: por debajo del stack WiFi (5) para no bloquear ACKs */
    xTaskCreate(capture_task, "capture", 8192, NULL, 4, NULL);

    ESP_LOGI(TAG, "Sistema listo — abre http://192.168.4.1/ en el navegador");
}
