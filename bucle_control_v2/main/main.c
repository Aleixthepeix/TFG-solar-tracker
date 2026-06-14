#include "config.h"
#include "stepper.h"
#include "homing.h"
#include "control.h"
#include "console.h"
#include "wifi.h"
#include "cmd_server.h"

#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "=== Solar Tracker v2 arrancando ===");

    /* ── NVS ── */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupta — borrando y reiniciando");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    /* ── WDT ── */
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = CFG_WDT_TIMEOUT_MS,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_cfg));

    /* ── Infraestructura de red ── */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ── Motores paso a paso ── */
    ESP_ERROR_CHECK(step_init());

    /* ── Homing — referencia absoluta al arrancar ── */
    ESP_ERROR_CHECK(home_all_axes());

    /* ── Consola de depuración ── */
    if (console_start() != ESP_OK) {
        ESP_LOGW(TAG, "Consola no disponible — continuando sin ella");
    }

    /* ── WiFi → AP del ESP32-CAM ── */
    if (wifi_connect() != ESP_OK) {
        ESP_LOGW(TAG, "Sin WiFi — el bucle de control reintentará cada ciclo");
    }

    /* ── Servidor HTTP de comandos remotos (solar_monitor) ── */
    if (cmd_server_start() != ESP_OK) {
        ESP_LOGW(TAG, "Servidor de comandos no disponible");
    }

    /* ── Bucle de control (alta prioridad) ── */
    ESP_ERROR_CHECK(ctrl_start());
}
