#include "cam_client.h"
#include "config.h"
#include "wifi.h"

#include "esp_http_client.h"
#include "esp_log.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "cam_client";

/* ================================================================== */
/* Buffer de respuesta HTTP                                           */
/* ================================================================== */

/*
 * El JSON de /centroid tiene ~120 caracteres. 512 B es más que suficiente
 * y evita asignación dinámica (malloc) en heap embebido.
 */
#define RESP_BUF_SIZE 512

static char s_buf[RESP_BUF_SIZE];
static int  s_buf_len;

/* ================================================================== */
/* Callback del cliente HTTP                                          */
/* ================================================================== */

static esp_err_t http_event_cb(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        int to_copy = evt->data_len;
        if (s_buf_len + to_copy > RESP_BUF_SIZE - 1) {
            to_copy = RESP_BUF_SIZE - 1 - s_buf_len;
        }
        if (to_copy > 0) {
            memcpy(s_buf + s_buf_len, evt->data, to_copy);
            s_buf_len += to_copy;
        }
    }
    return ESP_OK;
}

/* ================================================================== */
/* API pública                                                        */
/* ================================================================== */

cam_status_t cam_get_centroid(cam_data_t *out)
{
    if (!wifi_is_connected()) {
        return CAM_ERR_HTTP;
    }

    s_buf_len = 0;
    memset(s_buf, 0, sizeof(s_buf));

    esp_http_client_config_t cfg = {
        .url           = CFG_CAM_CENTROID_URL,
        .timeout_ms    = CFG_CAM_HTTP_TIMEOUT,
        .event_handler = http_event_cb,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return CAM_ERR_HTTP;

    esp_err_t err    = esp_http_client_perform(client);
    int       status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200) {
        ESP_LOGW(TAG, "HTTP error — err=0x%x status=%d", err, status);
        return CAM_ERR_HTTP;
    }

    /*
     * Parseo manual del JSON de formato fijo:
     *   {"dx":f,"dy":f,"cx":f,"cy":f,"area":u,"valid":b,...}
     *
     * strstr localiza el campo por nombre, strtof/strtoul parsean el valor.
     * Robusto al orden de campos porque busca cada etiqueta de forma
     * independiente.
     */
    char *p;

    p = strstr(s_buf, "\"dx\":");
    if (!p) return CAM_ERR_PARSE;
    out->dx = strtof(p + 5, NULL);

    p = strstr(s_buf, "\"dy\":");
    if (!p) return CAM_ERR_PARSE;
    out->dy = strtof(p + 5, NULL);

    p = strstr(s_buf, "\"area\":");
    if (!p) return CAM_ERR_PARSE;
    out->area = (uint32_t)strtoul(p + 7, NULL, 10);

    p = strstr(s_buf, "\"valid\":");
    if (!p) return CAM_ERR_PARSE;
    out->valid = (strncmp(p + 8, "true", 4) == 0);

    ESP_LOGD(TAG, "dx=%.2f dy=%.2f area=%lu valid=%d",
             out->dx, out->dy, (unsigned long)out->area, out->valid);

    return CAM_OK;
}

/* ================================================================== */
/* cam_post_position                                                   */
/* ================================================================== */

cam_status_t cam_post_position(float theta_mm, float phi_mm)
{
    if (!wifi_is_connected()) return CAM_ERR_HTTP;

    char body[48];
    int body_len = snprintf(body, sizeof(body),
                            "{\"theta\":%.2f,\"phi\":%.2f}", theta_mm, phi_mm);

    esp_http_client_config_t cfg = {
        .url        = CFG_CAM_POSITION_URL,
        .timeout_ms = CFG_CAM_HTTP_TIMEOUT,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return CAM_ERR_HTTP;

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, body_len);

    esp_err_t err    = esp_http_client_perform(client);
    int       status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200) {
        ESP_LOGW(TAG, "POST /position error — err=0x%x status=%d", err, status);
        return CAM_ERR_HTTP;
    }

    ESP_LOGD(TAG, "Posicion reportada theta=%.2f phi=%.2f", theta_mm, phi_mm);
    return CAM_OK;
}
