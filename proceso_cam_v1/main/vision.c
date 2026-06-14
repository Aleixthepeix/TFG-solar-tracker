#include "vision.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "driver/ledc.h"

static const char *TAG = "VISION";

/* ── Inicializacion de camara ───────────────────────────────────── */
esp_err_t vision_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn       = CAM_PIN_PWDN,
        .pin_reset      = CAM_PIN_RESET,
        .pin_xclk       = CAM_PIN_XCLK,
        .pin_sccb_sda   = CAM_PIN_SIOD,
        .pin_sccb_scl   = CAM_PIN_SIOC,
        .pin_d7         = CAM_PIN_D7,
        .pin_d6         = CAM_PIN_D6,
        .pin_d5         = CAM_PIN_D5,
        .pin_d4         = CAM_PIN_D4,
        .pin_d3         = CAM_PIN_D3,
        .pin_d2         = CAM_PIN_D2,
        .pin_d1         = CAM_PIN_D1,
        .pin_d0         = CAM_PIN_D0,
        .pin_vsync      = CAM_PIN_VSYNC,
        .pin_href       = CAM_PIN_HREF,
        .pin_pclk       = CAM_PIN_PCLK,

        .xclk_freq_hz   = 20000000,
        .ledc_timer     = LEDC_TIMER_1,    /* LEDC_TIMER_0 reservado para motor */
        .ledc_channel   = LEDC_CHANNEL_1,

        .pixel_format   = PIXFORMAT_GRAYSCALE,
        .frame_size     = FRAMESIZE_QQVGA, /* 160 x 120 — sin PSRAM */
        .jpeg_quality   = 0,               /* irrelevante en GRAYSCALE */
        .fb_count       = 1,
        .fb_location    = CAMERA_FB_IN_DRAM,
        .grab_mode      = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init falló: 0x%x", err);
    } else {
        ESP_LOGI(TAG, "Camara OK — QVGA escala de grises (%dx%d) umbral=%d",
                 CAM_FRAME_W, CAM_FRAME_H, CAM_THRESHOLD);
    }
    return err;
}

/* ── Calculo del centroide por brillo ponderado ─────────────────── *
 *
 *  Cada pixel con valor >= CAM_THRESHOLD contribuye al centroide
 *  con un peso igual a su intensidad. Esto localiza la region mas
 *  brillante de la imagen, util para seguimiento de LEDs, marcadores
 *  retro-reflectantes o cualquier objeto mas luminoso que el fondo.
 *
 *  dx = cx - W/2  (positivo → objeto a la derecha del centro)
 *  dy = cy - H/2  (positivo → objeto por debajo del centro)
 */
centroid_t vision_compute_centroid(const uint8_t *buf, int w, int h)
{
    centroid_t c = {0};
    double sum_x = 0.0, sum_y = 0.0, sum_w = 0.0;
    uint32_t area = 0;

    for (int y = 0; y < h; y++) {
        const uint8_t *row = buf + y * w;
        for (int x = 0; x < w; x++) {
            uint8_t p = row[x];
            if (p >= CAM_THRESHOLD) {
                sum_x += (double)x * p;
                sum_y += (double)y * p;
                sum_w += p;
                area++;
            }
        }
    }

    if (area > 0 && sum_w > 0.0) {
        c.cx    = (float)(sum_x / sum_w);
        c.cy    = (float)(sum_y / sum_w);
        c.dx    = c.cx - w * 0.5f;
        c.dy    = c.cy - h * 0.5f;
        c.area  = area;
        c.valid = true;
    }
    return c;
}

/* ── Dibuja una cruz sobre el buffer de pixeles (in-place) ─────── */
void vision_draw_cross(uint8_t *buf, int w, int h,
                       int cx, int cy, int size, uint8_t color)
{
    for (int x = cx - size; x <= cx + size; x++) {
        if (x >= 0 && x < w && cy >= 0 && cy < h) {
            buf[cy * w + x] = color;
        }
    }
    for (int y = cy - size; y <= cy + size; y++) {
        if (y >= 0 && y < h && cx >= 0 && cx < w) {
            buf[y * w + cx] = color;
        }
    }
}
