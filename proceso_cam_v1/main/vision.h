#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ── Pinout AI-Thinker ESP32-CAM ────────────────────────────────── */
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

/* ── Parametros de captura ──────────────────────────────────────── */
#define CAM_FRAME_W     160     /* QQVGA — sin PSRAM es el maximo viable */
#define CAM_FRAME_H     120
#define CAM_THRESHOLD   200     /* 0-255: pixels >= umbral contribuyen al centroide */

/* Dibuja un marcador en el frame procesado antes de enviar (1 = activo) */
#define ANNOTATE_FRAME  0

/* ── Resultado del analisis por frame ───────────────────────────── */
typedef struct {
    float    cx;      /* centroide x (px, origen esquina sup-izq)   */
    float    cy;      /* centroide y (px)                            */
    float    dx;      /* desviacion horizontal del centro de imagen  */
    float    dy;      /* desviacion vertical del centro de imagen    */
    uint32_t area;    /* numero de pixels sobre el umbral            */
    bool     valid;   /* false si no se detecto ningun objeto        */
} centroid_t;

/* ── API ────────────────────────────────────────────────────────── */
esp_err_t  vision_init(void);
centroid_t vision_compute_centroid(const uint8_t *buf, int w, int h);
void       vision_draw_cross(uint8_t *buf, int w, int h,
                             int cx, int cy, int size, uint8_t color);
