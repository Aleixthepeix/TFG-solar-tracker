#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ================================================================== */
/* cam_client.h — cliente HTTP para el endpoint /centroid             */
/*                                                                    */
/* Hace GET http://192.168.4.1/centroid y parsea el JSON que devuelve */
/* proceso_cam_v1:                                                    */
/*   {"dx":f,"dy":f,"cx":f,"cy":f,"area":u,"valid":b,                */
/*    "frame_w":160,"frame_h":120}                                    */
/*                                                                    */
/* dx, dy: desviación del centroide solar respecto al centro [px].   */
/*   Positivo → derecha / abajo. Rango ±80 px / ±60 px (QQVGA).     */
/* valid: false si no se detectó ningún objeto brillante.            */
/* ================================================================== */

typedef struct {
    float    dx;      /* desviación horizontal del centro [px] */
    float    dy;      /* desviación vertical del centro [px]   */
    uint32_t area;    /* píxeles sobre el umbral               */
    bool     valid;   /* false = sol no detectado              */
} cam_data_t;

typedef enum {
    CAM_OK = 0,
    CAM_ERR_HTTP,    /* fallo de red o código HTTP != 200     */
    CAM_ERR_PARSE,   /* respuesta recibida pero JSON inválido */
} cam_status_t;

/*
 * Realiza un GET síncrono a CFG_CAM_CENTROID_URL y rellena *out.
 * Bloquea hasta obtener respuesta o hasta CFG_CAM_HTTP_TIMEOUT ms.
 * Devuelve CAM_ERR_HTTP si no hay WiFi o la petición falla.
 */
cam_status_t cam_get_centroid(cam_data_t *out);

/*
 * Envía un POST a CFG_CAM_POSITION_URL con la posición actual de
 * ambos actuadores.  Cuerpo: {"theta":<mm>,"phi":<mm>}
 * No bloquea mas de CFG_CAM_HTTP_TIMEOUT ms.
 */
cam_status_t cam_post_position(float theta_mm, float phi_mm);
