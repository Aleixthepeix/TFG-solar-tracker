#pragma once

/* ================================================================== */
/* config.h — constantes de hardware del proyecto                     */
/* ================================================================== */

/* ------------------------------------------------------------------ */
/* Stepper — Eje THETA (elevación)                                    */
/* ------------------------------------------------------------------ */
#define CFG_THETA_STEP_GPIO   14
#define CFG_THETA_DIR_GPIO    15
#define CFG_THETA_EN_GPIO     13
#define CFG_THETA_INVERT      false

/* ------------------------------------------------------------------ */
/* Stepper — Eje PHI (acimut)                                         */
/* ------------------------------------------------------------------ */
#define CFG_PHI_STEP_GPIO      2
#define CFG_PHI_DIR_GPIO       4
#define CFG_PHI_EN_GPIO       12
#define CFG_PHI_INVERT        false

/* ------------------------------------------------------------------ */
/* Stepper — velocidad y aceleración (FastAccelStepper)               */
/* ------------------------------------------------------------------ */
#define CFG_STEP_FREQ_MIN_HZ    80u
#define CFG_STEP_FREQ_MAX_HZ  1500u
#define CFG_STEP_ACCEL_HZ_S   2000u

/* ------------------------------------------------------------------ */
/* Actuador lineal — cinemática y límites de carrera                  */
/* ------------------------------------------------------------------ */
#define CFG_STEPS_PER_REV       1600u
#define CFG_ACTUATOR_MM_REV     5.0f
#define CFG_ACTUATOR_STROKE     300.0f
#define CFG_ACTUATOR_HOME_MM    150.0f
#define CFG_ACTUATOR_MARGIN_MM  5.0f

/*
 * GPIOs de los interruptores de fin de carrera (posición mínima = origen).
 * Cableado esperado: NC conectado a GND → activo en LOW con pull-up interno.
 * Poner a -1 para deshabilitar (homing por tiempo).
 */
#define CFG_THETA_ENDSTOP_GPIO  -1   /* TODO: asignar GPIO real al instalar */
#define CFG_PHI_ENDSTOP_GPIO    -1   /* TODO: asignar GPIO real al instalar */

/* ------------------------------------------------------------------ */
/* WiFi — conexión al AP del ESP32-CAM                                */
/* ------------------------------------------------------------------ */
#define CFG_WIFI_SSID                "ESP32-CAM-Vision"
#define CFG_WIFI_PASS                "12345678"
#define CFG_WIFI_CONNECT_TIMEOUT_MS  15000u

/*
 * IP estática del ESP32-Motor en la red del AP del ESP32-CAM.
 * Fija para que solar_monitor siempre sepa dónde encontrar el Motor.
 * El AP de ESP32-CAM asigna su propia IP en 192.168.4.1.
 */
#define CFG_MOTOR_STATIC_IP          "192.168.4.2"

/* ------------------------------------------------------------------ */
/* Cliente HTTP — servidor de centroide en el ESP32-CAM               */
/* ------------------------------------------------------------------ */
/*
 * El ESP32-CAM corre en modo AP con IP fija 192.168.4.1.
 * El endpoint /centroid devuelve JSON con dx, dy, area, valid cada 1 s.
 */
#define CFG_CAM_SERVER        "http://192.168.4.1"
#define CFG_CAM_CENTROID_URL  "http://192.168.4.1/centroid"
#define CFG_CAM_POSITION_URL  "http://192.168.4.1/position"  /* POST theta+phi → Kivy */
#define CFG_CAM_POLL_MS      30000u    /* periodo entre capturas del centroide [ms] */
#define CFG_CAM_HTTP_TIMEOUT  2000u    /* timeout HTTP [ms] */

/* ------------------------------------------------------------------ */
/* Cámara — geometría de imagen para conversión px → grados           */
/* ------------------------------------------------------------------ */
/*
 * CFG_CAM_FOV_H_DEG / CFG_CAM_FOV_V_DEG:
 *   Campo de visión horizontal y vertical de la óptica del ESP32-CAM.
 *   El valor por defecto (65° × 50°) es una estimación para la óptica
 *   estándar del módulo AI-Thinker con OV2640 en QQVGA.
 *   CALIBRAR midiendo cuántos grados de giro corresponden a cuántos
 *   píxeles de desplazamiento del centroide.
 *
 * CFG_CAM_FRAME_W / CFG_CAM_FRAME_H:
 *   Resolución de la imagen que usa proceso_cam_v1 (QQVGA = 160×120).
 *
 * Derivados:
 *   CFG_CAM_PX_TO_DEG_H = FOV_H / FRAME_W  [°/px horizontal]
 *   CFG_CAM_PX_TO_DEG_V = FOV_V / FRAME_H  [°/px vertical]
 */
#define CFG_CAM_FOV_H_DEG   65.0f
#define CFG_CAM_FOV_V_DEG   50.0f
#define CFG_CAM_FRAME_W     160
#define CFG_CAM_FRAME_H     120

#define CFG_CAM_PX_TO_DEG_H  (CFG_CAM_FOV_H_DEG / (float)CFG_CAM_FRAME_W)
#define CFG_CAM_PX_TO_DEG_V  (CFG_CAM_FOV_V_DEG / (float)CFG_CAM_FRAME_H)

/* ------------------------------------------------------------------ */
/* Control — posición predictiva con cinemática inversa               */
/* ------------------------------------------------------------------ */
/*
 * CFG_CTRL_DEADZONE_PX : zona muerta en píxeles.
 *   Correcciones menores a este umbral se ignoran para evitar
 *   vibraciones por ruido de detección.
 *
 * CFG_CTRL_SPEED_HZ : velocidad de los actuadores durante corrección.
 *   Debe ser suficiente para completar el movimiento en CFG_CAM_POLL_MS.
 *   A 600 Hz y 320 pasos/mm → 1.875 mm/s → ~1 mm/ciclo a 1 fps.
 */
#define CFG_CTRL_DEADZONE_PX   3
#define CFG_CTRL_SPEED_HZ      600u

/* ------------------------------------------------------------------ */
/* Homing — recorrido al fin de carrera mínimo al arrancar            */
/* ------------------------------------------------------------------ */
/*
 * A 800 Hz y 320 pasos/mm → 2.5 mm/s → carrera 330 mm en ~132 s.
 * Ambos ejes se mueven en paralelo; tiempo total ≤ 132 s.
 * Reducir si los actuadores no soportan impactar el tope a esa vel.
 */
#define CFG_HOME_SPEED_HZ         800u

/* ------------------------------------------------------------------ */
/* Barrido inicial — búsqueda del sol                                 */
/* ------------------------------------------------------------------ */
#define CFG_SCAN_SPEED_HZ         150u
#define CFG_SCAN_THETA_STEP_MM    10.0f
#define CFG_SCAN_LOST_FRAMES      5u    /* a 1 fps: 5 s sin sol antes de re-barrer */

/* ------------------------------------------------------------------ */
/* Robustez — Watchdog (WDT)                                          */
/* ------------------------------------------------------------------ */
#define CFG_WDT_TIMEOUT_MS    35000u   /* CFG_CAM_POLL_MS (30 s) + HTTP (2 s) + margen */
