#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* ================================================================== */
/* control.h — bucle de control FreeRTOS del seguidor solar v2        */
/* ================================================================== */

/*
 * Crea la tarea FreeRTOS que ejecuta el bucle de control en segundo
 * plano.  Llámala desde app_main después de step_init().
 *
 * La tarea conecta al ESP32-CAM vía WiFi, hace GET /centroid cada
 * CFG_CAM_POLL_MS ms, convierte dx/dy [px] a grados mediante FOV,
 * acumula el ángulo y usa kin_inverse() para comandar step_move_to_mm().
 */
esp_err_t ctrl_start(void);

/* Pausa el bucle (motores parados) — para operación manual por consola */
void ctrl_pause(void);

/* Reanuda el bucle tras una pausa */
void ctrl_resume(void);

/* Fuerza re-barrido inmediato desde la posición actual */
void ctrl_restart_scan(void);

/* Devuelve true si el bucle está pausado */
bool ctrl_is_paused(void);
