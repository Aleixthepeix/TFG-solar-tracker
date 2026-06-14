#pragma once
#include "esp_err.h"

/*
 * server_start() — inicializa mutex, registra rutas y arranca httpd.
 *   Llamar ANTES de crear capture_task.
 *
 * capture_task() — tarea FreeRTOS de captura continua.
 *   Crear con xTaskCreate(..., 8192, ...) despues de server_start().
 */
esp_err_t server_start(void);
void      capture_task(void *arg);
