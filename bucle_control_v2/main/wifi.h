#pragma once

#include "esp_err.h"
#include <stdbool.h>

/* Conecta en modo STA con las credenciales de config.h.
 * Bloquea hasta obtener IP o hasta CFG_WIFI_CONNECT_TIMEOUT_MS. */
esp_err_t wifi_connect(void);

/* Devuelve true si la IP está asignada. */
bool wifi_is_connected(void);
