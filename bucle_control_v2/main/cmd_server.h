#pragma once

#include "esp_err.h"

/* ================================================================== */
/* cmd_server.h — servidor HTTP de comandos remotos                   */
/*                                                                    */
/* Expone dos endpoints sobre el puerto 80:                           */
/*                                                                    */
/*   POST /cmd   cuerpo: texto del comando (p.ej. "ctrl pause")      */
/*               respuesta: texto plano con el resultado              */
/*                                                                    */
/*   GET  /status  respuesta JSON: posicion + estado del bucle        */
/*                 {"theta":<mm>,"phi":<mm>,"paused":<bool>}         */
/*                                                                    */
/* Llamar después de wifi_connect() y step_init().                   */
/* ================================================================== */

esp_err_t cmd_server_start(void);
