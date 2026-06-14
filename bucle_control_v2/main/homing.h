#pragma once

#include "esp_err.h"

/*
 * Mueve ambos ejes al fin de carrera mecánico mínimo y establece
 * posición = 0 como referencia absoluta.
 *
 * Bloqueante — llamar desde app_main después de step_init() y antes
 * de ctrl_start().  Ambos ejes se mueven en paralelo; el tiempo máximo
 * es (CFG_ACTUATOR_STROKE + 30 mm) / CFG_HOME_SPEED_HZ.
 */
esp_err_t home_all_axes(void);
