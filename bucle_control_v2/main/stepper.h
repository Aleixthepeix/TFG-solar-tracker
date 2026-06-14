#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ================================================================== */
/* stepper.h — driver de motores paso a paso via FastAccelStepper     */
/* ================================================================== */

typedef enum {
    STEP_OK = 0,
    STEP_ERR_INVALID,
} step_status_t;

typedef enum {
    STEP_THETA = 0,
    STEP_PHI   = 1,
    STEP_AXIS_COUNT,
} step_axis_t;

/* Inicializa ambos ejes y activa drivers (EN en bajo) */
step_status_t step_init(void);

/* Velocidad continua — usado por el barrido */
void step_set_speed(step_axis_t axis, uint32_t freq_hz, bool dir_positive);

/* Para con rampa de deceleración */
void step_stop(step_axis_t axis);

/*
 * Mueve a una posición ABSOLUTA en mm a la velocidad indicada (asíncrono).
 * Usa moveTo() internamente: si el motor ya se mueve, sobreescribe el
 * destino anterior en lugar de sumar desplazamiento. Ideal para el bucle
 * de control de posición donde cada ciclo recalcula el objetivo.
 */
void step_move_to_mm(step_axis_t axis, float target_mm, uint32_t speed_hz);

/*
 * Mueve un desplazamiento RELATIVO en mm (asíncrono).
 * Útil para el paso THETA entre filas del barrido.
 */
void step_move_mm(step_axis_t axis, float delta_mm, uint32_t speed_hz);

/* true mientras el eje está ejecutando un movimiento */
bool step_is_moving(step_axis_t axis);

/* Posición estimada en mm (muertos de pasos = error acumulado) */
float step_get_pos_mm(step_axis_t axis);

/* Corrige la posición estimada a un valor conocido */
void step_set_pos_mm(step_axis_t axis, float mm);

/* Persiste la posición actual de ambos ejes en NVS */
void step_save_pos(void);

/* true si el endstop hardware del eje está activo (GPIO en LOW) */
bool step_endstop_triggered(step_axis_t axis);

/* true si el actuador está en la zona de seguridad del extremo indicado */
bool step_at_limit(step_axis_t axis, bool dir_positive);

#ifdef __cplusplus
}
#endif
