#pragma once

#include "esp_err.h"

/* ================================================================== */
/* console.h — consola de depuración via UART (esp_console / REPL)   */
/*                                                                    */
/* Comandos disponibles:                                              */
/*   step <eje> <hz> <+|->  — mueve un motor (pausa ctrl auto)       */
/*   stop [theta|phi]       — para uno o ambos motores               */
/*   ctrl <pause|resume>    — pausa o reanuda el bucle de control     */
/*   kin  <theta|phi> <deg> — cinemática inversa: ángulo → A [mm]    */
/*   pos                    — posición estimada de ambos actuadores   */
/*   setpos <eje> <mm>      — corrige la posición estimada            */
/*   scan                   — fuerza re-barrido inmediato             */
/*   help                   — lista todos los comandos                */
/* ================================================================== */

esp_err_t console_start(void);
