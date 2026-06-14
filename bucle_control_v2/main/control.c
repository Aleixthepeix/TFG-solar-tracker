#include "control.h"
#include "config.h"
#include "stepper.h"
#include "cam_client.h"
#include "kinematics.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include <stdint.h>
#include <stdbool.h>

static const char *TAG = "control";

/* ================================================================== */
/* Máquina de estados                                                  */
/* ================================================================== */

typedef enum {
    CTRL_STATE_SCAN_PHI,
    CTRL_STATE_SCAN_THETA_STEP,
    CTRL_STATE_TRACK,
} ctrl_state_t;

static ctrl_state_t  s_state     = CTRL_STATE_SCAN_PHI;
static bool          s_phi_fwd   = true;
static bool          s_theta_fwd = true;
static uint32_t      s_lost      = 0;

/* Acumuladores de ángulo para control predictivo */
static float s_angle_theta = 0.0f;
static float s_angle_phi   = 0.0f;


static volatile bool s_paused       = true;   /* espera comando desde solar_monitor */
static volatile bool s_restart_scan = false;

/* ================================================================== */
/* API pública — pausa / reanuda / reinicia barrido                   */
/* ================================================================== */

void ctrl_pause(void)
{
    s_paused = true;
    vTaskDelay(pdMS_TO_TICKS(50));
}

void ctrl_resume(void)
{
    s_paused = false;
}

void ctrl_restart_scan(void)
{
    s_restart_scan = true;
}

bool ctrl_is_paused(void)
{
    return s_paused;
}

/* ================================================================== */
/* Helper: avance THETA al siguiente nivel de barrido                 */
/* ================================================================== */

static bool scan_step_theta(void)
{
    if (!step_at_limit(STEP_THETA, s_theta_fwd)) {
        float delta = s_theta_fwd ? CFG_SCAN_THETA_STEP_MM : -CFG_SCAN_THETA_STEP_MM;
        step_move_mm(STEP_THETA, delta, CFG_SCAN_SPEED_HZ);
        return true;
    }
    s_theta_fwd = !s_theta_fwd;
    if (!step_at_limit(STEP_THETA, s_theta_fwd)) {
        float delta = s_theta_fwd ? CFG_SCAN_THETA_STEP_MM : -CFG_SCAN_THETA_STEP_MM;
        step_move_mm(STEP_THETA, delta, CFG_SCAN_SPEED_HZ);
        return true;
    }
    return false;
}

/* ================================================================== */
/* Tarea FreeRTOS de control                                          */
/* ================================================================== */

static void ctrl_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_LOGI(TAG, "Control v2 iniciado — esperando 5 s para que el AP se estabilice");
    /* Dar tiempo al AP del ESP32-CAM (y al PC) para terminar la asociación
     * WiFi antes de lanzar el primer request HTTP. */
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Iniciando barrido");

    while (1) {
        esp_task_wdt_reset();

        if (s_paused) {
            step_stop(STEP_THETA);
            step_stop(STEP_PHI);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (s_restart_scan) {
            s_restart_scan = false;
            s_state        = CTRL_STATE_SCAN_PHI;
            s_phi_fwd      = true;
            s_theta_fwd    = true;
            s_lost         = 0;
            step_stop(STEP_THETA);
            step_stop(STEP_PHI);
            ESP_LOGI(TAG, "Re-barrido iniciado por consola");
        }

        switch (s_state) {

        /* ────────────────────────────────────────────────────────── */
        /* SCAN_PHI: PHI barre buscando el sol fila a fila            */
        /* ────────────────────────────────────────────────────────── */
        case CTRL_STATE_SCAN_PHI: {
            cam_data_t    cam;
            cam_status_t  cs = cam_get_centroid(&cam);

            if (cs == CAM_ERR_HTTP) {
                /* CAM no accesible — esperar sin saturar el AP */
                ESP_LOGW(TAG, "CAM no responde — reintentando en 3 s");
                vTaskDelay(pdMS_TO_TICKS(3000));
                break;
            }

            /* Reportar posición solo cuando la CAM está disponible */
            cam_post_position(step_get_pos_mm(STEP_THETA),
                              step_get_pos_mm(STEP_PHI));

            if (cs == CAM_OK && cam.valid) {
                step_stop(STEP_THETA);
                step_stop(STEP_PHI);
                s_angle_theta = kin_solve_theta(step_get_pos_mm(STEP_THETA));
                s_angle_phi   = kin_solve_phi  (step_get_pos_mm(STEP_PHI));
                s_state       = CTRL_STATE_TRACK;
                s_lost        = 0;
                ESP_LOGI(TAG, "Sol encontrado — theta=%.1f° phi=%.1f° → TRACK",
                         (double)s_angle_theta, (double)s_angle_phi);
                break;
            }

            /* Sol no detectado — seguir barriendo PHI */
            if (!step_at_limit(STEP_PHI, s_phi_fwd)) {
                step_set_speed(STEP_PHI, CFG_SCAN_SPEED_HZ, s_phi_fwd);
            } else {
                step_stop(STEP_PHI);
                if (scan_step_theta()) {
                    s_phi_fwd = !s_phi_fwd;
                    s_state   = CTRL_STATE_SCAN_THETA_STEP;
                    ESP_LOGD(TAG, "Nueva fila — THETA→%.1f mm  PHI_fwd=%d",
                             (double)step_get_pos_mm(STEP_THETA), s_phi_fwd);
                } else {
                    ESP_LOGW(TAG, "Barrido completo sin sol — esperando 5 s...");
                    step_stop(STEP_THETA);
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    s_phi_fwd   = true;
                    s_theta_fwd = true;
                    ESP_LOGI(TAG, "Reiniciando barrido desde posición actual");
                }
            }
            break;
        }

        /* ────────────────────────────────────────────────────────── */
        /* SCAN_THETA_STEP: esperar que THETA complete el desplaz.    */
        /* ────────────────────────────────────────────────────────── */
        case CTRL_STATE_SCAN_THETA_STEP:
            if (!step_is_moving(STEP_THETA)) {
                s_state = CTRL_STATE_SCAN_PHI;
            } else {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            break;

        /* ────────────────────────────────────────────────────────── */
        /* TRACK: control predictivo con cinemática inversa           */
        /* Cadencia: CFG_CAM_POLL_MS (= 1000 ms por defecto)         */
        /* ────────────────────────────────────────────────────────── */
        case CTRL_STATE_TRACK: {
            cam_data_t   cam;
            cam_status_t cs = cam_get_centroid(&cam);

            if (cs != CAM_OK || !cam.valid) {
                s_lost++;
                if (s_lost > CFG_SCAN_LOST_FRAMES) {
                    ESP_LOGW(TAG, "Sol perdido (%lu frames) — re-barrido",
                             (unsigned long)s_lost);
                    s_state = CTRL_STATE_SCAN_PHI;
                    s_lost  = 0;
                    step_stop(STEP_THETA);
                    step_stop(STEP_PHI);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(CFG_CAM_POLL_MS));
                break;
            }
            s_lost = 0;

            /* Filtrar zona muerta */
            float dx = (cam.dx > CFG_CTRL_DEADZONE_PX || cam.dx < -CFG_CTRL_DEADZONE_PX)
                       ? cam.dx : 0.0f;
            float dy = (cam.dy > CFG_CTRL_DEADZONE_PX || cam.dy < -CFG_CTRL_DEADZONE_PX)
                       ? cam.dy : 0.0f;

            /* px → grados → acumular */
            float dtheta = dy * CFG_CAM_PX_TO_DEG_V;
            float dphi   = dx * CFG_CAM_PX_TO_DEG_H;
            s_angle_theta += dtheta;
            s_angle_phi   += dphi;

            /* Cinemática inversa y movimiento absoluto */
            float A_theta, A_phi;

            if (kin_inverse(s_angle_theta, false, &A_theta) == KIN_OK) {
                step_move_to_mm(STEP_THETA, A_theta, CFG_CTRL_SPEED_HZ);
            } else {
                ESP_LOGW(TAG, "kin_inverse theta sin solución (%.1f°)", (double)s_angle_theta);
                s_angle_theta -= dtheta;
            }

            if (kin_inverse(s_angle_phi, true, &A_phi) == KIN_OK) {
                step_move_to_mm(STEP_PHI, A_phi, CFG_CTRL_SPEED_HZ);
            } else {
                ESP_LOGW(TAG, "kin_inverse phi sin solución (%.1f°)", (double)s_angle_phi);
                s_angle_phi -= dphi;
            }

            /* Reportar posicion absoluta al ESP32-CAM para que Kivy la lea */
            cam_post_position(step_get_pos_mm(STEP_THETA),
                              step_get_pos_mm(STEP_PHI));

            ESP_LOGD(TAG, "dx=%.1f dy=%.1f  θ=%.1f° φ=%.1f°  A_θ=%.1fmm A_φ=%.1fmm",
                     (double)cam.dx, (double)cam.dy,
                     (double)s_angle_theta, (double)s_angle_phi,
                     (double)A_theta, (double)A_phi);
            step_save_pos();
            vTaskDelay(pdMS_TO_TICKS(CFG_CAM_POLL_MS));
            break;
        }

        } /* switch */
    }
}

/* ================================================================== */
/* API pública — arranque                                             */
/* ================================================================== */

esp_err_t ctrl_start(void)
{
    BaseType_t ret = xTaskCreate(ctrl_task, "ctrl", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate falló — heap insuficiente");
        return ESP_FAIL;
    }
    return ESP_OK;
}
