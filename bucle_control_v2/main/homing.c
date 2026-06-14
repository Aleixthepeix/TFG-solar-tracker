#include "homing.h"
#include "stepper.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "homing";

#define POLL_MS  10u

esp_err_t home_all_axes(void)
{
    const float steps_per_mm = (float)CFG_STEPS_PER_REV / CFG_ACTUATOR_MM_REV;
    const float speed_mm_s   = (float)CFG_HOME_SPEED_HZ / steps_per_mm;
    const uint32_t timeout_ms = (uint32_t)(
        ((CFG_ACTUATOR_STROKE + 30.0f) / speed_mm_s) * 1000.0f);

    const bool has_endstop[STEP_AXIS_COUNT] = {
        CFG_THETA_ENDSTOP_GPIO >= 0,
        CFG_PHI_ENDSTOP_GPIO   >= 0,
    };

    ESP_LOGI(TAG, "Homing — %.2f mm/s  timeout %.0f s  endstops[theta=%d phi=%d]",
             (double)speed_mm_s, (double)(timeout_ms / 1000.0f),
             has_endstop[STEP_THETA], has_endstop[STEP_PHI]);

    step_set_speed(STEP_THETA, CFG_HOME_SPEED_HZ, false);
    step_set_speed(STEP_PHI,   CFG_HOME_SPEED_HZ, false);

    if (!has_endstop[STEP_THETA] && !has_endstop[STEP_PHI]) {
        /* Homing ciego por tiempo — ningún eje tiene endstop */
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        step_stop(STEP_THETA);
        step_stop(STEP_PHI);
    } else {
        /* Polling de endstops con timeout de seguridad */
        bool done[STEP_AXIS_COUNT] = { false, false };
        uint32_t elapsed = 0;

        while (elapsed < timeout_ms) {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
            elapsed += POLL_MS;

            for (int ax = 0; ax < STEP_AXIS_COUNT; ax++) {
                if (done[ax]) continue;
                if (has_endstop[ax] && step_endstop_triggered((step_axis_t)ax)) {
                    step_stop((step_axis_t)ax);
                    done[ax] = true;
                    ESP_LOGI(TAG, "Eje %d — endstop alcanzado (%.1f s)",
                             ax, (double)(elapsed / 1000.0f));
                }
            }

            if (done[STEP_THETA] && done[STEP_PHI]) break;
        }

        /* Parar ejes que no llegaron antes del timeout */
        for (int ax = 0; ax < STEP_AXIS_COUNT; ax++) {
            if (!done[ax]) {
                step_stop((step_axis_t)ax);
                if (has_endstop[ax])
                    ESP_LOGW(TAG, "Eje %d — timeout sin endstop (%.0f s)",
                             ax, (double)(timeout_ms / 1000.0f));
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(300));   /* esperar fin de rampa de deceleración */

    step_set_pos_mm(STEP_THETA, 0.0f);
    step_set_pos_mm(STEP_PHI,   0.0f);
    step_save_pos();

    ESP_LOGI(TAG, "Homing completo — theta=0 mm  phi=0 mm");
    return ESP_OK;
}
