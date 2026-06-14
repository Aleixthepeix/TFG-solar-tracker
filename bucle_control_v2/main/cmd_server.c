#include "cmd_server.h"
#include "config.h"
#include "stepper.h"
#include "control.h"
#include "kinematics.h"
#include "homing.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "cmd_server";

/* ================================================================== */
/* Tarea de calibración — homing + centrado en 150 mm                 */
/* ================================================================== */

static volatile bool s_homing = false;

static void homing_task(void *arg)
{
    ctrl_pause();
    step_stop(STEP_THETA);
    step_stop(STEP_PHI);

    /* Mueve ambos ejes al fin de carrera mínimo y establece pos = 0 mm */
    home_all_axes();

    /* Desplaza al centro de carrera (150 mm) y espera que lleguen */
    step_move_to_mm(STEP_THETA, CFG_ACTUATOR_HOME_MM, CFG_HOME_SPEED_HZ);
    step_move_to_mm(STEP_PHI,   CFG_ACTUATOR_HOME_MM, CFG_HOME_SPEED_HZ);
    while (step_is_moving(STEP_THETA) || step_is_moving(STEP_PHI))
        vTaskDelay(pdMS_TO_TICKS(50));

    step_save_pos();
    ESP_LOGI(TAG, "Calibración completa — ambos ejes en %.0f mm, control pausado",
             (double)CFG_ACTUATOR_HOME_MM);
    s_homing = false;
    vTaskDelete(NULL);
}

/* ================================================================== */
/* Motor de comandos — parsea texto y llama a las funciones C         */
/* ================================================================== */

static void execute_cmd(const char *input, char *out, size_t out_len)
{
    char buf[128];
    strncpy(buf, input, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *word = strtok(buf, " \t\r\n");
    if (!word) { snprintf(out, out_len, "error: comando vacío"); return; }

    char *a1 = strtok(NULL, " \t\r\n");
    char *a2 = strtok(NULL, " \t\r\n");
    char *a3 = strtok(NULL, " \t\r\n");

    if (strcmp(word, "pos") == 0) {
        snprintf(out, out_len,
            "{\"theta\":%.2f,\"phi\":%.2f}",
            (double)step_get_pos_mm(STEP_THETA),
            (double)step_get_pos_mm(STEP_PHI));

    } else if (strcmp(word, "status") == 0) {
        snprintf(out, out_len,
            "{\"theta\":%.2f,\"phi\":%.2f,\"paused\":%s,\"homing\":%s}",
            (double)step_get_pos_mm(STEP_THETA),
            (double)step_get_pos_mm(STEP_PHI),
            ctrl_is_paused() ? "true" : "false",
            s_homing        ? "true" : "false");

    } else if (strcmp(word, "ctrl") == 0) {
        if (!a1) { snprintf(out, out_len, "error: ctrl pause|resume"); return; }
        if (strcmp(a1, "pause") == 0) {
            ctrl_pause();
            step_save_pos();   /* persistir posición al pausar */
            snprintf(out, out_len, "ok: bucle pausado (posicion guardada)");
        } else if (strcmp(a1, "resume") == 0) {
            ctrl_resume();
            snprintf(out, out_len, "ok: bucle reanudado");
        } else {
            snprintf(out, out_len, "error: acción desconocida '%s'", a1);
        }

    } else if (strcmp(word, "stop") == 0) {
        if (!a1) {
            step_stop(STEP_THETA);
            step_stop(STEP_PHI);
            snprintf(out, out_len, "ok: ambos ejes parados");
        } else if (strcmp(a1, "theta") == 0) {
            step_stop(STEP_THETA);
            snprintf(out, out_len, "ok: theta parado");
        } else if (strcmp(a1, "phi") == 0) {
            step_stop(STEP_PHI);
            snprintf(out, out_len, "ok: phi parado");
        } else {
            snprintf(out, out_len, "error: eje '%s' desconocido", a1);
        }

    } else if (strcmp(word, "scan") == 0) {
        ctrl_restart_scan();
        snprintf(out, out_len, "ok: re-barrido iniciado");

    } else if (strcmp(word, "setpos") == 0) {
        if (!a1 || !a2) {
            snprintf(out, out_len, "error: setpos theta|phi <mm>");
            return;
        }
        step_axis_t axis = (strcmp(a1, "phi") == 0) ? STEP_PHI : STEP_THETA;
        float mm = strtof(a2, NULL);
        if (mm < 0.0f || mm > CFG_ACTUATOR_STROKE) {
            snprintf(out, out_len, "error: fuera de rango [0, %.0f] mm",
                     (double)CFG_ACTUATOR_STROKE);
            return;
        }
        step_set_pos_mm(axis, mm);
        snprintf(out, out_len, "ok: %s fijado a %.2f mm", a1, (double)mm);

    } else if (strcmp(word, "step") == 0) {
        if (!a1 || !a2 || !a3) {
            snprintf(out, out_len, "error: step theta|phi <hz> <+|->");
            return;
        }
        step_axis_t axis = (strcmp(a1, "phi") == 0) ? STEP_PHI : STEP_THETA;
        uint32_t hz = (uint32_t)strtoul(a2, NULL, 10);
        if (hz == 0 || hz > CFG_STEP_FREQ_MAX_HZ) {
            snprintf(out, out_len, "error: hz fuera de rango [1, %u]",
                     CFG_STEP_FREQ_MAX_HZ);
            return;
        }
        bool dir = (a3[0] == '+');
        ctrl_pause();
        step_set_speed(axis, hz, dir);
        snprintf(out, out_len, "ok: %s a %lu Hz dir=%c (ctrl pausado)",
                 a1, (unsigned long)hz, a3[0]);

    } else if (strcmp(word, "kin") == 0) {
        if (!a1 || !a2) {
            snprintf(out, out_len, "error: kin theta|phi <grados>");
            return;
        }
        bool use_phi = (strcmp(a1, "phi") == 0);
        float deg = strtof(a2, NULL);
        float A;
        kin_status_t s = kin_inverse(deg, use_phi, &A);
        if (s == KIN_OK)
            snprintf(out, out_len, "%.3f mm", (double)A);
        else
            snprintf(out, out_len, "error: sin solución para %.2f°", (double)deg);

    } else if (strcmp(word, "goto") == 0) {
        /* goto theta|phi <mm>  →  mueve eje a posición absoluta en mm */
        if (!a1 || !a2) {
            snprintf(out, out_len, "error: goto theta|phi <mm>"); return;
        }
        step_axis_t axis = (strcmp(a1, "phi") == 0) ? STEP_PHI : STEP_THETA;
        float mm = strtof(a2, NULL);
        float margin = CFG_ACTUATOR_MARGIN_MM;
        float stroke = CFG_ACTUATOR_STROKE;
        if (mm < margin || mm > stroke - margin) {
            snprintf(out, out_len,
                "error: %.1f mm fuera de rango [%.0f, %.0f]",
                (double)mm, (double)margin, (double)(stroke - margin));
            return;
        }
        ctrl_pause();
        step_move_to_mm(axis, mm, CFG_CTRL_SPEED_HZ);
        snprintf(out, out_len, "ok: %s → %.1f mm", a1, (double)mm);

    } else if (strcmp(word, "angle") == 0) {
        /* angle theta|phi <grados>  →  convierte a mm y mueve */
        if (!a1 || !a2) {
            snprintf(out, out_len, "error: angle theta|phi <grados>"); return;
        }
        bool        use_phi2 = (strcmp(a1, "phi") == 0);
        step_axis_t axis2    = use_phi2 ? STEP_PHI : STEP_THETA;
        float deg = strtof(a2, NULL);
        float mm;
        if (kin_inverse(deg, use_phi2, &mm) != KIN_OK) {
            snprintf(out, out_len,
                "error: %s sin solucion para %.1f grados", a1, (double)deg);
            return;
        }
        float margin2 = CFG_ACTUATOR_MARGIN_MM;
        float stroke2 = CFG_ACTUATOR_STROKE;
        if (mm < margin2 || mm > stroke2 - margin2) {
            snprintf(out, out_len,
                "error: %.1f mm (para %.1f grados) fuera de rango [%.0f, %.0f]",
                (double)mm, (double)deg, (double)margin2, (double)(stroke2 - margin2));
            return;
        }
        ctrl_pause();
        step_move_to_mm(axis2, mm, CFG_CTRL_SPEED_HZ);
        snprintf(out, out_len, "ok: %s %.1f grados -> %.1f mm", a1, (double)deg, (double)mm);

    } else if (strcmp(word, "solar") == 0) {
        if (!a1 || !a2) {
            snprintf(out, out_len,
                "error: solar <elevacion_deg> <acimut_deg>");
            return;
        }
        float elev_deg = strtof(a1, NULL);
        float acim_deg = strtof(a2, NULL);

        float theta_mm, phi_mm;
        if (kin_inverse(elev_deg, false, &theta_mm) != KIN_OK) {
            snprintf(out, out_len,
                "error: theta sin solucion para %.1f°", (double)elev_deg);
            return;
        }
        if (kin_inverse(acim_deg, true, &phi_mm) != KIN_OK) {
            snprintf(out, out_len,
                "error: phi sin solucion para %.1f°", (double)acim_deg);
            return;
        }

        /* Verificar que los targets están dentro de márgenes de seguridad */
        float margin = CFG_ACTUATOR_MARGIN_MM;
        float stroke = CFG_ACTUATOR_STROKE;
        if (theta_mm < margin || theta_mm > stroke - margin ||
            phi_mm   < margin || phi_mm   > stroke - margin) {
            snprintf(out, out_len,
                "error: posicion fuera de limites "
                "theta=%.1fmm phi=%.1fmm (rango [%.0f, %.0f])",
                (double)theta_mm, (double)phi_mm,
                (double)margin, (double)(stroke - margin));
            return;
        }

        /* Advertir si algún eje está en 0 mm (contador probablemente sin calibrar) */
        float cur_theta = step_get_pos_mm(STEP_THETA);
        float cur_phi   = step_get_pos_mm(STEP_PHI);
        if (cur_theta == 0.0f || cur_phi == 0.0f) {
            snprintf(out, out_len,
                "error: posicion actual theta=%.1fmm phi=%.1fmm — "
                "calibra con 'setpos theta <mm>' antes de usar solar",
                (double)cur_theta, (double)cur_phi);
            return;
        }

        ctrl_pause();
        step_save_pos();   /* guardar posición antes de mover */
        step_move_to_mm(STEP_THETA, theta_mm, CFG_CTRL_SPEED_HZ);
        step_move_to_mm(STEP_PHI,   phi_mm,   CFG_CTRL_SPEED_HZ);

        ESP_LOGI(TAG, "solar: elev=%.1f°→%.1fmm  acim=%.1f°→%.1fmm",
                 (double)elev_deg, (double)theta_mm,
                 (double)acim_deg, (double)phi_mm);

        snprintf(out, out_len,
            "ok: solar elev=%.1f(%.1fmm) acim=%.1f(%.1fmm) — usa scan para retomar",
            (double)elev_deg, (double)theta_mm,
            (double)acim_deg, (double)phi_mm);

    } else if (strcmp(word, "home") == 0) {
        if (s_homing) {
            snprintf(out, out_len, "error: calibración ya en curso");
            return;
        }
        s_homing = true;
        if (xTaskCreate(homing_task, "home_task", 4096, NULL, 4, NULL) != pdPASS) {
            s_homing = false;
            snprintf(out, out_len, "error: sin heap para tarea de calibración");
            return;
        }
        snprintf(out, out_len,
            "ok: calibración iniciada (~3 min) — ambos ejes iran a 0 mm y luego a %.0f mm",
            (double)CFG_ACTUATOR_HOME_MM);

    } else if (strcmp(word, "help") == 0) {
        snprintf(out, out_len,
            "home\n"
            "goto  <theta|phi> <mm>\n"
            "angle <theta|phi> <grados>\n"
            "solar <elev_deg> <acim_deg>\n"
            "step  <theta|phi> <hz> <+|->\n"
            "stop  [theta|phi]\n"
            "ctrl  <pause|resume>\n"
            "scan\n"
            "pos\n"
            "status\n"
            "setpos <theta|phi> <mm>\n"
            "kin    <theta|phi> <grados>\n"
            "help");
    } else {
        snprintf(out, out_len, "error: comando '%s' desconocido", word);
    }
}

/* ================================================================== */
/* Handler: POST /cmd                                                  */
/* ================================================================== */

#define CMD_IN_SIZE  128
#define CMD_OUT_SIZE 512

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char in_buf[CMD_IN_SIZE];
    int len = httpd_req_recv(req, in_buf, sizeof(in_buf) - 1);
    if (len <= 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "sin cuerpo");
    in_buf[len] = '\0';

    char out_buf[CMD_OUT_SIZE];
    execute_cmd(in_buf, out_buf, sizeof(out_buf));

    ESP_LOGI(TAG, "cmd='%s' resp='%s'", in_buf, out_buf);

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, out_buf);
}

/* ================================================================== */
/* Handler: GET /status                                               */
/* ================================================================== */

static esp_err_t status_handler(httpd_req_t *req)
{
    char json[128];
    snprintf(json, sizeof(json),
        "{\"theta\":%.2f,\"phi\":%.2f,\"paused\":%s,\"homing\":%s}",
        (double)step_get_pos_mm(STEP_THETA),
        (double)step_get_pos_mm(STEP_PHI),
        ctrl_is_paused() ? "true" : "false",
        s_homing        ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

/* ================================================================== */
/* Arranque del servidor                                              */
/* ================================================================== */

esp_err_t cmd_server_start(void)
{
    httpd_config_t cfg      = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size          = 6144;
    cfg.lru_purge_enable    = true;
    cfg.recv_wait_timeout   = 5;
    cfg.send_wait_timeout   = 5;

    ESP_LOGI(TAG, "Heap libre antes de httpd_start: %lu B",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    httpd_handle_t srv = NULL;
    esp_err_t res = httpd_start(&srv, &cfg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start falló: 0x%x  (heap libre: %lu B)",
                 res, (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
        return res;
    }

    static const httpd_uri_t routes[] = {
        { .uri = "/cmd",    .method = HTTP_POST, .handler = cmd_handler    },
        { .uri = "/status", .method = HTTP_GET,  .handler = status_handler },
    };
    for (int i = 0; i < (int)(sizeof(routes) / sizeof(routes[0])); i++)
        httpd_register_uri_handler(srv, &routes[i]);

    ESP_LOGI(TAG, "Servidor de comandos listo en :80");
    ESP_LOGI(TAG, "  POST /cmd    → ejecuta comando, devuelve respuesta");
    ESP_LOGI(TAG, "  GET  /status → JSON {theta, phi, paused}");
    return ESP_OK;
}
