#include "console.h"
#include "config.h"
#include "stepper.h"
#include "control.h"
#include "kinematics.h"

#include "esp_console.h"
#include "esp_log.h"
#include "argtable3/argtable3.h"

#include <string.h>
#include <stdio.h>

static const char *TAG = "console";

/* ================================================================== */
/* Comando: step <theta|phi> <hz> <+|->                               */
/* ================================================================== */

static struct {
    struct arg_str *axis;
    struct arg_int *hz;
    struct arg_str *dir;
    struct arg_end *end;
} step_args;

static int cmd_step_fn(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&step_args);
    if (nerrors) {
        arg_print_errors(stdout, step_args.end, argv[0]);
        return 1;
    }

    const char *axis_str = step_args.axis->sval[0];
    step_axis_t axis;
    if (strcmp(axis_str, "theta") == 0)      axis = STEP_THETA;
    else if (strcmp(axis_str, "phi") == 0)   axis = STEP_PHI;
    else {
        printf("Error: eje debe ser 'theta' o 'phi'\n");
        return 1;
    }

    int hz = step_args.hz->ival[0];
    if (hz <= 0) {
        printf("Error: hz debe ser > 0\n");
        return 1;
    }

    const char *dir_str = step_args.dir->sval[0];
    bool dir;
    if (strcmp(dir_str, "+") == 0)      dir = true;
    else if (strcmp(dir_str, "-") == 0) dir = false;
    else {
        printf("Error: dirección debe ser '+' o '-'\n");
        return 1;
    }

    ctrl_pause();
    step_set_speed(axis, (uint32_t)hz, dir);
    printf("Moviendo %s a %d Hz dir=%s  (ctrl pausado — usa 'stop' y 'ctrl resume')\n",
           axis_str, hz, dir_str);
    return 0;
}

/* ================================================================== */
/* Comando: stop [theta|phi]                                          */
/* ================================================================== */

static struct {
    struct arg_str *axis;
    struct arg_end *end;
} stop_args;

static int cmd_stop_fn(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&stop_args);
    if (nerrors) {
        arg_print_errors(stdout, stop_args.end, argv[0]);
        return 1;
    }

    const char *a = (stop_args.axis->count > 0) ? stop_args.axis->sval[0] : "all";

    if (strcmp(a, "theta") == 0) {
        step_stop(STEP_THETA);
        printf("Theta parado\n");
    } else if (strcmp(a, "phi") == 0) {
        step_stop(STEP_PHI);
        printf("Phi parado\n");
    } else {
        step_stop(STEP_THETA);
        step_stop(STEP_PHI);
        printf("Ambos ejes parados\n");
    }
    return 0;
}

/* ================================================================== */
/* Comando: ctrl <pause|resume>                                       */
/* ================================================================== */

static struct {
    struct arg_str *action;
    struct arg_end *end;
} ctrl_args;

static int cmd_ctrl_fn(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&ctrl_args);
    if (nerrors) {
        arg_print_errors(stdout, ctrl_args.end, argv[0]);
        return 1;
    }

    const char *action = ctrl_args.action->sval[0];
    if (strcmp(action, "pause") == 0) {
        ctrl_pause();
        printf("Bucle de control pausado\n");
    } else if (strcmp(action, "resume") == 0) {
        ctrl_resume();
        printf("Bucle de control reanudado\n");
    } else {
        printf("Error: acción debe ser 'pause' o 'resume'\n");
        return 1;
    }
    return 0;
}

/* ================================================================== */
/* Comando: kin <theta|phi> <grados>                                  */
/* ================================================================== */

static struct {
    struct arg_str *axis;
    struct arg_dbl *angle;
    struct arg_end *end;
} kin_args;

static int cmd_kin_fn(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&kin_args);
    if (nerrors) {
        arg_print_errors(stdout, kin_args.end, argv[0]);
        return 1;
    }

    const char *axis_str = kin_args.axis->sval[0];
    bool use_phi;
    if (strcmp(axis_str, "phi") == 0)        use_phi = true;
    else if (strcmp(axis_str, "theta") == 0) use_phi = false;
    else {
        printf("Error: eje debe ser 'theta' o 'phi'\n");
        return 1;
    }

    float angle = (float)kin_args.angle->dval[0];
    float A_out;
    kin_status_t s = kin_inverse(angle, use_phi, &A_out);

    if (s == KIN_OK)
        printf("kin_inverse(%s, %.2f deg) → A = %.3f mm\n", axis_str, angle, A_out);
    else
        printf("Sin solución para %s = %.2f deg (fuera del rango mecánico)\n",
               axis_str, angle);
    return 0;
}

/* ================================================================== */
/* Comando: pos                                                        */
/* ================================================================== */

static int cmd_pos_fn(int argc, char **argv)
{
    printf("THETA: %.2f mm\n", (double)step_get_pos_mm(STEP_THETA));
    printf("PHI  : %.2f mm\n", (double)step_get_pos_mm(STEP_PHI));
    return 0;
}

/* ================================================================== */
/* Comando: setpos <theta|phi> <mm>                                    */
/* ================================================================== */

static struct {
    struct arg_str *axis;
    struct arg_dbl *mm;
    struct arg_end *end;
} setpos_args;

static int cmd_setpos_fn(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&setpos_args);
    if (nerrors) {
        arg_print_errors(stdout, setpos_args.end, argv[0]);
        return 1;
    }

    const char *axis_str = setpos_args.axis->sval[0];
    step_axis_t axis;
    if (strcmp(axis_str, "theta") == 0)      axis = STEP_THETA;
    else if (strcmp(axis_str, "phi") == 0)   axis = STEP_PHI;
    else {
        printf("Error: eje debe ser 'theta' o 'phi'\n");
        return 1;
    }

    float mm = (float)setpos_args.mm->dval[0];
    if (mm < 0.0f || mm > CFG_ACTUATOR_STROKE) {
        printf("Error: posición fuera de rango [0, %.0f] mm\n",
               (double)CFG_ACTUATOR_STROKE);
        return 1;
    }

    step_set_pos_mm(axis, mm);
    printf("Posición de %s fijada a %.2f mm\n", axis_str, (double)mm);
    return 0;
}

/* ================================================================== */
/* Comando: scan                                                       */
/* ================================================================== */

static int cmd_scan_fn(int argc, char **argv)
{
    ctrl_restart_scan();
    printf("Re-barrido iniciado\n");
    return 0;
}

/* ================================================================== */
/* Registro de comandos                                               */
/* ================================================================== */

static void register_commands(void)
{
    /* step */
    step_args.axis = arg_str1(NULL, NULL, "<theta|phi>", "Eje a mover");
    step_args.hz   = arg_int1(NULL, NULL, "<hz>",        "Frecuencia de pulsos [Hz]");
    step_args.dir  = arg_str1(NULL, NULL, "<+|->",       "Dirección positiva (+) o negativa (-)");
    step_args.end  = arg_end(3);
    const esp_console_cmd_t cmd_step = {
        .command  = "step",
        .help     = "Mueve un eje a velocidad constante (pausa ctrl automáticamente)",
        .argtable = &step_args,
        .func     = cmd_step_fn,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_step));

    /* stop */
    stop_args.axis = arg_str0(NULL, NULL, "[theta|phi]", "Eje a parar (omitir = ambos)");
    stop_args.end  = arg_end(1);
    const esp_console_cmd_t cmd_stop_cmd = {
        .command  = "stop",
        .help     = "Para uno o ambos motores",
        .argtable = &stop_args,
        .func     = cmd_stop_fn,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_stop_cmd));

    /* ctrl */
    ctrl_args.action = arg_str1(NULL, NULL, "<pause|resume>", "Pausa o reanuda el bucle");
    ctrl_args.end    = arg_end(1);
    const esp_console_cmd_t cmd_ctrl_cmd = {
        .command  = "ctrl",
        .help     = "Pausa o reanuda el bucle de control",
        .argtable = &ctrl_args,
        .func     = cmd_ctrl_fn,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_ctrl_cmd));

    /* kin */
    kin_args.axis  = arg_str1(NULL, NULL, "<theta|phi>", "Eje");
    kin_args.angle = arg_dbl1(NULL, NULL, "<grados>",    "Ángulo objetivo [°]");
    kin_args.end   = arg_end(2);
    const esp_console_cmd_t cmd_kin_cmd = {
        .command  = "kin",
        .help     = "Cinemática inversa: ángulo [°] → longitud de actuador A [mm]",
        .argtable = &kin_args,
        .func     = cmd_kin_fn,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_kin_cmd));

    /* pos */
    const esp_console_cmd_t cmd_pos = {
        .command = "pos",
        .help    = "Muestra la posición estimada de ambos actuadores en mm",
        .func    = cmd_pos_fn,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_pos));

    /* setpos */
    setpos_args.axis = arg_str1(NULL, NULL, "<theta|phi>", "Eje");
    setpos_args.mm   = arg_dbl1(NULL, NULL, "<mm>",        "Posición real conocida [mm]");
    setpos_args.end  = arg_end(2);
    const esp_console_cmd_t cmd_setpos = {
        .command  = "setpos",
        .help     = "Fija la posición estimada al valor real (usa con ctrl pause)",
        .argtable = &setpos_args,
        .func     = cmd_setpos_fn,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_setpos));

    /* scan */
    const esp_console_cmd_t cmd_scan = {
        .command = "scan",
        .help    = "Fuerza un re-barrido desde la posición actual",
        .func    = cmd_scan_fn,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd_scan));
}

/* ================================================================== */
/* API pública                                                         */
/* ================================================================== */

esp_err_t console_start(void)
{
    esp_console_repl_t *repl = NULL;

    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt           = "solar> ";
    repl_cfg.max_cmdline_length = 128;

    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    esp_err_t err = esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_console_new_repl_uart falló: 0x%x", err);
        return err;
    }

    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_commands();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    ESP_LOGI(TAG, "Consola lista — escribe 'help' para ver los comandos disponibles");
    return ESP_OK;
}
