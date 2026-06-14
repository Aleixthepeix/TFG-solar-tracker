#include "stepper.h"
#include "config.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "FastAccelStepper.h"

static const char *TAG = "stepper";

static const int STEP_PIN    [STEP_AXIS_COUNT] = { CFG_THETA_STEP_GPIO,     CFG_PHI_STEP_GPIO     };
static const int DIR_PIN     [STEP_AXIS_COUNT] = { CFG_THETA_DIR_GPIO,      CFG_PHI_DIR_GPIO      };
static const int EN_PIN      [STEP_AXIS_COUNT] = { CFG_THETA_EN_GPIO,       CFG_PHI_EN_GPIO       };
static const int ENDSTOP_PIN [STEP_AXIS_COUNT] = { CFG_THETA_ENDSTOP_GPIO,  CFG_PHI_ENDSTOP_GPIO  };

static FastAccelStepperEngine s_engine;
static FastAccelStepper       *s_ax[STEP_AXIS_COUNT];

/* 1600 pulsos/vuelta ÷ 5 mm/vuelta = 320 pasos/mm */
static constexpr float STEPS_PER_MM =
    (float)CFG_STEPS_PER_REV / CFG_ACTUATOR_MM_REV;

/* ================================================================== */
/* API pública                                                        */
/* ================================================================== */

step_status_t step_init(void)
{
    s_engine.init();

    for (int i = 0; i < STEP_AXIS_COUNT; i++) {
        gpio_set_direction((gpio_num_t)EN_PIN[i], GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)EN_PIN[i], 0);

        s_ax[i] = s_engine.stepperConnectToPin(STEP_PIN[i]);
        if (!s_ax[i]) {
            ESP_LOGE(TAG, "stepperConnectToPin falló (eje %d, GPIO %d)", i, STEP_PIN[i]);
            return STEP_ERR_INVALID;
        }

        s_ax[i]->setDirectionPin(DIR_PIN[i]);
        s_ax[i]->setAcceleration(CFG_STEP_ACCEL_HZ_S);

        float init_mm = CFG_ACTUATOR_HOME_MM;
        nvs_handle_t nvs;
        if (nvs_open("stepper", NVS_READONLY, &nvs) == ESP_OK) {
            uint32_t raw = 0;
            const char *key = (i == STEP_THETA) ? "theta_mm" : "phi_mm";
            if (nvs_get_u32(nvs, key, &raw) == ESP_OK)
                memcpy(&init_mm, &raw, sizeof(float));
            nvs_close(nvs);
        }
        s_ax[i]->setCurrentPosition((int32_t)(init_mm * STEPS_PER_MM));

        ESP_LOGI(TAG, "Eje %d OK — STEP=GPIO%d  DIR=GPIO%d  EN=GPIO%d  pos_init=%.1f mm",
                 i, STEP_PIN[i], DIR_PIN[i], EN_PIN[i], (double)init_mm);
    }

    /* Configurar GPIOs de endstop (pull-up interno, activo en LOW) */
    for (int i = 0; i < STEP_AXIS_COUNT; i++) {
        if (ENDSTOP_PIN[i] < 0) continue;
        gpio_config_t io = {
            .pin_bit_mask  = (1ULL << ENDSTOP_PIN[i]),
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_ENABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        ESP_LOGI(TAG, "Endstop eje %d — GPIO%d", i, ENDSTOP_PIN[i]);
    }

    return STEP_OK;
}

void step_set_speed(step_axis_t axis, uint32_t freq_hz, bool dir_positive)
{
    if (axis >= STEP_AXIS_COUNT || !s_ax[axis]) return;
    if (freq_hz < CFG_STEP_FREQ_MIN_HZ) freq_hz = CFG_STEP_FREQ_MIN_HZ;
    if (freq_hz > CFG_STEP_FREQ_MAX_HZ) freq_hz = CFG_STEP_FREQ_MAX_HZ;
    s_ax[axis]->setSpeedInHz(freq_hz);
    if (dir_positive) s_ax[axis]->runForward();
    else              s_ax[axis]->runBackward();
}

void step_stop(step_axis_t axis)
{
    if (axis >= STEP_AXIS_COUNT || !s_ax[axis]) return;
    s_ax[axis]->stopMove();
}

void step_move_to_mm(step_axis_t axis, float target_mm, uint32_t speed_hz)
{
    if (axis >= STEP_AXIS_COUNT || !s_ax[axis]) return;
    s_ax[axis]->setSpeedInHz(speed_hz);
    s_ax[axis]->moveTo((int32_t)(target_mm * STEPS_PER_MM));
}

void step_move_mm(step_axis_t axis, float delta_mm, uint32_t speed_hz)
{
    if (axis >= STEP_AXIS_COUNT || !s_ax[axis]) return;
    s_ax[axis]->setSpeedInHz(speed_hz);
    s_ax[axis]->move((int32_t)(delta_mm * STEPS_PER_MM));
}

bool step_is_moving(step_axis_t axis)
{
    if (axis >= STEP_AXIS_COUNT || !s_ax[axis]) return false;
    return s_ax[axis]->isRunning();
}

float step_get_pos_mm(step_axis_t axis)
{
    if (axis >= STEP_AXIS_COUNT || !s_ax[axis]) return 0.0f;
    return (float)s_ax[axis]->getCurrentPosition() / STEPS_PER_MM;
}

void step_set_pos_mm(step_axis_t axis, float mm)
{
    if (axis >= STEP_AXIS_COUNT || !s_ax[axis]) return;
    s_ax[axis]->setCurrentPosition((int32_t)(mm * STEPS_PER_MM));
}

void step_save_pos(void)
{
    nvs_handle_t nvs;
    if (nvs_open("stepper", NVS_READWRITE, &nvs) != ESP_OK) return;
    float v;
    uint32_t raw;
    v = step_get_pos_mm(STEP_THETA); memcpy(&raw, &v, sizeof(float));
    nvs_set_u32(nvs, "theta_mm", raw);
    v = step_get_pos_mm(STEP_PHI);   memcpy(&raw, &v, sizeof(float));
    nvs_set_u32(nvs, "phi_mm", raw);
    nvs_commit(nvs);
    nvs_close(nvs);
}

bool step_endstop_triggered(step_axis_t axis)
{
    if (axis >= STEP_AXIS_COUNT) return false;
    int pin = ENDSTOP_PIN[axis];
    if (pin < 0) return false;
    return gpio_get_level((gpio_num_t)pin) == 0;   /* activo en LOW */
}

bool step_at_limit(step_axis_t axis, bool dir_positive)
{
    /* Endstop hardware tiene prioridad en el extremo mínimo */
    if (!dir_positive && step_endstop_triggered(axis)) return true;

    float pos = step_get_pos_mm(axis);
    if (dir_positive && pos >= CFG_ACTUATOR_STROKE - CFG_ACTUATOR_MARGIN_MM) return true;
    if (!dir_positive && pos <= CFG_ACTUATOR_MARGIN_MM)                       return true;
    return false;
}
