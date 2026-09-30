#include "state_machine.h"

#include "config_manager.h"
#include "global_state.h"
#include "esp_system.h"
#include "ir_ctrl.h"
#include "outputs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "state_machine"

static enum {
    SM_IDLE,
    SM_PRESENT,
} sm_state = SM_IDLE;

static TickType_t s_vacancy_deadline = 0;

/* Manual override timers for relay, IR fan, light. */
static TickType_t s_manual_override_until[3] = {0};

#define MANUAL_OVERRIDE_MS 30000

static void apply_manual(control_cmd_t *cmd) {
    global_state_t *st = global_state_get();
    switch (cmd->type) {
    case CMD_SET_RELAY:
        st->config.fan_relay_manual = (cmd->value != 0);
        s_manual_override_until[0] = xTaskGetTickCount() + pdMS_TO_TICKS(MANUAL_OVERRIDE_MS);
        outputs_set_relay(st->config.fan_relay_manual);
        break;
    case CMD_SET_IR_FAN:
        st->config.fan_ir_manual = (cmd->value != 0);
        s_manual_override_until[1] = xTaskGetTickCount() + pdMS_TO_TICKS(MANUAL_OVERRIDE_MS);
        outputs_set_ir_fan(st->config.fan_ir_manual);
        break;
    case CMD_SET_LIGHT:
        st->config.light_manual = (cmd->value != 0);
        s_manual_override_until[2] = xTaskGetTickCount() + pdMS_TO_TICKS(MANUAL_OVERRIDE_MS);
        outputs_set_light(st->config.light_manual);
        break;
    case CMD_SET_MODE:
        /* value is packed bits for auto/fan/light; update config */
        st->config.auto_mode = (cmd->value & 0x01) != 0;
        st->config.fan_auto = (cmd->value & 0x02) != 0;
        st->config.light_auto = (cmd->value & 0x04) != 0;
        break;
    case CMD_SET_THRESHOLD:
        /* value encodes type/value; not implemented in first pass */
        break;
    case CMD_LEARN_IR_ON:
        ir_ctrl_learn(true);
        break;
    case CMD_LEARN_IR_OFF:
        ir_ctrl_learn(false);
        break;
    case CMD_FACTORY_RESET:
        app_config_load_defaults(&st->config);
        app_config_save(&st->config);
        esp_restart();
        break;
    default:
        break;
    }
}

static bool is_manual_override(int idx) {
    return s_manual_override_until[idx] != 0 &&
           xTaskGetTickCount() < s_manual_override_until[idx];
}

static void update_automatic(void) {
    global_state_t *st = global_state_get();
    TickType_t now = xTaskGetTickCount();

    bool presence = (st->presence != PRESENCE_NONE);
    bool is_dark = st->sensors.lux < st->config.light_thr_lux;
    bool is_hot_humid = (st->sensors.temperature > st->config.temp_thr_c) &&
                        (st->sensors.humidity > st->config.hum_thr_pct);

    if (presence) {
        sm_state = SM_PRESENT;
        s_vacancy_deadline = 0;

        if (st->config.light_auto && !is_manual_override(2)) {
            outputs_set_light(is_dark);
        }

        if (st->config.fan_auto && !is_manual_override(0)) {
            outputs_set_relay(is_hot_humid);
        }
        if (st->config.fan_auto && !is_manual_override(1)) {
            outputs_set_ir_fan(is_hot_humid);
        }
    } else {
        if (sm_state == SM_PRESENT) {
            sm_state = SM_IDLE;
            s_vacancy_deadline = now + pdMS_TO_TICKS(st->config.vacancy_timeout_ms);
        }

        if (s_vacancy_deadline != 0 && now >= s_vacancy_deadline) {
            s_vacancy_deadline = 0;
            if (st->config.fan_auto && !is_manual_override(0)) {
                outputs_set_relay(false);
            }
            if (st->config.fan_auto && !is_manual_override(1)) {
                outputs_set_ir_fan(false);
            }
            if (st->config.light_auto && !is_manual_override(2)) {
                outputs_set_light(false);
            }
        }
    }
}

static void state_machine_task(void *arg) {
    (void)arg;
    control_cmd_t cmd;

    for (;;) {
        if (xQueueReceive(cmd_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            apply_manual(&cmd);
        }

        if (global_state_get()->config.auto_mode) {
            update_automatic();
        }
    }
}

esp_err_t state_machine_init(void) {
    ESP_LOGI(TAG, "state_machine init");
    return ESP_OK;
}

void state_machine_start_task(void) {
    xTaskCreatePinnedToCore(state_machine_task, "state_machine", 4096, NULL, 6, NULL, 1);
}
