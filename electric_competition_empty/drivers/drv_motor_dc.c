#include "drivers/drv_motor_dc.h"

#include "common/math_util.h"

void motor_dc_init(motor_dc_handle_t *handle, const motor_dc_config_t *cfg)
{
    handle->cfg = *cfg;
    handle->command = 0.0f;
    motor_dc_set_output(handle, 0.0f);
}

void motor_dc_set_output(motor_dc_handle_t *handle, float normalized_output)
{
    float duty = math_clampf(normalized_output, -handle->cfg.max_duty, handle->cfg.max_duty);

    /* Zero small commands before polarity handling to avoid idle-region chatter. */
    if (math_absf(duty) < handle->cfg.deadband) {
        duty = 0.0f;
    }

    if (handle->cfg.invert_direction) {
        duty = -duty;
    }

    if (duty > 0.0f) {
        /* AT8236 uses AIN1/AIN2 or BIN1/BIN2 as an H-bridge pair. PWM the forward leg only. */
        bsp_pwm_set_motor_bridge(handle->cfg.pwm_channel, duty, 0.0f);
    } else if (duty < 0.0f) {
        /* Reverse by PWM-ing the opposite leg while keeping the first leg low. */
        bsp_pwm_set_motor_bridge(handle->cfg.pwm_channel, 0.0f, -duty);
    } else {
        /* Coast at zero command instead of actively braking the bridge. */
        bsp_pwm_set_motor_bridge(handle->cfg.pwm_channel, 0.0f, 0.0f);
    }

    handle->command = duty;
}

void motor_dc_brake(motor_dc_handle_t *handle)
{
    bsp_pwm_set_motor_bridge(handle->cfg.pwm_channel, 1.0f, 1.0f);
    handle->command = 0.0f;
}

float motor_dc_get_output(const motor_dc_handle_t *handle)
{
    return handle->command;
}
