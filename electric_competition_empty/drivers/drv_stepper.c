#include "drivers/drv_stepper.h"

#include "common/math_util.h"

void stepper_init(stepper_handle_t *handle, const stepper_config_t *cfg)
{
    handle->cfg = *cfg;
    handle->target_frequency_hz = 0.0f;
    handle->current_frequency_hz = 0.0f;
    handle->enabled = false;
    handle->reversing = false;
    stepper_stop(handle);
}

void stepper_enable(stepper_handle_t *handle, bool enable)
{
    handle->enabled = enable;
    if (!enable) {
        stepper_stop(handle);
    }
}

void stepper_set_speed(stepper_handle_t *handle, float frequency_hz)
{
    handle->target_frequency_hz = math_clampf(frequency_hz,
        -handle->cfg.max_frequency_hz,
        handle->cfg.max_frequency_hz);
}

void stepper_stop(stepper_handle_t *handle)
{
    handle->target_frequency_hz = 0.0f;
    handle->current_frequency_hz = 0.0f;
    handle->reversing = false;
    bsp_pwm_stop_step(handle->cfg.axis);
}

void stepper_update(stepper_handle_t *handle, float dt_s)
{
    float target;
    float accel_hz_per_s;
    float step_delta;
    bool positive;

    if (!handle->enabled) {
        stepper_stop(handle);
        return;
    }

    target = handle->target_frequency_hz;
    if ((target * handle->current_frequency_hz) < 0.0f) {
        handle->reversing = true;
    }
    accel_hz_per_s = handle->cfg.accel_hz_per_s;
    if (handle->reversing && (handle->cfg.reverse_accel_hz_per_s > 0.0f)) {
        accel_hz_per_s = handle->cfg.reverse_accel_hz_per_s;
    }
    if (accel_hz_per_s > 0.0f) {
        /* Use a linear ramp so direction and speed changes do not jump the pulse generator. */
        step_delta = accel_hz_per_s * dt_s;
        handle->current_frequency_hz += math_clampf(target - handle->current_frequency_hz, -step_delta, step_delta);
    } else {
        handle->current_frequency_hz = target;
    }
    if ((target == 0.0f) || (handle->current_frequency_hz == target)) {
        handle->reversing = false;
    }

    if (math_absf(handle->current_frequency_hz) < handle->cfg.min_frequency_hz) {
        /* Stop output entirely below the effective motion band to avoid meaningless pulses. */
        bsp_pwm_stop_step(handle->cfg.axis);
        return;
    }

    positive = (handle->current_frequency_hz >= 0.0f);
    if (handle->cfg.invert_direction) {
        positive = !positive;
    }

    bsp_gpio_set_dir_output(handle->cfg.dir_output, positive);
    bsp_pwm_set_step_frequency(handle->cfg.axis, math_absf(handle->current_frequency_hz));
}
