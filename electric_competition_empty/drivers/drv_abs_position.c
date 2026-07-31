#include "drivers/drv_abs_position.h"

#include "common/math_util.h"

void abs_position_init(abs_position_handle_t *handle, float min_duty, float max_duty)
{
    handle->min_duty = min_duty;
    handle->max_duty = max_duty;
    handle->multi_turn_position = 0.0F;
    handle->position = 0.0f;
    handle->last_position = 0.0F;
    handle->position_initialized = false;
    handle->valid = false;
}

void abs_position_update_pwm(abs_position_handle_t *handle,
    uint32_t high_ticks,
    uint32_t period_ticks)
{
    float position;
    float duty;

    if ((period_ticks == 0U) || (handle->max_duty <= handle->min_duty)) {
        handle->valid = false;
        return;
    }

    duty = (float) high_ticks / (float) period_ticks;
    if ((duty < handle->min_duty) || (duty > handle->max_duty)) {
        handle->valid = false;
        return;
    }

    position = math_clampf((duty - handle->min_duty) /
        (handle->max_duty - handle->min_duty), 0.0f, 1.0f);
    if (handle->position_initialized) {
        float position_delta = position - handle->last_position;

        if (position_delta > 0.5F) {
            position_delta -= 1.0F;
        } else if (position_delta < -0.5F) {
            position_delta += 1.0F;
        }
        handle->multi_turn_position += position_delta;
    } else {
        handle->multi_turn_position = 0.0F;
        handle->position_initialized = true;
    }
    handle->position = position;
    handle->last_position = position;
    handle->valid = true;
}
