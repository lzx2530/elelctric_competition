#include "drivers/drv_abs_position.h"

#include "common/math_util.h"

void abs_position_init(abs_position_handle_t *handle, float min_duty, float max_duty)
{
    handle->min_duty = min_duty;
    handle->max_duty = max_duty;
    handle->position = 0.0f;
    handle->valid = false;
}

void abs_position_update_pwm(abs_position_handle_t *handle,
    uint32_t high_ticks,
    uint32_t period_ticks)
{
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

    handle->position = math_clampf((duty - handle->min_duty) /
        (handle->max_duty - handle->min_duty), 0.0f, 1.0f);
    handle->valid = true;
}
