#ifndef DRV_ABS_POSITION_H
#define DRV_ABS_POSITION_H

#include "common/types.h"

typedef struct {
    float min_duty;
    float max_duty;
    float position;
    bool valid;
} abs_position_handle_t;

void abs_position_init(abs_position_handle_t *handle, float min_duty, float max_duty);
void abs_position_update_pwm(abs_position_handle_t *handle,
    uint32_t high_ticks,
    uint32_t period_ticks);

#endif
