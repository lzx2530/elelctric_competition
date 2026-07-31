#ifndef DRV_STEPPER_H
#define DRV_STEPPER_H

#include "bsp/bsp_gpio.h"
#include "bsp/bsp_pwm.h"

typedef struct {
    bsp_stepper_axis_t axis;
    bsp_dir_output_t dir_output;
    bool invert_direction;
    float min_frequency_hz;
    float max_frequency_hz;
    float accel_hz_per_s;
    float reverse_accel_hz_per_s;
} stepper_config_t;

typedef struct {
    stepper_config_t cfg;
    float target_frequency_hz;
    float current_frequency_hz;
    bool enabled;
    bool reversing;
} stepper_handle_t;

void stepper_init(stepper_handle_t *handle, const stepper_config_t *cfg);
void stepper_enable(stepper_handle_t *handle, bool enable);
/* 传入带符号频率，符号决定方向，绝对值决定 STEP 频率 */
void stepper_set_speed(stepper_handle_t *handle, float frequency_hz);
void stepper_stop(stepper_handle_t *handle);
/* 需要在周期任务中调用，用于线性加减速 */
void stepper_update(stepper_handle_t *handle, float dt_s);

#endif
