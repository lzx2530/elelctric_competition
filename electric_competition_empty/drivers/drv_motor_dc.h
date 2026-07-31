#ifndef DRV_MOTOR_DC_H
#define DRV_MOTOR_DC_H

#include "bsp/bsp_pwm.h"

typedef struct {
    bsp_motor_pwm_t pwm_channel;
    bool invert_direction;
    float deadband;
    float max_duty;
} motor_dc_config_t;

typedef struct {
    motor_dc_config_t cfg;
    float command;
} motor_dc_handle_t;

void motor_dc_init(motor_dc_handle_t *handle, const motor_dc_config_t *cfg);
/* 输入范围建议为 -1.0 ~ 1.0，内部会做死区和限幅 */
void motor_dc_set_output(motor_dc_handle_t *handle, float normalized_output);
void motor_dc_brake(motor_dc_handle_t *handle);
float motor_dc_get_output(const motor_dc_handle_t *handle);

#endif
