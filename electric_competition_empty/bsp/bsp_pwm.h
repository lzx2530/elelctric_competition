#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "common/types.h"

typedef enum {
    BSP_MOTOR_PWM_LEFT = 0,
    BSP_MOTOR_PWM_RIGHT,
} bsp_motor_pwm_t;

typedef enum {
    BSP_STEPPER_AXIS_YAW = 0,
    BSP_STEPPER_AXIS_PITCH,
} bsp_stepper_axis_t;

void bsp_pwm_init(void);
void bsp_pwm_start_all(void);
void bsp_pwm_set_motor_bridge(bsp_motor_pwm_t motor, float in1_duty, float in2_duty);
void bsp_pwm_set_motor_duty(bsp_motor_pwm_t motor, float duty);
void bsp_pwm_set_step_frequency(bsp_stepper_axis_t axis, float frequency_hz);
void bsp_pwm_stop_step(bsp_stepper_axis_t axis);
float bsp_pwm_get_step_frequency(bsp_stepper_axis_t axis);

#endif
