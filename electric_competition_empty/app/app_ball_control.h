#ifndef APP_BALL_CONTROL_H
#define APP_BALL_CONTROL_H

#include "app/app_imu.h"
#include "protocol/proto_k230.h"

typedef enum {
    APP_BALL_FAULT_NONE = 0U,
    APP_BALL_FAULT_VISION_TIMEOUT = 1U,
    APP_BALL_FAULT_ACTUATOR_FEEDBACK = 2U,
} app_ball_fault_t;

typedef struct {
    bool enabled;
    bool vision_valid;
    bool vision_stable;
    bool reference_ready;
    bool stop_requested;
    bool actuator_feedback_valid;
    int16_t target_mm;
    int16_t ball_position_mm;
    float ball_velocity_mmps;
    float tilt_command_rad;
    float actuator_target;
    float actuator_feedback;
    float stepper_command_hz;
    app_ball_fault_t fault;
} ball_control_snapshot_t;

void app_ball_control_init(void);
void app_ball_control_set_enabled(bool enabled);
void app_ball_control_set_target_mm(int16_t target_mm);
void app_ball_control_set_vision(const k230_frame_t *frame, uint32_t tick_ms);
void app_ball_control_reset_vision(void);
void app_ball_control_set_actuator_pwm(uint32_t high_ticks, uint32_t period_ticks);
void app_ball_control_outer_task(const imu_snapshot_t *imu, uint32_t tick_ms);
void app_ball_control_inner_task(float dt_s);
const ball_control_snapshot_t *app_ball_control_get_snapshot(void);

#endif
