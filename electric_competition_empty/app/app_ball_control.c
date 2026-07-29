#include "app/app_ball_control.h"

#include "algo/algo_filter.h"
#include "algo/algo_pid.h"
#include "common/math_util.h"
#include "drivers/drv_abs_position.h"
#include "drivers/drv_stepper.h"

#define APP_BALL_VISION_TIMEOUT_MS       (120U)
#define APP_BALL_MAX_TILT_RAD            (0.140F)
#define APP_BALL_NEUTRAL_ACTUATOR        (0.500F)
#define APP_BALL_ACTUATOR_PER_RAD        (1.250F)
#define APP_BALL_COMMAND_SIGN            (1.0F)

typedef struct {
    stepper_handle_t stepper;
    abs_position_handle_t actuator_encoder;
    pid_handle_t ball_pid;
    pid_handle_t actuator_pid;
    lpf1_handle_t ball_velocity_filter;
    ball_control_snapshot_t snapshot;
    uint32_t last_vision_tick_ms;
    int16_t previous_ball_position_mm;
    bool ball_position_initialized;
} ball_control_app_t;

static ball_control_app_t g_ball;

static const stepper_config_t g_pitch_stepper_cfg = {
    .axis = BSP_STEPPER_AXIS_PITCH,
    .dir_output = BSP_DIR_PITCH,
    .invert_direction = false,
    .min_frequency_hz = 5.0F,
    .max_frequency_hz = 4000.0F,
    .accel_hz_per_s = 8000.0F,
};

static const pid_config_t g_ball_pid_cfg = {
    .kp = 0.0018F,
    .ki = 0.0003F,
    .kd = 0.0009F,
    .dt_s = 0.01F,
    .output_limit = APP_BALL_MAX_TILT_RAD,
    .integral_limit = 0.050F,
    .integral_separation = 25.0F,
    .derivative_lpf_alpha = 0.20F,
    .setpoint_slew_rate = 0.0F,
    .deadband = 1.0F,
    .derivative_on_measurement = true,
    .enable_integral_separation = true,
    .enable_output_limit = true,
    .enable_integral_limit = true,
    .enable_deadband = true,
    .enable_setpoint_ramp = false,
};

static const pid_config_t g_actuator_pid_cfg = {
    .kp = 7000.0F,
    .ki = 300.0F,
    .kd = 40.0F,
    .dt_s = 0.001F,
    .output_limit = 3500.0F,
    .integral_limit = 0.15F,
    .integral_separation = 0.10F,
    .derivative_lpf_alpha = 0.10F,
    .setpoint_slew_rate = 0.0F,
    .deadband = 0.001F,
    .derivative_on_measurement = true,
    .enable_integral_separation = true,
    .enable_output_limit = true,
    .enable_integral_limit = true,
    .enable_deadband = true,
    .enable_setpoint_ramp = false,
};

void app_ball_control_init(void)
{
    stepper_init(&g_ball.stepper, &g_pitch_stepper_cfg);
    stepper_enable(&g_ball.stepper, true);
    abs_position_init(&g_ball.actuator_encoder, 0.02F, 0.98F);
    pid_init(&g_ball.ball_pid, &g_ball_pid_cfg, PID_MODE_POSITION);
    pid_init(&g_ball.actuator_pid, &g_actuator_pid_cfg, PID_MODE_POSITION);
    lpf1_init(&g_ball.ball_velocity_filter, 0.25F, 0.0F);
    g_ball.snapshot.target_mm = 0;
    g_ball.snapshot.actuator_target = APP_BALL_NEUTRAL_ACTUATOR;
}

void app_ball_control_set_enabled(bool enabled)
{
    g_ball.snapshot.enabled = enabled;
    if (!enabled) {
        pid_reset(&g_ball.ball_pid);
        pid_reset(&g_ball.actuator_pid);
        g_ball.snapshot.actuator_target = APP_BALL_NEUTRAL_ACTUATOR;
    }
}

void app_ball_control_set_target_mm(int16_t target_mm)
{
    g_ball.snapshot.target_mm = target_mm;
}

void app_ball_control_set_vision(const k230_frame_t *frame, uint32_t tick_ms)
{
    if (frame == NULL) {
        return;
    }

    g_ball.snapshot.vision_valid = frame->ball.valid;
    g_ball.snapshot.vision_stable = frame->ball.stable;
    g_ball.snapshot.reference_ready = frame->ball.reference_ready;
    if (frame->ball.stop_requested) {
        g_ball.snapshot.stop_requested = true;
    }
    if (!frame->ball.valid) {
        return;
    }

    if (g_ball.ball_position_initialized) {
        float velocity = ((float) frame->ball.position_mm -
            (float) g_ball.previous_ball_position_mm) / 0.033F;
        g_ball.snapshot.ball_velocity_mmps = lpf1_update(&g_ball.ball_velocity_filter, velocity);
    } else {
        g_ball.ball_position_initialized = true;
        lpf1_init(&g_ball.ball_velocity_filter, 0.25F, 0.0F);
    }

    g_ball.previous_ball_position_mm = frame->ball.position_mm;
    g_ball.snapshot.ball_position_mm = frame->ball.position_mm;
    g_ball.last_vision_tick_ms = tick_ms;
}

void app_ball_control_reset_vision(void)
{
    g_ball.snapshot.vision_valid = false;
    g_ball.snapshot.vision_stable = false;
    g_ball.snapshot.reference_ready = false;
    g_ball.snapshot.stop_requested = false;
    g_ball.snapshot.ball_position_mm = 0;
    g_ball.snapshot.ball_velocity_mmps = 0.0F;
    g_ball.last_vision_tick_ms = 0U;
    g_ball.ball_position_initialized = false;
    lpf1_init(&g_ball.ball_velocity_filter, 0.25F, 0.0F);
}

void app_ball_control_set_actuator_pwm(uint32_t high_ticks, uint32_t period_ticks)
{
    abs_position_update_pwm(&g_ball.actuator_encoder, high_ticks, period_ticks);
    g_ball.snapshot.actuator_feedback_valid = g_ball.actuator_encoder.valid;
    g_ball.snapshot.actuator_feedback = g_ball.actuator_encoder.position;
}

void app_ball_control_outer_task(const imu_snapshot_t *imu, uint32_t tick_ms)
{
    float feedback;
    float feedforward;
    float tilt_command;

    if (!g_ball.snapshot.enabled) {
        return;
    }
    if (!g_ball.snapshot.actuator_feedback_valid) {
        g_ball.snapshot.fault = APP_BALL_FAULT_ACTUATOR_FEEDBACK;
        return;
    }
    if ((!g_ball.snapshot.vision_valid) ||
        ((tick_ms - g_ball.last_vision_tick_ms) > APP_BALL_VISION_TIMEOUT_MS)) {
        g_ball.snapshot.fault = APP_BALL_FAULT_VISION_TIMEOUT;
        return;
    }

    feedback = (float) g_ball.snapshot.ball_position_mm;
    tilt_command = APP_BALL_COMMAND_SIGN * pid_update(&g_ball.ball_pid,
        (float) g_ball.snapshot.target_mm, feedback);
    feedforward = imu != NULL && imu->online ?
        math_clampf(-imu->longitudinal_accel_mps2 / 9.80665F,
            -APP_BALL_MAX_TILT_RAD, APP_BALL_MAX_TILT_RAD) : 0.0F;
    tilt_command = math_clampf(tilt_command + feedforward,
        -APP_BALL_MAX_TILT_RAD, APP_BALL_MAX_TILT_RAD);
    g_ball.snapshot.tilt_command_rad = tilt_command;
    g_ball.snapshot.actuator_target = math_clampf(APP_BALL_NEUTRAL_ACTUATOR +
        APP_BALL_ACTUATOR_PER_RAD * tilt_command, 0.05F, 0.95F);
    g_ball.snapshot.fault = APP_BALL_FAULT_NONE;
}

void app_ball_control_inner_task(float dt_s)
{
    float command_hz = 0.0F;

    if (g_ball.snapshot.enabled && g_ball.snapshot.actuator_feedback_valid &&
        g_ball.snapshot.fault == APP_BALL_FAULT_NONE) {
        command_hz = pid_update(&g_ball.actuator_pid,
            g_ball.snapshot.actuator_target, g_ball.snapshot.actuator_feedback);
    } else {
        pid_reset(&g_ball.actuator_pid);
    }

    stepper_set_speed(&g_ball.stepper, command_hz);
    stepper_update(&g_ball.stepper, dt_s);
    g_ball.snapshot.stepper_command_hz = command_hz;
}

const ball_control_snapshot_t *app_ball_control_get_snapshot(void)
{
    return &g_ball.snapshot;
}
