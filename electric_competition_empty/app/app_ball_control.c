#include "app/app_ball_control.h"

#include "algo/algo_pid.h"
#include "common/math_util.h"
#include "drivers/drv_abs_position.h"
#include "drivers/drv_stepper.h"

#define APP_BALL_VISION_TIMEOUT_MS       (120U)
#define APP_BALL_CONTROL_PERIOD_S         (0.03333333F)
#define APP_BALL_CAMERA_DELAY_FRAMES      (2U)
#define APP_BALL_OBSERVER_HISTORY_LENGTH  (APP_BALL_CAMERA_DELAY_FRAMES)
#define APP_BALL_MAX_TILT_RAD            (0.069813F)
#define APP_BALL_ACTUATOR_TURNS_PER_RAD  (152.000F)
#define APP_BALL_MAX_ACTUATOR_OFFSET_TURNS (16.000F)
#define APP_BALL_COMMAND_SIGN            (1.0F)
#define APP_BALL_G_MPS2                  (9.80665F)
#define APP_BALL_BETA                    (1.400F)
#define APP_BALL_KG_MPS2_PER_RAD         (APP_BALL_G_MPS2 / APP_BALL_BETA)
#define APP_BALL_KP                       (24.150F)
#define APP_BALL_KD                       (7.950F)
#define APP_BALL_KI                       (24.500F)
#define APP_BALL_INTEGRAL_LIMIT_MS        (0.500F)
#define APP_BALL_KF_K_POSITION            (0.541505F)
#define APP_BALL_KF_K_VELOCITY            (6.254990F)
#define APP_BALL_PREVIEW_TIME_S            (1.000F)
#define APP_BALL_FRICTION_BREAK_RAD       (0.011999F)
#define APP_BALL_BREAK_ENGAGE_M           (0.0010F)
#define APP_BALL_BREAK_RELEASE_M          (0.0003F)
#define APP_BALL_BREAK_GAIN               (1.15F)

typedef struct {
    stepper_handle_t stepper;
    abs_position_handle_t actuator_encoder;
    pid_handle_t actuator_pid;
    ball_control_snapshot_t snapshot;
    uint32_t last_vision_tick_ms;
    float delayed_position_mm;
    float delayed_velocity_mmps;
    float input_tilt_rad[APP_BALL_OBSERVER_HISTORY_LENGTH];
    float input_ax_mps2[APP_BALL_OBSERVER_HISTORY_LENGTH];
    float error_integral_ms;
    float actuator_reference_turns;
    int8_t break_direction;
    bool observer_initialized;
    bool vision_sample_pending;
    bool actuator_reference_valid;
} ball_control_app_t;

static ball_control_app_t g_ball;

static const stepper_config_t g_pitch_stepper_cfg = {
    .axis = BSP_STEPPER_AXIS_PITCH,
    .dir_output = BSP_DIR_PITCH,
    .invert_direction = false,
    .min_frequency_hz = 5.0F,
    .max_frequency_hz = 8000.0F,
    .accel_hz_per_s = 16000.0F,
};

static const pid_config_t g_actuator_pid_cfg = {
    .kp = 7000.0F,
    .ki = 300.0F,
    .kd = 40.0F,
    .dt_s = 0.001F,
    .output_limit = 8000.0F,
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

static void ball_observer_reset(void)
{
    uint8_t index;

    for (index = 0U; index < APP_BALL_OBSERVER_HISTORY_LENGTH; index++) {
        g_ball.input_tilt_rad[index] = 0.0F;
        g_ball.input_ax_mps2[index] = 0.0F;
    }
    g_ball.delayed_position_mm = 0.0F;
    g_ball.delayed_velocity_mmps = 0.0F;
    g_ball.error_integral_ms = 0.0F;
    g_ball.break_direction = 0;
    g_ball.observer_initialized = false;
    g_ball.vision_sample_pending = false;
    g_ball.snapshot.observer_ready = false;
    g_ball.snapshot.estimated_error_mm = 0.0F;
    g_ball.snapshot.ball_velocity_mmps = 0.0F;
}

static void ball_observer_predict(float *position_mm, float *velocity_mmps,
    float tilt_rad, float ax_mps2)
{
    float acceleration_mps2 = -APP_BALL_KG_MPS2_PER_RAD * tilt_rad -
        ax_mps2 / APP_BALL_BETA;

    *position_mm += *velocity_mmps * APP_BALL_CONTROL_PERIOD_S +
        500.0F * acceleration_mps2 * APP_BALL_CONTROL_PERIOD_S * APP_BALL_CONTROL_PERIOD_S;
    *velocity_mmps += 1000.0F * acceleration_mps2 * APP_BALL_CONTROL_PERIOD_S;
}

static void ball_observer_correct_and_predict(int16_t measured_error_mm,
    float *present_position_mm, float *present_velocity_mmps)
{
    uint8_t index;

    if (!g_ball.observer_initialized) {
        g_ball.delayed_position_mm = (float) measured_error_mm;
        g_ball.delayed_velocity_mmps = 0.0F;
        g_ball.observer_initialized = true;
    } else {
        float innovation = (float) measured_error_mm - g_ball.delayed_position_mm;

        g_ball.delayed_position_mm += APP_BALL_KF_K_POSITION * innovation;
        g_ball.delayed_velocity_mmps +=
            APP_BALL_KF_K_VELOCITY * innovation;
    }

    *present_position_mm = g_ball.delayed_position_mm;
    *present_velocity_mmps = g_ball.delayed_velocity_mmps;
    for (index = 0U; index < APP_BALL_OBSERVER_HISTORY_LENGTH; index++) {
        ball_observer_predict(present_position_mm, present_velocity_mmps,
            g_ball.input_tilt_rad[index], g_ball.input_ax_mps2[index]);
    }
}

static void ball_observer_advance(float tilt_rad, float ax_mps2)
{
    uint8_t index;

    /* Keep the delayed state aligned with the next delayed camera frame. */
    ball_observer_predict(&g_ball.delayed_position_mm, &g_ball.delayed_velocity_mmps,
        g_ball.input_tilt_rad[0], g_ball.input_ax_mps2[0]);
    for (index = 0U; index < (APP_BALL_OBSERVER_HISTORY_LENGTH - 1U); index++) {
        g_ball.input_tilt_rad[index] = g_ball.input_tilt_rad[index + 1U];
        g_ball.input_ax_mps2[index] = g_ball.input_ax_mps2[index + 1U];
    }
    g_ball.input_tilt_rad[APP_BALL_OBSERVER_HISTORY_LENGTH - 1U] = tilt_rad;
    g_ball.input_ax_mps2[APP_BALL_OBSERVER_HISTORY_LENGTH - 1U] = ax_mps2;
}

void app_ball_control_init(void)
{
    stepper_init(&g_ball.stepper, &g_pitch_stepper_cfg);
    stepper_enable(&g_ball.stepper, true);
    abs_position_init(&g_ball.actuator_encoder, 0.01F, 0.99F);
    pid_init(&g_ball.actuator_pid, &g_actuator_pid_cfg, PID_MODE_POSITION);
    ball_observer_reset();
    g_ball.snapshot.target_mm = 0;
    g_ball.snapshot.actuator_target = 0.0F;
}

void app_ball_control_set_enabled(bool enabled)
{
    bool was_enabled = g_ball.snapshot.enabled;

    g_ball.snapshot.enabled = enabled;
    if (!enabled || !was_enabled) {
        pid_reset(&g_ball.actuator_pid);
        ball_observer_reset();
        g_ball.snapshot.tilt_command_rad = 0.0F;
        g_ball.snapshot.tilt_feedback_rad = 0.0F;
        g_ball.snapshot.tilt_feedforward_rad = 0.0F;
        g_ball.snapshot.actuator_target = 0.0F;
        if (enabled && g_ball.actuator_encoder.valid) {
            g_ball.actuator_reference_turns = g_ball.actuator_encoder.multi_turn_position;
            g_ball.actuator_reference_valid = true;
            g_ball.snapshot.actuator_feedback = 0.0F;
        } else {
            g_ball.actuator_reference_valid = false;
        }
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

    /* K230 sends ball_position - current_target, so zero is always the control setpoint. */
    g_ball.snapshot.ball_position_mm = frame->ball.position_mm;
    g_ball.last_vision_tick_ms = tick_ms;
    g_ball.vision_sample_pending = true;
}

void app_ball_control_reset_vision(void)
{
    g_ball.snapshot.vision_valid = false;
    g_ball.snapshot.vision_stable = false;
    g_ball.snapshot.reference_ready = false;
    g_ball.snapshot.stop_requested = false;
    g_ball.snapshot.ball_position_mm = 0;
    g_ball.last_vision_tick_ms = 0U;
    ball_observer_reset();
}

void app_ball_control_set_actuator_pwm(uint32_t high_ticks, uint32_t period_ticks)
{
    abs_position_update_pwm(&g_ball.actuator_encoder, high_ticks, period_ticks);
    g_ball.snapshot.actuator_feedback_valid = g_ball.actuator_encoder.valid;
    if (!g_ball.actuator_encoder.valid) {
        return;
    }

    g_ball.snapshot.actuator_phase = g_ball.actuator_encoder.position;
    if (g_ball.snapshot.enabled && !g_ball.actuator_reference_valid) {
        g_ball.actuator_reference_turns = g_ball.actuator_encoder.multi_turn_position;
        g_ball.actuator_reference_valid = true;
    }
    if (g_ball.actuator_reference_valid) {
        g_ball.snapshot.actuator_feedback = g_ball.actuator_encoder.multi_turn_position -
            g_ball.actuator_reference_turns;
    }
}

void app_ball_control_outer_task(const imu_snapshot_t *imu, uint32_t tick_ms)
{
    float ax_mps2;
    float error_m;
    float velocity_error_mps;
    float desired_accel_mps2;
    float unsaturated_tilt_rad;
    float tilt_command_rad;
    float actual_tilt_rad;
    float estimated_error_mm;
    float estimated_velocity_mmps;
    float preview_error_mm;

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
    if (!g_ball.vision_sample_pending) {
        return;
    }

    ax_mps2 = imu != NULL && imu->online ? imu->longitudinal_accel_mps2 : 0.0F;
    ball_observer_correct_and_predict(g_ball.snapshot.ball_position_mm,
        &estimated_error_mm, &estimated_velocity_mmps);
    g_ball.vision_sample_pending = false;

    g_ball.snapshot.observer_ready = g_ball.observer_initialized;
    g_ball.snapshot.estimated_error_mm = estimated_error_mm;
    g_ball.snapshot.ball_velocity_mmps = estimated_velocity_mmps;

    preview_error_mm = estimated_error_mm +
        APP_BALL_PREVIEW_TIME_S * estimated_velocity_mmps;
    error_m = -0.001F * preview_error_mm;
    velocity_error_mps = -0.001F * estimated_velocity_mmps;
    desired_accel_mps2 = APP_BALL_KP * error_m + APP_BALL_KD * velocity_error_mps +
        APP_BALL_KI * g_ball.error_integral_ms;
    g_ball.snapshot.tilt_feedback_rad = -desired_accel_mps2 * APP_BALL_BETA / APP_BALL_G_MPS2;
    g_ball.snapshot.tilt_feedforward_rad = math_clampf(-ax_mps2 / APP_BALL_G_MPS2,
        -APP_BALL_MAX_TILT_RAD, APP_BALL_MAX_TILT_RAD);

    if (g_ball.break_direction == 0) {
        if (math_absf(error_m) > APP_BALL_BREAK_ENGAGE_M) {
            g_ball.break_direction = error_m > 0.0F ? 1 : -1;
        }
    } else if (((error_m > 0.0F ? 1 : (error_m < 0.0F ? -1 : 0)) != g_ball.break_direction) ||
        (math_absf(error_m) < APP_BALL_BREAK_RELEASE_M)) {
        g_ball.break_direction = 0;
    }

    if (g_ball.break_direction != 0) {
        g_ball.snapshot.tilt_feedforward_rad -= (float) g_ball.break_direction *
            APP_BALL_BREAK_GAIN * APP_BALL_FRICTION_BREAK_RAD;
    }
    unsaturated_tilt_rad = g_ball.snapshot.tilt_feedback_rad +
        g_ball.snapshot.tilt_feedforward_rad;
    tilt_command_rad = math_clampf(unsaturated_tilt_rad,
        -APP_BALL_MAX_TILT_RAD, APP_BALL_MAX_TILT_RAD);
    if ((tilt_command_rad == unsaturated_tilt_rad) ||
        (error_m * (unsaturated_tilt_rad - tilt_command_rad) < 0.0F)) {
        g_ball.error_integral_ms += error_m * APP_BALL_CONTROL_PERIOD_S;
        g_ball.error_integral_ms = math_clampf(g_ball.error_integral_ms,
            -APP_BALL_INTEGRAL_LIMIT_MS, APP_BALL_INTEGRAL_LIMIT_MS);
    }
    g_ball.snapshot.tilt_command_rad = tilt_command_rad;
    g_ball.snapshot.actuator_target = math_clampf(
        APP_BALL_COMMAND_SIGN * APP_BALL_ACTUATOR_TURNS_PER_RAD * tilt_command_rad,
        -APP_BALL_MAX_ACTUATOR_OFFSET_TURNS, APP_BALL_MAX_ACTUATOR_OFFSET_TURNS);
    actual_tilt_rad = APP_BALL_COMMAND_SIGN * g_ball.snapshot.actuator_feedback /
        APP_BALL_ACTUATOR_TURNS_PER_RAD;
    ball_observer_advance(actual_tilt_rad, ax_mps2);
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
