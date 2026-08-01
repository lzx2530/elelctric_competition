#include "app/app_ball_control.h"

#include "algo/algo_pid.h"
#include "common/math_util.h"
#include "drivers/drv_abs_position.h"
#include "drivers/drv_stepper.h"

#define APP_BALL_VISION_TIMEOUT_MS       (0U)
#define APP_BALL_CONTROL_PERIOD_S         (0.03333333F)
#define APP_BALL_CAMERA_DELAY_FRAMES      (2U)
#define APP_BALL_OBSERVER_HISTORY_LENGTH  (APP_BALL_CAMERA_DELAY_FRAMES)
#define APP_BALL_MAX_TILT_RAD            (0.034907F)
#define APP_BALL_ACTUATOR_TURNS_PER_RAD  (152.000F)
#define APP_BALL_MAX_ACTUATOR_OFFSET_TURNS (16.000F)
#define APP_BALL_COMMAND_SIGN            (1.0F)
#define APP_BALL_G_MPS2                  (9.80665F)
#define APP_BALL_BETA                    (1.400F)
#define APP_BALL_KG_MPS2_PER_RAD         (APP_BALL_G_MPS2 / APP_BALL_BETA)
#define APP_BALL_KP                       (14.000F)
#define APP_BALL_KD_POSITIVE_TARGET       (6.000F)
#define APP_BALL_KD_NEGATIVE_TARGET       (8.000F)
#define APP_BALL_KI                       (0.0F)
#define APP_BALL_INTEGRAL_LIMIT_MS        (0.500F)
#define APP_BALL_KF_K_POSITION            (0.541505F)
#define APP_BALL_KF_K_VELOCITY            (6.254990F)
/* Extra braking look-ahead after the two-frame camera-delay compensation. */
#define APP_BALL_PREVIEW_TIME_S            (0.100F)
#define APP_BALL_FRICTION_BREAK_RAD       (0.011999F)
#define APP_BALL_BREAK_ENGAGE_M           (0.0005F)
#define APP_BALL_BREAK_RELEASE_M          (0.0003F)
#define APP_BALL_BREAK_GAIN               (1.30F)
#define APP_BALL_TRAJ_MAX_SPEED_MMPS      (10.0F)
#define APP_BALL_TRAJ_ACCEL_MMPS2         (30.0F)
#define APP_BALL_TRAJ_NEGATIVE_MAX_SPEED_MMPS (9.0F)
#define APP_BALL_TRAJ_NEGATIVE_ACCEL_MMPS2     (20.0F)
#define APP_BALL_TRAJ_POSITION_GAIN       (1.25F)
#define APP_BALL_TRAJ_TERMINAL_WINDOW_MM  (25.0F)
#define APP_BALL_TRAJ_TERMINAL_POSITION_GAIN (0.55F)
#define APP_BALL_STATIC_POSITIVE_TARGET_MM (50.0F)
#define APP_BALL_STATIC_NEGATIVE_TARGET_MM (-50.0F)
#define APP_BALL_STATIC_PHASE_SPEED_HZ      (12000.0F)
#define APP_BALL_STATIC_PHASE_1_TIME_S     (0.86F)
#define APP_BALL_STATIC_PHASE_2_TIME_S     (1.69F)
#define APP_BALL_STATIC_PHASE_3_TIME_S     (1.36F)
#define APP_BALL_STATIC_PHASE_4_TIME_S     (0.58F)
#define APP_BALL_STATIC_PHASE_LEFT_ACCEL   (1U)
#define APP_BALL_STATIC_PHASE_RIGHT_BRAKE  (2U)
#define APP_BALL_STATIC_PHASE_LEFT_BRAKE   (3U)
#define APP_BALL_STATIC_PHASE_RIGHT_LEVEL  (4U)
#define APP_BALL_STATIC_PHASE_COMPLETE     (5U)
#define APP_BALL_SETTLE_WINDOW_MM         (10.0F)
#define APP_BALL_SETTLE_MAX_TILT_RAD      (0.020944F)
/* Keep correcting at the final point instead of forcing the rod horizontal. */
#define APP_BALL_ENABLE_LEVEL_HOLD        (0U)
#define APP_BALL_LEVEL_HOLD_ENTER_ERROR_MM (10.0F)
#define APP_BALL_LEVEL_HOLD_EXIT_ERROR_MM  (15.0F)
#define APP_BALL_LEVEL_HOLD_ENTER_SPEED_MMPS (5.0F)
#define APP_BALL_LEVEL_HOLD_EXIT_SPEED_MMPS  (8.0F)
#define APP_BALL_LEVEL_HOLD_MAX_TILT_RAD  (0.008727F)
#define APP_BALL_LEVEL_HOLD_CONFIRM_SAMPLES (4U)
#define APP_BALL_VISION_MIN_CONFIDENCE_PERMILLE (150U)
#define APP_BALL_VISION_MAX_POSITION_STEP_MM   (30)

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
    float control_error_mm;
    float trajectory_target_mm;
    float trajectory_velocity_mmps;
    float trajectory_acceleration_mmps2;
    uint32_t trajectory_tick_ms;
    uint32_t static_phase_start_tick_ms;
    int16_t last_accepted_actual_position_mm;
    int8_t break_direction;
    uint8_t level_hold_samples;
    uint8_t static_motion_phase;
    bool observer_initialized;
    bool vision_sample_pending;
    bool actuator_reference_valid;
    bool trajectory_initialized;
    bool static_motion_active;
    bool last_accepted_actual_position_valid;
    bool level_hold_active;
} ball_control_app_t;

static ball_control_app_t g_ball;

static const stepper_config_t g_pitch_stepper_cfg = {
    .axis = BSP_STEPPER_AXIS_PITCH,
    .dir_output = BSP_DIR_PITCH,
    .invert_direction = false,
    .min_frequency_hz = 5.0F,
    .max_frequency_hz = 16000.0F,
    .accel_hz_per_s = 96000.0F,
    .reverse_accel_hz_per_s = 192000.0F,
};

static const pid_config_t g_actuator_pid_cfg = {
    .kp = 7000.0F,
    .ki = 300.0F,
    .kd = 40.0F,
    .dt_s = 0.001F,
    .output_limit = 16000.0F,
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
    g_ball.level_hold_samples = 0U;
    g_ball.level_hold_active = false;
    g_ball.observer_initialized = false;
    g_ball.vision_sample_pending = false;
    g_ball.snapshot.observer_ready = false;
    g_ball.snapshot.level_hold_active = false;
    g_ball.snapshot.estimated_error_mm = 0.0F;
    g_ball.snapshot.ball_velocity_mmps = 0.0F;
}

static int16_t ball_round_to_i16(float value)
{
    return (int16_t) (value >= 0.0F ? value + 0.5F : value - 0.5F);
}

static bool ball_actual_position_acceptable(const k230_frame_t *frame)
{
    int32_t position_step_mm;

    if (!frame->ball.actual_position_valid ||
        frame->ball.confidence_permille < APP_BALL_VISION_MIN_CONFIDENCE_PERMILLE) {
        return false;
    }
    if (!g_ball.last_accepted_actual_position_valid) {
        return true;
    }

    position_step_mm = (int32_t) frame->ball.actual_position_mm -
        (int32_t) g_ball.last_accepted_actual_position_mm;
    return position_step_mm >= -APP_BALL_VISION_MAX_POSITION_STEP_MM &&
        position_step_mm <= APP_BALL_VISION_MAX_POSITION_STEP_MM;
}

static void ball_trajectory_reset(void)
{
    g_ball.control_error_mm = 0.0F;
    g_ball.trajectory_target_mm = 0.0F;
    g_ball.trajectory_velocity_mmps = 0.0F;
    g_ball.trajectory_acceleration_mmps2 = 0.0F;
    g_ball.snapshot.trajectory_velocity_mmps = 0.0F;
    g_ball.trajectory_tick_ms = 0U;
    g_ball.static_phase_start_tick_ms = 0U;
    g_ball.static_motion_phase = 0U;
    g_ball.trajectory_initialized = false;
    g_ball.static_motion_active = false;
}

static void ball_static_motion_update(uint32_t tick_ms)
{
    float phase_elapsed_s;

    if (!g_ball.static_motion_active) {
        g_ball.static_motion_active = true;
        g_ball.static_phase_start_tick_ms = tick_ms;
        g_ball.static_motion_phase = APP_BALL_STATIC_PHASE_LEFT_ACCEL;
    }

    phase_elapsed_s = (float) (tick_ms - g_ball.static_phase_start_tick_ms) * 0.001F;
    if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_LEFT_ACCEL &&
        phase_elapsed_s >= APP_BALL_STATIC_PHASE_1_TIME_S) {
        g_ball.static_phase_start_tick_ms = tick_ms;
        g_ball.static_motion_phase = APP_BALL_STATIC_PHASE_RIGHT_BRAKE;
    } else if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_RIGHT_BRAKE &&
        phase_elapsed_s >= APP_BALL_STATIC_PHASE_2_TIME_S) {
        g_ball.static_phase_start_tick_ms = tick_ms;
        g_ball.static_motion_phase = APP_BALL_STATIC_PHASE_LEFT_BRAKE;
    } else if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_LEFT_BRAKE &&
        phase_elapsed_s >= APP_BALL_STATIC_PHASE_3_TIME_S) {
        g_ball.static_phase_start_tick_ms = tick_ms;
        g_ball.static_motion_phase = APP_BALL_STATIC_PHASE_RIGHT_LEVEL;
    } else if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_RIGHT_LEVEL &&
        phase_elapsed_s >= APP_BALL_STATIC_PHASE_4_TIME_S) {
        g_ball.static_motion_phase = APP_BALL_STATIC_PHASE_COMPLETE;
    }

    if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_LEFT_ACCEL) {
        g_ball.trajectory_target_mm = APP_BALL_STATIC_POSITIVE_TARGET_MM;
    } else if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_RIGHT_BRAKE) {
        g_ball.trajectory_target_mm = APP_BALL_STATIC_POSITIVE_TARGET_MM;
    } else if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_LEFT_BRAKE) {
        g_ball.trajectory_target_mm = APP_BALL_STATIC_NEGATIVE_TARGET_MM;
    } else {
        g_ball.trajectory_target_mm = APP_BALL_STATIC_NEGATIVE_TARGET_MM;
    }

    g_ball.trajectory_velocity_mmps = 0.0F;
    g_ball.trajectory_acceleration_mmps2 = 0.0F;
}

static void ball_trajectory_update(int16_t actual_position_mm, int16_t position_error_mm,
    uint32_t tick_ms)
{
    (void) position_error_mm;
    (void) tick_ms;
    g_ball.trajectory_initialized = true;
    g_ball.control_error_mm = (float) actual_position_mm - g_ball.trajectory_target_mm;
    g_ball.snapshot.target_mm = ball_round_to_i16(g_ball.trajectory_target_mm);
    g_ball.snapshot.trajectory_velocity_mmps = g_ball.trajectory_velocity_mmps;
}

#if APP_BALL_ENABLE_LEVEL_HOLD
static void ball_level_hold_update(float estimated_velocity_mmps, float actual_tilt_rad)
{
    float final_error_mm = (float) g_ball.snapshot.ball_position_mm;

    if (!g_ball.snapshot.stop_requested) {
        g_ball.level_hold_samples = 0U;
        g_ball.level_hold_active = false;
        return;
    }

    if (g_ball.level_hold_active) {
        if ((math_absf(final_error_mm) > APP_BALL_LEVEL_HOLD_EXIT_ERROR_MM) ||
            (math_absf(estimated_velocity_mmps) > APP_BALL_LEVEL_HOLD_EXIT_SPEED_MMPS)) {
            g_ball.level_hold_active = false;
            g_ball.level_hold_samples = 0U;
        }
        return;
    }

    if ((math_absf(final_error_mm) <= APP_BALL_LEVEL_HOLD_ENTER_ERROR_MM) &&
        (math_absf(estimated_velocity_mmps) <= APP_BALL_LEVEL_HOLD_ENTER_SPEED_MMPS) &&
        (math_absf(actual_tilt_rad) <= APP_BALL_LEVEL_HOLD_MAX_TILT_RAD) &&
        (math_absf(g_ball.trajectory_velocity_mmps) <= 1.0F)) {
        if (g_ball.level_hold_samples < APP_BALL_LEVEL_HOLD_CONFIRM_SAMPLES) {
            g_ball.level_hold_samples++;
        }
        if (g_ball.level_hold_samples >= APP_BALL_LEVEL_HOLD_CONFIRM_SAMPLES) {
            g_ball.level_hold_active = true;
            g_ball.error_integral_ms = 0.0F;
            g_ball.break_direction = 0;
        }
    } else {
        g_ball.level_hold_samples = 0U;
    }
}
#endif

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
    abs_position_init(&g_ball.actuator_encoder, 0.0F, 1.0F);
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

void app_ball_control_set_trajectory_enabled(bool enabled)
{
    g_ball.snapshot.trajectory_enabled = enabled;
    ball_trajectory_reset();
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

    g_ball.snapshot.ball_position_mm = frame->ball.position_mm;
    g_ball.snapshot.actual_position_valid = frame->ball.actual_position_valid;
    g_ball.snapshot.actual_position_mm = frame->ball.actual_position_mm;
    if (g_ball.snapshot.trajectory_enabled && g_ball.snapshot.enabled &&
        frame->ball.actual_position_present) {
        if (!ball_actual_position_acceptable(frame)) {
            return;
        }
        g_ball.last_accepted_actual_position_mm = frame->ball.actual_position_mm;
        g_ball.last_accepted_actual_position_valid = true;
        ball_trajectory_update(frame->ball.actual_position_mm, frame->ball.position_mm,
            tick_ms);
    } else {
        g_ball.control_error_mm = (float) frame->ball.position_mm;
    }
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
    g_ball.snapshot.actual_position_mm = 0;
    g_ball.snapshot.actual_position_valid = false;
    g_ball.last_accepted_actual_position_mm = 0;
    g_ball.last_accepted_actual_position_valid = false;
    g_ball.last_vision_tick_ms = 0U;
    ball_trajectory_reset();
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
        g_ball.snapshot.actuator_tilt_rad = APP_BALL_COMMAND_SIGN *
            g_ball.snapshot.actuator_feedback / APP_BALL_ACTUATOR_TURNS_PER_RAD;
    }
}

void app_ball_control_outer_task(const imu_snapshot_t *imu, uint32_t tick_ms)
{
    float ax_mps2;
    float error_m;
    float velocity_error_mps;
    float desired_accel_mps2;
    float velocity_gain;
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
    if ((APP_BALL_VISION_TIMEOUT_MS > 0U) &&
        ((!g_ball.snapshot.vision_valid) ||
            ((tick_ms - g_ball.last_vision_tick_ms) > APP_BALL_VISION_TIMEOUT_MS))) {
        g_ball.snapshot.fault = APP_BALL_FAULT_VISION_TIMEOUT;
        return;
    }
    if (g_ball.snapshot.trajectory_enabled) {
        ball_static_motion_update(tick_ms);
    }
    if (!g_ball.vision_sample_pending) {
        return;
    }

    /* Terminal trajectory is static; do not inject vehicle-IMU feedforward there. */
    ax_mps2 = !g_ball.snapshot.trajectory_enabled && (imu != NULL) &&
        imu->online && imu->accel_bias_ready ?
        imu->longitudinal_accel_mps2 : 0.0F;
    ball_observer_correct_and_predict(g_ball.control_error_mm,
        &estimated_error_mm, &estimated_velocity_mmps);
    g_ball.vision_sample_pending = false;

    g_ball.snapshot.observer_ready = g_ball.observer_initialized;
    g_ball.snapshot.estimated_error_mm = estimated_error_mm;
    g_ball.snapshot.ball_velocity_mmps = estimated_velocity_mmps;
    actual_tilt_rad = APP_BALL_COMMAND_SIGN * g_ball.snapshot.actuator_feedback /
        APP_BALL_ACTUATOR_TURNS_PER_RAD;
    g_ball.snapshot.actuator_tilt_rad = actual_tilt_rad;

#if APP_BALL_ENABLE_LEVEL_HOLD
    ball_level_hold_update(estimated_velocity_mmps, actual_tilt_rad);
    g_ball.snapshot.level_hold_active = g_ball.level_hold_active;
    if (g_ball.level_hold_active) {
        g_ball.snapshot.tilt_feedback_rad = 0.0F;
        g_ball.snapshot.tilt_feedforward_rad = 0.0F;
        g_ball.snapshot.tilt_command_rad = 0.0F;
        g_ball.snapshot.actuator_target = 0.0F;
        ball_observer_advance(actual_tilt_rad, 0.0F);
        g_ball.snapshot.fault = APP_BALL_FAULT_NONE;
        return;
    }
#else
    g_ball.snapshot.level_hold_active = false;
#endif

    if (g_ball.snapshot.trajectory_enabled && g_ball.static_motion_active) {
        g_ball.snapshot.tilt_feedback_rad = 0.0F;
        g_ball.snapshot.tilt_feedforward_rad = 0.0F;
        g_ball.snapshot.tilt_command_rad = actual_tilt_rad;
        g_ball.snapshot.actuator_target = 0.0F;
        ball_observer_advance(actual_tilt_rad, 0.0F);
        g_ball.snapshot.fault = APP_BALL_FAULT_NONE;
        return;
    }

    preview_error_mm = estimated_error_mm + APP_BALL_PREVIEW_TIME_S *
        (estimated_velocity_mmps - g_ball.trajectory_velocity_mmps);
    error_m = -0.001F * preview_error_mm;
    velocity_error_mps = -0.001F * (estimated_velocity_mmps -
        g_ball.trajectory_velocity_mmps);
    velocity_gain = estimated_error_mm < 0.0F ?
        APP_BALL_KD_POSITIVE_TARGET : APP_BALL_KD_NEGATIVE_TARGET;
    desired_accel_mps2 = 0.001F * g_ball.trajectory_acceleration_mmps2 +
        APP_BALL_KP * error_m + velocity_gain * velocity_error_mps +
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
    if (math_absf(estimated_error_mm) <= APP_BALL_SETTLE_WINDOW_MM &&
        estimated_error_mm * estimated_velocity_mmps < 0.0F &&
        tilt_command_rad * estimated_velocity_mmps < 0.0F) {
        tilt_command_rad = math_clampf(tilt_command_rad,
            -APP_BALL_SETTLE_MAX_TILT_RAD, APP_BALL_SETTLE_MAX_TILT_RAD);
    }
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
    ball_observer_advance(actual_tilt_rad, ax_mps2);
    g_ball.snapshot.fault = APP_BALL_FAULT_NONE;
}

void app_ball_control_inner_task(float dt_s)
{
    float command_hz = 0.0F;

    if (g_ball.snapshot.enabled && g_ball.snapshot.actuator_feedback_valid &&
        g_ball.snapshot.fault == APP_BALL_FAULT_NONE) {
        if (g_ball.snapshot.trajectory_enabled && g_ball.static_motion_active) {
            pid_reset(&g_ball.actuator_pid);
            if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_LEFT_ACCEL ||
                g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_LEFT_BRAKE) {
                command_hz = -APP_BALL_STATIC_PHASE_SPEED_HZ;
            } else if (g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_RIGHT_BRAKE ||
                g_ball.static_motion_phase == APP_BALL_STATIC_PHASE_RIGHT_LEVEL) {
                command_hz = APP_BALL_STATIC_PHASE_SPEED_HZ;
            }
        } else {
            command_hz = pid_update(&g_ball.actuator_pid,
                g_ball.snapshot.actuator_target, g_ball.snapshot.actuator_feedback);
        }
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
