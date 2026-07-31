#include "app/app_chassis.h"

#include "algo/algo_filter.h"
#include "algo/algo_pid.h"
#include "common/math_util.h"
#include "drivers/drv_line_sensor.h"
#include "drivers/drv_motor_dc.h"

#define APP_CHASSIS_WHEEL_DIAMETER_M      (0.065F)
#define APP_CHASSIS_MAX_SPEED_MPS         (1.6F)
#define APP_CHASSIS_MAX_TARGET_RPS        (10.0F)
#define APP_CHASSIS_LINE_LOST_CONFIRM_SAMPLES (3U)
#define APP_CHASSIS_STRAIGHT_OUTPUT_BOOST (0.50F)
#define APP_CHASSIS_TURN_OUTPUT_BOOST     (0.70F)
#define APP_CHASSIS_OUTPUT_BOOST_SLEW_RATE (2.0F)
#define APP_CHASSIS_TURN_ENTER_RPS        (1.5F)
#define APP_CHASSIS_TURN_EXIT_RPS         (0.5F)
#define APP_CHASSIS_ENABLE_CURVE_SPEED_PLANNER (0U)
#define APP_CHASSIS_CURVE_SLOWDOWN_START_ERROR (0.50F)
#define APP_CHASSIS_CURVE_SLOWDOWN_FULL_ERROR  (2.50F)
#define APP_CHASSIS_CURVE_MIN_SPEED_SCALE      (0.65F)
#define APP_CHASSIS_CURVE_SPEED_FILTER_ALPHA    (0.20F)
#define APP_CHASSIS_CURVE_SPEED_SLOWDOWN_ALPHA  (0.35F)
#define APP_CHASSIS_LINE_SAMPLE_PERIOD_S        (0.005F)
#define APP_CHASSIS_MAX_BRAKE_DURATION_S        (0.20F)
#define APP_CHASSIS_LINE_ERROR_FILTER_ALPHA     (0.30F)
#define APP_CHASSIS_LINE_RATE_FILTER_ALPHA      (0.15F)
#define APP_CHASSIS_LINE_PREDICTION_HORIZON_S   (0.028F)
#define APP_CHASSIS_LINE_PREDICTION_LIMIT       (0.75F)
#define APP_CHASSIS_CURVE_SLOWDOWN_SLEW_RATE    (8.0F)
#define APP_CHASSIS_CURVE_RECOVERY_SLEW_RATE    (3.0F)

typedef struct {
    motor_dc_handle_t left_motor;
    motor_dc_handle_t right_motor;
    encoder_driver_t encoder_driver;
    line_sensor_handle_t line_sensor;
    pid_handle_t line_pid;
    pid_handle_t left_speed_pid;
    pid_handle_t right_speed_pid;
    lpf1_handle_t left_speed_filter;
    lpf1_handle_t right_speed_filter;
    lpf1_handle_t curve_speed_filter;
    lpf1_handle_t line_error_filter;
    lpf1_handle_t line_error_rate_filter;
    ramp_filter_handle_t curve_speed_ramp;
    ramp_filter_handle_t output_boost_ramp;
    int32_t travel_left_zero;
    int32_t travel_right_zero;
    float cruise_speed_mps;
    float last_line_error;
    float previous_line_error;
    float line_error_rate;
    float predicted_line_error;
    float curve_speed_ramp_scale;
    float curve_speed_scale;
    float output_boost;
    float brake_remaining_s;
    uint8_t line_lost_samples;
    uint8_t center_mark_count;
    bool enabled;
    bool line_lost_confirmed;
    bool line_error_rate_valid;
    bool turn_output_active;
    bool center_mark_active;
    bool start_line_latched;
    chassis_snapshot_t snapshot;
} chassis_app_t;

static chassis_app_t g_chassis;

static const motor_dc_config_t g_left_motor_cfg = {
    .pwm_channel = BSP_MOTOR_PWM_LEFT,
    .invert_direction = false,
    .deadband = 0.02F,
    .max_duty = 0.95F,
};

static const motor_dc_config_t g_right_motor_cfg = {
    .pwm_channel = BSP_MOTOR_PWM_RIGHT,
    .invert_direction = true,
    .deadband = 0.02F,
    .max_duty = 0.95F,
};

static const encoder_config_t g_left_encoder_cfg = {
    .counts_per_revolution = 1760.0F,
    .invert_direction = false,
};

static const encoder_config_t g_right_encoder_cfg = {
    .counts_per_revolution = 1760.0F,
    .invert_direction = true,
};

static const line_sensor_config_t g_line_sensor_cfg = {
    .active_high = true,
    .settle_cycles = 1600U,
    .weights = {-3.5F, -2.5F, -1.5F, -0.5F, 0.5F, 1.5F, 2.5F, 3.5F},
};

static const pid_config_t g_speed_pid_cfg = {
    .kp = 0.06F,
    .ki = 0.20F,
    .kd = 0.001F,
    .dt_s = 0.001F,
    .output_limit = 4.5F,
    .integral_limit = 0.25F,
    .integral_separation = 1.0F,
    .derivative_lpf_alpha = 0.15F,
    .setpoint_slew_rate = 50.0F,
    .deadband = 0.02F,
    .derivative_on_measurement = true,
    .enable_integral_separation = true,
    .enable_output_limit = true,
    .enable_integral_limit = true,
    .enable_deadband = true,
    .enable_setpoint_ramp = false,
};

static const pid_config_t g_line_pid_cfg = {
    .kp = 2.8F,
    .ki = 0.0F,
    .kd = 0.0F,
    .dt_s = 0.005F,
    .output_limit = 8.0F,
    .integral_limit = 0.0F,
    .integral_separation = 0.0F,
    .derivative_lpf_alpha = 0.0F,
    .setpoint_slew_rate = 0.0F,
    .deadband = 0.10F,
    .derivative_on_measurement = false,
    .enable_integral_separation = false,
    .enable_output_limit = true,
    .enable_integral_limit = false,
    .enable_deadband = true,
    .enable_setpoint_ramp = false,
};

static uint8_t chassis_count_bits(uint8_t bits)
{
    uint8_t count = 0U;

    while (bits != 0U) {
        count += (uint8_t) (bits & 0x01U);
        bits >>= 1U;
    }
    return count;
}

static bool chassis_has_consecutive_line_mark(uint8_t bits)
{
    uint8_t overlapping_bits = bits;

    overlapping_bits &= (uint8_t) (bits >> 1U);
    overlapping_bits &= (uint8_t) (bits >> 2U);
    overlapping_bits &= (uint8_t) (bits >> 3U);

    return overlapping_bits != 0U;
}

#if (APP_CHASSIS_ENABLE_CURVE_SPEED_PLANNER != 0U)
static float chassis_get_curve_speed_scale(float line_error)
{
    float normalized_error;
    float error_magnitude = math_absf(line_error);

    if (error_magnitude <= APP_CHASSIS_CURVE_SLOWDOWN_START_ERROR) {
        return 1.0F;
    }

    normalized_error = (error_magnitude - APP_CHASSIS_CURVE_SLOWDOWN_START_ERROR) /
        (APP_CHASSIS_CURVE_SLOWDOWN_FULL_ERROR - APP_CHASSIS_CURVE_SLOWDOWN_START_ERROR);
    normalized_error = math_clampf(normalized_error, 0.0F, 1.0F);
    return 1.0F - ((1.0F - APP_CHASSIS_CURVE_MIN_SPEED_SCALE) * normalized_error);
}
#endif

static float chassis_get_travel_mm(void)
{
    float left_turns = (float) (g_chassis.encoder_driver.left.count -
        g_chassis.travel_left_zero) / g_chassis.encoder_driver.left.cfg.counts_per_revolution;
    float right_turns = (float) (g_chassis.encoder_driver.right.count -
        g_chassis.travel_right_zero) / g_chassis.encoder_driver.right.cfg.counts_per_revolution;
    return (left_turns + right_turns) * 0.5F * APP_CHASSIS_WHEEL_DIAMETER_M * 3.1415926F * 1000.0F;
}

void app_chassis_init(void)
{
    motor_dc_init(&g_chassis.left_motor, &g_left_motor_cfg);
    motor_dc_init(&g_chassis.right_motor, &g_right_motor_cfg);
    encoder_driver_init(&g_chassis.encoder_driver, &g_left_encoder_cfg, &g_right_encoder_cfg);
    line_sensor_init(&g_chassis.line_sensor, &g_line_sensor_cfg);
    pid_init(&g_chassis.line_pid, &g_line_pid_cfg, PID_MODE_POSITION);
    pid_init(&g_chassis.left_speed_pid, &g_speed_pid_cfg, PID_MODE_POSITION);
    pid_init(&g_chassis.right_speed_pid, &g_speed_pid_cfg, PID_MODE_POSITION);
    lpf1_init(&g_chassis.left_speed_filter, 0.20F, 0.0F);
    lpf1_init(&g_chassis.right_speed_filter, 0.20F, 0.0F);
    lpf1_init(&g_chassis.line_error_filter, APP_CHASSIS_LINE_ERROR_FILTER_ALPHA, 0.0F);
    lpf1_init(&g_chassis.line_error_rate_filter, APP_CHASSIS_LINE_RATE_FILTER_ALPHA, 0.0F);
    ramp_filter_init(&g_chassis.output_boost_ramp, APP_CHASSIS_OUTPUT_BOOST_SLEW_RATE,
        APP_CHASSIS_STRAIGHT_OUTPUT_BOOST);
    g_chassis.output_boost = APP_CHASSIS_STRAIGHT_OUTPUT_BOOST;
#if (APP_CHASSIS_ENABLE_CURVE_SPEED_PLANNER != 0U)
    lpf1_init(&g_chassis.curve_speed_filter, APP_CHASSIS_CURVE_SPEED_FILTER_ALPHA, 1.0F);
    ramp_filter_init(&g_chassis.curve_speed_ramp, APP_CHASSIS_CURVE_RECOVERY_SLEW_RATE, 1.0F);
    g_chassis.curve_speed_ramp_scale = 1.0F;
    g_chassis.curve_speed_scale = 1.0F;
#endif
    g_chassis.line_lost_confirmed = true;
    g_chassis.start_line_latched = false;
    app_chassis_reset_travel();
}

void app_chassis_set_enabled(bool enabled)
{
    g_chassis.enabled = enabled;
    if (enabled) {
        g_chassis.brake_remaining_s = 0.0F;
        g_chassis.center_mark_count = 0U;
        g_chassis.center_mark_active = false;
        g_chassis.start_line_latched = false;
    }
    if (!enabled) {
        pid_reset(&g_chassis.left_speed_pid);
        pid_reset(&g_chassis.right_speed_pid);
        g_chassis.turn_output_active = false;
        ramp_filter_init(&g_chassis.output_boost_ramp, APP_CHASSIS_OUTPUT_BOOST_SLEW_RATE,
            APP_CHASSIS_STRAIGHT_OUTPUT_BOOST);
        g_chassis.output_boost = APP_CHASSIS_STRAIGHT_OUTPUT_BOOST;
        motor_dc_set_output(&g_chassis.left_motor, 0.0F);
        motor_dc_set_output(&g_chassis.right_motor, 0.0F);
    }
}

void app_chassis_brake(float duration_s)
{
    app_chassis_set_enabled(false);
    g_chassis.brake_remaining_s = math_clampf(duration_s, 0.0F,
        APP_CHASSIS_MAX_BRAKE_DURATION_S);
}

void app_chassis_set_cruise_speed_mps(float speed_mps)
{
    g_chassis.cruise_speed_mps = math_clampf(speed_mps, 0.0F, APP_CHASSIS_MAX_SPEED_MPS);
}

void app_chassis_reset_travel(void)
{
    g_chassis.travel_left_zero = g_chassis.encoder_driver.left.count;
    g_chassis.travel_right_zero = g_chassis.encoder_driver.right.count;
    g_chassis.snapshot.travel_mm = 0.0F;
}

void app_chassis_line_task(void)
{
#if (APP_CHASSIS_ENABLE_CURVE_SPEED_PLANNER != 0U)
    float curve_speed_target;
#endif
    bool line_mark_detected;

    line_sensor_update(&g_chassis.line_sensor);
    line_mark_detected =
        chassis_has_consecutive_line_mark(g_chassis.line_sensor.raw_bits);
    if (line_mark_detected) {
        if (!g_chassis.center_mark_active) {
            g_chassis.center_mark_active = true;
            if (g_chassis.center_mark_count < UINT8_MAX) {
                g_chassis.center_mark_count++;
            }
            if (g_chassis.center_mark_count >= 2U) {
                g_chassis.start_line_latched = true;
            }
        }
    } else {
        g_chassis.center_mark_active = false;
    }
    if (g_chassis.line_sensor.line_lost) {
        if (g_chassis.line_lost_samples < APP_CHASSIS_LINE_LOST_CONFIRM_SAMPLES) {
            g_chassis.line_lost_samples++;
        }
        if (g_chassis.line_lost_samples >= APP_CHASSIS_LINE_LOST_CONFIRM_SAMPLES) {
            g_chassis.line_lost_confirmed = true;
        }
        g_chassis.line_error_rate = lpf1_update(&g_chassis.line_error_rate_filter, 0.0F);
        g_chassis.line_error_rate_valid = false;
    } else {
        g_chassis.line_lost_samples = 0U;
        g_chassis.line_lost_confirmed = false;
        g_chassis.last_line_error = lpf1_update(&g_chassis.line_error_filter,
            g_chassis.line_sensor.line_error);
        if (g_chassis.line_error_rate_valid) {
            g_chassis.line_error_rate = lpf1_update(&g_chassis.line_error_rate_filter,
                (g_chassis.last_line_error - g_chassis.previous_line_error) /
                    APP_CHASSIS_LINE_SAMPLE_PERIOD_S);
        } else {
            lpf1_init(&g_chassis.line_error_rate_filter, APP_CHASSIS_LINE_RATE_FILTER_ALPHA, 0.0F);
            g_chassis.line_error_rate = 0.0F;
            g_chassis.line_error_rate_valid = true;
        }
        g_chassis.previous_line_error = g_chassis.last_line_error;
    }
    g_chassis.predicted_line_error = g_chassis.last_line_error + math_clampf(
        g_chassis.line_error_rate * APP_CHASSIS_LINE_PREDICTION_HORIZON_S,
        -APP_CHASSIS_LINE_PREDICTION_LIMIT, APP_CHASSIS_LINE_PREDICTION_LIMIT);
#if (APP_CHASSIS_ENABLE_CURVE_SPEED_PLANNER != 0U)
    curve_speed_target = chassis_get_curve_speed_scale(g_chassis.predicted_line_error);
    g_chassis.curve_speed_ramp.slew_rate = curve_speed_target < g_chassis.curve_speed_ramp_scale ?
        APP_CHASSIS_CURVE_SLOWDOWN_SLEW_RATE : APP_CHASSIS_CURVE_RECOVERY_SLEW_RATE;
    g_chassis.curve_speed_ramp_scale = ramp_filter_update(&g_chassis.curve_speed_ramp,
        curve_speed_target, APP_CHASSIS_LINE_SAMPLE_PERIOD_S);
    g_chassis.curve_speed_filter.alpha = g_chassis.curve_speed_ramp_scale < g_chassis.curve_speed_scale ?
        APP_CHASSIS_CURVE_SPEED_SLOWDOWN_ALPHA : APP_CHASSIS_CURVE_SPEED_FILTER_ALPHA;
    g_chassis.curve_speed_scale = lpf1_update(&g_chassis.curve_speed_filter,
        g_chassis.curve_speed_ramp_scale);
#endif
    g_chassis.snapshot.line_bits = g_chassis.line_sensor.raw_bits;
    g_chassis.snapshot.line_error = g_chassis.last_line_error;
    g_chassis.snapshot.line_lost = g_chassis.line_lost_confirmed;
    g_chassis.snapshot.line_state = g_chassis.line_lost_confirmed ?
        APP_LINE_FOLLOW_LOST : APP_LINE_FOLLOW_TRACK;
    g_chassis.snapshot.start_line_detected = g_chassis.start_line_latched;
    g_chassis.snapshot.line_offset_m = g_chassis.last_line_error * 0.005F;
}

void app_chassis_control_task(float control_dt_s, float elapsed_s)
{
    float left_speed;
    float right_speed;
    float base_rps;
    float turn_rps = 0.0F;
    float left_target = 0.0F;
    float right_target = 0.0F;
    float left_output;
    float right_output;

    (void) elapsed_s;
    encoder_driver_update_speed(&g_chassis.encoder_driver, control_dt_s);
    left_speed = lpf1_update(&g_chassis.left_speed_filter, g_chassis.encoder_driver.left.speed_rps);
    right_speed = lpf1_update(&g_chassis.right_speed_filter, g_chassis.encoder_driver.right.speed_rps);

    if (g_chassis.brake_remaining_s > 0.0F) {
        if (g_chassis.brake_remaining_s <= control_dt_s) {
            g_chassis.brake_remaining_s = 0.0F;
        } else {
            g_chassis.brake_remaining_s -= control_dt_s;
        }
        motor_dc_brake(&g_chassis.left_motor);
        motor_dc_brake(&g_chassis.right_motor);
        g_chassis.snapshot.enabled = false;
        g_chassis.snapshot.left_speed_rps = left_speed;
        g_chassis.snapshot.right_speed_rps = right_speed;
        g_chassis.snapshot.left_target_rps = 0.0F;
        g_chassis.snapshot.right_target_rps = 0.0F;
        g_chassis.snapshot.left_output = 0.0F;
        g_chassis.snapshot.right_output = 0.0F;
        g_chassis.snapshot.travel_mm = chassis_get_travel_mm();
        return;
    }

    if (g_chassis.enabled && !g_chassis.line_lost_confirmed) {
        base_rps = g_chassis.cruise_speed_mps /
            (APP_CHASSIS_WHEEL_DIAMETER_M * 3.1415926F);
#if (APP_CHASSIS_ENABLE_CURVE_SPEED_PLANNER != 0U)
        base_rps *= g_chassis.curve_speed_scale;
#endif
        turn_rps = pid_update(&g_chassis.line_pid, g_chassis.predicted_line_error, 0.0F);
        if (g_chassis.turn_output_active) {
            if (math_absf(turn_rps) < APP_CHASSIS_TURN_EXIT_RPS) {
                g_chassis.turn_output_active = false;
            }
        } else if (math_absf(turn_rps) > APP_CHASSIS_TURN_ENTER_RPS) {
            g_chassis.turn_output_active = true;
        }
        left_target = math_clampf(base_rps + turn_rps,
            -APP_CHASSIS_MAX_TARGET_RPS, APP_CHASSIS_MAX_TARGET_RPS);
        right_target = math_clampf(base_rps - turn_rps,
            -APP_CHASSIS_MAX_TARGET_RPS, APP_CHASSIS_MAX_TARGET_RPS);
    } else {
        pid_reset(&g_chassis.line_pid);
        g_chassis.turn_output_active = false;
    }

    if ((left_target == 0.0F) && (right_target == 0.0F)) {
        pid_reset(&g_chassis.left_speed_pid);
        pid_reset(&g_chassis.right_speed_pid);
        motor_dc_set_output(&g_chassis.left_motor, 0.0F);
        motor_dc_set_output(&g_chassis.right_motor, 0.0F);
    } else {
        left_output = pid_update(&g_chassis.left_speed_pid, left_target, left_speed);
        right_output = pid_update(&g_chassis.right_speed_pid, right_target, right_speed);
        if (!g_chassis.line_lost_confirmed) {
            float output_boost_target = g_chassis.turn_output_active ?
                APP_CHASSIS_TURN_OUTPUT_BOOST : APP_CHASSIS_STRAIGHT_OUTPUT_BOOST;
            g_chassis.output_boost = ramp_filter_update(&g_chassis.output_boost_ramp,
                output_boost_target, control_dt_s);
            left_output *= g_chassis.output_boost;
            right_output *= g_chassis.output_boost;
        }
        motor_dc_set_output(&g_chassis.left_motor, left_output);
        motor_dc_set_output(&g_chassis.right_motor, right_output);
    }

    g_chassis.snapshot.enabled = g_chassis.enabled;
    g_chassis.snapshot.left_speed_rps = left_speed;
    g_chassis.snapshot.right_speed_rps = right_speed;
    g_chassis.snapshot.left_target_rps = left_target;
    g_chassis.snapshot.right_target_rps = right_target;
    g_chassis.snapshot.left_output = motor_dc_get_output(&g_chassis.left_motor);
    g_chassis.snapshot.right_output = motor_dc_get_output(&g_chassis.right_motor);
    g_chassis.snapshot.travel_mm = chassis_get_travel_mm();
}

const chassis_snapshot_t *app_chassis_get_snapshot(void)
{
    return &g_chassis.snapshot;
}

encoder_driver_t *app_chassis_get_encoder_driver(void)
{
    return &g_chassis.encoder_driver;
}
