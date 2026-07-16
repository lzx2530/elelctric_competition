#include "app/app_chassis.h"

#include "algo/algo_filter.h"
#include "algo/algo_pid.h"
#include "common/math_util.h"
#include "drivers/drv_line_sensor.h"
#include "drivers/drv_motor_dc.h"

static const motor_dc_config_t g_left_motor_cfg = {
    .pwm_channel = BSP_MOTOR_PWM_LEFT,
    .invert_direction = false,
    .deadband = 0.02f,
    .max_duty = 0.95f,
};

static const motor_dc_config_t g_right_motor_cfg = {
    .pwm_channel = BSP_MOTOR_PWM_RIGHT,
    .invert_direction = true,
    .deadband = 0.02f,
    .max_duty = 0.95f,
};

static const encoder_config_t g_left_encoder_cfg = {
    .counts_per_revolution = 1760.0f,
    .invert_direction = false,
};

static const encoder_config_t g_right_encoder_cfg = {
    .counts_per_revolution = 1760.0f,
    .invert_direction = true,
};

static const line_sensor_config_t g_line_sensor_cfg = {
    .active_high = true,
    .settle_cycles = 1600U,
    .weights = {-3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f},
};

static const pid_config_t g_speed_pid_cfg = {
    .kp = 0.12f,
    .ki = 0.40f,
    .kd = 0.001f,
    .dt_s = 0.001f,
    .output_limit = 1.0f,
    .integral_limit = 0.6f,
    .integral_separation = 3.0f,
    .derivative_lpf_alpha = 0.15f,
    .setpoint_slew_rate = 50.0f,
    .deadband = 0.02f,
    .derivative_on_measurement = true,
    .enable_integral_separation = true,
    .enable_output_limit = true,
    .enable_integral_limit = true,
    .enable_deadband = true,
    .enable_setpoint_ramp = false,
};

static const float g_track_speed_rps = 3.0f;
static const float g_corner_pivot_speed_rps = 4.0f;
static const float g_lost_speed_rps = 1.0f;
static const float g_recover_turn_speed_rps = 2.5f;
static const float g_recover_inner_speed_rps = 0.0f;
// static const float g_track_line_gain = 0.85f;
static const float g_line_deadband = 0.10f;
static const float g_line_error_limit = 2.50f;
static const uint16_t g_lost_hold_samples = 40U;
static const uint16_t g_recover_flip_samples = 80U;
static const uint8_t g_corner_confirm_samples = 3U;
static const uint8_t g_corner_reacquire_samples = 3U;
static const float g_speed_filter_alpha = 0.20f;
static const float g_track_gain_straight = 0.50f;   // 直道小增益
static const float g_track_gain_curve = 0.85f;      // 弯道大增益
const float g_dynamic_gain_threshold = 1.5f;


#define LINE_SENSOR_LEFT_HALF_MASK     (0x0FU)
#define LINE_SENSOR_CENTER_MASK        (0x18U)
#define LINE_SENSOR_RIGHT_HALF_MASK    (0xF0U)
#define LINE_SENSOR_HISTORY_MASK       (0x1FU)

typedef struct {
    motor_dc_handle_t left_motor;
    motor_dc_handle_t right_motor;
    encoder_driver_t encoder_driver;
    line_sensor_handle_t line_sensor;
    pid_handle_t left_speed_pid;
    pid_handle_t right_speed_pid;
    lpf1_handle_t left_speed_filter;
    lpf1_handle_t right_speed_filter;
    app_line_follow_state_t line_state;
    uint16_t line_state_samples;
    float last_valid_line_error;
    float search_bias;
    float corner_bias;
    int8_t pending_corner_direction;
    uint8_t corner_confirm_count;
    bool corner_entry_cleared;
    uint8_t corner_reacquire_count;
    uint8_t center_history;
    chassis_snapshot_t snapshot;
} chassis_app_t;

static chassis_app_t g_chassis;

static float chassis_get_control_error(float line_error)
{
    return line_error;
}

static void chassis_update_sensor_history(chassis_app_t *chassis)
{
    uint8_t raw_bits = chassis->line_sensor.raw_bits;
    bool center_hit = (raw_bits & LINE_SENSOR_CENTER_MASK) != 0U;

    chassis->center_history = (uint8_t) (((chassis->center_history << 1U) |
        (center_hit ? 1U : 0U)) & LINE_SENSOR_HISTORY_MASK);
}

static uint8_t chassis_count_bits(uint8_t bits)
{
    uint8_t count = 0U;

    while (bits != 0U) {
        count += (uint8_t) (bits & 0x01U);
        bits >>= 1U;
    }

    return count;
}

static bool chassis_bits_are_contiguous(uint8_t bits)
{
    if (bits == 0U) {
        return false;
    }

    while ((bits & 0x01U) == 0U) {
        bits >>= 1U;
    }
    while ((bits & 0x01U) != 0U) {
        bits >>= 1U;
    }

    return bits == 0U;
}

static int8_t chassis_get_corner_candidate(uint8_t raw_bits)
{
    uint8_t total_hits = chassis_count_bits(raw_bits);
    uint8_t left_hits = chassis_count_bits(raw_bits & LINE_SENSOR_LEFT_HALF_MASK);
    uint8_t right_hits = chassis_count_bits(raw_bits & LINE_SENSOR_RIGHT_HALF_MASK);

    if ((total_hits < 4U) || (total_hits > 6U) || !chassis_bits_are_contiguous(raw_bits)) {
        return 0;
    }

    if (left_hits > right_hits) {
        return 1;
    }
    if (right_hits > left_hits) {
        return -1;
    }

    return 0;
}

static int8_t chassis_confirm_corner_candidate(chassis_app_t *chassis, int8_t candidate)
{
    if (candidate == 0) {
        chassis->pending_corner_direction = 0;
        chassis->corner_confirm_count = 0U;
        return 0;
    }

    if (candidate != chassis->pending_corner_direction) {
        chassis->pending_corner_direction = candidate;
        chassis->corner_confirm_count = 1U;
        return 0;
    }

    if (chassis->corner_confirm_count < UINT8_MAX) {
        chassis->corner_confirm_count++;
    }
    if (chassis->corner_confirm_count < g_corner_confirm_samples) {
        return 0;
    }

    chassis->pending_corner_direction = 0;
    chassis->corner_confirm_count = 0U;
    return candidate;
}

static bool chassis_corner_entry_present(const chassis_app_t *chassis)
{
    int8_t candidate = chassis_get_corner_candidate(chassis->line_sensor.raw_bits);

    if (chassis->corner_bias < 0.0f) {
        return candidate < 0;
    }

    return candidate > 0;
}

static bool chassis_corner_complete(chassis_app_t *chassis)
{
    uint8_t raw_bits = chassis->line_sensor.raw_bits;

    if (!chassis->corner_entry_cleared) {
        if (!chassis_corner_entry_present(chassis)) {
            chassis->corner_entry_cleared = true;
        }
        chassis->corner_reacquire_count = 0U;
        return false;
    }

    if ((raw_bits & LINE_SENSOR_CENTER_MASK) != LINE_SENSOR_CENTER_MASK) {
        chassis->corner_reacquire_count = 0U;
        return false;
    }

    if (chassis->corner_reacquire_count < UINT8_MAX) {
        chassis->corner_reacquire_count++;
    }

    return chassis->corner_reacquire_count >= g_corner_reacquire_samples;
}

static bool chassis_center_reacquired(const chassis_app_t *chassis)
{
    return (chassis->center_history & 0x07U) == 0x07U;
}

static void chassis_enter_state(chassis_app_t *chassis, app_line_follow_state_t state)
{
    chassis->line_state = state;
    chassis->line_state_samples = 0U;
}

static void chassis_update_line_state(chassis_app_t *chassis)
{
    bool line_lost = chassis->line_sensor.line_lost;
    float line_error = chassis->line_sensor.line_error;
    int8_t corner_candidate;

    chassis_update_sensor_history(chassis);
    if (chassis->line_state == APP_LINE_FOLLOW_TRACK) {
        corner_candidate = chassis_confirm_corner_candidate(
            chassis,
            chassis_get_corner_candidate(chassis->line_sensor.raw_bits));
    } else {
        chassis->pending_corner_direction = 0;
        chassis->corner_confirm_count = 0U;
        corner_candidate = 0;
    }

    if (!line_lost && (chassis->line_state == APP_LINE_FOLLOW_TRACK) &&
        (math_absf(line_error) >= g_line_deadband)) {
        chassis->last_valid_line_error = line_error;
        chassis->search_bias = chassis_get_control_error(line_error);
    }

    switch (chassis->line_state) {
        case APP_LINE_FOLLOW_CORNER:
            if (chassis_corner_complete(chassis)) {
                chassis_enter_state(chassis, APP_LINE_FOLLOW_TRACK);
            }
            break;
        case APP_LINE_FOLLOW_LOST:
            if (!line_lost) {
                chassis_enter_state(chassis, APP_LINE_FOLLOW_TRACK);
            } else if (chassis->line_state_samples >= g_lost_hold_samples) {
                chassis_enter_state(chassis, APP_LINE_FOLLOW_RECOVER);
            }
            break;
        case APP_LINE_FOLLOW_RECOVER:
            if (!line_lost && chassis_center_reacquired(chassis)) {
                chassis_enter_state(chassis, APP_LINE_FOLLOW_TRACK);
            }
            break;
        case APP_LINE_FOLLOW_TRACK:
        default:
            if (corner_candidate != 0) {
                chassis->corner_bias = (float) corner_candidate;
                chassis->search_bias = (float) corner_candidate;
                chassis->corner_entry_cleared = false;
                chassis->corner_reacquire_count = 0U;
                chassis->center_history = 0U;
                chassis_enter_state(chassis, APP_LINE_FOLLOW_CORNER);
            } else if (line_lost) {
                if (math_absf(chassis->search_bias) < g_line_deadband) {
                    chassis->search_bias = (math_absf(chassis->last_valid_line_error) >= g_line_deadband)
                        ? chassis_get_control_error(chassis->last_valid_line_error)
                        : 1.0f;
                }
                chassis_enter_state(chassis, APP_LINE_FOLLOW_LOST);
            }
            break;
    }

    if (chassis->line_state_samples < UINT16_MAX) {
        chassis->line_state_samples++;
    }
}

static void chassis_get_wheel_targets(
    const chassis_app_t *chassis,
    float *left_ref,
    float *right_ref)
{
    float control_error;

    switch (chassis->line_state) {
        case APP_LINE_FOLLOW_CORNER:
            if (chassis->corner_bias >= 0.0f) {
                *left_ref = g_corner_pivot_speed_rps;
                *right_ref = 0.0f;
            } else {
                *left_ref = 0.0f;
                *right_ref = g_corner_pivot_speed_rps;
            }
            break;

        case APP_LINE_FOLLOW_LOST:
        case APP_LINE_FOLLOW_RECOVER:
            // 丢线和恢复状态：绝不能直走！根据丢线前的最后一刻误差，原地打转找线
            control_error = chassis->search_bias;
            if (chassis->line_state == APP_LINE_FOLLOW_RECOVER &&
                chassis->line_state_samples >= g_recover_flip_samples) {
                control_error = -control_error; // 找太久没找到，反向找
            }

            if (control_error >= 0.0f) { // 丢线前线在右边，原地向右打转
                *left_ref = g_recover_turn_speed_rps;
                *right_ref = -g_recover_turn_speed_rps;
            } else {                     // 丢线前线在左边，原地向左打转
                *left_ref = -g_recover_turn_speed_rps;
                *right_ref = g_recover_turn_speed_rps;
            }
            break;

        case APP_LINE_FOLLOW_TRACK:
        default:
            // 1. 获取原始误差
            control_error = chassis_get_control_error(chassis->line_sensor.line_error);
            control_error = math_clampf(control_error, -g_line_error_limit, g_line_error_limit);

            // 2. 死区过滤：防止直道微小震荡
            if ((control_error < g_line_deadband) && (control_error > -g_line_deadband)) {
                control_error = 0.0f;
            }

            // 3. 动态增益与速度分配
            float current_gain;
            float current_base_speed;

            if (fabsf(control_error) > g_dynamic_gain_threshold) {
                // 偏离较大：降回基础速度，大增益拉回
                current_base_speed = g_track_speed_rps - 0.5f;
                current_gain = g_track_gain_curve;
            } else {
                // 走得很直：提速，小增益
                current_base_speed = g_track_speed_rps;
                current_gain = g_track_gain_straight;
            }

            // 4. 计算最终参考速度
            float turn_adjustment = current_gain * control_error;
            *left_ref = current_base_speed + turn_adjustment;
            *right_ref = current_base_speed - turn_adjustment;
            break;
    }

    // 安全锁：允许倒转（提供强心向心力），同时限制最高速度防越界
    *left_ref = math_clampf(*left_ref, -3.0f, 4.5f);
    *right_ref = math_clampf(*right_ref, -3.0f, 4.5f);
}
static float chassis_get_forward_output(pid_handle_t *pid, float target_rps, float speed_rps)
{
    if (target_rps == 0.0f) {
        pid_reset(pid);
        return 0.0f;
    }

    return pid_update(pid, target_rps, speed_rps);
}

void app_chassis_init(void)
{
    motor_dc_init(&g_chassis.left_motor, &g_left_motor_cfg);
    motor_dc_init(&g_chassis.right_motor, &g_right_motor_cfg);
    encoder_driver_init(&g_chassis.encoder_driver, &g_left_encoder_cfg, &g_right_encoder_cfg);
    line_sensor_init(&g_chassis.line_sensor, &g_line_sensor_cfg);
    pid_init(&g_chassis.left_speed_pid, &g_speed_pid_cfg, PID_MODE_POSITION);
    pid_init(&g_chassis.right_speed_pid, &g_speed_pid_cfg, PID_MODE_POSITION);
    lpf1_init(&g_chassis.left_speed_filter, g_speed_filter_alpha, 0.0f);
    lpf1_init(&g_chassis.right_speed_filter, g_speed_filter_alpha, 0.0f);

    g_chassis.line_state = APP_LINE_FOLLOW_TRACK;
    g_chassis.line_state_samples = 0U;
    g_chassis.last_valid_line_error = 0.0f;
    g_chassis.search_bias = 0.0f;
    g_chassis.corner_bias = 1.0f;
    g_chassis.pending_corner_direction = 0;
    g_chassis.corner_confirm_count = 0U;
    g_chassis.corner_entry_cleared = false;
    g_chassis.corner_reacquire_count = 0U;
    g_chassis.center_history = 0U;
}

void app_chassis_line_task(void)
{
    /* Update line position at 5 ms so mux settling jitter stays out of the 1 kHz speed loop. */
    line_sensor_update(&g_chassis.line_sensor);
    chassis_update_line_state(&g_chassis);
    g_chassis.snapshot.line_bits = g_chassis.line_sensor.raw_bits;
    g_chassis.snapshot.line_error = g_chassis.line_sensor.line_error;
    g_chassis.snapshot.line_state = g_chassis.line_state;
    g_chassis.snapshot.line_lost = g_chassis.line_sensor.line_lost;
}

void app_chassis_control_task(float dt_s)
{
    float left_speed;
    float right_speed;
    float left_ref;
    float right_ref;
    float line_error;

    // encoder_driver_poll(&g_chassis.encoder_driver);
    encoder_driver_update_speed(&g_chassis.encoder_driver, dt_s);
    left_speed = lpf1_update(&g_chassis.left_speed_filter, g_chassis.encoder_driver.left.speed_rps);
    right_speed = lpf1_update(&g_chassis.right_speed_filter, g_chassis.encoder_driver.right.speed_rps);

    line_error = g_chassis.line_sensor.line_error;
    chassis_get_wheel_targets(&g_chassis, &left_ref, &right_ref);

    motor_dc_set_output(&g_chassis.left_motor,
        chassis_get_forward_output(&g_chassis.left_speed_pid, left_ref, left_speed));
    motor_dc_set_output(&g_chassis.right_motor,
        chassis_get_forward_output(&g_chassis.right_speed_pid, right_ref, right_speed));

    g_chassis.snapshot.left_speed_rps = left_speed;
    g_chassis.snapshot.right_speed_rps = right_speed;
    g_chassis.snapshot.left_target_rps = left_ref;
    g_chassis.snapshot.right_target_rps = right_ref;
    g_chassis.snapshot.line_error = line_error;
    g_chassis.snapshot.line_bits = g_chassis.line_sensor.raw_bits;
    g_chassis.snapshot.line_state = g_chassis.line_state;
    g_chassis.snapshot.line_lost = g_chassis.line_sensor.line_lost;
    g_chassis.snapshot.left_output = motor_dc_get_output(&g_chassis.left_motor);
    g_chassis.snapshot.right_output = motor_dc_get_output(&g_chassis.right_motor);
}

const chassis_snapshot_t *app_chassis_get_snapshot(void)
{
    return &g_chassis.snapshot;
}

encoder_driver_t *app_chassis_get_encoder_driver(void)
{
    return &g_chassis.encoder_driver;
}
