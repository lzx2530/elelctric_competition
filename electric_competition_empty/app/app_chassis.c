#include "app/app_chassis.h"

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
    .kp = 0.07f,//越小越快，待修正
    .ki = 0.20f,
    .kd = 0.001f,
    .dt_s = 0.001f,
    .output_limit = 1.5f,
    .integral_limit = 0.25f,
    .integral_separation = 1.0f,
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

static const pid_config_t g_line_pid_cfg = {
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .dt_s = 0.005f,
    .output_limit = 2.4f,
    .integral_limit = 0.0f,
    .integral_separation = 0.0f,
    .derivative_lpf_alpha = 0.0f,
    .setpoint_slew_rate = 0.0f,
    .deadband = 0.10f,
    .derivative_on_measurement = false,
    .enable_integral_separation = false,
    .enable_output_limit = true,
    .enable_integral_limit = false,
    .enable_deadband = true,
    .enable_setpoint_ramp = false,
};

static const float g_track_speed_rps = 5.5f;
static const float g_track_large_error_speed_rps = 3.0f;
static const float g_wheel_target_limit_rps = 6.0f;
static const float g_corner_pivot_speed_rps = 1.0f;
// static const float g_lost_speed_rps = 1.0f;
static const uint8_t g_corner_stop_confirm_samples = 20U;
static const float g_corner_dash_speed_rps = 2.5f;
static const uint16_t g_corner_dash_samples = 80U;
static const float g_corner_stop_speed_threshold_rps = 0.15f;
static const float g_wheel_track_m = 0.120f;
static const float g_wheel_diameter_m = 0.065f;
static const float g_corner_pivot_pause_s = 0.5f;
static const float g_recover_turn_speed_rps = 1.0f;
// static const float g_recover_inner_speed_rps = 0.0f;
// static const float g_track_line_gain = 0.85f;
static const float g_line_deadband = 0.10f;
static const float g_line_error_limit = 2.50f;
static const uint16_t g_lost_hold_samples = 40U;
static const uint16_t g_recover_flip_samples = 500U;
static const uint8_t g_corner_confirm_samples = 2U;
static const uint8_t g_corner_reacquire_samples = 2U;
static const float g_speed_filter_alpha = 0.20f;
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
    pid_handle_t line_pid;
    pid_handle_t left_speed_pid;
    pid_handle_t right_speed_pid;
    lpf1_handle_t left_speed_filter;
    lpf1_handle_t right_speed_filter;
    app_line_follow_state_t line_state;
    uint16_t line_state_samples;
    float last_valid_line_error;
    float line_turn_adjustment;
    float search_bias;
    float corner_bias;
    int8_t pending_corner_direction;
    uint8_t corner_confirm_count;
    uint8_t corner_stop_confirm_count;
    int32_t corner_pivot_left_start_count;
    int32_t corner_pivot_right_start_count;
    bool corner_pivot_angle_ready;
    float corner_pivot_pause_elapsed_s;
    bool corner_entry_cleared;
    uint8_t corner_reacquire_count;
    uint8_t center_history;
    app_chassis_mode_t mode;
    float external_forward_rps;
    float external_yaw_rps;
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

static int8_t chassis_get_corner_candidate(uint8_t raw_bits)
{
    uint8_t total_hits = chassis_count_bits(raw_bits);

   // 【补丁】：处理拐弯处粗线导致大量传感器亮灯的情况
    if (total_hits >= 5U) {
        bool left_side  = (raw_bits & 0x03U) != 0U;   // 左侧最外侧2个传感器
        bool right_side = (raw_bits & 0xC0U) != 0U;   // 右侧最外侧2个传感器
        
        if (left_side && right_side) {
            // 左右都压到，可能是非常粗的弯道或暂时性全覆盖
            // 这里可以选择直行，或者根据之前的方向惯性，但最简单先返回0
            return 0;
        }
        
        if (left_side)  return -1;   // 左侧压线 → 往左转（跟随左弯）
        if (right_side) return 1;    // 右侧压线 → 往右转（跟随右弯）
        
        // 亮灯很多但没有明显压到两侧，可能是完全压在粗线中间
        return 0;   // 保持直行
    }

    if (total_hits < 3U) {
        return 0;
    }

    uint8_t left_hits = chassis_count_bits(raw_bits & LINE_SENSOR_LEFT_HALF_MASK);
    uint8_t right_hits = chassis_count_bits(raw_bits & LINE_SENSOR_RIGHT_HALF_MASK);
    bool touches_left_edge = (raw_bits & 0x01U) != 0U;
    bool touches_right_edge = (raw_bits & 0x80U) != 0U;

    if (touches_left_edge && (left_hits >= 3U) && (left_hits > right_hits)) {
        return -1;
    }
    
    if (touches_right_edge && (right_hits >= 3U) && (right_hits > left_hits)) {
        return 1;
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
        // 等待车头完全离开入弯时的横向黑线
        if (!chassis_corner_entry_present(chassis)) {
            chassis->corner_entry_cleared = true;
        }
        chassis->corner_reacquire_count = 0U;
        return false;
    }

    if (!chassis->corner_pivot_angle_ready) {
        chassis->corner_reacquire_count = 0U;
        return false;
    }

    // --- 滤除砖缝与噪点干扰的核心逻辑 ---
    // 1. 中心必须踩到线（只要中间两个有一个亮就行）
    bool center_hit = (raw_bits & LINE_SENSOR_CENTER_MASK) != 0U;
    
    // 2. 最左(bit0)和最右(bit7)的边缘绝对不能踩到线
    // 真正的中心纵线是不可能让边缘灯亮的，如果边缘亮了，说明是扫过了砖缝或横线
    uint8_t right_hits = chassis_count_bits(raw_bits & LINE_SENSOR_RIGHT_HALF_MASK);
    bool edge_clear = (raw_bits & 0x81U) == 0U; 

    // 如果中心没对准，或者边缘还在踩脏东西，直接清零稳定计数器
    if ((!center_hit || !edge_clear) && (right_hits < 2U)) {
        chassis->corner_reacquire_count = 0U;
        return false;
    }

    // 只有姿态干净（中心亮且边缘灭），才开始累加稳定度
    if (chassis->corner_reacquire_count < UINT8_MAX) {
        chassis->corner_reacquire_count++;
    }

    // 判断是否达到了稳定样本数要求
    return (chassis->corner_pivot_pause_elapsed_s >= g_corner_pivot_pause_s) &&
        (chassis->corner_reacquire_count >= g_corner_reacquire_samples);
}

static bool chassis_center_reacquired(const chassis_app_t *chassis)
{
    return (chassis->center_history & 0x07U) == 0x07U;
}

static void chassis_enter_state(chassis_app_t *chassis, app_line_follow_state_t state)
{
    chassis->line_state = state;
    chassis->line_state_samples = 0U;

    if (state == APP_LINE_FOLLOW_TRACK) {
        pid_reset(&chassis->line_pid);
        chassis->line_turn_adjustment = 0.0f;
    }
}

static void chassis_update_corner_stop_state(
    chassis_app_t *chassis,
    float left_speed_rps,
    float right_speed_rps)
{
    if ((chassis->line_state != APP_LINE_FOLLOW_CORNER) ||
        (chassis->line_state_samples < g_corner_dash_samples)) {
        return;
    }

    if (chassis->corner_stop_confirm_count >= g_corner_stop_confirm_samples) {
        return;
    }

    if ((math_absf(left_speed_rps) <= g_corner_stop_speed_threshold_rps) &&
        (math_absf(right_speed_rps) <= g_corner_stop_speed_threshold_rps)) {
        if (chassis->corner_stop_confirm_count < UINT8_MAX) {
            chassis->corner_stop_confirm_count++;
        }
    } else {
        chassis->corner_stop_confirm_count = 0U;
    }

    if (chassis->corner_stop_confirm_count == g_corner_stop_confirm_samples) {
        chassis->corner_pivot_left_start_count = chassis->encoder_driver.left.count;
        chassis->corner_pivot_right_start_count = chassis->encoder_driver.right.count;
    }
}

static void chassis_update_corner_pivot_angle(chassis_app_t *chassis)
{
    float left_turns;
    float right_turns;
    float required_turns;

    if ((chassis->line_state != APP_LINE_FOLLOW_CORNER) ||
        (chassis->corner_stop_confirm_count < g_corner_stop_confirm_samples) ||
        chassis->corner_pivot_angle_ready) {
        return;
    }

    left_turns = math_absf((float) (chassis->encoder_driver.left.count -
        chassis->corner_pivot_left_start_count)) /
        chassis->encoder_driver.left.cfg.counts_per_revolution;
    right_turns = math_absf((float) (chassis->encoder_driver.right.count -
        chassis->corner_pivot_right_start_count)) /
        chassis->encoder_driver.right.cfg.counts_per_revolution;
    required_turns = g_wheel_track_m / (4.0f * g_wheel_diameter_m);

    if (((left_turns + right_turns) * 0.5f) >= required_turns) {
        chassis->corner_pivot_angle_ready = true;
        chassis->corner_pivot_pause_elapsed_s = 0.0f;
    }
}

static void chassis_update_corner_pivot_pause(chassis_app_t *chassis, float dt_s)
{
    if ((chassis->line_state != APP_LINE_FOLLOW_CORNER) ||
        !chassis->corner_pivot_angle_ready ||
        (chassis->corner_pivot_pause_elapsed_s >= g_corner_pivot_pause_s)) {
        return;
    }

    chassis->corner_pivot_pause_elapsed_s += dt_s;
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
                chassis->corner_stop_confirm_count = 0U;
                chassis->corner_pivot_left_start_count = 0;
                chassis->corner_pivot_right_start_count = 0;
                chassis->corner_pivot_angle_ready = false;
                chassis->corner_pivot_pause_elapsed_s = 0.0f;
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

static void chassis_update_track_turn_adjustment(chassis_app_t *chassis)
{
    float control_error;

    if (chassis->line_state != APP_LINE_FOLLOW_TRACK) {
        return;
    }

    control_error = chassis_get_control_error(chassis->line_sensor.line_error);
    control_error = math_clampf(control_error, -g_line_error_limit, g_line_error_limit);
    /* Runs at the 5 ms line-sensor rate; the 1 kHz speed loop uses the cached result. */
    chassis->line_turn_adjustment = pid_update(&chassis->line_pid, control_error, 0.0f);
}

static void chassis_get_wheel_targets(
    const chassis_app_t *chassis,
    float *left_ref,
    float *right_ref)
{
    float control_error;

    switch (chassis->line_state) {
        case APP_LINE_FOLLOW_CORNER:
            if (chassis->line_state_samples < g_corner_dash_samples) {
                *left_ref = g_corner_dash_speed_rps;
                *right_ref = g_corner_dash_speed_rps;
            } else if (chassis->corner_stop_confirm_count < g_corner_stop_confirm_samples) {
                *left_ref = 0.0f;
                *right_ref = 0.0f;
            } else if (chassis->corner_pivot_angle_ready) {
                *left_ref = 0.0f;
                *right_ref = 0.0f;
            } else {
            // 1. 冲弯逻辑
            if (chassis->corner_bias > 0.0f) {
                *left_ref = -g_corner_pivot_speed_rps;
                *right_ref = g_corner_pivot_speed_rps;
            } 
            // 2. 原地打转逻辑
            else {
                if (chassis->corner_bias > 0.0f) {
                    // bias > 0 是右转：左轮正转，右轮反转
                    *left_ref = g_corner_pivot_speed_rps;
                    *right_ref = -g_corner_pivot_speed_rps; 
                } else {
                    // bias < 0 是左转：左轮反转，右轮正转
                    *left_ref = g_corner_pivot_speed_rps;
                    *right_ref = -g_corner_pivot_speed_rps;
                }
            }
            }
            break;

        case APP_LINE_FOLLOW_LOST:
        case APP_LINE_FOLLOW_RECOVER:
            // 丢线和恢复状态：原地打转找线
            control_error = chassis->search_bias;
            if (chassis->line_state == APP_LINE_FOLLOW_RECOVER &&
                chassis->line_state_samples >= g_recover_flip_samples) {
                control_error = -control_error; // 找太久没找到，反向找
            }

            if (control_error >= 0.0f) { 
                *left_ref = g_recover_turn_speed_rps;
                *right_ref = -g_recover_turn_speed_rps;
            } else {                     
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

            // 3. 动态降速
            float current_base_speed;

            if (fabsf(control_error) > g_dynamic_gain_threshold) {
                // 偏离较大：主动降速（比原来多降一点以稳住车身），大增益拉回
                current_base_speed = g_track_large_error_speed_rps;
            } else {
                // 走得很直：提速狂奔，小增益
                current_base_speed = g_track_speed_rps;
            }

            // 4. 计算最终参考速度
            float turn_adjustment = chassis->line_turn_adjustment;
            *left_ref = current_base_speed + turn_adjustment;
            *right_ref = current_base_speed - turn_adjustment;
            break;
    }

    // 安全锁：放宽下限，允许 PID 有足够的倒转空间来完成原地旋转
    *left_ref = math_clampf(*left_ref, -g_wheel_target_limit_rps, g_wheel_target_limit_rps);
    *right_ref = math_clampf(*right_ref, -g_wheel_target_limit_rps, g_wheel_target_limit_rps);
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
    pid_init(&g_chassis.line_pid, &g_line_pid_cfg, PID_MODE_POSITION);
    pid_init(&g_chassis.left_speed_pid, &g_speed_pid_cfg, PID_MODE_POSITION);
    pid_init(&g_chassis.right_speed_pid, &g_speed_pid_cfg, PID_MODE_POSITION);
    lpf1_init(&g_chassis.left_speed_filter, g_speed_filter_alpha, 0.0f);
    lpf1_init(&g_chassis.right_speed_filter, g_speed_filter_alpha, 0.0f);

    g_chassis.line_state = APP_LINE_FOLLOW_TRACK;
    g_chassis.line_state_samples = 0U;
    g_chassis.last_valid_line_error = 0.0f;
    g_chassis.line_turn_adjustment = 0.0f;
    g_chassis.search_bias = 0.0f;
    g_chassis.corner_bias = 1.0f;
    g_chassis.pending_corner_direction = 0;
    g_chassis.corner_confirm_count = 0U;
    g_chassis.corner_stop_confirm_count = 0U;
    g_chassis.corner_pivot_left_start_count = 0;
    g_chassis.corner_pivot_right_start_count = 0;
    g_chassis.corner_pivot_angle_ready = false;
    g_chassis.corner_pivot_pause_elapsed_s = 0.0f;
    g_chassis.corner_entry_cleared = false;
    g_chassis.corner_reacquire_count = 0U;
    g_chassis.center_history = 0U;
    g_chassis.mode = APP_CHASSIS_MODE_STOP;
    g_chassis.external_forward_rps = 0.0f;
    g_chassis.external_yaw_rps = 0.0f;
}

void app_chassis_line_task(void)
{
    /* Update line position at 5 ms so mux settling jitter stays out of the 1 kHz speed loop. */
    line_sensor_update(&g_chassis.line_sensor);
    chassis_update_line_state(&g_chassis);
    chassis_update_track_turn_adjustment(&g_chassis);
    g_chassis.snapshot.line_bits = g_chassis.line_sensor.raw_bits;
    g_chassis.snapshot.line_error = g_chassis.line_sensor.line_error;
    g_chassis.snapshot.line_state = g_chassis.line_state;
    g_chassis.snapshot.line_lost = g_chassis.line_sensor.line_lost;
}

void app_chassis_control_task(float control_dt_s, float elapsed_s)
{
    float left_speed;
    float right_speed;
    float left_ref;
    float right_ref;
    float line_error;

    // encoder_driver_poll(&g_chassis.encoder_driver);
    encoder_driver_update_speed(&g_chassis.encoder_driver, control_dt_s);
    left_speed = lpf1_update(&g_chassis.left_speed_filter, g_chassis.encoder_driver.left.speed_rps);
    right_speed = lpf1_update(&g_chassis.right_speed_filter, g_chassis.encoder_driver.right.speed_rps);
    chassis_update_corner_stop_state(&g_chassis, left_speed, right_speed);
    chassis_update_corner_pivot_angle(&g_chassis);
    chassis_update_corner_pivot_pause(&g_chassis, elapsed_s);

    line_error = g_chassis.line_sensor.line_error;
    if (g_chassis.mode == APP_CHASSIS_MODE_LINE_FOLLOW) {
        chassis_get_wheel_targets(&g_chassis, &left_ref, &right_ref);
    } else if (g_chassis.mode == APP_CHASSIS_MODE_EXTERNAL) {
        left_ref = math_clampf(g_chassis.external_forward_rps + g_chassis.external_yaw_rps,
            -g_wheel_target_limit_rps, g_wheel_target_limit_rps);
        right_ref = math_clampf(g_chassis.external_forward_rps - g_chassis.external_yaw_rps,
            -g_wheel_target_limit_rps, g_wheel_target_limit_rps);
    } else {
        left_ref = 0.0f;
        right_ref = 0.0f;
    }

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

void app_chassis_enable_line_follow(void)
{
    g_chassis.mode = APP_CHASSIS_MODE_LINE_FOLLOW;
}

void app_chassis_set_external_drive(float forward_rps, float yaw_rps)
{
    g_chassis.external_forward_rps = forward_rps;
    g_chassis.external_yaw_rps = yaw_rps;
    g_chassis.mode = APP_CHASSIS_MODE_EXTERNAL;
}

void app_chassis_stop(void)
{
    g_chassis.external_forward_rps = 0.0f;
    g_chassis.external_yaw_rps = 0.0f;
    g_chassis.mode = APP_CHASSIS_MODE_STOP;
}

void app_chassis_get_encoder_counts(int32_t *left, int32_t *right)
{
    if (left != NULL) {
        *left = g_chassis.encoder_driver.left.count;
    }
    if (right != NULL) {
        *right = g_chassis.encoder_driver.right.count;
    }
}

float app_chassis_get_average_distance_mm(int32_t left_start, int32_t right_start)
{
    float left_turns = math_absf((float)(g_chassis.encoder_driver.left.count - left_start)) /
        g_chassis.encoder_driver.left.cfg.counts_per_revolution;
    float right_turns = math_absf((float)(g_chassis.encoder_driver.right.count - right_start)) /
        g_chassis.encoder_driver.right.cfg.counts_per_revolution;

    return 0.5f * (left_turns + right_turns) * 3.1415926f * g_wheel_diameter_m * 1000.0f;
}
