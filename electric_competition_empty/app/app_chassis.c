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

static const float g_track_speed_rps = 7.5f;
static const float g_track_large_error_speed_rps = 3.0f;
static const float g_wheel_target_limit_rps = 10.0f;
static const float g_corner_pivot_speed_rps = 1.5f;
// static const float g_lost_speed_rps = 1.0f;
static const float g_corner_dash_speed_rps = 2.5f;
static const uint16_t g_corner_dash_samples = 60U;
static const uint8_t g_corner_stop_confirm_samples = 20U;
static const float g_corner_stop_speed_threshold_rps = 0.15f;
static const float g_wheel_track_m = 0.110f;
static const float g_wheel_diameter_m = 0.065f;
static const float g_corner_pivot_pause_s = 0.5f;
static const uint8_t g_corner_reacquire_samples = 3U;
static const float g_recover_turn_speed_rps = 1.0f;
// static const float g_recover_inner_speed_rps = 0.0f;
// static const float g_track_line_gain = 0.85f;
static const float g_line_deadband = 0.10f;
static const float g_line_error_limit = 2.50f;
static const uint16_t g_lost_hold_samples = 40U;
static const uint16_t g_recover_flip_samples = 2U;
static const uint8_t g_corner_confirm_samples = 4U;
static const float g_speed_filter_alpha = 0.20f;
static const float g_track_output_boost = 0.40f;
const float g_dynamic_gain_threshold = 2.6f;


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
    int8_t locked_turn;
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

static int8_t chassis_get_corner_candidate(uint8_t raw_bits, chassis_app_t *chassis)
{
    // locked_turn：直角弯方向锁定 (-1:左转中, 1:右转中, 0:未锁)
    // 作用：防止转弯转到一半，车头甩过去导致传感器误判回打！ 
    uint8_t total_hits = chassis_count_bits(raw_bits);

    // -------------------------------------------------------------
    // 1. 如果处于直角弯锁定状态，优先检查是否“过弯完成”
    // -------------------------------------------------------------
    if (chassis->locked_turn != 0) {
        // 静态计数器：记录在时间维度上“连续有灯亮”的次数
        static uint8_t unlock_confirm_count = 0;

        // 只要有任意传感器亮灯（说明车头开始切入黑线）
        if (total_hits > 0U) {
            unlock_confirm_count++;
        } else {
            // 中途若断线/全灭（如遇到砖缝间隙），瞬间重置计数，防抖动误解锁
            unlock_confirm_count = 0U;
        }

        // 连续 2 帧（如果觉得不够稳，可以改为 3U）都检测到黑线，确认过弯完成！
        if (unlock_confirm_count >= 2U) {
            chassis->locked_turn = 0;   // 解锁，恢复正常 PID 巡线
            unlock_confirm_count = 0U;  // 清空计数器，供下次使用
        } else {
            // 还没达到连续确认次数，坚决保持之前的转向，绝不中途反打！
            return chassis->locked_turn;
        }
    }

    // -------------------------------------------------------------
    // 2. 统计左右半边的灯数
    // -------------------------------------------------------------
    // 0x0F (0000 1111) 为左半边 4 个传感器 (Bit 0~3)
    // 0xF0 (1111 0000) 为右半边 4 个传感器 (Bit 4~7)
    uint8_t left_hits  = chassis_count_bits(raw_bits & 0x0FU); 
    uint8_t right_hits = chassis_count_bits(raw_bits & 0xF0U); 

    bool touches_left_edge  = (raw_bits & 0x01U) != 0U; // Bit 0 最左
    bool touches_right_edge = (raw_bits & 0x80U) != 0U; // Bit 7 最右

    // -------------------------------------------------------------
    // 3. 优先判定“直角弯特征”（比较左右相对优势，而不是只看极值）
    // -------------------------------------------------------------
    // 左直角弯：左半边亮了 3 个及以上，且最左侧触线，且左边数量绝对多于右边
    if (touches_left_edge && (left_hits >= 3U) && (left_hits > right_hits)) {
        chassis->locked_turn = -1; // 锁定左转
        return -1;
    }

    // 右直角弯：右半边亮了 3 个及以上，且最右侧触线，且右边数量绝对多于左边
    if (touches_right_edge && (right_hits >= 3U) && (right_hits > left_hits)) {
        chassis->locked_turn = 1;  // 锁定右转
        return 1;
    }

    // -------------------------------------------------------------
    // 4. 极粗线/全亮（全黑线或交叉口）保底处理
    // -------------------------------------------------------------
    if (total_hits >= 6U) {
        // 如果之前有锁定，沿用锁定；否则返回 0 保持直行
        return chassis->locked_turn;
    }

    // 5. 没达到直角弯标准，返回 0，交给普通 PID 或巡线逻辑
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

static bool chassis_corner_entry_present(chassis_app_t *chassis)
{
    int8_t candidate = chassis_get_corner_candidate(chassis->line_sensor.raw_bits,chassis);

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

    if (chassis->line_state_samples < g_corner_dash_samples) {
        chassis->corner_reacquire_count = 0U;
        return false;
    }

    if (chassis->corner_stop_confirm_count < g_corner_stop_confirm_samples) {
        chassis->corner_reacquire_count = 0U;
        return false;
    }

    if (raw_bits == 0U) {
        chassis->corner_reacquire_count = 0U;
        return false;
    }

    if (chassis->corner_reacquire_count < UINT8_MAX) {
        chassis->corner_reacquire_count++;
    }

    return chassis->corner_reacquire_count >= g_corner_reacquire_samples;

    // 1. 等待车头完全离开入弯时的横向黑线
#if 0
    if (!chassis->corner_entry_cleared) {
        if (!chassis_corner_entry_present(chassis)) {
            chassis->corner_entry_cleared = true;
        }
        chassis->corner_sensor_sequence_step = 0U;
        return false;
    }

    // 2. 等待打转角度或旋转逻辑准备就绪
    if (chassis->corner_stop_confirm_count < g_corner_stop_confirm_samples) {
        chassis->corner_sensor_sequence_step = 0U;
        return false;
    }

    // ==================== 核心判定逻辑修复 ====================

    // A. 抓线判定：只要传感阵列抓到了黑线（1 到 4 个灯亮均算有效抓线）
    return chassis_corner_sensor_sequence_complete(chassis, raw_bits);

#if 0
    uint8_t total_hits = chassis_count_bits(raw_bits);
    bool line_detected = (total_hits >= 1U) && (total_hits <= 4U);

    // B. 边缘防干扰判定：使用 0x81U (二进制 1000 0001)，仅要求绝对最外侧的 Bit 7 和 Bit 0 处于灭状态
    // 这样哪怕姿态稍偏、压在右侧倒数第二个灯（Bit 6 或 Bit 5）上，也能被正常认可！
    bool edge_clean = (raw_bits & 0x81U) == 0U;

    // C. 如果没有抓到线，或者最外侧踩到了干扰线，重置稳定计数器
    if (!line_detected || !edge_clean) {
        chassis->corner_sensor_sequence_step = 0U;
        return false;
    }

    // D. 姿态合格（抓到线且边缘干净），开始累加稳定采样次数
    if (chassis->corner_sensor_sequence_step < UINT8_MAX) {
        chassis->corner_sensor_sequence_step++;
    }

    // E. 同时满足“打转暂停时间”和“重新抓线稳定次数”后，退出转弯！
    return (chassis->corner_pivot_pause_elapsed_s >= g_corner_pivot_pause_s) &&
           (chassis->corner_sensor_sequence_step >= 4U);
#endif
#endif
}

static bool chassis_center_reacquired(const chassis_app_t *chassis)
{
    return (chassis->center_history & 0x07U) == 0x07U;
}

static void chassis_enter_state(chassis_app_t *chassis, app_line_follow_state_t state)
{
    chassis->line_state = state;
    chassis->line_state_samples = 0U;

    chassis->locked_turn = 0; // 切换状态时强制解除锁定，防止死锁
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
        (chassis->line_state_samples < g_corner_dash_samples) ||
        (chassis->corner_stop_confirm_count >= g_corner_stop_confirm_samples)) {
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
            chassis_get_corner_candidate(chassis->line_sensor.raw_bits,chassis));
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
    float left_output;
    float right_output;
    float line_error;

    // encoder_driver_poll(&g_chassis.encoder_driver);
    encoder_driver_update_speed(&g_chassis.encoder_driver, control_dt_s);
    left_speed = lpf1_update(&g_chassis.left_speed_filter, g_chassis.encoder_driver.left.speed_rps);
    right_speed = lpf1_update(&g_chassis.right_speed_filter, g_chassis.encoder_driver.right.speed_rps);
    chassis_update_corner_stop_state(&g_chassis, left_speed, right_speed);
    chassis_update_corner_pivot_angle(&g_chassis);
    chassis_update_corner_pivot_pause(&g_chassis, elapsed_s);

    line_error = g_chassis.line_sensor.line_error;
    chassis_get_wheel_targets(&g_chassis, &left_ref, &right_ref);

    left_output = chassis_get_forward_output(&g_chassis.left_speed_pid, left_ref, left_speed);
    right_output = chassis_get_forward_output(&g_chassis.right_speed_pid, right_ref, right_speed);

    if ((g_chassis.line_state == APP_LINE_FOLLOW_TRACK) &&
        (left_ref > 0.0f) && (right_ref > 0.0f)) {
        left_output = math_clampf(left_output * g_track_output_boost, 0.0f, 0.95f);
        right_output = math_clampf(right_output * g_track_output_boost, 0.0f, 0.95f);
    }

    motor_dc_set_output(&g_chassis.left_motor, left_output);
    motor_dc_set_output(&g_chassis.right_motor, right_output);

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
