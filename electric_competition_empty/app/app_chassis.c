#include "app/app_chassis.h"

#include "algo/algo_filter.h"
#include "algo/algo_pid.h"
#include "common/math_util.h"
#include "drivers/drv_line_sensor.h"
#include "drivers/drv_motor_dc.h"

#define APP_CHASSIS_WHEEL_DIAMETER_M      (0.065F)
#define APP_CHASSIS_MAX_SPEED_MPS         (0.50F)
#define APP_CHASSIS_LINE_MARK_HITS        (6U)

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
    int32_t travel_left_zero;
    int32_t travel_right_zero;
    float cruise_speed_mps;
    bool enabled;
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
    .kp = 0.07F,
    .ki = 0.20F,
    .kd = 0.001F,
    .dt_s = 0.001F,
    .output_limit = 0.95F,
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
    .kp = 0.23F,
    .ki = 0.0F,
    .kd = 0.012F,
    .dt_s = 0.005F,
    .output_limit = 1.20F,
    .integral_limit = 0.0F,
    .integral_separation = 0.0F,
    .derivative_lpf_alpha = 0.20F,
    .setpoint_slew_rate = 0.0F,
    .deadband = 0.08F,
    .derivative_on_measurement = true,
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
    app_chassis_reset_travel();
}

void app_chassis_set_enabled(bool enabled)
{
    g_chassis.enabled = enabled;
    if (!enabled) {
        pid_reset(&g_chassis.left_speed_pid);
        pid_reset(&g_chassis.right_speed_pid);
        motor_dc_set_output(&g_chassis.left_motor, 0.0F);
        motor_dc_set_output(&g_chassis.right_motor, 0.0F);
    }
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
    uint8_t hits;

    line_sensor_update(&g_chassis.line_sensor);
    hits = chassis_count_bits(g_chassis.line_sensor.raw_bits);
    g_chassis.snapshot.line_bits = g_chassis.line_sensor.raw_bits;
    g_chassis.snapshot.line_error = g_chassis.line_sensor.line_error;
    g_chassis.snapshot.line_lost = g_chassis.line_sensor.line_lost;
    g_chassis.snapshot.line_state = g_chassis.line_sensor.line_lost ?
        APP_LINE_FOLLOW_LOST : APP_LINE_FOLLOW_TRACK;
    g_chassis.snapshot.start_line_detected = hits >= APP_CHASSIS_LINE_MARK_HITS;
    g_chassis.snapshot.line_offset_m = g_chassis.line_sensor.line_error * 0.005F;
}

void app_chassis_control_task(float control_dt_s, float elapsed_s)
{
    float left_speed;
    float right_speed;
    float base_rps;
    float turn_rps = 0.0F;
    float left_target = 0.0F;
    float right_target = 0.0F;

    (void) elapsed_s;
    encoder_driver_update_speed(&g_chassis.encoder_driver, control_dt_s);
    left_speed = lpf1_update(&g_chassis.left_speed_filter, g_chassis.encoder_driver.left.speed_rps);
    right_speed = lpf1_update(&g_chassis.right_speed_filter, g_chassis.encoder_driver.right.speed_rps);

    if (g_chassis.enabled && !g_chassis.line_sensor.line_lost) {
        base_rps = g_chassis.cruise_speed_mps /
            (APP_CHASSIS_WHEEL_DIAMETER_M * 3.1415926F);
        turn_rps = pid_update(&g_chassis.line_pid, 0.0F, g_chassis.line_sensor.line_error);
        left_target = math_clampf(base_rps + turn_rps, -3.0F, 3.0F);
        right_target = math_clampf(base_rps - turn_rps, -3.0F, 3.0F);
    } else {
        pid_reset(&g_chassis.line_pid);
    }

    if ((left_target == 0.0F) && (right_target == 0.0F)) {
        pid_reset(&g_chassis.left_speed_pid);
        pid_reset(&g_chassis.right_speed_pid);
        motor_dc_set_output(&g_chassis.left_motor, 0.0F);
        motor_dc_set_output(&g_chassis.right_motor, 0.0F);
    } else {
        motor_dc_set_output(&g_chassis.left_motor,
            pid_update(&g_chassis.left_speed_pid, left_target, left_speed));
        motor_dc_set_output(&g_chassis.right_motor,
            pid_update(&g_chassis.right_speed_pid, right_target, right_speed));
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
