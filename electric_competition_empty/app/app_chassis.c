#include "app/app_chassis.h"

#include "algo/algo_filter.h"
#include "algo/algo_pid.h"
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
    .invert_direction = false,
    .deadband = 0.02f,
    .max_duty = 0.95f,
};

static const encoder_config_t g_left_encoder_cfg = {
    .counts_per_revolution = 780.0f,
    .invert_direction = true,
};

static const encoder_config_t g_right_encoder_cfg = {
    .counts_per_revolution = 780.0f,
    .invert_direction = false,
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

static const float g_base_speed_rps = 6.0f;
static const float g_line_gain = 0.9f;
static const float g_speed_filter_alpha = 0.20f;

typedef struct {
    motor_dc_handle_t left_motor;
    motor_dc_handle_t right_motor;
    encoder_driver_t encoder_driver;
    line_sensor_handle_t line_sensor;
    pid_handle_t left_speed_pid;
    pid_handle_t right_speed_pid;
    lpf1_handle_t left_speed_filter;
    lpf1_handle_t right_speed_filter;
    float base_speed_rps;
    float line_gain;
    chassis_snapshot_t snapshot;
} chassis_app_t;

static chassis_app_t g_chassis;

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

    g_chassis.base_speed_rps = g_base_speed_rps;
    g_chassis.line_gain = g_line_gain;
}

void app_chassis_line_task(void)
{
    /* Update line position at 5 ms so mux settling jitter stays out of the 1 kHz speed loop. */
    line_sensor_update(&g_chassis.line_sensor);
    g_chassis.snapshot.line_bits = g_chassis.line_sensor.raw_bits;
    g_chassis.snapshot.line_error = g_chassis.line_sensor.line_error;
    g_chassis.snapshot.line_lost = g_chassis.line_sensor.line_lost;
}

void app_chassis_control_task(float dt_s)
{
    float left_speed;
    float right_speed;
    float left_ref;
    float right_ref;

    encoder_driver_poll(&g_chassis.encoder_driver);
    encoder_driver_update_speed(&g_chassis.encoder_driver, dt_s);
    left_speed = lpf1_update(&g_chassis.left_speed_filter, g_chassis.encoder_driver.left.speed_rps);
    right_speed = lpf1_update(&g_chassis.right_speed_filter, g_chassis.encoder_driver.right.speed_rps);

    /* Stop both wheel targets when the line is lost so off-track behavior stays predictable. */
    if (g_chassis.line_sensor.line_lost) {
        left_ref = 0.0f;
        right_ref = 0.0f;
    } else {
        /* Convert line error to differential wheel-speed targets; speed PIDs track wheel rps. */
        left_ref = g_chassis.base_speed_rps - g_chassis.line_gain * g_chassis.line_sensor.line_error;
        right_ref = g_chassis.base_speed_rps + g_chassis.line_gain * g_chassis.line_sensor.line_error;
    }

    motor_dc_set_output(&g_chassis.left_motor, pid_update(&g_chassis.left_speed_pid, left_ref, left_speed));
    motor_dc_set_output(&g_chassis.right_motor, pid_update(&g_chassis.right_speed_pid, right_ref, right_speed));

    g_chassis.snapshot.left_speed_rps = left_speed;
    g_chassis.snapshot.right_speed_rps = right_speed;
    g_chassis.snapshot.left_target_rps = left_ref;
    g_chassis.snapshot.right_target_rps = right_ref;
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
