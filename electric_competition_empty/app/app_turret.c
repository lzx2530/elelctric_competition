#include "app/app_turret.h"

#include "algo/algo_pid.h"
#include "bsp/bsp_gpio.h"
#include "drivers/drv_stepper.h"

typedef struct {
    stepper_handle_t yaw_stepper;
    stepper_handle_t pitch_stepper;
    pid_handle_t yaw_pid;
    pid_handle_t pitch_pid;
    k230_frame_t target;
    turret_snapshot_t snapshot;
} turret_app_t;

static turret_app_t g_turret;

void app_turret_init(void)
{
    static const stepper_config_t yaw_cfg = {
        .axis = BSP_STEPPER_AXIS_YAW,
        .dir_output = BSP_DIR_YAW,
        .invert_direction = false,
        .min_frequency_hz = 5.0f,
        .max_frequency_hz = 4000.0f,
        .accel_hz_per_s = 8000.0f,
    };
    static const stepper_config_t pitch_cfg = {
        .axis = BSP_STEPPER_AXIS_PITCH,
        .dir_output = BSP_DIR_PITCH,
        .invert_direction = false,
        .min_frequency_hz = 5.0f,
        .max_frequency_hz = 4000.0f,
        .accel_hz_per_s = 8000.0f,
    };
    static const pid_config_t turret_pid_cfg = {
        .kp = 25.0f,
        .ki = 0.0f,
        .kd = 0.8f,
        .dt_s = 0.001f,
        .output_limit = 3000.0f,
        .integral_limit = 500.0f,
        .integral_separation = 30.0f,
        .derivative_lpf_alpha = 0.15f,
        .setpoint_slew_rate = 0.0f,
        .deadband = 1.0f,
        .derivative_on_measurement = true,
        .enable_integral_separation = true,
        .enable_output_limit = true,
        .enable_integral_limit = true,
        .enable_deadband = true,
        .enable_setpoint_ramp = false,
    };

    stepper_init(&g_turret.yaw_stepper, &yaw_cfg);
    stepper_init(&g_turret.pitch_stepper, &pitch_cfg);
    stepper_enable(&g_turret.yaw_stepper, true);
    stepper_enable(&g_turret.pitch_stepper, true);
    pid_init(&g_turret.yaw_pid, &turret_pid_cfg, PID_MODE_POSITION);
    pid_init(&g_turret.pitch_pid, &turret_pid_cfg, PID_MODE_POSITION);
    bsp_gpio_set_laser(false);
}

void app_turret_set_target(const k230_frame_t *frame)
{
    g_turret.target = *frame;
}

void app_turret_control_task(float dt_s)
{
    float yaw_cmd = 0.0f;
    float pitch_cmd = 0.0f;

    if (g_turret.target.valid) {
        /* 视觉链路给的是偏差量，所以这里直接把 0 当目标做误差收敛。 */
        yaw_cmd = pid_update(&g_turret.yaw_pid, 0.0f, (float) g_turret.target.x_error);
        pitch_cmd = pid_update(&g_turret.pitch_pid, 0.0f, (float) g_turret.target.y_error);
        bsp_gpio_set_laser(true);
    } else {
        /* 丢目标时清空 PID 状态，避免目标恢复后沿用旧积分和旧微分。 */
        pid_reset(&g_turret.yaw_pid);
        pid_reset(&g_turret.pitch_pid);
        bsp_gpio_set_laser(false);
    }

    stepper_set_speed(&g_turret.yaw_stepper, yaw_cmd);
    stepper_set_speed(&g_turret.pitch_stepper, pitch_cmd);
    stepper_update(&g_turret.yaw_stepper, dt_s);
    stepper_update(&g_turret.pitch_stepper, dt_s);

    g_turret.snapshot.target_valid = g_turret.target.valid;
    g_turret.snapshot.x_error = g_turret.target.x_error;
    g_turret.snapshot.y_error = g_turret.target.y_error;
    g_turret.snapshot.yaw_cmd_hz = yaw_cmd;
    g_turret.snapshot.pitch_cmd_hz = pitch_cmd;
}

const turret_snapshot_t *app_turret_get_snapshot(void)
{
    return &g_turret.snapshot;
}
