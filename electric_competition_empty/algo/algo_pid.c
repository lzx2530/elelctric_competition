#include "algo/algo_pid.h"

#include <string.h>

#include "common/math_util.h"

static float pid_apply_ramp(pid_handle_t *pid, float ref)
{
    float max_delta;
    float delta;

    /* 设定值斜坡用于抑制目标突变，避免速度环或步进频率一下子冲满。 */
    if ((!pid->cfg.enable_setpoint_ramp) || (pid->cfg.setpoint_slew_rate <= 0.0f) || (pid->cfg.dt_s <= 0.0f)) {
        pid->setpoint = ref;
        return ref;
    }

    max_delta = pid->cfg.setpoint_slew_rate * pid->cfg.dt_s;
    delta = math_clampf(ref - pid->setpoint, -max_delta, max_delta);
    pid->setpoint += delta;
    return pid->setpoint;
}

static bool pid_integral_enabled(const pid_handle_t *pid, float error)
{
    if (!pid->cfg.enable_integral_separation) {
        return true;
    }

    return (math_absf(error) <= pid->cfg.integral_separation);
}

static float pid_limit_output(const pid_handle_t *pid, float value)
{
    if (!pid->cfg.enable_output_limit) {
        return value;
    }
    return math_clampf(value, -pid->cfg.output_limit, pid->cfg.output_limit);
}

static float pid_limit_integral(const pid_handle_t *pid, float value)
{
    if (!pid->cfg.enable_integral_limit) {
        return value;
    }
    return math_clampf(value, -pid->cfg.integral_limit, pid->cfg.integral_limit);
}

static float pid_compute_derivative(pid_handle_t *pid, float error, float feedback)
{
    float raw_derivative;

    if (pid->cfg.dt_s <= 0.0f) {
        return 0.0f;
    }

    /* 微分先行时对测量值求导，能减轻设定值突变带来的微分冲击。 */
    if (pid->cfg.derivative_on_measurement) {
        raw_derivative = -(feedback - pid->prev_feedback) / pid->cfg.dt_s;
    } else {
        raw_derivative = (error - pid->prev_error) / pid->cfg.dt_s;
    }

    /* 对微分项做一阶低通，减少编码器和 IMU 噪声直接放大。 */
    if (pid->cfg.derivative_lpf_alpha > 0.0f) {
        raw_derivative = pid->prev_derivative +
            pid->cfg.derivative_lpf_alpha * (raw_derivative - pid->prev_derivative);
    }

    return raw_derivative;
}

void pid_init(pid_handle_t *pid, const pid_config_t *cfg, pid_mode_t mode)
{
    memset(pid, 0, sizeof(*pid));
    pid->cfg = *cfg;
    pid->mode = mode;
    pid->initialized = true;
}

void pid_reset(pid_handle_t *pid)
{
    float kp = pid->cfg.kp;
    float ki = pid->cfg.ki;
    float kd = pid->cfg.kd;
    pid_config_t cfg = pid->cfg;
    pid_mode_t mode = pid->mode;

    memset(pid, 0, sizeof(*pid));
    pid->cfg = cfg;
    pid->mode = mode;
    pid->cfg.kp = kp;
    pid->cfg.ki = ki;
    pid->cfg.kd = kd;
    pid->initialized = true;
}

float pid_update(pid_handle_t *pid, float ref, float fdb)
{
    float error;
    float derivative;
    float proportional;
    float output;

    ref = pid_apply_ramp(pid, ref);
    error = ref - fdb;

    if ((pid->cfg.enable_deadband) && (math_absf(error) < pid->cfg.deadband)) {
        error = 0.0f;
    }

    derivative = pid_compute_derivative(pid, error, fdb);

    /* 积分分离只在误差进入可控区间后累加，减少大偏差时积分饱和。 */
    if (pid_integral_enabled(pid, error)) {
        pid->integrator += pid->cfg.ki * error * pid->cfg.dt_s;
        pid->integrator = pid_limit_integral(pid, pid->integrator);
    }

    proportional = pid->cfg.kp * error;

    if (pid->mode == PID_MODE_POSITION) {
        output = proportional + pid->integrator + pid->cfg.kd * derivative;
        output = pid_limit_output(pid, output);
    } else {
        /* 增量式输出的是“本周期增量”，适合部分执行器做平滑更新。 */
        float delta_p = pid->cfg.kp * (error - pid->prev_error);
        float delta_i = pid_integral_enabled(pid, error) ? (pid->cfg.ki * error * pid->cfg.dt_s) : 0.0f;
        float delta_d = pid->cfg.kd * (derivative - pid->prev_derivative);
        output = pid->output + delta_p + delta_i + delta_d;
        output = pid_limit_output(pid, output);
    }

    pid->error = error;
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_feedback = fdb;
    pid->derivative = derivative;
    pid->prev_derivative = derivative;
    pid->output = output;

    return output;
}

void cascade_pid_init(cascade_pid_t *cascade,
    const pid_config_t *outer_cfg,
    const pid_config_t *inner_cfg,
    pid_mode_t outer_mode,
    pid_mode_t inner_mode)
{
    memset(cascade, 0, sizeof(*cascade));
    pid_init(&cascade->outer_pid, outer_cfg, outer_mode);
    pid_init(&cascade->inner_pid, inner_cfg, inner_mode);
}

void cascade_pid_reset(cascade_pid_t *cascade)
{
    pid_reset(&cascade->outer_pid);
    pid_reset(&cascade->inner_pid);
    cascade->outer_ref = 0.0f;
    cascade->inner_ref = 0.0f;
    cascade->inner_fdb = 0.0f;
    cascade->output = 0.0f;
}

float cascade_pid_update(cascade_pid_t *cascade, float outer_ref, float outer_fdb, float inner_fdb)
{
    cascade->outer_ref = outer_ref;
    cascade->inner_fdb = inner_fdb;
    /* 串级结构中，外环先给出内环目标，再由内环驱动最终执行器。 */
    cascade->inner_ref = pid_update(&cascade->outer_pid, outer_ref, outer_fdb);
    cascade->output = pid_update(&cascade->inner_pid, cascade->inner_ref, inner_fdb);
    return cascade->output;
}
