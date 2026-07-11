#ifndef ALGO_PID_H
#define ALGO_PID_H

#include "common/types.h"

typedef enum {
    PID_MODE_POSITION = 0,
    PID_MODE_INCREMENTAL,
} pid_mode_t;

/* 统一收口所有调参项，避免散落宏定义 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float dt_s;
    float output_limit;
    float integral_limit;
    float integral_separation;
    float derivative_lpf_alpha;
    float setpoint_slew_rate;
    float deadband;
    bool derivative_on_measurement;
    bool enable_integral_separation;
    bool enable_output_limit;
    bool enable_integral_limit;
    bool enable_deadband;
    bool enable_setpoint_ramp;
} pid_config_t;

/* 运行态句柄，保留误差、积分、微分和输出缓存 */
typedef struct {
    pid_config_t cfg;
    pid_mode_t mode;
    float setpoint;
    float error;
    float prev_error;
    float prev_prev_error;
    float prev_feedback;
    float integrator;
    float derivative;
    float prev_derivative;
    float output;
    bool initialized;
} pid_handle_t;

/* 串级 PID：外环输出作为内环设定值 */
typedef struct {
    pid_handle_t outer_pid;
    pid_handle_t inner_pid;
    float outer_ref;
    float inner_ref;
    float inner_fdb;
    float output;
} cascade_pid_t;

void pid_init(pid_handle_t *pid, const pid_config_t *cfg, pid_mode_t mode);
void pid_reset(pid_handle_t *pid);
/* ref 为目标值，fdb 为反馈值，返回当前控制输出 */
float pid_update(pid_handle_t *pid, float ref, float fdb);

void cascade_pid_init(cascade_pid_t *cascade,
    const pid_config_t *outer_cfg,
    const pid_config_t *inner_cfg,
    pid_mode_t outer_mode,
    pid_mode_t inner_mode);
void cascade_pid_reset(cascade_pid_t *cascade);
/* outer_fdb 常为位置/误差反馈，inner_fdb 常为速度反馈 */
float cascade_pid_update(cascade_pid_t *cascade, float outer_ref, float outer_fdb, float inner_fdb);

#endif
