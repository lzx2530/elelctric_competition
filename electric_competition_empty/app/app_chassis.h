#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "drivers/drv_encoder_ab.h"

typedef enum {
    APP_LINE_FOLLOW_TRACK = 0U,
    APP_LINE_FOLLOW_CORNER = 1U,
    APP_LINE_FOLLOW_LOST = 2U,
    APP_LINE_FOLLOW_RECOVER = 3U,
} app_line_follow_state_t;

typedef enum {
    APP_CHASSIS_MODE_LINE_FOLLOW = 0U,
    APP_CHASSIS_MODE_EXTERNAL = 1U,
    APP_CHASSIS_MODE_STOP = 2U,
} app_chassis_mode_t;

/* Chassis telemetry snapshot for UI, VOFA+, and debug output. */
typedef struct {
    float left_speed_rps;
    float right_speed_rps;
    float left_target_rps;
    float right_target_rps;
    float line_error;
    float line_offset_m;
    float line_curvature_1pm;
    uint8_t line_bits;
    app_line_follow_state_t line_state;
    bool line_lost;
    float left_output;
    float right_output;
} chassis_snapshot_t;

void app_chassis_init(void);
/* Low-rate task: read grayscale sensors and update line offset. */
void app_chassis_line_task(void);
/* High-rate task: update wheel speed estimates and run left/right speed loops. */
void app_chassis_control_task(float control_dt_s, float elapsed_s);
const chassis_snapshot_t *app_chassis_get_snapshot(void);
encoder_driver_t *app_chassis_get_encoder_driver(void);
void app_chassis_enable_line_follow(void);
void app_chassis_set_external_drive(float forward_rps, float yaw_rps);
void app_chassis_stop(void);
void app_chassis_get_encoder_counts(int32_t *left, int32_t *right);
float app_chassis_get_average_distance_mm(int32_t left_start, int32_t right_start);

#endif
