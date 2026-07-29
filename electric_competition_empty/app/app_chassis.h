#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "drivers/drv_encoder_ab.h"

typedef enum {
    APP_LINE_FOLLOW_TRACK = 0U,
    APP_LINE_FOLLOW_LOST = 1U,
} app_line_follow_state_t;

typedef struct {
    float left_speed_rps;
    float right_speed_rps;
    float left_target_rps;
    float right_target_rps;
    float line_error;
    float line_offset_m;
    float line_curvature_1pm;
    float travel_mm;
    uint8_t line_bits;
    app_line_follow_state_t line_state;
    bool line_lost;
    bool start_line_detected;
    bool enabled;
    float left_output;
    float right_output;
} chassis_snapshot_t;

void app_chassis_init(void);
void app_chassis_set_enabled(bool enabled);
void app_chassis_set_cruise_speed_mps(float speed_mps);
void app_chassis_reset_travel(void);
void app_chassis_line_task(void);
void app_chassis_control_task(float control_dt_s, float elapsed_s);
const chassis_snapshot_t *app_chassis_get_snapshot(void);
encoder_driver_t *app_chassis_get_encoder_driver(void);

#endif
