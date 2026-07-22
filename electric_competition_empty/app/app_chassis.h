#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "drivers/drv_encoder_ab.h"

typedef enum {
    APP_LINE_FOLLOW_TRACK = 0U,
    APP_LINE_FOLLOW_CORNER = 1U,
    APP_LINE_FOLLOW_LOST = 2U,
    APP_LINE_FOLLOW_RECOVER = 3U,
} app_line_follow_state_t;

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

#endif
