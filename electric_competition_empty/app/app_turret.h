#ifndef APP_TURRET_H
#define APP_TURRET_H

#include "protocol/proto_k230.h"

typedef struct {
    bool target_valid;
    float x_error_cm;
    float y_error_cm;
    float yaw_cmd_hz;
    float pitch_cmd_hz;
} turret_snapshot_t;

void app_turret_init(void);
/* Push the newest K230 vision target into the turret controller. */
void app_turret_set_target(const k230_frame_t *frame);
/* Convert vision error into two stepper speed commands. */
void app_turret_control_task(float dt_s);
const turret_snapshot_t *app_turret_get_snapshot(void);

#endif
