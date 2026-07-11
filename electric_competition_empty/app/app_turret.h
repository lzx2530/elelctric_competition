#ifndef APP_TURRET_H
#define APP_TURRET_H

#include "protocol/proto_k230.h"

typedef struct {
    bool target_valid;
    int16_t x_error;
    int16_t y_error;
    float yaw_cmd_hz;
    float pitch_cmd_hz;
} turret_snapshot_t;

void app_turret_init(void);
/* 由视觉协议层推送最新目标偏差 */
void app_turret_set_target(const k230_frame_t *frame);
/* 把视觉误差转换成双轴步进频率命令 */
void app_turret_control_task(float dt_s);
const turret_snapshot_t *app_turret_get_snapshot(void);

#endif
