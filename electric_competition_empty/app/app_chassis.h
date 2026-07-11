#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "drivers/drv_encoder_ab.h"

/* 供 UI/VOFA+ 读取的底盘观测快照 */
typedef struct {
    float left_speed_rps;
    float right_speed_rps;
    float left_target_rps;
    float right_target_rps;
    float line_error;
    uint8_t line_bits;
    bool line_lost;
    float left_output;
    float right_output;
} chassis_snapshot_t;

void app_chassis_init(void);
/* 低频任务：读取灰度并更新巡线偏差 */
void app_chassis_line_task(void);
/* 高频任务：更新轮速估计并运行左右轮速度环 */
void app_chassis_control_task(float dt_s);
const chassis_snapshot_t *app_chassis_get_snapshot(void);
encoder_driver_t *app_chassis_get_encoder_driver(void);

#endif
