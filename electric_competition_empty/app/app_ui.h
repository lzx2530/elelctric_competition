#ifndef APP_UI_H
#define APP_UI_H

#include "app/app_chassis.h"
#include "app/app_ball_control.h"
#include "app/app_mission.h"

void app_ui_init(void);
void app_ui_process(void);
/* 统一刷新 OLED 状态页，避免上层直接依赖显示驱动细节 */
void app_ui_refresh(const chassis_snapshot_t *chassis,
    const ball_control_snapshot_t *ball,
    const mission_snapshot_t *mission);

#endif
