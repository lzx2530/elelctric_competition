#ifndef APP_UI_H
#define APP_UI_H

#include "app/app_chassis.h"
#include "app/app_imu.h"
#include "app/app_turret.h"

void app_ui_init(void);
/* 统一刷新 OLED 状态页，避免上层直接依赖显示驱动细节 */
void app_ui_refresh(const chassis_snapshot_t *chassis,
    const turret_snapshot_t *turret,
    const imu_snapshot_t *imu);

#endif
