#ifndef APP_MISSION_H
#define APP_MISSION_H

#include "app/app_imu.h"

typedef enum {
    APP_MISSION_VIDEO_RECORD = 1U,
    APP_MISSION_LINE_LOOP = 2U,
    APP_MISSION_STATIC_SWEEP = 3U,
    APP_MISSION_AB_CENTER = 4U,
    APP_MISSION_LOOP_CENTER = 5U,
    APP_MISSION_LOOP_HOLD = 6U,
} app_mission_mode_t;

typedef enum {
    APP_MISSION_IDLE = 0U,
    APP_MISSION_ARMING = 1U,
    APP_MISSION_RUNNING = 2U,
    APP_MISSION_FINISHED = 3U,
    APP_MISSION_FAULT = 4U,
} app_mission_state_t;

typedef struct {
    app_mission_mode_t mode;
    app_mission_state_t state;
    uint32_t elapsed_ms;
    int16_t target_mm;
    uint8_t fault_code;
} mission_snapshot_t;

void app_mission_init(void);
void app_mission_next_mode(void);
void app_mission_start(uint32_t tick_ms);
void app_mission_start_from_k230(uint8_t task_flag, uint32_t tick_ms);
void app_mission_abort(void);
void app_mission_task(const imu_snapshot_t *imu, uint32_t tick_ms);
const mission_snapshot_t *app_mission_get_snapshot(void);

#endif
