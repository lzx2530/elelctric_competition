#ifndef STEELBALL_TASK_H
#define STEELBALL_TASK_H

#include "common/types.h"

typedef struct {
    uint8_t mission_id;
    uint8_t vision_state;
    uint8_t vision_flags;
    int16_t center_x_permille;
    int16_t center_y_permille;
    uint16_t ball_diameter_px;
    uint16_t confidence_permille;
    uint8_t stable_frames;
    uint8_t target_count;
} steelball_vision_snapshot_t;

void steelball_init(void);
void steelball_rx_bytes(const uint8_t *data, uint16_t length);
void steelball_task_10ms(uint32_t now_ms);
bool steelball_start_mission(void);
void steelball_abort_mission(void);
void steelball_reset_mission(void);
uint8_t steelball_get_state(void);
void steelball_get_vision_snapshot(steelball_vision_snapshot_t *snapshot);

#endif
