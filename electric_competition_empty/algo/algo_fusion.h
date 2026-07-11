#ifndef ALGO_FUSION_H
#define ALGO_FUSION_H

#include "common/types.h"

typedef struct {
    float dt_s;
    float accel_weight;
    float yaw_correction_weight;
} fusion6_config_t;

typedef struct {
    fusion6_config_t cfg;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    bool initialized;
} fusion6_handle_t;

void fusion6_init(fusion6_handle_t *handle, const fusion6_config_t *cfg);
void fusion6_reset(fusion6_handle_t *handle);
/* 输入单位固定为 g 和 dps，输出欧拉角单位为 deg */
void fusion6_update(fusion6_handle_t *handle,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps);

#endif
