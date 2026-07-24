#ifndef ALGO_FUSION_H
#define ALGO_FUSION_H

#include "algo/algo_filter.h"
#include "common/types.h"

typedef struct {
    float dt_s;
    float accel_weight;
    float yaw_correction_weight;
    /* (0, 1], smaller values smooth more; 1.0 disables gyro filtering. */
    float gyro_filter_alpha;
    bool enable_yaw_stationary_lock;
    float yaw_rotate_enter_dps;
    float yaw_stationary_enter_dps;
    float yaw_bias_alpha;
} fusion6_config_t;

typedef struct {
    fusion6_config_t cfg;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float mag_yaw_reference_deg;
    float stable_yaw_deg;
    float yaw_gyro_bias_dps;
    lpf1_handle_t gyro_x_filter;
    lpf1_handle_t gyro_y_filter;
    lpf1_handle_t gyro_z_filter;
    bool initialized;
    bool mag_yaw_reference_valid;
    bool yaw_rotating;
} fusion6_handle_t;

void fusion6_init(fusion6_handle_t *handle, const fusion6_config_t *cfg);
void fusion6_reset(fusion6_handle_t *handle);
/* 输入单位固定为 g 和 dps，输出欧拉角单位为 deg */
void fusion6_update(fusion6_handle_t *handle,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps);
void fusion6_update_9axis(fusion6_handle_t *handle,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps,
    float mx_uT, float my_uT, float mz_uT);

#endif
