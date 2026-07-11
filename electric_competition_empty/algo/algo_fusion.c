#include "algo/algo_fusion.h"

#include <math.h>
#include <string.h>

#include "common/math_util.h"

#define RAD_TO_DEG    (57.2957795f)

void fusion6_init(fusion6_handle_t *handle, const fusion6_config_t *cfg)
{
    memset(handle, 0, sizeof(*handle));
    handle->cfg = *cfg;
    handle->initialized = true;
}

void fusion6_reset(fusion6_handle_t *handle)
{
    handle->roll_deg = 0.0f;
    handle->pitch_deg = 0.0f;
    handle->yaw_deg = 0.0f;
    handle->initialized = true;
}

void fusion6_update(fusion6_handle_t *handle,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps)
{
    float accel_roll;
    float accel_pitch;
    float accel_weight = math_clampf(handle->cfg.accel_weight, 0.0f, 1.0f);
    float gyro_weight = 1.0f - accel_weight;

    if (!handle->initialized) {
        handle->initialized = true;
    }

    /* Use accelerometer tilt as the long-term reference for roll/pitch drift correction. */
    accel_roll = atan2f(ay_g, az_g) * RAD_TO_DEG;
    accel_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;

    /* Integrate gyro first, then blend back toward the low-frequency accel estimate. */
    handle->roll_deg += gx_dps * handle->cfg.dt_s;
    handle->pitch_deg += gy_dps * handle->cfg.dt_s;
    handle->yaw_deg += gz_dps * handle->cfg.dt_s;

    handle->roll_deg = gyro_weight * handle->roll_deg + accel_weight * accel_roll;
    handle->pitch_deg = gyro_weight * handle->pitch_deg + accel_weight * accel_pitch;
    handle->yaw_deg = math_wrap_deg(handle->yaw_deg);
}
