#include "algo/algo_fusion.h"

#include <math.h>
#include <string.h>

#include "common/math_util.h"

#define RAD_TO_DEG    (57.2957795f)
#define DEG_TO_RAD    (0.0174532925f)

void fusion6_init(fusion6_handle_t *handle, const fusion6_config_t *cfg)
{
    memset(handle, 0, sizeof(*handle));
    handle->cfg = *cfg;
    if ((handle->cfg.gyro_filter_alpha <= 0.0f) ||
        (handle->cfg.gyro_filter_alpha > 1.0f)) {
        handle->cfg.gyro_filter_alpha = 1.0f;
    }
    if ((handle->cfg.yaw_rotate_enter_dps <= 0.0f) ||
        (handle->cfg.yaw_stationary_enter_dps <= 0.0f) ||
        (handle->cfg.yaw_stationary_enter_dps >= handle->cfg.yaw_rotate_enter_dps)) {
        handle->cfg.yaw_rotate_enter_dps = 3.0f;
        handle->cfg.yaw_stationary_enter_dps = 1.5f;
    }
    handle->cfg.yaw_bias_alpha = math_clampf(handle->cfg.yaw_bias_alpha, 0.0f, 1.0f);
    lpf1_init(&handle->gyro_x_filter, handle->cfg.gyro_filter_alpha, 0.0f);
    lpf1_init(&handle->gyro_y_filter, handle->cfg.gyro_filter_alpha, 0.0f);
    lpf1_init(&handle->gyro_z_filter, handle->cfg.gyro_filter_alpha, 0.0f);
    handle->initialized = true;
}

void fusion6_reset(fusion6_handle_t *handle)
{
    handle->roll_deg = 0.0f;
    handle->pitch_deg = 0.0f;
    handle->yaw_deg = 0.0f;
    handle->mag_yaw_reference_deg = 0.0f;
    handle->stable_yaw_deg = 0.0f;
    handle->yaw_gyro_bias_dps = 0.0f;
    lpf1_init(&handle->gyro_x_filter, handle->cfg.gyro_filter_alpha, 0.0f);
    lpf1_init(&handle->gyro_y_filter, handle->cfg.gyro_filter_alpha, 0.0f);
    lpf1_init(&handle->gyro_z_filter, handle->cfg.gyro_filter_alpha, 0.0f);
    handle->initialized = true;
    handle->mag_yaw_reference_valid = false;
    handle->yaw_rotating = false;
}

void fusion6_update(fusion6_handle_t *handle,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps)
{
    float accel_roll;
    float accel_pitch;
    float yaw_rate_dps;
    float accel_weight = math_clampf(handle->cfg.accel_weight, 0.0f, 1.0f);
    float gyro_weight = 1.0f - accel_weight;

    if (!handle->initialized) {
        handle->initialized = true;
    }

    /* Use accelerometer tilt as the long-term reference for roll/pitch drift correction. */
    accel_roll = atan2f(ay_g, az_g) * RAD_TO_DEG;
    accel_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;
    gx_dps = lpf1_update(&handle->gyro_x_filter, gx_dps);
    gy_dps = lpf1_update(&handle->gyro_y_filter, gy_dps);
    gz_dps = lpf1_update(&handle->gyro_z_filter, gz_dps);

    /* Integrate gyro first, then blend back toward the low-frequency accel estimate. */
    handle->roll_deg += gx_dps * handle->cfg.dt_s;
    handle->pitch_deg += gy_dps * handle->cfg.dt_s;
    if (!handle->cfg.enable_yaw_stationary_lock) {
        handle->yaw_deg += gz_dps * handle->cfg.dt_s;
    } else {
        yaw_rate_dps = gz_dps - handle->yaw_gyro_bias_dps;
        if (handle->yaw_rotating) {
            if (math_absf(yaw_rate_dps) < handle->cfg.yaw_stationary_enter_dps) {
                handle->yaw_rotating = false;
                handle->stable_yaw_deg = handle->yaw_deg;
            } else {
                handle->yaw_deg += yaw_rate_dps * handle->cfg.dt_s;
            }
        } else if (math_absf(yaw_rate_dps) > handle->cfg.yaw_rotate_enter_dps) {
            handle->yaw_rotating = true;
            handle->yaw_deg += yaw_rate_dps * handle->cfg.dt_s;
        } else {
            /* Zero-rate update: stationary samples refine the residual gyro bias. */
            handle->yaw_gyro_bias_dps += handle->cfg.yaw_bias_alpha *
                (gz_dps - handle->yaw_gyro_bias_dps);
            handle->yaw_deg = handle->stable_yaw_deg;
        }
    }

    handle->roll_deg = gyro_weight * handle->roll_deg + accel_weight * accel_roll;
    handle->pitch_deg = gyro_weight * handle->pitch_deg + accel_weight * accel_pitch;
    handle->yaw_deg = math_wrap_deg(handle->yaw_deg);
}

void fusion6_update_9axis(fusion6_handle_t *handle,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps,
    float mx_uT, float my_uT, float mz_uT)
{
    float roll_rad;
    float pitch_rad;
    float horizontal_x;
    float horizontal_y;
    float magnetic_yaw_deg;
    float yaw_error_deg;
    float yaw_correction_weight;

    fusion6_update(handle, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);

    roll_rad = handle->roll_deg * DEG_TO_RAD;
    pitch_rad = handle->pitch_deg * DEG_TO_RAD;
    horizontal_x = mx_uT * cosf(pitch_rad) + mz_uT * sinf(pitch_rad);
    horizontal_y = mx_uT * sinf(roll_rad) * sinf(pitch_rad) +
        my_uT * cosf(roll_rad) - mz_uT * sinf(roll_rad) * cosf(pitch_rad);
    if ((horizontal_x * horizontal_x + horizontal_y * horizontal_y) < 1.0e-6f) {
        return;
    }

    magnetic_yaw_deg = atan2f(horizontal_y, horizontal_x) * RAD_TO_DEG;
    if (!handle->mag_yaw_reference_valid) {
        handle->mag_yaw_reference_deg = magnetic_yaw_deg;
        handle->mag_yaw_reference_valid = true;
        return;
    }

    yaw_correction_weight = math_clampf(handle->cfg.yaw_correction_weight, 0.0f, 1.0f);
    yaw_error_deg = math_wrap_deg(magnetic_yaw_deg - handle->mag_yaw_reference_deg - handle->yaw_deg);
    handle->yaw_deg = math_wrap_deg(handle->yaw_deg + yaw_correction_weight * yaw_error_deg);
}
