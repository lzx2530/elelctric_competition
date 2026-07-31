#include "app/app_imu.h"

#include "algo/algo_fusion.h"
#include "algo/algo_filter.h"
#include "common/math_util.h"
#include "drivers/drv_mpu9250.h"

#define APP_IMU_ACCEL_BIAS_SAMPLES              (200U)
#define APP_IMU_STATIONARY_GYRO_DPS             (3.0F)
#define APP_IMU_STATIONARY_ACCEL_NORM_SQ_MIN    (0.90F)
#define APP_IMU_STATIONARY_ACCEL_NORM_SQ_MAX    (1.10F)

typedef struct {
    mpu9250_handle_t imu;
    fusion6_handle_t fusion;
    lpf1_handle_t longitudinal_accel_filter;
    float longitudinal_accel_bias_g;
    float longitudinal_accel_bias_sum_g;
    uint16_t accel_bias_samples;
    imu_snapshot_t snapshot;
} imu_app_t;

static imu_app_t g_imu;

static bool app_imu_is_stationary(void)
{
    float accel_norm_sq = g_imu.imu.accel_g.x * g_imu.imu.accel_g.x +
        g_imu.imu.accel_g.y * g_imu.imu.accel_g.y +
        g_imu.imu.accel_g.z * g_imu.imu.accel_g.z;

    return (accel_norm_sq >= APP_IMU_STATIONARY_ACCEL_NORM_SQ_MIN) &&
        (accel_norm_sq <= APP_IMU_STATIONARY_ACCEL_NORM_SQ_MAX) &&
        (math_absf(g_imu.imu.gyro_dps.x) <= APP_IMU_STATIONARY_GYRO_DPS) &&
        (math_absf(g_imu.imu.gyro_dps.y) <= APP_IMU_STATIONARY_GYRO_DPS) &&
        (math_absf(g_imu.imu.gyro_dps.z) <= APP_IMU_STATIONARY_GYRO_DPS);
}

void app_imu_init(void)
{
    static const fusion6_config_t fusion_cfg = {
        .dt_s = 0.01f,
        .accel_weight = 0.02f,
        .yaw_correction_weight = 0.0f,
    };

    fusion6_init(&g_imu.fusion, &fusion_cfg);
    lpf1_init(&g_imu.longitudinal_accel_filter, 0.20F, 0.0F);
    g_imu.longitudinal_accel_bias_g = 0.0F;
    g_imu.longitudinal_accel_bias_sum_g = 0.0F;
    g_imu.accel_bias_samples = 0U;
    g_imu.snapshot.accel_bias_ready = false;
    g_imu.snapshot.online = (mpu9250_init(&g_imu.imu) == STATUS_OK);
    if (g_imu.snapshot.online) {
        /* Bring-up keeps the online flag strict so later tasks can fail fast. */
        g_imu.snapshot.online = (mpu9250_read_who_am_i(&g_imu.imu, &g_imu.snapshot.who_am_i) == STATUS_OK);
        if (g_imu.snapshot.online) {
            (void) mpu9250_calibrate_gyro_bias(&g_imu.imu, 32U);
        }
    }
}

void app_imu_task(void)
{
    if (!g_imu.snapshot.online) {
        return;
    }

    if (mpu9250_update(&g_imu.imu) != STATUS_OK) {
        g_imu.snapshot.online = false;
        return;
    }

    /* V1 uses a lightweight complementary fusion path instead of a full AHRS stack. */
    fusion6_update(&g_imu.fusion,
        g_imu.imu.accel_g.x,
        g_imu.imu.accel_g.y,
        g_imu.imu.accel_g.z,
        g_imu.imu.gyro_dps.x,
        g_imu.imu.gyro_dps.y,
        g_imu.imu.gyro_dps.z);

    if (!g_imu.snapshot.accel_bias_ready) {
        if (app_imu_is_stationary()) {
            g_imu.longitudinal_accel_bias_sum_g += -g_imu.imu.accel_g.y;
            g_imu.accel_bias_samples++;
            if (g_imu.accel_bias_samples >= APP_IMU_ACCEL_BIAS_SAMPLES) {
                g_imu.longitudinal_accel_bias_g =
                    g_imu.longitudinal_accel_bias_sum_g / (float) g_imu.accel_bias_samples;
                g_imu.snapshot.accel_bias_ready = true;
            }
        } else {
            g_imu.longitudinal_accel_bias_sum_g = 0.0F;
            g_imu.accel_bias_samples = 0U;
        }
    }

    g_imu.snapshot.roll_deg = g_imu.fusion.roll_deg;
    g_imu.snapshot.pitch_deg = g_imu.fusion.pitch_deg;
    g_imu.snapshot.yaw_deg = g_imu.fusion.yaw_deg;
    g_imu.snapshot.gyro_z_dps = g_imu.imu.gyro_dps.z;
    g_imu.snapshot.accel_x_g = g_imu.imu.accel_g.x;
    g_imu.snapshot.accel_y_g = g_imu.imu.accel_g.y;
    g_imu.snapshot.accel_z_g = g_imu.imu.accel_g.z;
    if (g_imu.snapshot.accel_bias_ready) {
        g_imu.snapshot.longitudinal_accel_mps2 = lpf1_update(
            &g_imu.longitudinal_accel_filter,
            (-g_imu.imu.accel_g.y - g_imu.longitudinal_accel_bias_g) * 9.80665F);
    } else {
        g_imu.snapshot.longitudinal_accel_mps2 = 0.0F;
    }
}

const imu_snapshot_t *app_imu_get_snapshot(void)
{
    return &g_imu.snapshot;
}
