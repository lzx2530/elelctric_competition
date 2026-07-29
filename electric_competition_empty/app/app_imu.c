#include "app/app_imu.h"

#include "algo/algo_fusion.h"
#include "algo/algo_filter.h"
#include "drivers/drv_mpu9250.h"

typedef struct {
    mpu9250_handle_t imu;
    fusion6_handle_t fusion;
    lpf1_handle_t longitudinal_accel_filter;
    float longitudinal_accel_bias_g;
    uint8_t accel_bias_samples;
    imu_snapshot_t snapshot;
} imu_app_t;

static imu_app_t g_imu;

void app_imu_init(void)
{
    static const fusion6_config_t fusion_cfg = {
        .dt_s = 0.01f,
        .accel_weight = 0.02f,
        .yaw_correction_weight = 0.0f,
    };

    fusion6_init(&g_imu.fusion, &fusion_cfg);
    lpf1_init(&g_imu.longitudinal_accel_filter, 0.20F, 0.0F);
    g_imu.snapshot.online = (mpu9250_init(&g_imu.imu, 0x68U) == STATUS_OK);
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

    if (g_imu.accel_bias_samples < 100U) {
        float samples = (float) g_imu.accel_bias_samples;
        g_imu.longitudinal_accel_bias_g =
            (g_imu.longitudinal_accel_bias_g * samples + g_imu.imu.accel_g.x) /
            (samples + 1.0F);
        g_imu.accel_bias_samples++;
    }

    g_imu.snapshot.roll_deg = g_imu.fusion.roll_deg;
    g_imu.snapshot.pitch_deg = g_imu.fusion.pitch_deg;
    g_imu.snapshot.yaw_deg = g_imu.fusion.yaw_deg;
    g_imu.snapshot.gyro_z_dps = g_imu.imu.gyro_dps.z;
    g_imu.snapshot.longitudinal_accel_mps2 = lpf1_update(
        &g_imu.longitudinal_accel_filter,
        (g_imu.imu.accel_g.x - g_imu.longitudinal_accel_bias_g) * 9.80665F);
}

const imu_snapshot_t *app_imu_get_snapshot(void)
{
    return &g_imu.snapshot;
}
