#ifndef APP_IMU_H
#define APP_IMU_H

#include "common/types.h"

typedef struct {
    bool online;
    bool accel_bias_ready;
    uint8_t who_am_i;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_z_dps;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float longitudinal_accel_mps2;
} imu_snapshot_t;

void app_imu_init(void);
/* 周期采样 MPU9250 并刷新姿态融合结果 */
void app_imu_task(void);
const imu_snapshot_t *app_imu_get_snapshot(void);

#endif
