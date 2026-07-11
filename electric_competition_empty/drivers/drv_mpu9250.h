#ifndef DRV_MPU9250_H
#define DRV_MPU9250_H

#include "common/status.h"
#include "common/types.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu9250_vec3i16_t;

typedef struct {
    float x;
    float y;
    float z;
} mpu9250_vec3f_t;

/* 当前只面向六轴使用，磁力计与 DMP 不在这个句柄中维护 */
typedef struct {
    uint8_t i2c_addr;
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;
    mpu9250_vec3f_t gyro_bias_dps;
    mpu9250_vec3i16_t accel_raw;
    mpu9250_vec3i16_t gyro_raw;
    int16_t temp_raw;
    mpu9250_vec3f_t accel_g;
    mpu9250_vec3f_t gyro_dps;
} mpu9250_handle_t;

status_t mpu9250_init(mpu9250_handle_t *handle, uint8_t i2c_addr);
status_t mpu9250_read_who_am_i(mpu9250_handle_t *handle, uint8_t *who_am_i);
status_t mpu9250_read_raw(mpu9250_handle_t *handle);
/* update 会同时刷新 raw、物理量换算值和陀螺仪偏置补偿结果 */
status_t mpu9250_update(mpu9250_handle_t *handle);
/* 标定时应保持静止，sample_count 越大结果越稳但耗时更长 */
status_t mpu9250_calibrate_gyro_bias(mpu9250_handle_t *handle, uint16_t sample_count);

#endif
