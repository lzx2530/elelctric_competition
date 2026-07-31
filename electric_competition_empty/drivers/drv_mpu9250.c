#include "drivers/drv_mpu9250.h"

#include <string.h>

#include "bsp/bsp_spi_imu.h"
#include <ti/driverlib/dl_common.h>
#include <ti/driverlib/m0p/dl_core.h>

#define MPU9250_REG_WHO_AM_I       (0x75U)
#define MPU9250_REG_PWR_MGMT_1     (0x6BU)
#define MPU9250_REG_PWR_MGMT_2     (0x6CU)
#define MPU9250_REG_CONFIG         (0x1AU)
#define MPU9250_REG_GYRO_CONFIG    (0x1BU)
#define MPU9250_REG_ACCEL_CONFIG   (0x1CU)
#define MPU9250_REG_ACCEL_CONFIG2  (0x1DU)
#define MPU9250_REG_ACCEL_XOUT_H   (0x3BU)

status_t mpu9250_init(mpu9250_handle_t *handle)
{
    uint8_t value;

    memset(handle, 0, sizeof(*handle));
    /* 起步版固定用 ±2g / ±250dps，换算简单，调试也直观。 */
    handle->accel_lsb_per_g = 16384.0f;
    handle->gyro_lsb_per_dps = 131.0f;

    value = 0x80U;
    if (bsp_spi_imu_write(MPU9250_REG_PWR_MGMT_1, &value, 1U) != STATUS_OK) {
        return STATUS_ERROR;
    }
    delay_cycles(320000U);

    value = 0x01U;
    if (bsp_spi_imu_write(MPU9250_REG_PWR_MGMT_1, &value, 1U) != STATUS_OK) {
        return STATUS_ERROR;
    }

    value = 0x00U;
    if (bsp_spi_imu_write(MPU9250_REG_PWR_MGMT_2, &value, 1U) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (bsp_spi_imu_write(MPU9250_REG_CONFIG, &value, 1U) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (bsp_spi_imu_write(MPU9250_REG_GYRO_CONFIG, &value, 1U) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (bsp_spi_imu_write(MPU9250_REG_ACCEL_CONFIG, &value, 1U) != STATUS_OK) {
        return STATUS_ERROR;
    }
    value = 0x03U;
    if (bsp_spi_imu_write(MPU9250_REG_ACCEL_CONFIG2, &value, 1U) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

status_t mpu9250_read_who_am_i(mpu9250_handle_t *handle, uint8_t *who_am_i)
{
    return bsp_spi_imu_read(MPU9250_REG_WHO_AM_I, who_am_i, 1U);
}

status_t mpu9250_read_raw(mpu9250_handle_t *handle)
{
    uint8_t raw[14];

    if (bsp_spi_imu_read(MPU9250_REG_ACCEL_XOUT_H, raw, sizeof(raw)) != STATUS_OK) {
        return STATUS_ERROR;
    }

    handle->accel_raw.x = (int16_t) ((raw[0] << 8) | raw[1]);
    handle->accel_raw.y = (int16_t) ((raw[2] << 8) | raw[3]);
    handle->accel_raw.z = (int16_t) ((raw[4] << 8) | raw[5]);
    handle->temp_raw = (int16_t) ((raw[6] << 8) | raw[7]);
    handle->gyro_raw.x = (int16_t) ((raw[8] << 8) | raw[9]);
    handle->gyro_raw.y = (int16_t) ((raw[10] << 8) | raw[11]);
    handle->gyro_raw.z = (int16_t) ((raw[12] << 8) | raw[13]);
    return STATUS_OK;
}

status_t mpu9250_update(mpu9250_handle_t *handle)
{
    if (mpu9250_read_raw(handle) != STATUS_OK) {
        return STATUS_ERROR;
    }

    /* 先换算物理量，再减去静态零偏，给上层姿态融合直接使用。 */
    handle->accel_g.x = (float) handle->accel_raw.x / handle->accel_lsb_per_g;
    handle->accel_g.y = (float) handle->accel_raw.y / handle->accel_lsb_per_g;
    handle->accel_g.z = (float) handle->accel_raw.z / handle->accel_lsb_per_g;

    handle->gyro_dps.x = (float) handle->gyro_raw.x / handle->gyro_lsb_per_dps - handle->gyro_bias_dps.x;
    handle->gyro_dps.y = (float) handle->gyro_raw.y / handle->gyro_lsb_per_dps - handle->gyro_bias_dps.y;
    handle->gyro_dps.z = (float) handle->gyro_raw.z / handle->gyro_lsb_per_dps - handle->gyro_bias_dps.z;

    return STATUS_OK;
}

status_t mpu9250_calibrate_gyro_bias(mpu9250_handle_t *handle, uint16_t sample_count)
{
    mpu9250_vec3f_t sum = {0};

    if (sample_count == 0U) {
        return STATUS_INVALID_ARG;
    }

    for (uint16_t i = 0U; i < sample_count; i++) {
        if (mpu9250_update(handle) != STATUS_OK) {
            return STATUS_ERROR;
        }
        /* 这里假定标定期间静止，所以均值可直接作为陀螺仪零偏。 */
        sum.x += handle->gyro_dps.x;
        sum.y += handle->gyro_dps.y;
        sum.z += handle->gyro_dps.z;
        delay_cycles(32000U);
    }

    handle->gyro_bias_dps.x = sum.x / (float) sample_count;
    handle->gyro_bias_dps.y = sum.y / (float) sample_count;
    handle->gyro_bias_dps.z = sum.z / (float) sample_count;
    return STATUS_OK;
}
