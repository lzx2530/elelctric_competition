#ifndef BSP_SPI_IMU_H
#define BSP_SPI_IMU_H

#include "common/status.h"
#include "common/types.h"

void bsp_spi_imu_init(void);
status_t bsp_spi_imu_write(uint8_t reg_addr, const uint8_t *data, uint16_t length);
status_t bsp_spi_imu_read(uint8_t reg_addr, uint8_t *data, uint16_t length);

#endif
