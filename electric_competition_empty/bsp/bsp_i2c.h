#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "common/status.h"
#include "common/types.h"

void bsp_i2c_init(void);
/* 用于设备探测；成功只表示地址应答正常 */
status_t bsp_i2c_probe(uint8_t dev_addr);
status_t bsp_i2c_write_bytes(uint8_t dev_addr, const uint8_t *data, uint16_t length);
/* mem_read/mem_write 面向常见寄存器设备，reg_addr 为 8bit 寄存器地址 */
status_t bsp_i2c_mem_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t length);
status_t bsp_i2c_mem_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length);

#endif
