#ifndef DRV_ENCODER_AB_H
#define DRV_ENCODER_AB_H

#include "common/types.h"
#include <ti/devices/msp/msp.h>

typedef struct {
    float counts_per_revolution;
    bool invert_direction;
} encoder_config_t;

/* 单个编码器句柄，同时缓存计数、增量和速度估计 */
typedef struct {
    encoder_config_t cfg;
    volatile int32_t count;
    int32_t prev_count;
    int32_t delta_count;
    float speed_rps;
    uint8_t prev_state;
} encoder_handle_t;

typedef struct {
    encoder_handle_t left;
    encoder_handle_t right;
} encoder_driver_t;

void encoder_driver_init(encoder_driver_t *driver, const encoder_config_t *left_cfg, const encoder_config_t *right_cfg);
/* 在 GPIO 中断中调用，按端口和 IIDX 分发到左右编码器 */
void encoder_driver_handle_gpio_interrupt(encoder_driver_t *driver, GPIO_Regs *port, uint32_t pin_iidx);
/* 在固定周期任务中调用，把累计计数换算成速度 */
void encoder_driver_update_speed(encoder_driver_t *driver, float dt_s);

#endif
