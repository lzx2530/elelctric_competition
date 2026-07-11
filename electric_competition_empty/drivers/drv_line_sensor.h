#ifndef DRV_LINE_SENSOR_H
#define DRV_LINE_SENSOR_H

#include "common/types.h"

typedef struct {
    bool active_high;
    float weights[8];
} line_sensor_config_t;

/* line_error 为加权平均后的巡线偏差，line_lost 表示 8 路均未命中 */
typedef struct {
    line_sensor_config_t cfg;
    uint8_t raw_bits;
    uint8_t hit_count;
    float line_error;
    bool line_lost;
} line_sensor_handle_t;

void line_sensor_init(line_sensor_handle_t *handle, const line_sensor_config_t *cfg);
void line_sensor_update(line_sensor_handle_t *handle);

#endif
