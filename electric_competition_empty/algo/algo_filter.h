#ifndef ALGO_FILTER_H
#define ALGO_FILTER_H

#include "common/types.h"

#define ALGO_FILTER_MAX_WINDOW    (9U)

typedef struct {
    float alpha;
    bool initialized;
    float output;
} lpf1_handle_t;

/* 滑动均值和中值滤波窗口都使用固定大小数组，避免动态内存 */
typedef struct {
    float buffer[ALGO_FILTER_MAX_WINDOW];
    uint8_t window_size;
    uint8_t index;
    uint8_t count;
} moving_average_handle_t;

typedef struct {
    float buffer[ALGO_FILTER_MAX_WINDOW];
    uint8_t window_size;
    uint8_t index;
    uint8_t count;
} median_filter_handle_t;

typedef struct {
    float limit_step;
    bool initialized;
    float output;
} limit_filter_handle_t;

typedef struct {
    float slew_rate;
    bool initialized;
    float output;
} ramp_filter_handle_t;

/* alpha 越大响应越快，越小越平滑 */
void lpf1_init(lpf1_handle_t *handle, float alpha, float init_value);
float lpf1_update(lpf1_handle_t *handle, float input);

void moving_average_init(moving_average_handle_t *handle, uint8_t window_size);
float moving_average_update(moving_average_handle_t *handle, float input);

void median_filter_init(median_filter_handle_t *handle, uint8_t window_size);
float median_filter_update(median_filter_handle_t *handle, float input);

void limit_filter_init(limit_filter_handle_t *handle, float limit_step, float init_value);
float limit_filter_update(limit_filter_handle_t *handle, float input);

/* slew_rate 单位为“每秒允许变化量” */
void ramp_filter_init(ramp_filter_handle_t *handle, float slew_rate, float init_value);
float ramp_filter_update(ramp_filter_handle_t *handle, float input, float dt_s);

#endif
