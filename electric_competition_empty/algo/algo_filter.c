#include "algo/algo_filter.h"

#include <string.h>

#include "common/math_util.h"

static void algo_filter_sort(float *data, uint8_t count)
{
    uint8_t i;
    uint8_t j;

    /* 窗口很小，直接用简单排序，代码体积和可读性都更合适。 */
    for (i = 0U; i < count; i++) {
        for (j = (uint8_t) (i + 1U); j < count; j++) {
            if (data[j] < data[i]) {
                float temp = data[i];
                data[i] = data[j];
                data[j] = temp;
            }
        }
    }
}

void lpf1_init(lpf1_handle_t *handle, float alpha, float init_value)
{
    handle->alpha = math_clampf(alpha, 0.0f, 1.0f);
    handle->initialized = true;
    handle->output = init_value;
}

float lpf1_update(lpf1_handle_t *handle, float input)
{
    if (!handle->initialized) {
        handle->output = input;
        handle->initialized = true;
        return input;
    }

    handle->output += handle->alpha * (input - handle->output);
    return handle->output;
}

void moving_average_init(moving_average_handle_t *handle, uint8_t window_size)
{
    memset(handle, 0, sizeof(*handle));
    handle->window_size = (window_size == 0U) ? 1U :
        ((window_size > ALGO_FILTER_MAX_WINDOW) ? ALGO_FILTER_MAX_WINDOW : window_size);
}

float moving_average_update(moving_average_handle_t *handle, float input)
{
    float sum = 0.0f;
    uint8_t i;

    /* 用循环数组保存最近窗口，避免动态内存和搬移数据。 */
    handle->buffer[handle->index] = input;
    handle->index = (uint8_t) ((handle->index + 1U) % handle->window_size);
    if (handle->count < handle->window_size) {
        handle->count++;
    }

    for (i = 0U; i < handle->count; i++) {
        sum += handle->buffer[i];
    }

    return sum / (float) handle->count;
}

void median_filter_init(median_filter_handle_t *handle, uint8_t window_size)
{
    memset(handle, 0, sizeof(*handle));
    handle->window_size = (window_size == 0U) ? 1U :
        ((window_size > ALGO_FILTER_MAX_WINDOW) ? ALGO_FILTER_MAX_WINDOW : window_size);
}

float median_filter_update(median_filter_handle_t *handle, float input)
{
    float sorted[ALGO_FILTER_MAX_WINDOW];

    /* 中值滤波先复制再排序，保持原窗口顺序不被破坏。 */
    handle->buffer[handle->index] = input;
    handle->index = (uint8_t) ((handle->index + 1U) % handle->window_size);
    if (handle->count < handle->window_size) {
        handle->count++;
    }

    memcpy(sorted, handle->buffer, sizeof(float) * handle->count);
    algo_filter_sort(sorted, handle->count);

    return sorted[handle->count / 2U];
}

void limit_filter_init(limit_filter_handle_t *handle, float limit_step, float init_value)
{
    handle->limit_step = math_absf(limit_step);
    handle->initialized = true;
    handle->output = init_value;
}

float limit_filter_update(limit_filter_handle_t *handle, float input)
{
    float delta;

    if (!handle->initialized) {
        handle->initialized = true;
        handle->output = input;
        return input;
    }

    delta = input - handle->output;
    delta = math_clampf(delta, -handle->limit_step, handle->limit_step);
    handle->output += delta;
    return handle->output;
}

void ramp_filter_init(ramp_filter_handle_t *handle, float slew_rate, float init_value)
{
    handle->slew_rate = math_absf(slew_rate);
    handle->initialized = true;
    handle->output = init_value;
}

float ramp_filter_update(ramp_filter_handle_t *handle, float input, float dt_s)
{
    float max_delta;
    float delta;

    if ((!handle->initialized) || (dt_s <= 0.0f)) {
        handle->initialized = true;
        handle->output = input;
        return input;
    }

    max_delta = handle->slew_rate * dt_s;
    delta = math_clampf(input - handle->output, -max_delta, max_delta);
    handle->output += delta;
    return handle->output;
}
