#include "drivers/drv_line_sensor.h"

#include "bsp/bsp_gpio.h"

void line_sensor_init(line_sensor_handle_t *handle, const line_sensor_config_t *cfg)
{
    handle->cfg = *cfg;
    handle->raw_bits = 0U;
    handle->hit_count = 0U;
    handle->line_error = 0.0f;
    handle->line_lost = true;
}

void line_sensor_update(line_sensor_handle_t *handle)
{
    uint8_t raw = bsp_gpio_read_line_bits();
    float sum = 0.0f;
    uint8_t hits = 0U;

    handle->raw_bits = raw;
    for (uint8_t i = 0U; i < 8U; i++) {
        bool active = ((raw >> i) & 0x01U) != 0U;
        if (!handle->cfg.active_high) {
            active = !active;
        }

        if (active) {
            hits++;
            sum += handle->cfg.weights[i];
        }
    }

    handle->hit_count = hits;
    handle->line_lost = (hits == 0U);
    if (!handle->line_lost) {
        /* Weighted average gives a compact line offset without hardcoding pattern tables. */
        handle->line_error = sum / (float) hits;
    }
}
