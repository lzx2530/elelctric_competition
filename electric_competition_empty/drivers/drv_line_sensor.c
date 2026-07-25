#include "drivers/drv_line_sensor.h"

#include "bsp/bsp_gpio.h"

static void line_sensor_delay_cycles(uint16_t cycles)
{
    volatile uint16_t wait = cycles;

    while (wait > 0U) {
        wait--;
        __asm(" nop");
    }
}

void line_sensor_init(line_sensor_handle_t *handle, const line_sensor_config_t *cfg)
{
    handle->cfg = *cfg;
    if (handle->cfg.settle_cycles == 0U) {
        handle->cfg.settle_cycles = 1600U;
    }
    for (uint8_t i = 0U; i < 8U; i++) {
        handle->raw_state[i] = 0U;
    }
    handle->raw_bits = 0U;
    handle->hit_count = 0U;
    handle->line_error = 0.0f;
    handle->line_lost = true;

    bsp_gpio_init_line_mux();
}

void line_sensor_update(line_sensor_handle_t *handle)
{
    float sum = 0.0f;
    uint8_t raw = 0U;
    uint8_t hits = 0U;

    uint8_t direct_bits;

    line_sensor_delay_cycles(handle->cfg.settle_cycles);
    direct_bits = bsp_gpio_read_line_bits();
    for (uint8_t i = 0U; i < 8U; i++) {
        bool active;

        handle->raw_state[i] = (uint8_t)((direct_bits >> i) & 0x01U);

        active = (handle->raw_state[i] != 0U);
        if (!handle->cfg.active_high) {
            active = !active;
        }

        if (active) {
            raw |= (uint8_t) (1U << i);
            hits++;
            sum += handle->cfg.weights[i];
        }
    }

    handle->raw_bits = raw;
    handle->hit_count = hits;
    handle->line_lost = (hits == 0U);
    if (!handle->line_lost) {
        /* Weighted average gives a compact line offset without hardcoding pattern tables. */
        handle->line_error = sum / (float) hits;
    } else {
        handle->line_error = 0.0f;
    }
}
