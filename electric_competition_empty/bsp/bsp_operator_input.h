#ifndef BSP_OPERATOR_INPUT_H
#define BSP_OPERATOR_INPUT_H

#include "common/types.h"

#define BSP_OPERATOR_EVENT_MODE     (1U << 0)
#define BSP_OPERATOR_EVENT_START    (1U << 1)

void bsp_operator_input_init(void);
void bsp_operator_input_handle_gpio_interrupt(uint32_t iidx);
void bsp_operator_input_capture_irq_handler(void);
uint8_t bsp_operator_input_take_events(void);
bool bsp_operator_input_take_abs_pwm(uint32_t *high_ticks, uint32_t *period_ticks);
void bsp_operator_input_get_abs_pwm_diagnostics(uint32_t *capture_count,
    uint32_t *timeout_count, uint32_t *invalid_count);

#endif
