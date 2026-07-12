#include "drivers/drv_encoder_ab.h"

#include <string.h>

#include "bsp/bsp_gpio.h"
#include "ti_msp_dl_config.h"

static const int8_t g_qei_table[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

static uint8_t encoder_read_state_left(void)
{
    return (uint8_t) ((bsp_gpio_read_encoder_left_a() ? 0x02U : 0U) |
        (bsp_gpio_read_encoder_left_b() ? 0x01U : 0U));
}

static uint8_t encoder_read_state_right(void)
{
    return (uint8_t) ((bsp_gpio_read_encoder_right_a() ? 0x02U : 0U) |
        (bsp_gpio_read_encoder_right_b() ? 0x01U : 0U));
}

static void encoder_apply_transition(encoder_handle_t *encoder, uint8_t new_state)
{
    uint8_t index = (uint8_t) ((encoder->prev_state << 2U) | new_state);
    int8_t delta = g_qei_table[index];
    /* 四倍频软件 AB 解码：查表直接得到本次边沿对应的 +1/-1/0。 */
    if (encoder->cfg.invert_direction) {
        delta = (int8_t) (-delta);
    }
    encoder->count += delta;
    encoder->prev_state = new_state;
}

void encoder_driver_init(encoder_driver_t *driver, const encoder_config_t *left_cfg, const encoder_config_t *right_cfg)
{
    memset(driver, 0, sizeof(*driver));
    driver->left.cfg = *left_cfg;
    driver->right.cfg = *right_cfg;
    driver->left.prev_state = encoder_read_state_left();
    driver->right.prev_state = encoder_read_state_right();
}

void encoder_driver_poll(encoder_driver_t *driver)
{
    uint8_t left_state = encoder_read_state_left();
    uint8_t right_state = encoder_read_state_right();

    if (left_state != driver->left.prev_state) {
        encoder_apply_transition(&driver->left, left_state);
    }
    if (right_state != driver->right.prev_state) {
        encoder_apply_transition(&driver->right, right_state);
    }
}

void encoder_driver_handle_gpio_interrupt(encoder_driver_t *driver, GPIO_Regs *port, uint32_t pin_iidx)
{
    if (port == GPIOB) {
        if ((pin_iidx == (uint32_t) GPIO_ENCODER_ENC_L_A_IIDX) ||
            (pin_iidx == (uint32_t) GPIO_ENCODER_ENC_L_B_IIDX)) {
            encoder_apply_transition(&driver->left, encoder_read_state_left());
        }

        if (pin_iidx == (uint32_t) GPIO_ENCODER_ENC_R_A_IIDX) {
            encoder_apply_transition(&driver->right, encoder_read_state_right());
        }
    } else if ((port == GPIOA) && (pin_iidx == (uint32_t) GPIO_ENCODER_ENC_R_B_IIDX)) {
        encoder_apply_transition(&driver->right, encoder_read_state_right());
    }
}

void encoder_driver_update_speed(encoder_driver_t *driver, float dt_s)
{
    if (dt_s <= 0.0f) {
        return;
    }

    /* 速度估计直接由固定周期内的增量计数换算得到。 */
    driver->left.delta_count = driver->left.count - driver->left.prev_count;
    driver->right.delta_count = driver->right.count - driver->right.prev_count;
    driver->left.prev_count = driver->left.count;
    driver->right.prev_count = driver->right.count;

    driver->left.speed_rps = (float) driver->left.delta_count / driver->left.cfg.counts_per_revolution / dt_s;
    driver->right.speed_rps = (float) driver->right.delta_count / driver->right.cfg.counts_per_revolution / dt_s;
}
