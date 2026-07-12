#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "common/types.h"

typedef enum {
    BSP_DIR_LEFT = 0,
    BSP_DIR_RIGHT,
    BSP_DIR_YAW,
    BSP_DIR_PITCH,
} bsp_dir_output_t;

void bsp_gpio_init(void);
void bsp_gpio_set_led(bool on);
void bsp_gpio_toggle_led(void);
void bsp_gpio_set_motor_dir(bool left_forward, bool right_forward);
void bsp_gpio_set_turret_dir(bool yaw_positive, bool pitch_positive);
void bsp_gpio_set_dir_output(bsp_dir_output_t output, bool high);
void bsp_gpio_set_laser(bool on);
void bsp_gpio_set_buzzer(bool on);
void bsp_gpio_init_line_mux(void);
void bsp_gpio_set_line_mux_address(uint8_t address);
bool bsp_gpio_read_line_mux_out(void);
uint8_t bsp_gpio_read_line_bits(void);
bool bsp_gpio_read_encoder_left_a(void);
bool bsp_gpio_read_encoder_left_b(void);
bool bsp_gpio_read_encoder_right_a(void);
bool bsp_gpio_read_encoder_right_b(void);

#endif
