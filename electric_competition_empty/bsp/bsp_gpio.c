#include "bsp/bsp_gpio.h"

#include "ti_msp_dl_config.h"

void bsp_gpio_init(void)
{
    bsp_gpio_set_led(false);
    bsp_gpio_set_motor_dir(true, true);
    bsp_gpio_set_turret_dir(true, true);
    bsp_gpio_set_laser(false);
    bsp_gpio_set_buzzer(false);
}

void bsp_gpio_set_led(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_LED_STATUS_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_LED_STATUS_PIN);
    }
}

void bsp_gpio_toggle_led(void)
{
    DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_LED_STATUS_PIN);
}

void bsp_gpio_set_motor_dir(bool left_forward, bool right_forward)
{
    bsp_gpio_set_dir_output(BSP_DIR_LEFT, left_forward);
    bsp_gpio_set_dir_output(BSP_DIR_RIGHT, right_forward);
}

void bsp_gpio_set_turret_dir(bool yaw_positive, bool pitch_positive)
{
    bsp_gpio_set_dir_output(BSP_DIR_YAW, yaw_positive);
    bsp_gpio_set_dir_output(BSP_DIR_PITCH, pitch_positive);
}

void bsp_gpio_set_dir_output(bsp_dir_output_t output, bool high)
{
    GPIO_Regs *port = GPIOA;
    uint32_t pin = 0U;

    /* Centralize semantic output selection here so upper layers never touch raw pins. */
    switch (output) {
        case BSP_DIR_LEFT:
            port = GPIO_MOTOR_PORT;
            pin = GPIO_MOTOR_DIR_LEFT_PIN;
            break;
        case BSP_DIR_RIGHT:
            port = GPIO_MOTOR_PORT;
            pin = GPIO_MOTOR_DIR_RIGHT_PIN;
            break;
        case BSP_DIR_YAW:
            port = GPIO_TURRET_PORT;
            pin = GPIO_TURRET_DIR_YAW_PIN;
            break;
        case BSP_DIR_PITCH:
            port = GPIO_TURRET_PORT;
            pin = GPIO_TURRET_DIR_PITCH_PIN;
            break;
        default:
            return;
    }

    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

void bsp_gpio_set_laser(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_TURRET_PORT, GPIO_TURRET_LASER_EN_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TURRET_PORT, GPIO_TURRET_LASER_EN_PIN);
    }
}

void bsp_gpio_set_buzzer(bool on)
{
    if (on) {
        DL_GPIO_setPins(GPIO_TURRET_PORT, GPIO_TURRET_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TURRET_PORT, GPIO_TURRET_BUZZER_PIN);
    }
}

uint8_t bsp_gpio_read_line_bits(void)
{
    uint8_t bits = 0U;

    /* Pack the 8-way digital grayscale inputs into one byte for fast control-loop use. */
    bits |= (uint8_t) (DL_GPIO_readPins(GPIO_LINE_LINE0_PORT, GPIO_LINE_LINE0_PIN) ? (1U << 0) : 0U);
    bits |= (uint8_t) (DL_GPIO_readPins(GPIO_LINE_LINE1_PORT, GPIO_LINE_LINE1_PIN) ? (1U << 1) : 0U);
    bits |= (uint8_t) (DL_GPIO_readPins(GPIO_LINE_LINE2_PORT, GPIO_LINE_LINE2_PIN) ? (1U << 2) : 0U);
    bits |= (uint8_t) (DL_GPIO_readPins(GPIO_LINE_LINE3_PORT, GPIO_LINE_LINE3_PIN) ? (1U << 3) : 0U);
    bits |= (uint8_t) (DL_GPIO_readPins(GPIO_LINE_LINE4_PORT, GPIO_LINE_LINE4_PIN) ? (1U << 4) : 0U);
    bits |= (uint8_t) (DL_GPIO_readPins(GPIO_LINE_LINE5_PORT, GPIO_LINE_LINE5_PIN) ? (1U << 5) : 0U);
    bits |= (uint8_t) (DL_GPIO_readPins(GPIO_LINE_LINE6_PORT, GPIO_LINE_LINE6_PIN) ? (1U << 6) : 0U);
    bits |= (uint8_t) (DL_GPIO_readPins(GPIO_LINE_LINE7_PORT, GPIO_LINE_LINE7_PIN) ? (1U << 7) : 0U);

    return bits;
}

bool bsp_gpio_read_encoder_left_a(void)
{
    return (DL_GPIO_readPins(GPIO_ENCODER_ENC_L_A_PORT, GPIO_ENCODER_ENC_L_A_PIN) != 0U);
}

bool bsp_gpio_read_encoder_left_b(void)
{
    return (DL_GPIO_readPins(GPIO_ENCODER_ENC_L_B_PORT, GPIO_ENCODER_ENC_L_B_PIN) != 0U);
}

bool bsp_gpio_read_encoder_right_a(void)
{
    return (DL_GPIO_readPins(GPIO_ENCODER_ENC_R_A_PORT, GPIO_ENCODER_ENC_R_A_PIN) != 0U);
}

bool bsp_gpio_read_encoder_right_b(void)
{
    return (DL_GPIO_readPins(GPIO_ENCODER_ENC_R_B_PORT, GPIO_ENCODER_ENC_R_B_PIN) != 0U);
}
