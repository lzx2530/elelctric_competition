/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "app/app_chassis.h"
#include "app/app_ball_control.h"
#include "app/app_control_scheduler.h"
#include "app/app_imu.h"
#include "app/app_isr.h"
#include "app/app_mission.h"
#include "app/app_ui.h"
#include "algo/algo_filter.h"
#include "algo/algo_fusion.h"
#include "algo/algo_pid.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_i2c.h"
#include "bsp/bsp_operator_input.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_spi_imu.h"
#include "bsp/bsp_uart.h"
#include "common/math_util.h"
#include "drivers/drv_encoder_ab.h"
#include "drivers/drv_abs_position.h"
#include "drivers/drv_line_sensor.h"
#include "drivers/drv_motor_dc.h"
#include "drivers/drv_mpu9250.h"
#include "drivers/drv_oled_ssd1306.h"
#include "drivers/drv_stepper.h"
#include "protocol/proto_k230.h"
#include "protocol/proto_vofa_firewater.h"
#include "ti_msp_dl_config.h"

#include <stdlib.h>
#include <string.h>

typedef enum {
    APP_RUN_MODE_BRINGUP_TEST = 0,
    APP_RUN_MODE_VEHICLE = 1,
    APP_RUN_MODE_ACTUATOR_SINE_TEST = 2,
    APP_RUN_MODE_PA8_INPUT_TEST = 3,
    APP_RUN_MODE_LINE_TRACKING_TEST = 4,
    APP_RUN_MODE_LEAD_SCREW_TEST = 5,
    APP_RUN_MODE_ACTUATOR_RESPONSE_TEST = 6,
} app_run_mode_t;

#define APP_ENABLE_OLED_UI       (1U)
#define APP_ENABLE_VOFA_STREAM   (0U)
#define APP_ENABLE_TEXT_DEBUG    (0U)

typedef struct {
    oled_handle_t oled;
    mpu9250_handle_t imu;
    motor_dc_handle_t left_motor;
    motor_dc_handle_t right_motor;
    encoder_driver_t encoder_driver;
    line_sensor_handle_t line_sensor;
    fusion6_handle_t imu_fusion;
    lpf1_handle_t left_speed_filter;
    limit_filter_handle_t left_output_filter;
    cascade_pid_t left_motor_pos_loop;
    stepper_handle_t stepper;
    uint8_t imu_who_am_i;
    uint8_t motor_stage;
    uint8_t stepper_stage;
    bool oled_probe_ok;
    bool oled_ready;
    bool imu_probe_ok;
    bool imu_ready;
    float left_command;
    float right_command;
    float stepper_command_hz;
    float position_target_rev;
    float position_actual_rev;
    float position_error_rev;
    float left_speed_rps;
    float left_speed_target_rps;
    float imu_roll_deg;
    float imu_pitch_deg;
    float imu_yaw_deg;
    uint8_t debug_log_divider;
} bringup_test_context_t;

typedef enum {
    BRINGUP_OUTPUT_TEST_DC_MOTOR = 0,
    BRINGUP_OUTPUT_TEST_SINGLE_STEPPER = 1,
    BRINGUP_OUTPUT_TEST_GRAY_SENSOR = 2,
    BRINGUP_OUTPUT_TEST_IMU = 3,
} bringup_output_test_t;

#define BRINGUP_TEST_MODE_DEFAULT    BRINGUP_OUTPUT_TEST_DC_MOTOR
#define BRINGUP_IMU_TEXT_LOG_ENABLE  (0U)
#define BRINGUP_MOTOR_TEXT_LOG_ENABLE (0U)

static bringup_test_context_t g_bringup_test;

static void run_vehicle_app(bool line_tracking_test);
static void run_bringup_test(void);
static void run_actuator_sine_test(void);
static void run_pa8_input_test(void);
static void run_lead_screw_test(void);
static void run_actuator_response_test(void);
static void bringup_test_init(bringup_test_context_t *ctx);
static void bringup_test_process(bringup_test_context_t *ctx, const scheduler_flags_t *flags);
static void bringup_test_poll_debug_command(bringup_test_context_t *ctx);
static void bringup_test_update_motor_pattern(bringup_test_context_t *ctx, uint32_t tick_ms);
static void bringup_test_update_stepper_pattern(bringup_test_context_t *ctx, uint32_t tick_ms);
static void bringup_test_update_gray_sensor(bringup_test_context_t *ctx);
static void bringup_test_update_oled(bringup_test_context_t *ctx, uint32_t tick_ms);
static void bringup_test_send_vofa(const bringup_test_context_t *ctx, uint32_t tick_ms);
static void bringup_test_set_position_target(bringup_test_context_t *ctx, float target_rev);
static bool bringup_test_parse_target_line(bringup_test_context_t *ctx, const char *line);
static const char *bringup_test_get_stepper_stage_name(uint8_t stage);
static const char *bringup_test_get_stepper_axis_name(const stepper_handle_t *handle);

int main(void)
{
    static const app_run_mode_t app_mode = APP_RUN_MODE_VEHICLE;

    if (app_mode == APP_RUN_MODE_BRINGUP_TEST) {
        run_bringup_test();
    } else if (app_mode == APP_RUN_MODE_ACTUATOR_SINE_TEST) {
        run_actuator_sine_test();
    } else if (app_mode == APP_RUN_MODE_PA8_INPUT_TEST) {
        run_pa8_input_test();
    } else if (app_mode == APP_RUN_MODE_LINE_TRACKING_TEST) {
        run_vehicle_app(true);
    } else if (app_mode == APP_RUN_MODE_LEAD_SCREW_TEST) {
        run_lead_screw_test();
    } else if (app_mode == APP_RUN_MODE_ACTUATOR_RESPONSE_TEST) {
        run_actuator_response_test();
    } else {
        run_vehicle_app(false);
    }

    while (1) {
    }
}

static void run_vehicle_app(bool line_tracking_test)
{
    scheduler_flags_t scheduler_flags;
    uint32_t last_chassis_control_tick_ms = 0U;
    uint32_t last_k230_log_tick_ms = 0U;
    uint32_t k230_position_frame_count = 0U;
#if APP_ENABLE_TEXT_DEBUG
    uint32_t k230_task_start_frame_count = 0U;
#endif
#if APP_ENABLE_TEXT_DEBUG
    uint32_t last_abs_pwm_high_ticks = 0U;
    uint32_t last_abs_pwm_period_ticks = 0U;
#endif
    k230_parser_t k230_parser;
    ringbuf_t *k230_ringbuf;
#if APP_ENABLE_VOFA_STREAM
    static const proto_vofa_firewater_mode_t vofa_mode = PROTO_VOFA_FIREWATER_MODE_RAW;
    static const char *const vofa_names[10] = {
        "control_dt_ms",
        "left_pid_setpoint_rps",
        "right_pid_setpoint_rps",
        "output_boost",
        "left_speed_rps",
        "right_speed_rps",
        "left_target_rps",
        "right_target_rps",
        "left_output",
        "right_output",
    };
#endif

    SYSCFG_DL_init();

    bsp_gpio_init();
    bsp_operator_input_init();
    bsp_pwm_init();
    bsp_i2c_init();
    bsp_spi_imu_init();
    bsp_uart_init();
    bsp_uart_enable_irqs();
    bsp_pwm_start_all();

    proto_k230_init(&k230_parser);
    k230_ringbuf = bsp_uart_get_k230_ringbuf();

    app_chassis_init();
    app_isr_set_encoder_driver(app_chassis_get_encoder_driver());
    app_imu_init();
    app_ball_control_init();
    app_mission_init();
    if (line_tracking_test) {
        app_mission_start_from_k230(APP_MISSION_LINE_LOOP, 0U);
    }
#if APP_ENABLE_OLED_UI
    app_ui_init();
#endif
    app_control_scheduler_init();

    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);

#if APP_ENABLE_TEXT_DEBUG
    bsp_uart_debug_printf("system init done; K230 UART2 RX=PB18 115200/8N1\r\n");
#endif

    while (1) {
        k230_frame_t frame;
        uint8_t operator_events;
        uint32_t high_ticks;
        uint32_t period_ticks;
        static bool mode_press_seen;
        static bool start_press_seen;
        static uint32_t last_mode_press_ms;
        static uint32_t last_start_press_ms;

        app_control_scheduler_fetch(&scheduler_flags);

        operator_events = bsp_operator_input_take_events();
        if ((operator_events & BSP_OPERATOR_EVENT_MODE) != 0U &&
            (!mode_press_seen ||
                ((scheduler_flags.tick_ms - last_mode_press_ms) >= 200U))) {
            app_mission_next_mode();
#if APP_ENABLE_TEXT_DEBUG
            bsp_uart_debug_printf("[LOCAL] MODE task=%u\r\n",
                (unsigned) app_mission_get_snapshot()->mode);
#endif
            mode_press_seen = true;
            last_mode_press_ms = scheduler_flags.tick_ms;
        }
        if ((operator_events & BSP_OPERATOR_EVENT_START) != 0U &&
            (!start_press_seen ||
                ((scheduler_flags.tick_ms - last_start_press_ms) >= 200U))) {
            app_mission_start(scheduler_flags.tick_ms);
#if APP_ENABLE_TEXT_DEBUG
            bsp_uart_debug_printf("[LOCAL] START task=%u\r\n",
                (unsigned) app_mission_get_snapshot()->mode);
#endif
            start_press_seen = true;
            last_start_press_ms = scheduler_flags.tick_ms;
        }
        if (bsp_operator_input_take_abs_pwm(&high_ticks, &period_ticks)) {
#if APP_ENABLE_TEXT_DEBUG
            last_abs_pwm_high_ticks = high_ticks;
            last_abs_pwm_period_ticks = period_ticks;
#endif
            app_ball_control_set_actuator_pwm(high_ticks, period_ticks);
        }

        while ((!line_tracking_test) &&
            proto_k230_process_ringbuf(&k230_parser, k230_ringbuf, &frame)) {
            if (frame.type == K230_PROTOCOL_TYPE_TASK_START) {
#if APP_ENABLE_TEXT_DEBUG
                k230_task_start_frame_count++;
#endif
                app_mission_start_from_k230(frame.task_flag, scheduler_flags.tick_ms);
#if APP_ENABLE_TEXT_DEBUG
                bsp_uart_debug_printf("[K230] TASK_START flag=%u\r\n", (unsigned) frame.task_flag);
#endif
            } else if (frame.type == K230_PROTOCOL_TYPE_BALL_REPORT) {
                app_ball_control_set_vision(&frame, scheduler_flags.tick_ms);
                k230_position_frame_count++;
                if (APP_ENABLE_TEXT_DEBUG && ((k230_position_frame_count == 1U) ||
                    ((scheduler_flags.tick_ms - last_k230_log_tick_ms) >= 100U))) {
                    bsp_uart_debug_printf(
                        "[K230] BALL seq=%u flags=0x%02X err=%dmm conf=%u valid=%u stable=%u ref=%u stop=%u\r\n",
                        (unsigned) frame.ball.sequence,
                        (unsigned) frame.ball.status,
                        (int) frame.ball.position_mm,
                        (unsigned) frame.ball.confidence_permille,
                        frame.ball.valid ? 1U : 0U,
                        frame.ball.stable ? 1U : 0U,
                        frame.ball.reference_ready ? 1U : 0U,
                        frame.ball.stop_requested ? 1U : 0U);
                    last_k230_log_tick_ms = scheduler_flags.tick_ms;
                }
            }
        }

        if (scheduler_flags.line_5ms) {
            app_chassis_line_task();
        }
        if (scheduler_flags.imu_10ms) {
            app_imu_task();
            app_mission_task(app_imu_get_snapshot(), scheduler_flags.tick_ms);
        }
        if (scheduler_flags.control_1khz) {
            uint32_t elapsed_ms = scheduler_flags.tick_ms - last_chassis_control_tick_ms;
            float chassis_elapsed_s;

            last_chassis_control_tick_ms = scheduler_flags.tick_ms;
            chassis_elapsed_s = 0.001f * (float) elapsed_ms;
            app_chassis_control_task(chassis_elapsed_s, chassis_elapsed_s);
            app_ball_control_inner_task(chassis_elapsed_s);
        }
        if ((APP_ENABLE_OLED_UI != 0U) && scheduler_flags.oled_50ms) {
            app_ui_refresh(app_chassis_get_snapshot(), app_ball_control_get_snapshot(),
                app_mission_get_snapshot());
        }
#if APP_ENABLE_OLED_UI
        app_ui_process();
#endif
        if (scheduler_flags.debug_100ms) {
#if APP_ENABLE_TEXT_DEBUG
            const mission_snapshot_t *mission = app_mission_get_snapshot();
            const ball_control_snapshot_t *ball = app_ball_control_get_snapshot();
#endif
#if APP_ENABLE_VOFA_STREAM
            const chassis_snapshot_t *chassis = app_chassis_get_snapshot();
#endif
#if APP_ENABLE_TEXT_DEBUG
            uint32_t capture_count;
            uint32_t timeout_count;
            uint32_t invalid_count;
            uint32_t duty_permille = 0U;
            uint32_t k230_rx_byte_count;
            uint32_t k230_rx_drop_count;
            uint32_t k230_rx_error_count;
            bsp_operator_input_get_abs_pwm_diagnostics(&capture_count, &timeout_count, &invalid_count);
            bsp_uart_get_k230_rx_diagnostics(&k230_rx_byte_count, &k230_rx_drop_count,
                &k230_rx_error_count);
            if (last_abs_pwm_period_ticks != 0U) {
                duty_permille = (last_abs_pwm_high_ticks * 1000U) / last_abs_pwm_period_ticks;
            }
            bsp_uart_debug_printf(
                "[CTRL] task=%u state=%u en=%u enc=%u fb=%ld/1000 tgt=%ld/1000 cmd=%ldHz fault=%u\r\n",
                (unsigned) mission->mode,
                (unsigned) mission->state,
                ball->enabled ? 1U : 0U,
                ball->actuator_feedback_valid ? 1U : 0U,
                (long) (ball->actuator_feedback * 1000.0F),
                (long) (ball->actuator_target * 1000.0F),
                (long) ball->stepper_command_hz,
                (unsigned) ball->fault);
            bsp_uart_debug_printf("[CAP] count=%lu timeout=%lu invalid=%lu high=%lu period=%lu duty=%lu/1000\r\n",
                (unsigned long) capture_count,
                (unsigned long) timeout_count,
                (unsigned long) invalid_count,
                (unsigned long) last_abs_pwm_high_ticks,
                (unsigned long) last_abs_pwm_period_ticks,
                (unsigned long) duty_permille);
            bsp_uart_debug_printf("[K230-RX] bytes=%lu drop=%lu error=%lu queued=%u task=%lu ball=%lu\r\n",
                (unsigned long) k230_rx_byte_count,
                (unsigned long) k230_rx_drop_count,
                (unsigned long) k230_rx_error_count,
                (unsigned) ringbuf_size(k230_ringbuf),
                (unsigned long) k230_task_start_frame_count,
                (unsigned long) k230_position_frame_count);
#endif
#if APP_ENABLE_VOFA_STREAM
            proto_vofa_firewater_packet_t vofa_packet;
            float vofa_channels[10] = {
                1000.0F * chassis->control_dt_s,
                chassis->left_pid_setpoint_rps,
                chassis->right_pid_setpoint_rps,
                chassis->output_boost,
                chassis->left_speed_rps,
                chassis->right_speed_rps,
                chassis->left_target_rps,
                chassis->right_target_rps,
                chassis->left_output,
                chassis->right_output,
            };
            vofa_packet.mode = vofa_mode;
            vofa_packet.names = vofa_names;
            vofa_packet.data = vofa_channels;
            vofa_packet.count = 10U;
            proto_vofa_firewater_send_packet(&vofa_packet);
#endif
            bsp_gpio_toggle_led();
        }
    }
}

static void run_pa8_input_test(void)
{
    scheduler_flags_t scheduler_flags;

    SYSCFG_DL_init();
    bsp_gpio_init();
    bsp_uart_init();
    bsp_uart_enable_irqs();
    app_control_scheduler_init();

    /* Temporarily override TIMA0 capture mux to prove the PA8/J1.4 signal level. */
    DL_GPIO_initDigitalInputFeatures(GPIO_CAPTURE_ABS_PWM_C0_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    bsp_uart_debug_printf("PA8 input test: disconnect encoder PWM, expected level=1\r\n");

    while (1) {
        app_control_scheduler_fetch(&scheduler_flags);
        if (scheduler_flags.debug_100ms) {
            uint32_t level = DL_GPIO_readPins(GPIO_CAPTURE_ABS_PWM_C0_PORT,
                GPIO_CAPTURE_ABS_PWM_C0_PIN) != 0U ? 1U : 0U;
            bsp_uart_debug_printf("PA8 level=%lu\r\n", (unsigned long) level);
        }
    }
}

static void run_actuator_sine_test(void)
{
    scheduler_flags_t scheduler_flags;
    stepper_handle_t stepper;
    static const stepper_config_t stepper_cfg = {
        .axis = BSP_STEPPER_AXIS_PITCH,
        .dir_output = BSP_DIR_PITCH,
        .invert_direction = false,
        .min_frequency_hz = 5.0F,
        .max_frequency_hz = 300.0F,
        .accel_hz_per_s = 800.0F,
    };
    const float test_period_s = 8.0F;
    const float max_frequency_hz = 250.0F;
    const float two_pi = 6.283185307F;
    uint32_t last_high_ticks = 0U;
    uint32_t last_period_ticks = 0U;
    uint16_t pa8_high_samples = 0U;
    uint16_t pa8_total_samples = 0U;

    SYSCFG_DL_init();
    bsp_gpio_init();
    bsp_operator_input_init();
    bsp_pwm_init();
    bsp_uart_init();
    bsp_uart_enable_irqs();
    bsp_pwm_start_all();
    app_control_scheduler_init();

    stepper_init(&stepper, &stepper_cfg);
    stepper_enable(&stepper, true);
    bsp_uart_debug_printf("actuator sine test: period=8s max=250Hz, K230/chassis disabled\r\n");

    while (1) {
        uint32_t high_ticks;
        uint32_t period_ticks;

        app_control_scheduler_fetch(&scheduler_flags);
        if (scheduler_flags.control_1khz) {
            float phase = two_pi * (float) scheduler_flags.tick_ms / (1000.0F * test_period_s);
            float command_hz = max_frequency_hz * sinf(phase);

            if (DL_GPIO_readPins(GPIO_CAPTURE_ABS_PWM_C0_PORT,
                    GPIO_CAPTURE_ABS_PWM_C0_PIN) != 0U) {
                pa8_high_samples++;
            }
            pa8_total_samples++;
            stepper_set_speed(&stepper, command_hz);
            stepper_update(&stepper, 0.001F);
        }

        if (bsp_operator_input_take_abs_pwm(&high_ticks, &period_ticks)) {
            /* Captured PWM comes directly from the absolute encoder on PA8. */
            last_high_ticks = high_ticks;
            last_period_ticks = period_ticks;
        }

        if (scheduler_flags.debug_100ms) {
            uint32_t capture_count;
            uint32_t timeout_count;
            uint32_t invalid_count;
            float command_hz = max_frequency_hz * sinf(two_pi *
                (float) scheduler_flags.tick_ms / (1000.0F * test_period_s));
            bsp_operator_input_get_abs_pwm_diagnostics(&capture_count,
                &timeout_count, &invalid_count);
            if (last_period_ticks != 0U) {
                float duty = 100.0F * (float) last_high_ticks / (float) last_period_ticks;
                bsp_uart_debug_printf("sine t=%lums cmd=%.1fHz pwm=%lu/%lu duty=%.2f%% cap=%lu zero=%lu bad=%lu pa8=%u/%u\r\n",
                    (unsigned long) scheduler_flags.tick_ms, command_hz,
                    (unsigned long) last_high_ticks, (unsigned long) last_period_ticks, duty,
                    (unsigned long) capture_count, (unsigned long) timeout_count,
                    (unsigned long) invalid_count, pa8_high_samples, pa8_total_samples);
            } else {
                bsp_uart_debug_printf("sine t=%lums cmd=%.1fHz pwm=waiting cap=%lu zero=%lu bad=%lu pa8=%u/%u\r\n",
                    (unsigned long) scheduler_flags.tick_ms, command_hz,
                    (unsigned long) capture_count, (unsigned long) timeout_count,
                    (unsigned long) invalid_count, pa8_high_samples, pa8_total_samples);
            }
            pa8_high_samples = 0U;
            pa8_total_samples = 0U;
        }
    }
}

static void run_actuator_response_test(void)
{
    scheduler_flags_t scheduler_flags;
    abs_position_handle_t encoder;
    stepper_handle_t stepper;
    pid_handle_t position_pid;
    static const stepper_config_t stepper_cfg = {
        .axis = BSP_STEPPER_AXIS_PITCH,
        .dir_output = BSP_DIR_PITCH,
        .invert_direction = false,
        .min_frequency_hz = 5.0F,
        .max_frequency_hz = 8000.0F,
        .accel_hz_per_s = 16000.0F,
    };
    static const pid_config_t position_pid_cfg = {
        .kp = 7000.0F,
        .ki = 300.0F,
        .kd = 40.0F,
        .dt_s = 0.001F,
        .output_limit = 8000.0F,
        .integral_limit = 0.15F,
        .integral_separation = 0.10F,
        .derivative_lpf_alpha = 0.10F,
        .setpoint_slew_rate = 0.0F,
        .deadband = 0.001F,
        .derivative_on_measurement = true,
        .enable_integral_separation = true,
        .enable_output_limit = true,
        .enable_integral_limit = true,
        .enable_deadband = true,
        .enable_setpoint_ramp = false,
    };
    const float test_turns = 2.0F;
    const float settle_tolerance_turns = 0.03F;
    const uint32_t settle_time_ms = 300U;
    const uint32_t phase_timeout_ms = 6000U;
    const uint32_t capture_timeout_ms = 300U;
    float feedback_turns = 0.0F;
    float reference_turns = 0.0F;
    float target_turns = 0.0F;
    float command_hz = 0.0F;
    uint32_t start_tick_ms = 0U;
    uint32_t phase_tick_ms = 0U;
    uint32_t last_capture_tick_ms = 0U;
    uint32_t last_control_tick_ms = 0U;
    uint8_t stage = 0U;

    SYSCFG_DL_init();
    bsp_gpio_init();
    bsp_operator_input_init();
    bsp_pwm_init();
    bsp_uart_init();
    bsp_uart_enable_irqs();
    bsp_pwm_start_all();
    app_control_scheduler_init();
    abs_position_init(&encoder, 0.01F, 0.99F);
    stepper_init(&stepper, &stepper_cfg);
    stepper_enable(&stepper, true);
    pid_init(&position_pid, &position_pid_cfg, PID_MODE_POSITION);
    bsp_uart_debug_printf("actuator response: waiting encoder, then +2.0 -> -2.0 turns\r\n");

    while (1) {
        uint32_t high_ticks;
        uint32_t period_ticks;

        app_control_scheduler_fetch(&scheduler_flags);
        if (bsp_operator_input_take_abs_pwm(&high_ticks, &period_ticks)) {
            abs_position_update_pwm(&encoder, high_ticks, period_ticks);
            if (encoder.valid) {
                last_capture_tick_ms = scheduler_flags.tick_ms;
                if (stage == 0U) {
                    reference_turns = encoder.multi_turn_position;
                    start_tick_ms = scheduler_flags.tick_ms;
                    phase_tick_ms = scheduler_flags.tick_ms;
                    stage = 1U;
                    target_turns = test_turns;
                    bsp_uart_debug_printf("actuator response: start +2.0 turns\r\n");
                }
                feedback_turns = encoder.multi_turn_position - reference_turns;
            }
        }

        if (scheduler_flags.control_1khz) {
            uint32_t elapsed_ms = scheduler_flags.tick_ms - last_control_tick_ms;
            float elapsed_s = 0.001F * (float) elapsed_ms;

            last_control_tick_ms = scheduler_flags.tick_ms;
            if ((stage >= 1U) && (stage <= 4U) &&
                ((scheduler_flags.tick_ms - last_capture_tick_ms) > capture_timeout_ms ||
                    (scheduler_flags.tick_ms - phase_tick_ms) > phase_timeout_ms)) {
                stage = 6U;
                stepper_stop(&stepper);
                bsp_uart_debug_printf("actuator response: capture/phase timeout\r\n");
            } else if (stage == 1U) {
                if (math_absf(target_turns - feedback_turns) <= settle_tolerance_turns) {
                    stage = 2U;
                    phase_tick_ms = scheduler_flags.tick_ms;
                }
            } else if (stage == 2U) {
                if ((scheduler_flags.tick_ms - phase_tick_ms) >= settle_time_ms) {
                    stage = 3U;
                    target_turns = -test_turns;
                    phase_tick_ms = scheduler_flags.tick_ms;
                    bsp_uart_debug_printf("actuator response: switch -2.0 turns\r\n");
                }
            } else if (stage == 3U) {
                if (math_absf(target_turns - feedback_turns) <= settle_tolerance_turns) {
                    stage = 4U;
                    phase_tick_ms = scheduler_flags.tick_ms;
                }
            } else if (stage == 4U &&
                (scheduler_flags.tick_ms - phase_tick_ms) >= settle_time_ms) {
                stage = 5U;
                stepper_stop(&stepper);
                bsp_uart_debug_printf("actuator response: done\r\n");
            }

            if ((stage >= 1U) && (stage <= 4U)) {
                command_hz = pid_update(&position_pid, target_turns, feedback_turns);
                stepper_set_speed(&stepper, command_hz);
                stepper_update(&stepper, elapsed_s);
            } else if (stage != 5U) {
                command_hz = 0.0F;
            }
        }

        if (scheduler_flags.debug_100ms) {
            float vofa_channels[7] = {
                0.001F * (float) (scheduler_flags.tick_ms - start_tick_ms),
                target_turns,
                feedback_turns,
                target_turns - feedback_turns,
                command_hz,
                bsp_pwm_get_step_frequency(BSP_STEPPER_AXIS_PITCH),
                (float) stage,
            };

            proto_vofa_firewater_send(vofa_channels, 7U);
            bsp_uart_debug_printf("response t=%.3fs stage=%u fb=%.3f tgt=%.3f cmd=%.0fHz pwm=%.0fHz\r\n",
                vofa_channels[0], (unsigned) stage, feedback_turns, target_turns,
                command_hz, vofa_channels[5]);
        }
    }
}

static void run_lead_screw_test(void)
{
    scheduler_flags_t scheduler_flags;
    abs_position_handle_t encoder;
    const float target_revolutions = 12.0F;
    const float test_command_hz = -400.0F;
    const uint32_t encoder_timeout_ms = 300U;
    const uint32_t test_timeout_ms = 120000U;
    bool encoder_started = false;
    bool test_done = false;
    float traveled_revolutions = 0.0F;
    float last_position = 0.0F;
    float encoder_duty = 0.0F;
    uint32_t start_tick_ms = 0U;
    uint32_t last_encoder_tick_ms = 0U;
    uint32_t last_timer_count = 0U;
    uint16_t timer_count_changes = 0U;
    bool test_timed_out = false;
    bool pwm_started = false;

    SYSCFG_DL_init();
    bsp_gpio_init();
    bsp_operator_input_init();
    bsp_pwm_init();
    bsp_uart_init();
    bsp_uart_enable_irqs();
    bsp_pwm_start_all();
    app_control_scheduler_init();

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_STEP_PITCH_C0_IOMUX,
        GPIO_PWM_STEP_PITCH_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_STEP_PITCH_C0_PORT, GPIO_PWM_STEP_PITCH_C0_PIN);
    bsp_gpio_set_dir_output(BSP_DIR_PITCH, false);
    abs_position_init(&encoder, 0.01F, 0.99F);
    bsp_uart_debug_printf(
        "lead screw return: DIR=LOW, STEP=400Hz for %.1f motor turns\r\n",
        target_revolutions);

    while (1) {
        uint32_t high_ticks;
        uint32_t period_ticks;

        app_control_scheduler_fetch(&scheduler_flags);
        if (bsp_operator_input_take_abs_pwm(&high_ticks, &period_ticks)) {
            encoder_duty = period_ticks != 0U ?
                (float) high_ticks / (float) period_ticks : 0.0F;
            abs_position_update_pwm(&encoder, high_ticks, period_ticks);
            last_encoder_tick_ms = scheduler_flags.tick_ms;
            if (encoder.valid) {
                if (!encoder_started) {
                    encoder_started = true;
                    start_tick_ms = scheduler_flags.tick_ms;
                    last_position = encoder.position;
                    bsp_uart_debug_printf("lead screw test: encoder ready, starting\r\n");
                } else if (!test_done) {
                    float position_delta = encoder.position - last_position;

                    if (position_delta > 0.50F) {
                        position_delta -= 1.0F;
                    } else if (position_delta < -0.50F) {
                        position_delta += 1.0F;
                    }
                    if (math_absf(position_delta) <= 0.20F) {
                        traveled_revolutions += position_delta;
                    }
                    if (math_absf(traveled_revolutions) >= target_revolutions) {
                        test_done = true;
                        bsp_pwm_stop_step(BSP_STEPPER_AXIS_PITCH);
                        bsp_uart_debug_printf(
                            "lead screw test: done %.3f turns, measure nut travel in mm\r\n",
                            math_absf(traveled_revolutions));
                    }
                    last_position = encoder.position;
                }
            }
        }

        if (scheduler_flags.control_1khz && encoder_started && !test_done) {
            if (((scheduler_flags.tick_ms - start_tick_ms) > test_timeout_ms) ||
                ((scheduler_flags.tick_ms - last_encoder_tick_ms) > encoder_timeout_ms)) {
                test_done = true;
                test_timed_out = true;
                bsp_pwm_stop_step(BSP_STEPPER_AXIS_PITCH);
                bsp_uart_debug_printf("lead screw test: stopped, encoder/test timeout\r\n");
            } else {
                if (!pwm_started) {
                    bsp_pwm_set_step_frequency(BSP_STEPPER_AXIS_PITCH, 400.0F);
                    pwm_started = true;
                    last_timer_count = DL_TimerG_getTimerCount(PWM_STEP_PITCH_INST);
                } else {
                    uint32_t timer_count = DL_TimerG_getTimerCount(PWM_STEP_PITCH_INST);

                    if (timer_count != last_timer_count) {
                        timer_count_changes++;
                    }
                    last_timer_count = timer_count;
                }
            }
        }

        if (scheduler_flags.debug_100ms) {
            float vofa_channels[9] = {
                encoder_duty,
                encoder.position,
                encoder.valid ? 1.0F : 0.0F,
                math_absf(traveled_revolutions),
                test_command_hz,
                test_timed_out ? 3.0F : (test_done ? 2.0F : (encoder_started ? 1.0F : 0.0F)),
                DL_TimerG_isRunning(PWM_STEP_PITCH_INST) ? 1.0F : 0.0F,
                (float) timer_count_changes,
                0.001F * (float) DL_TimerG_getLoadValue(PWM_STEP_PITCH_INST),
            };

            proto_vofa_firewater_send(vofa_channels, 9U);
            bsp_uart_debug_printf("lead screw turns=%.3f/%.1f pos=%ld/1000 cmd=%.0fHz run=%u ctrchg=%u load=%lu%s\r\n",
                math_absf(traveled_revolutions),
                target_revolutions,
                (long) (encoder.position * 1000.0F),
                test_command_hz,
                DL_TimerG_isRunning(PWM_STEP_PITCH_INST) ? 1U : 0U,
                (unsigned) timer_count_changes,
                (unsigned long) DL_TimerG_getLoadValue(PWM_STEP_PITCH_INST),
                test_done ? " done" : "");
            timer_count_changes = 0U;
        }
    }
}

static void run_bringup_test(void)
{
    scheduler_flags_t scheduler_flags;

    SYSCFG_DL_init();

    bsp_gpio_init();
    bsp_operator_input_init();
    bsp_pwm_init();
    bsp_i2c_init();
    bsp_uart_init();
    bsp_uart_enable_irqs();
    bsp_pwm_start_all();

    bringup_test_init(&g_bringup_test);
    app_isr_set_encoder_driver(&g_bringup_test.encoder_driver);
    app_control_scheduler_init();

    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);

    bsp_uart_debug_printf("bringup test start\r\n");

    while (1) {
        app_control_scheduler_fetch(&scheduler_flags);
        bringup_test_process(&g_bringup_test, &scheduler_flags);
    }
}

static void bringup_test_init(bringup_test_context_t *ctx)
{
    static const bringup_output_test_t test_mode = BRINGUP_TEST_MODE_DEFAULT;
    static const encoder_config_t left_encoder_cfg = {
        .counts_per_revolution = 780.0f,
        .invert_direction = false,
    };
    static const encoder_config_t right_encoder_cfg = {
        .counts_per_revolution = 780.0f,
        .invert_direction = true,
    };
    static const line_sensor_config_t line_cfg = {
        .active_high = true,
        .settle_cycles = 1600U,
        .weights = {-3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f},
    };
    static const fusion6_config_t imu_fusion_cfg = {
        .dt_s = 0.01f,
        .accel_weight = 0.02f,
        .yaw_correction_weight = 0.0f,
    };
    static const float left_speed_filter_alpha = 0.75f;
    static const float left_output_limit_step = 1.0f;
    static const pid_config_t pos_outer_cfg = {
        .kp = 2.7f,
        .ki = 0.0f,
        .kd = 0.0f,
        .dt_s = 0.001f,
        .output_limit = 4.0f,
        .integral_limit = 1.2f,
        .integral_separation = 0.8f,
        .derivative_lpf_alpha = 0.12f,
        .setpoint_slew_rate = 0.0f,
        .deadband = 0.01f,
        .derivative_on_measurement = true,
        .enable_integral_separation = true,
        .enable_output_limit = true,
        .enable_integral_limit = true,
        .enable_deadband = true,
        .enable_setpoint_ramp = false,
    };
    static const pid_config_t speed_inner_cfg = {
        .kp = 0.45f,
        .ki = 0.02f,
        .kd = 0.001f,
        .dt_s = 0.001f,
        .output_limit = 0.45f,
        .integral_limit = 0.3f,
        .integral_separation = 2.0f,
        .derivative_lpf_alpha = 0.15f,
        .setpoint_slew_rate = 55.0f,
        .deadband = 0.04f,
        .derivative_on_measurement = true,
        .enable_integral_separation = true,
        .enable_output_limit = true,
        .enable_integral_limit = true,
        .enable_deadband = true,
        .enable_setpoint_ramp = false,
    };
    static const stepper_config_t single_stepper_cfg = {
        .axis = BSP_STEPPER_AXIS_PITCH,
        .dir_output = BSP_DIR_PITCH,
        .invert_direction = false,
        .min_frequency_hz = 5.0f,
        .max_frequency_hz = 2000.0f,
        .accel_hz_per_s = 4000.0f,
    };
    static const motor_dc_config_t left_motor_cfg = {
        .pwm_channel = BSP_MOTOR_PWM_LEFT,
        .invert_direction = false,
        .deadband = 0.02f,
        .max_duty = 0.45f,
    };
    static const motor_dc_config_t right_motor_cfg = {
        .pwm_channel = BSP_MOTOR_PWM_RIGHT,
        .invert_direction = true,
        .deadband = 0.02f,
        .max_duty = 0.45f,
    };
    status_t oled_init_ret = STATUS_NOT_READY;
    const uint8_t oled_addr = 0x3CU;

    /*
     * The previous CubeMX SSD1306 driver used 8-bit address 0x78, i.e. 7-bit address 0x3C.
     * The current low-level probe implementation can report false positives, so do not auto-switch to 0x3D.
     */
    ctx->oled_probe_ok = true;
    ctx->imu_probe_ok = (bsp_i2c_probe(0x68U) == STATUS_OK);
    ctx->oled_ready = false;
    ctx->imu_ready = false;
    ctx->imu_who_am_i = 0U;
    ctx->motor_stage = 0U;
    ctx->stepper_stage = 0U;
    ctx->left_command = 0.0f;
    ctx->right_command = 0.0f;
    ctx->stepper_command_hz = 0.0f;
    ctx->position_target_rev = 0.0f;
    ctx->position_actual_rev = 0.0f;
    ctx->position_error_rev = 0.0f;
    ctx->left_speed_rps = 0.0f;
    ctx->left_speed_target_rps = 0.0f;
    ctx->imu_roll_deg = 0.0f;
    ctx->imu_pitch_deg = 0.0f;
    ctx->imu_yaw_deg = 0.0f;
    ctx->debug_log_divider = 0U;

    motor_dc_init(&ctx->left_motor, &left_motor_cfg);
    motor_dc_init(&ctx->right_motor, &right_motor_cfg);
    encoder_driver_init(&ctx->encoder_driver, &left_encoder_cfg, &right_encoder_cfg);
    line_sensor_init(&ctx->line_sensor, &line_cfg);
    fusion6_init(&ctx->imu_fusion, &imu_fusion_cfg);
    lpf1_init(&ctx->left_speed_filter, left_speed_filter_alpha, 0.0f);
    limit_filter_init(&ctx->left_output_filter, left_output_limit_step, 0.0f);
    cascade_pid_init(&ctx->left_motor_pos_loop, &pos_outer_cfg, &speed_inner_cfg, PID_MODE_POSITION, PID_MODE_POSITION);
    stepper_init(&ctx->stepper, &single_stepper_cfg);
    stepper_enable(&ctx->stepper, true);

    if (ctx->oled_probe_ok) {
        oled_init_ret = oled_init(&ctx->oled, oled_addr);
        ctx->oled_ready = (oled_init_ret == STATUS_OK);
    }

    if (ctx->imu_probe_ok) {
        ctx->imu_ready = (mpu9250_init(&ctx->imu) == STATUS_OK);
        if (ctx->imu_ready) {
            ctx->imu_ready = (mpu9250_read_who_am_i(&ctx->imu, &ctx->imu_who_am_i) == STATUS_OK);
        }
        if (ctx->imu_ready) {
            ctx->imu_ready = (mpu9250_calibrate_gyro_bias(&ctx->imu, 32U) == STATUS_OK);
        }
    }

    bsp_uart_debug_printf("oled addr=0x%02X ready=%u init_ret=%d fail_idx=%u fail_cmd=0x%02X last=%d\r\n",
        oled_addr,
        ctx->oled_ready ? 1U : 0U,
        oled_init_ret,
        oled_get_last_failed_index(),
        oled_get_last_failed_command(),
        oled_get_last_status());
    bsp_uart_debug_printf("mpu probe=%u ready=%u who_am_i=0x%02X\r\n",
        ctx->imu_probe_ok ? 1U : 0U,
        ctx->imu_ready ? 1U : 0U,
        ctx->imu_who_am_i);
    bsp_uart_debug_printf("bringup output test=%u\r\n", (unsigned) test_mode);
    if (test_mode == BRINGUP_OUTPUT_TEST_DC_MOTOR) {
        bsp_uart_debug_printf("motor=LEFT pos-loop AIN1/AIN2\r\n");
        bsp_uart_debug_printf("send target rev as ASCII line, ex: 1.25 or zero\r\n");
    } else if (test_mode == BRINGUP_OUTPUT_TEST_GRAY_SENSOR) {
        bsp_uart_debug_printf("gray sensor mux: AD0=PA15 AD1=PA16 AD2=PB16 OUT=PA25(GPIO IN)\r\n");
        bsp_uart_debug_printf("gray digital active_high=%u settle~50us(eq=%u)\r\n",
            line_cfg.active_high ? 1U : 0U,
            line_cfg.settle_cycles);
        bsp_uart_debug_printf("vendor sample uses digital scan + 5V module supply; current project keeps remapped pins for your wiring\r\n");
    } else if (test_mode == BRINGUP_OUTPUT_TEST_IMU) {
        bsp_uart_debug_printf("imu bringup mode: VOFA ch1=roll ch2=pitch ch3=yaw\r\n");
    }
}

static void bringup_test_process(bringup_test_context_t *ctx, const scheduler_flags_t *flags)
{
    static const bringup_output_test_t test_mode = BRINGUP_TEST_MODE_DEFAULT;

    if (test_mode == BRINGUP_OUTPUT_TEST_DC_MOTOR) {
        bringup_test_poll_debug_command(ctx);
    }

    if (flags->imu_10ms && ctx->imu_ready) {
        if (mpu9250_update(&ctx->imu) != STATUS_OK) {
            ctx->imu_ready = false;
            bsp_uart_debug_printf("mpu update failed\r\n");
            fusion6_reset(&ctx->imu_fusion);
            ctx->imu_roll_deg = 0.0f;
            ctx->imu_pitch_deg = 0.0f;
            ctx->imu_yaw_deg = 0.0f;
        } else {
            fusion6_update(&ctx->imu_fusion,
                ctx->imu.accel_g.x,
                ctx->imu.accel_g.y,
                ctx->imu.accel_g.z,
                ctx->imu.gyro_dps.x,
                ctx->imu.gyro_dps.y,
                ctx->imu.gyro_dps.z);
            ctx->imu_roll_deg = ctx->imu_fusion.roll_deg;
            ctx->imu_pitch_deg = ctx->imu_fusion.pitch_deg;
            ctx->imu_yaw_deg = ctx->imu_fusion.yaw_deg;
        }
    }

    if ((test_mode == BRINGUP_OUTPUT_TEST_GRAY_SENSOR) && flags->line_5ms) {
        bringup_test_update_gray_sensor(ctx);
    }

    if (flags->control_1khz) {
        if (test_mode == BRINGUP_OUTPUT_TEST_SINGLE_STEPPER) {
            bringup_test_update_stepper_pattern(ctx, flags->tick_ms);
        } else if (test_mode == BRINGUP_OUTPUT_TEST_GRAY_SENSOR) {
            /* Gray scanning is done in the 5 ms task so the 1 kHz path stays lightweight. */
        } else if (test_mode == BRINGUP_OUTPUT_TEST_IMU) {
            /* IMU bring-up only updates the sensor and prints telemetry. */
        } else {
            bringup_test_update_motor_pattern(ctx, flags->tick_ms);
        }
    }

    if (flags->oled_50ms) {
        bringup_test_update_oled(ctx, flags->tick_ms);
    }

    if (flags->debug_100ms) {
        ctx->debug_log_divider++;
        if (ctx->debug_log_divider >= 10U) {
            ctx->debug_log_divider = 0U;
            if (test_mode == BRINGUP_OUTPUT_TEST_SINGLE_STEPPER) {
                bsp_uart_debug_printf("status t=%lu oled=%u imu=%u wai=0x%02X axis=%s stage=%u cmd=%.1fHz\r\n",
                    (unsigned long) flags->tick_ms,
                    ctx->oled_ready ? 1U : 0U,
                    ctx->imu_ready ? 1U : 0U,
                    ctx->imu_who_am_i,
                    bringup_test_get_stepper_axis_name(&ctx->stepper),
                    ctx->stepper_stage,
                    ctx->stepper_command_hz);
            } else if (test_mode == BRINGUP_OUTPUT_TEST_GRAY_SENSOR) {
                bsp_uart_debug_printf("gray bits=0x%02X hit=%u lost=%u all=%u,%u,%u,%u,%u,%u,%u,%u\r\n",
                    ctx->line_sensor.raw_bits,
                    ctx->line_sensor.hit_count,
                    ctx->line_sensor.line_lost ? 1U : 0U,
                    ctx->line_sensor.raw_state[0],
                    ctx->line_sensor.raw_state[1],
                    ctx->line_sensor.raw_state[2],
                    ctx->line_sensor.raw_state[3],
                    ctx->line_sensor.raw_state[4],
                    ctx->line_sensor.raw_state[5],
                    ctx->line_sensor.raw_state[6],
                    ctx->line_sensor.raw_state[7]);
            } else if ((test_mode == BRINGUP_OUTPUT_TEST_IMU) && (BRINGUP_IMU_TEXT_LOG_ENABLE != 0U)) {
                bsp_uart_debug_printf("status t=%lu oled=%u imu=%u wai=0x%02X\r\n",
                    (unsigned long) flags->tick_ms,
                    ctx->oled_ready ? 1U : 0U,
                    ctx->imu_ready ? 1U : 0U,
                    ctx->imu_who_am_i);
            } else if (BRINGUP_MOTOR_TEXT_LOG_ENABLE != 0U) {
                bsp_uart_debug_printf("status t=%lu target=%.3f actual=%.3f err=%.3f spd_t=%.3f spd=%.3f out=%.3f\r\n",
                    (unsigned long) flags->tick_ms,
                    ctx->position_target_rev,
                    ctx->position_actual_rev,
                    ctx->position_error_rev,
                    ctx->left_speed_target_rps,
                    ctx->left_speed_rps,
                    ctx->left_command);
            }
            if ((ctx->imu_ready) &&
                (test_mode == BRINGUP_OUTPUT_TEST_IMU) &&
                (BRINGUP_IMU_TEXT_LOG_ENABLE != 0U)) {
                bsp_uart_debug_printf("imu roll=%.2f yaw=%.2f pitch=%.2f\r\n",
                    ctx->imu_roll_deg,
                    ctx->imu_yaw_deg,
                    ctx->imu_pitch_deg);
            }
        }
        bringup_test_send_vofa(ctx, flags->tick_ms);
        bsp_gpio_toggle_led();
    }
}

static void bringup_test_poll_debug_command(bringup_test_context_t *ctx)
{
    ringbuf_t *rb = bsp_uart_get_debug_ringbuf();
    static char line[48];
    static uint8_t length = 0U;
    uint8_t byte;

    while (ringbuf_pop_byte(rb, &byte)) {
        if ((byte == '\r') || (byte == '\n')) {
            if (length == 0U) {
                continue;
            }

            line[length] = '\0';
            (void) bringup_test_parse_target_line(ctx, line);
            length = 0U;
            continue;
        }

        if (length >= (uint8_t) (sizeof(line) - 1U)) {
            length = 0U;
            continue;
        }

        line[length++] = (char) byte;
    }
}

static void bringup_test_update_motor_pattern(bringup_test_context_t *ctx, uint32_t tick_ms)
{
    static const float speed_feedforward_gain = 0.09f;

    (void) tick_ms;

    encoder_driver_poll(&ctx->encoder_driver);
    encoder_driver_update_speed(&ctx->encoder_driver, 0.001f);

    ctx->position_actual_rev =
        (float) ctx->encoder_driver.left.count / ctx->encoder_driver.left.cfg.counts_per_revolution;
    ctx->left_speed_rps = lpf1_update(&ctx->left_speed_filter, ctx->encoder_driver.left.speed_rps);
    ctx->position_error_rev = ctx->position_target_rev - ctx->position_actual_rev;
    ctx->left_command = cascade_pid_update(&ctx->left_motor_pos_loop,
        ctx->position_target_rev,
        ctx->position_actual_rev,
        ctx->left_speed_rps);
    ctx->left_speed_target_rps = ctx->left_motor_pos_loop.inner_pid.setpoint;
    ctx->left_command += speed_feedforward_gain * ctx->left_speed_target_rps;
    ctx->left_command = math_clampf(ctx->left_command,
        -ctx->left_motor.cfg.max_duty,
        ctx->left_motor.cfg.max_duty);
    ctx->left_command = limit_filter_update(&ctx->left_output_filter, ctx->left_command);
    ctx->right_command = 0.0f;

    motor_dc_set_output(&ctx->left_motor, ctx->left_command);
    motor_dc_set_output(&ctx->right_motor, 0.0f);

    if ((ctx->position_error_rev > 0.02f) || (ctx->position_error_rev < -0.02f)) {
        ctx->motor_stage = 1U;
    } else if ((ctx->left_speed_rps > 0.05f) || (ctx->left_speed_rps < -0.05f)) {
        ctx->motor_stage = 2U;
    } else {
        ctx->motor_stage = 0U;
    }
}

static void bringup_test_update_stepper_pattern(bringup_test_context_t *ctx, uint32_t tick_ms)
{
    uint8_t stage = (uint8_t) ((tick_ms / 2000U) % 4U);
    float stepper_command_hz = 0.0f;

    switch (stage) {
        case 1U:
            stepper_command_hz = 400.0f;
            break;
        case 3U:
            stepper_command_hz = -400.0f;
            break;
        default:
            break;
    }

    if (stage != ctx->stepper_stage) {
        bsp_uart_debug_printf("stepper stage=%u %s axis=%s cmd=%.1fHz\r\n",
            stage,
            bringup_test_get_stepper_stage_name(stage),
            bringup_test_get_stepper_axis_name(&ctx->stepper),
            stepper_command_hz);
        ctx->stepper_stage = stage;
    }

    ctx->stepper_command_hz = stepper_command_hz;
    stepper_set_speed(&ctx->stepper, stepper_command_hz);
    stepper_update(&ctx->stepper, 0.001f);
}

static void bringup_test_update_gray_sensor(bringup_test_context_t *ctx)
{
    line_sensor_update(&ctx->line_sensor);
}

static void bringup_test_update_oled(bringup_test_context_t *ctx, uint32_t tick_ms)
{
    static const bringup_output_test_t test_mode = BRINGUP_TEST_MODE_DEFAULT;

    if (!ctx->oled_ready) {
        return;
    }

    /* Keep the bring-up page sparse first so display quality is easy to judge by eye. */
    oled_clear(&ctx->oled);
    oled_printf(&ctx->oled, 0U, 0U, "BRINGUP");
    oled_printf(&ctx->oled, 0U, 16U, "O%u M%u W%02X", ctx->oled_ready ? 1U : 0U, ctx->imu_ready ? 1U : 0U, ctx->imu_who_am_i);
    if (test_mode == BRINGUP_OUTPUT_TEST_SINGLE_STEPPER) {
        oled_printf(&ctx->oled, 0U, 32U, "%s S%u", bringup_test_get_stepper_axis_name(&ctx->stepper), ctx->stepper_stage);
        oled_printf(&ctx->oled, 0U, 48U, "F%.0f T%lu", ctx->stepper_command_hz, (unsigned long) (tick_ms / 1000U));
    } else if (test_mode == BRINGUP_OUTPUT_TEST_GRAY_SENSOR) {
        oled_printf(&ctx->oled, 0U, 32U, "B%02X H%u", ctx->line_sensor.raw_bits, ctx->line_sensor.hit_count);
        oled_printf(&ctx->oled, 0U, 48U, "%u%u%u%u%u%u%u%u",
            ctx->line_sensor.raw_state[0],
            ctx->line_sensor.raw_state[1],
            ctx->line_sensor.raw_state[2],
            ctx->line_sensor.raw_state[3],
            ctx->line_sensor.raw_state[4],
            ctx->line_sensor.raw_state[5],
            ctx->line_sensor.raw_state[6],
            ctx->line_sensor.raw_state[7]);
    } else if (test_mode == BRINGUP_OUTPUT_TEST_IMU) {
        oled_printf(&ctx->oled, 0U, 32U, "R%.1f P%.1f", ctx->imu_roll_deg, ctx->imu_pitch_deg);
        oled_printf(&ctx->oled, 0U, 48U, "Y%.1f T%lu", ctx->imu_yaw_deg, (unsigned long) (tick_ms / 1000U));
    } else {
        oled_printf(&ctx->oled, 0U, 32U, "TG%.2f AC%.2f", ctx->position_target_rev, ctx->position_actual_rev);
        oled_printf(&ctx->oled, 0U, 48U, "V%.2f O%.2f", ctx->left_speed_rps, ctx->left_command);
    }
    (void) oled_flush(&ctx->oled);
}

static void bringup_test_send_vofa(const bringup_test_context_t *ctx, uint32_t tick_ms)
{
    static const bringup_output_test_t test_mode = BRINGUP_TEST_MODE_DEFAULT;
#define BRINGUP_ENABLE_VOFA_STREAM    (1U)
#if BRINGUP_ENABLE_VOFA_STREAM
    static const proto_vofa_firewater_mode_t vofa_mode = PROTO_VOFA_FIREWATER_MODE_NAMED;
    static const proto_vofa_firewater_mode_t imu_vofa_mode = PROTO_VOFA_FIREWATER_MODE_RAW;
    static const proto_vofa_firewater_mode_t motor_vofa_mode = PROTO_VOFA_FIREWATER_MODE_RAW;
    static const char *const motor_vofa_names[2] = {
        "target_rev",
        "actual_rev",
    };
    static const char *const gray_vofa_names[8] = {
        "gray_1",
        "gray_2",
        "gray_3",
        "gray_4",
        "gray_5",
        "gray_6",
        "gray_7",
        "gray_8",
    };
    static const char *const imu_vofa_names[3] = {
        "roll",
        "pitch",
        "yaw",
    };
    proto_vofa_firewater_packet_t packet;
    float motor_vofa_channels[2] = {
        ctx->position_target_rev,
        ctx->position_actual_rev,
    };
    float gray_vofa_channels[8] = {
        (float) ctx->line_sensor.raw_state[0],
        (float) ctx->line_sensor.raw_state[1],
        (float) ctx->line_sensor.raw_state[2],
        (float) ctx->line_sensor.raw_state[3],
        (float) ctx->line_sensor.raw_state[4],
        (float) ctx->line_sensor.raw_state[5],
        (float) ctx->line_sensor.raw_state[6],
        (float) ctx->line_sensor.raw_state[7],
    };
    float imu_vofa_channels[3] = {
        ctx->imu_roll_deg,
        ctx->imu_pitch_deg,
        ctx->imu_yaw_deg,
    };

    /* Emit a compact bring-up packet so VOFA+/串口助手都能同时看外设状态和电机命令。 */
    packet.mode = vofa_mode;
    if (test_mode == BRINGUP_OUTPUT_TEST_GRAY_SENSOR) {
        packet.names = gray_vofa_names;
        packet.data = gray_vofa_channels;
        packet.count = 8U;
    } else if (test_mode == BRINGUP_OUTPUT_TEST_IMU) {
        packet.mode = imu_vofa_mode;
        packet.names = imu_vofa_names;
        packet.data = imu_vofa_channels;
        packet.count = 3U;
    } else {
        packet.mode = motor_vofa_mode;
        packet.names = motor_vofa_names;
        packet.data = motor_vofa_channels;
        packet.count = 2U;
    }
    proto_vofa_firewater_send_packet(&packet);
#else
    (void) ctx;
#endif
    (void) tick_ms;
}

static void bringup_test_set_position_target(bringup_test_context_t *ctx, float target_rev)
{
    ctx->position_target_rev = target_rev;
    if (BRINGUP_MOTOR_TEXT_LOG_ENABLE != 0U) {
        bsp_uart_debug_printf("target_rev=%.3f\r\n", ctx->position_target_rev);
    }
}

static bool bringup_test_parse_target_line(bringup_test_context_t *ctx, const char *line)
{
    const char *cursor = line;
    char *endptr;
    float value;

    if ((line[0] == 'z') || (line[0] == 'Z')) {
        ctx->encoder_driver.left.count = 0;
        ctx->encoder_driver.left.prev_count = 0;
        ctx->encoder_driver.left.delta_count = 0;
        ctx->encoder_driver.left.speed_rps = 0.0f;
        ctx->position_actual_rev = 0.0f;
        ctx->position_error_rev = 0.0f;
        ctx->left_speed_rps = 0.0f;
        ctx->left_speed_target_rps = 0.0f;
        cascade_pid_reset(&ctx->left_motor_pos_loop);
        bringup_test_set_position_target(ctx, 0.0f);
        if (BRINGUP_MOTOR_TEXT_LOG_ENABLE != 0U) {
            bsp_uart_debug_printf("encoder zeroed\r\n");
        }
        return true;
    }

    while ((*cursor != '\0') &&
        ((*cursor < '0') || (*cursor > '9')) &&
        (*cursor != '-') && (*cursor != '+') && (*cursor != '.')) {
        cursor++;
    }
    if (*cursor == '\0') {
        return false;
    }

    value = strtof(cursor, &endptr);
    if (endptr == cursor) {
        return false;
    }

    bringup_test_set_position_target(ctx, value);
    return true;
}

static const char *bringup_test_get_stepper_stage_name(uint8_t stage)
{
    switch (stage) {
        case 1U:
            return "FWD";
        case 3U:
            return "REV";
        default:
            return "IDLE";
    }
}

static const char *bringup_test_get_stepper_axis_name(const stepper_handle_t *handle)
{
    switch (handle->cfg.axis) {
        case BSP_STEPPER_AXIS_PITCH:
            return "PITCH";
        default:
            return "STEP";
    }
}
