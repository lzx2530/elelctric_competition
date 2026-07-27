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
#include "app/app_control_scheduler.h"
#include "app/app_imu.h"
#include "app/app_isr.h"
#include "app/steelball_task.h"
#include "app/app_turret.h"
#include "app/app_ui.h"
#include "algo/algo_filter.h"
#include "algo/algo_fusion.h"
#include "algo/algo_pid.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_i2c.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_uart.h"
#include "common/math_util.h"
#include "drivers/drv_encoder_ab.h"
#include "drivers/drv_line_sensor.h"
#include "drivers/drv_motor_dc.h"
#include "drivers/drv_mpu9250.h"
#include "drivers/drv_oled_ssd1306.h"
#include "drivers/drv_stepper.h"
#include "protocol/proto_esp_uart.h"
#include "protocol/proto_vofa_firewater.h"
#include "ti_msp_dl_config.h"

#include <stdlib.h>
#include <string.h>

typedef enum {
    APP_RUN_MODE_BRINGUP_TEST = 0,
    APP_RUN_MODE_VEHICLE = 1,
} app_run_mode_t;

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
static esp_uart_protocol_t g_esp_uart_protocol;

static void run_vehicle_app(void);
static void vehicle_poll_esp_uart(void);
static void vehicle_handle_esp_frame(const esp_uart_frame_t *frame);
static void run_bringup_test(void);
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
    } else {
        run_vehicle_app();
    }

    while (1) {
    }
}

static void run_vehicle_app(void)
{
    scheduler_flags_t scheduler_flags;
    uint32_t last_chassis_control_tick_ms = 0U;
    uint32_t last_esp_heartbeat_ms = 0U;
    ringbuf_t *k230_ringbuf;
    static const proto_vofa_firewater_mode_t vofa_mode = PROTO_VOFA_FIREWATER_MODE_NAMED;
    static const char *const vofa_names[15] = {
        "line_error",
        "line_bits",
        "line_state",
        "line_lost",
        "left_target_rps",
        "right_target_rps",
        "left_speed_rps",
        "right_speed_rps",
        "left_output",
        "right_output",
        "ball_diameter_px",
        "vision_state",
        "vision_center_x",
        "vision_flags",
        "vision_stable_frames",
    };

    SYSCFG_DL_init();

    bsp_gpio_init();
    bsp_pwm_init();
    bsp_i2c_init();
    bsp_uart_init();
    bsp_uart_enable_irqs();
    bsp_pwm_start_all();

    k230_ringbuf = bsp_uart_get_k230_ringbuf();

    app_chassis_init();
    app_isr_set_encoder_driver(app_chassis_get_encoder_driver());
    app_imu_init();
    app_turret_init();
    app_ui_init();
    app_control_scheduler_init();
    steelball_init();
    proto_esp_uart_init(&g_esp_uart_protocol, vehicle_handle_esp_frame);
    (void)proto_esp_uart_send(&g_esp_uart_protocol, ESP_UART_MESSAGE_HELLO,
        (const uint8_t *)"TI UART2 ready", 14U);

    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);

    bsp_uart_debug_printf("system init done\r\n");

    while (1) {
        uint8_t k230_bytes[64];
        uint16_t k230_length = 0U;
        uint8_t byte;

        bsp_uart_k230_poll_rx();
        while ((k230_length < sizeof(k230_bytes)) && ringbuf_pop_byte(k230_ringbuf, &byte)) {
            k230_bytes[k230_length++] = byte;
        }
        if (k230_length > 0U) {
            steelball_rx_bytes(k230_bytes, k230_length);
        }
        vehicle_poll_esp_uart();

        /* The scheduler snapshots and clears flags so each task consumes one tick at most once. */
        app_control_scheduler_fetch(&scheduler_flags);

        if (scheduler_flags.line_5ms) {
            app_chassis_line_task();
        }
        if (scheduler_flags.imu_10ms) {
            app_imu_task();
            steelball_task_10ms(scheduler_flags.tick_ms);
            if ((uint32_t)(scheduler_flags.tick_ms - last_esp_heartbeat_ms) >= 1000U) {
                last_esp_heartbeat_ms = scheduler_flags.tick_ms;
                (void)proto_esp_uart_send(&g_esp_uart_protocol, ESP_UART_MESSAGE_HEARTBEAT,
                    (const uint8_t *)"TI alive", 8U);
            }
        }
        if (scheduler_flags.control_1khz) {
            uint32_t elapsed_ms = scheduler_flags.tick_ms - last_chassis_control_tick_ms;
            float chassis_elapsed_s;

            last_chassis_control_tick_ms = scheduler_flags.tick_ms;
            chassis_elapsed_s = 0.001f * (float) elapsed_ms;
            app_chassis_control_task(0.001f, chassis_elapsed_s);
            app_turret_control_task(0.001f);
        }
        if (scheduler_flags.oled_50ms) {
            app_ui_refresh(app_chassis_get_snapshot(), app_turret_get_snapshot(), app_imu_get_snapshot());
        }
        if (scheduler_flags.debug_100ms) {
            const chassis_snapshot_t *chassis = app_chassis_get_snapshot();
            steelball_vision_snapshot_t vision;
            proto_vofa_firewater_packet_t vofa_packet;
            /* Keep one fixed set of debug variables and switch only the text formatting mode. */
            float vofa_channels[15] = {
                chassis->line_error,
                (float) chassis->line_bits,
                (float) chassis->line_state,
                chassis->line_lost ? 1.0f : 0.0f,
                chassis->left_target_rps,
                chassis->right_target_rps,
                chassis->left_speed_rps,
                chassis->right_speed_rps,
                chassis->left_output,
                chassis->right_output,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
            };
            steelball_get_vision_snapshot(&vision);
            vofa_channels[10] = (float)vision.ball_diameter_px;
            vofa_channels[11] = (float)vision.vision_state;
            vofa_channels[12] = (float)vision.center_x_permille;
            vofa_channels[13] = (float)vision.vision_flags;
            vofa_channels[14] = (float)vision.stable_frames;
            vofa_packet.mode = vofa_mode;
            vofa_packet.names = vofa_names;
            vofa_packet.data = vofa_channels;
            vofa_packet.count = 15U;
            proto_vofa_firewater_send_packet(&vofa_packet);
            bsp_gpio_toggle_led();
        }
    }
}

static void vehicle_poll_esp_uart(void)
{
    bsp_uart_esp_poll_rx();
    proto_esp_uart_process_ringbuf(&g_esp_uart_protocol, bsp_uart_get_esp_ringbuf());
}

static void vehicle_handle_esp_frame(const esp_uart_frame_t *frame)
{
    static const uint8_t ack_payload_length = 2U;
    uint8_t ack_payload[2];
    uint8_t payload_index;

    if (frame->type != ESP_UART_MESSAGE_TEXT) {
        return;
    }

    bsp_uart_debug_printf("esp text seq=%u len=%u: ", frame->sequence,
        frame->payload_length);
    for (payload_index = 0U; payload_index < frame->payload_length; ++payload_index) {
        uint8_t value = frame->payload[payload_index];
        bsp_uart_debug_write_byte((value >= 32U) && (value <= 126U) ? value : (uint8_t)'.');
    }
    bsp_uart_debug_write_str("\r\n");

    if ((frame->payload_length == 11U) &&
        (memcmp(frame->payload, "steel start", frame->payload_length) == 0)) {
        bsp_uart_debug_printf("steel start %s\r\n", steelball_start_mission() ? "sent" : "busy");
    } else if ((frame->payload_length == 11U) &&
        (memcmp(frame->payload, "steel abort", frame->payload_length) == 0)) {
        steelball_abort_mission();
        bsp_uart_debug_printf("steel abort\r\n");
    } else if ((frame->payload_length == 11U) &&
        (memcmp(frame->payload, "steel reset", frame->payload_length) == 0)) {
        steelball_reset_mission();
        bsp_uart_debug_printf("steel reset\r\n");
    }

    ack_payload[0] = frame->sequence;
    ack_payload[1] = (uint8_t)frame->type;
    (void)proto_esp_uart_send(&g_esp_uart_protocol, ESP_UART_MESSAGE_ACK,
        ack_payload, ack_payload_length);
}

static void run_bringup_test(void)
{
    scheduler_flags_t scheduler_flags;

    SYSCFG_DL_init();

    bsp_gpio_init();
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
        .axis = BSP_STEPPER_AXIS_YAW,
        .dir_output = BSP_DIR_YAW,
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
        ctx->imu_ready = (mpu9250_init(&ctx->imu, 0x68U) == STATUS_OK);
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
        case BSP_STEPPER_AXIS_YAW:
            return "YAW";
        case BSP_STEPPER_AXIS_PITCH:
            return "PITCH";
        default:
            return "STEP";
    }
}
