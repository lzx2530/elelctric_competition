#include "app/steelball_task.h"

#include <string.h>

#include "app/app_chassis.h"
#include "bsp/bsp_gpio.h"
#include "bsp/bsp_uart.h"
#include "common/math_util.h"
#include "protocol/ti_steelball_protocol.h"

#define STEELBALL_FRAME_MAX_LENGTH (SB_MAX_PAYLOAD_LENGTH + 8U)
#define STEELBALL_ACK_RETRY_MS 50U
#define STEELBALL_ACK_MAX_SENDS 3U
#define STEELBALL_STATUS_PERIOD_MS 100U
#define STEELBALL_VISION_TIMEOUT_MS 200U
#define STEELBALL_NEAR_CENTER_TOLERANCE_PERMILLE 80
#define STEELBALL_VISUAL_FORWARD_RPS 0.65f
#define STEELBALL_VISUAL_YAW_GAIN 0.0008f
#define STEELBALL_VISUAL_YAW_LIMIT_RPS 0.35f
#define STEELBALL_YAW_SIGN 1.0f
#define STEELBALL_NEAR_CREEP_CALIBRATED 0U
#define STEELBALL_NEAR_CREEP_DISTANCE_MM 0.0f
#define STEELBALL_NEAR_CREEP_SPEED_RPS 0.25f
#define STEELBALL_NEAR_CREEP_TIMEOUT_MS 1500U
#define STEELBALL_MAGNET_HOLD_MS 300U

#define STEELBALL_FAULT_ACK_TIMEOUT 1U
#define STEELBALL_FAULT_VISION_TIMEOUT 2U
#define STEELBALL_FAULT_VISION_LOST 3U
#define STEELBALL_FAULT_NEAR_UNCALIBRATED 4U
#define STEELBALL_FAULT_NEAR_TIMEOUT 5U
#define STEELBALL_FAULT_INVALID_STATE 6U

typedef struct {
    uint8_t bytes[STEELBALL_FRAME_MAX_LENGTH];
    uint8_t length;
} steelball_parser_t;

typedef struct {
    uint8_t state;
    uint8_t mission_id;
    uint8_t sequence;
    uint8_t start_sequence;
    uint8_t start_sends;
    bool mission_active;
    bool waiting_start_ack;
    bool magnet_on;
    uint8_t capture_result;
    uint32_t start_tx_ms;
    uint32_t now_ms;
    uint32_t last_status_tx_ms;
    uint32_t last_vision_rx_ms;
    uint32_t magnet_hold_start_ms;
    uint32_t near_start_ms;
    int32_t near_left_start_count;
    int32_t near_right_start_count;
    uint16_t near_travel_mm;
    uint8_t vision_state;
    uint8_t vision_flags;
    int16_t center_x_permille;
    int16_t center_y_permille;
    uint16_t ball_diameter_px;
    uint16_t confidence_permille;
    uint8_t stable_frames;
    uint8_t target_count;
    steelball_parser_t parser;
} steelball_context_t;

static steelball_context_t g_steelball;

static bool steelball_elapsed(uint32_t now_ms, uint32_t then_ms, uint32_t period_ms)
{
    return (uint32_t)(now_ms - then_ms) >= period_ms;
}

static int16_t steelball_read_i16_le(const uint8_t *data)
{
    return (int16_t)sb_read_u16_le(data);
}

static void steelball_set_safe_stop(void)
{
    app_chassis_stop();
    bsp_gpio_set_magnet(false);
    g_steelball.magnet_on = false;
}

static void steelball_enter_fault(uint8_t reason)
{
    bsp_uart_debug_printf("steel fault reason=%u state=%u vision=%u flags=0x%02X ball=%u cx=%d now=%lu last=%lu\r\n",
        reason, g_steelball.state, g_steelball.vision_state, g_steelball.vision_flags,
        g_steelball.ball_diameter_px, g_steelball.center_x_permille,
        (unsigned long)g_steelball.now_ms, (unsigned long)g_steelball.last_vision_rx_ms);
    steelball_set_safe_stop();
    g_steelball.waiting_start_ack = false;
    g_steelball.capture_result = SB_CAPTURE_FAILED;
    g_steelball.state = SB_TI_FAULT;
}

static void steelball_send_frame(uint8_t type, const uint8_t *payload, uint8_t payload_length)
{
    uint8_t frame[STEELBALL_FRAME_MAX_LENGTH];
    uint16_t crc;
    uint8_t index = 0U;

    if (payload_length > SB_MAX_PAYLOAD_LENGTH) {
        return;
    }

    frame[index++] = SB_SOF_0;
    frame[index++] = SB_SOF_1;
    frame[index++] = SB_PROTOCOL_VERSION;
    frame[index++] = type;
    frame[index++] = g_steelball.sequence++;
    frame[index++] = payload_length;
    if (payload_length > 0U) {
        memcpy(&frame[index], payload, payload_length);
        index += payload_length;
    }
    crc = sb_crc16_ccitt_false(&frame[2], (uint16_t)(4U + payload_length));
    sb_write_u16_le(&frame[index], crc);
    index += 2U;
    (void)bsp_uart_k230_write(frame, index);
}

static void steelball_send_mission_command(uint8_t command)
{
    uint8_t payload[SB_MISSION_COMMAND_LENGTH];

    payload[0] = g_steelball.mission_id;
    payload[1] = command;
    g_steelball.start_sequence = g_steelball.sequence;
    steelball_send_frame(SB_TYPE_MISSION_COMMAND, payload, sizeof(payload));
}

static void steelball_send_status(void)
{
    uint8_t payload[SB_TI_STATUS_LENGTH];
    uint8_t feedback_flags = (1U << 2);

    if (g_steelball.magnet_on) {
        feedback_flags |= (1U << 1);
    }
    payload[0] = g_steelball.mission_id;
    payload[1] = g_steelball.state;
    payload[2] = feedback_flags;
    payload[3] = g_steelball.capture_result;
    sb_write_u16_le(&payload[4], g_steelball.near_travel_mm);
    steelball_send_frame(SB_TYPE_TI_STATUS, payload, sizeof(payload));
}

static bool steelball_vision_is_stable_target(void)
{
    return ((g_steelball.vision_flags & (SB_VISION_FLAG_TARGET_VALID | SB_VISION_FLAG_TARGET_STABLE)) ==
        (SB_VISION_FLAG_TARGET_VALID | SB_VISION_FLAG_TARGET_STABLE)) &&
        ((g_steelball.vision_state == SB_VISION_TRACKING) ||
            (g_steelball.vision_state == SB_VISION_NEAR_HANDOFF));
}

static bool steelball_vision_is_near_centered(void)
{
    return steelball_vision_is_stable_target() &&
        (g_steelball.vision_state == SB_VISION_NEAR_HANDOFF) &&
        (g_steelball.center_x_permille <= STEELBALL_NEAR_CENTER_TOLERANCE_PERMILLE) &&
        (g_steelball.center_x_permille >= -STEELBALL_NEAR_CENTER_TOLERANCE_PERMILLE);
}

static void steelball_handle_frame(const uint8_t *frame, uint8_t length)
{
    uint8_t type;
    uint8_t sequence;
    uint8_t payload_length;
    const uint8_t *payload;

    if ((length < 8U) || (frame[2] != SB_PROTOCOL_VERSION)) {
        return;
    }
    type = frame[3];
    sequence = frame[4];
    payload_length = frame[5];
    payload = &frame[6];

    if ((type == SB_TYPE_ACK) && (payload_length == SB_ACK_LENGTH) &&
        g_steelball.waiting_start_ack && (payload[0] == SB_TYPE_MISSION_COMMAND) &&
        (payload[1] == g_steelball.start_sequence) && (payload[2] == SB_ACK_OK) &&
        (payload[3] == g_steelball.mission_id)) {
        bsp_uart_debug_printf("steel ack mid=%u seq=%u\r\n", g_steelball.mission_id, sequence);
        g_steelball.waiting_start_ack = false;
        g_steelball.mission_active = true;
        g_steelball.capture_result = SB_CAPTURE_NONE;
        g_steelball.state = SB_TI_FOLLOW_LINE;
        app_chassis_enable_line_follow();
        (void)sequence;
        return;
    }

    if ((type == SB_TYPE_VISION_REPORT) && (payload_length == SB_VISION_REPORT_LENGTH) &&
        g_steelball.mission_active && (payload[0] == g_steelball.mission_id)) {
        g_steelball.vision_state = payload[1];
        g_steelball.vision_flags = payload[2];
        g_steelball.center_x_permille = steelball_read_i16_le(&payload[3]);
        g_steelball.center_y_permille = steelball_read_i16_le(&payload[5]);
        g_steelball.ball_diameter_px = sb_read_u16_le(&payload[7]);
        g_steelball.confidence_permille = sb_read_u16_le(&payload[11]);
        g_steelball.stable_frames = payload[13];
        g_steelball.target_count = payload[14];
        /* A frame is trusted only after all framing checks and mission filtering above. */
        g_steelball.last_vision_rx_ms = g_steelball.now_ms;
    }
}

void steelball_init(void)
{
    memset(&g_steelball, 0, sizeof(g_steelball));
    g_steelball.state = SB_TI_IDLE;
    g_steelball.capture_result = SB_CAPTURE_NONE;
    steelball_set_safe_stop();
}

void steelball_rx_bytes(const uint8_t *data, uint16_t length)
{
    uint16_t input_index;

    for (input_index = 0U; input_index < length; ++input_index) {
        steelball_parser_t *parser = &g_steelball.parser;
        uint8_t byte = data[input_index];
        uint8_t payload_length;
        uint8_t frame_length;
        uint16_t received_crc;
        uint16_t calculated_crc;

        if ((parser->length == 0U) && (byte != SB_SOF_0)) {
            continue;
        }
        if (parser->length == 1U) {
            if (byte == SB_SOF_1) {
                parser->bytes[parser->length++] = byte;
            } else if (byte == SB_SOF_0) {
                parser->bytes[0] = byte;
            } else {
                parser->length = 0U;
            }
            continue;
        }
        if (parser->length == 0U) {
            parser->bytes[parser->length++] = byte;
            continue;
        }
        if (parser->length >= STEELBALL_FRAME_MAX_LENGTH) {
            parser->length = 0U;
            continue;
        }
        parser->bytes[parser->length++] = byte;
        if (parser->length < 6U) {
            continue;
        }
        payload_length = parser->bytes[5];
        if (payload_length > SB_MAX_PAYLOAD_LENGTH) {
            parser->length = 0U;
            continue;
        }
        frame_length = (uint8_t)(payload_length + 8U);
        if (parser->length < frame_length) {
            continue;
        }
        received_crc = sb_read_u16_le(&parser->bytes[6U + payload_length]);
        calculated_crc = sb_crc16_ccitt_false(&parser->bytes[2], (uint16_t)(4U + payload_length));
        if ((parser->bytes[2] == SB_PROTOCOL_VERSION) && (received_crc == calculated_crc)) {
            steelball_handle_frame(parser->bytes, frame_length);
        }
        parser->length = 0U;
    }
}

bool steelball_start_mission(void)
{
    if (g_steelball.waiting_start_ack || g_steelball.mission_active) {
        return false;
    }

    g_steelball.mission_id++;
    if (g_steelball.mission_id == 0U) {
        g_steelball.mission_id = 1U;
    }
    g_steelball.start_sends = 1U;
    g_steelball.waiting_start_ack = true;
    g_steelball.capture_result = SB_CAPTURE_NONE;
    g_steelball.vision_state = SB_VISION_READY;
    g_steelball.vision_flags = 0U;
    g_steelball.near_travel_mm = 0U;
    g_steelball.state = SB_TI_IDLE;
    g_steelball.start_tx_ms = g_steelball.now_ms;
    g_steelball.last_status_tx_ms = g_steelball.now_ms - STEELBALL_STATUS_PERIOD_MS;
    steelball_set_safe_stop();
    steelball_send_mission_command(SB_CMD_START);
    return true;
}

void steelball_abort_mission(void)
{
    if ((g_steelball.mission_active || g_steelball.waiting_start_ack) && (g_steelball.mission_id != 0U)) {
        steelball_send_mission_command(SB_CMD_ABORT);
    }
    steelball_set_safe_stop();
    g_steelball.waiting_start_ack = false;
    g_steelball.mission_active = false;
    g_steelball.capture_result = SB_CAPTURE_NONE;
    g_steelball.state = SB_TI_ABORTED;
}

void steelball_reset_mission(void)
{
    if (g_steelball.mission_id != 0U) {
        steelball_send_mission_command(SB_CMD_RESET);
    }
    steelball_set_safe_stop();
    g_steelball.waiting_start_ack = false;
    g_steelball.mission_active = false;
    g_steelball.capture_result = SB_CAPTURE_NONE;
    g_steelball.state = SB_TI_IDLE;
}

void steelball_task_10ms(uint32_t now_ms)
{
    float yaw_rps;

    g_steelball.now_ms = now_ms;

    if (g_steelball.waiting_start_ack) {
        if (g_steelball.start_sends >= STEELBALL_ACK_MAX_SENDS) {
            if (steelball_elapsed(now_ms, g_steelball.start_tx_ms, STEELBALL_ACK_RETRY_MS)) {
                bsp_uart_debug_printf("steel ack timeout\r\n");
                steelball_enter_fault(STEELBALL_FAULT_ACK_TIMEOUT);
            }
        } else if (steelball_elapsed(now_ms, g_steelball.start_tx_ms, STEELBALL_ACK_RETRY_MS)) {
            g_steelball.start_sends++;
            steelball_send_mission_command(SB_CMD_START);
            g_steelball.start_tx_ms = now_ms;
        }
        return;
    }

    if (!g_steelball.mission_active) {
        return;
    }

    if (steelball_elapsed(now_ms, g_steelball.last_status_tx_ms, STEELBALL_STATUS_PERIOD_MS)) {
        steelball_send_status();
        g_steelball.last_status_tx_ms = now_ms;
    }

    if ((g_steelball.state == SB_TI_VISUAL_APPROACH) &&
        steelball_elapsed(now_ms, g_steelball.last_vision_rx_ms, STEELBALL_VISION_TIMEOUT_MS)) {
        steelball_enter_fault(STEELBALL_FAULT_VISION_TIMEOUT);
        return;
    }

    switch (g_steelball.state) {
        case SB_TI_FOLLOW_LINE:
            app_chassis_enable_line_follow();
            if (steelball_vision_is_stable_target()) {
                g_steelball.state = SB_TI_VISUAL_APPROACH;
            }
            break;
        case SB_TI_VISUAL_APPROACH:
            if ((g_steelball.vision_state == SB_VISION_LOST) || (g_steelball.vision_state == SB_VISION_FAULT)) {
                steelball_enter_fault(STEELBALL_FAULT_VISION_LOST);
                break;
            }
            yaw_rps = math_clampf(STEELBALL_YAW_SIGN * STEELBALL_VISUAL_YAW_GAIN *
                (float)g_steelball.center_x_permille, -STEELBALL_VISUAL_YAW_LIMIT_RPS,
                STEELBALL_VISUAL_YAW_LIMIT_RPS);
            app_chassis_set_external_drive(STEELBALL_VISUAL_FORWARD_RPS, yaw_rps);
            if (steelball_vision_is_near_centered()) {
#if STEELBALL_NEAR_CREEP_CALIBRATED
                app_chassis_get_encoder_counts(&g_steelball.near_left_start_count,
                    &g_steelball.near_right_start_count);
                g_steelball.near_start_ms = now_ms;
                g_steelball.state = SB_TI_NEAR_CREEP;
#else
                steelball_enter_fault(STEELBALL_FAULT_NEAR_UNCALIBRATED);
#endif
            }
            break;
        case SB_TI_NEAR_CREEP:
#if STEELBALL_NEAR_CREEP_CALIBRATED
            g_steelball.near_travel_mm = (uint16_t)app_chassis_get_average_distance_mm(
                g_steelball.near_left_start_count, g_steelball.near_right_start_count);
            if (steelball_elapsed(now_ms, g_steelball.near_start_ms, STEELBALL_NEAR_CREEP_TIMEOUT_MS)) {
                steelball_enter_fault(STEELBALL_FAULT_NEAR_TIMEOUT);
            } else if ((float)g_steelball.near_travel_mm >= STEELBALL_NEAR_CREEP_DISTANCE_MM) {
                app_chassis_stop();
                bsp_gpio_set_magnet(true);
                g_steelball.magnet_on = true;
                g_steelball.magnet_hold_start_ms = now_ms;
                g_steelball.state = SB_TI_MAGNET_HOLD;
            } else {
                app_chassis_set_external_drive(STEELBALL_NEAR_CREEP_SPEED_RPS, 0.0f);
            }
#else
            steelball_enter_fault(STEELBALL_FAULT_NEAR_UNCALIBRATED);
#endif
            break;
        case SB_TI_MAGNET_HOLD:
            app_chassis_stop();
            if (steelball_elapsed(now_ms, g_steelball.magnet_hold_start_ms, STEELBALL_MAGNET_HOLD_MS)) {
                g_steelball.capture_result = SB_CAPTURE_EXECUTED_UNVERIFIED;
                g_steelball.state = SB_TI_SUCCESS_UNVERIFIED;
            }
            break;
        case SB_TI_SUCCESS_UNVERIFIED:
            app_chassis_stop();
            break;
        default:
            steelball_enter_fault(STEELBALL_FAULT_INVALID_STATE);
            break;
    }
}

uint8_t steelball_get_state(void)
{
    return g_steelball.state;
}

void steelball_get_vision_snapshot(steelball_vision_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    snapshot->mission_id = g_steelball.mission_id;
    snapshot->vision_state = g_steelball.vision_state;
    snapshot->vision_flags = g_steelball.vision_flags;
    snapshot->center_x_permille = g_steelball.center_x_permille;
    snapshot->center_y_permille = g_steelball.center_y_permille;
    snapshot->ball_diameter_px = g_steelball.ball_diameter_px;
    snapshot->confidence_permille = g_steelball.confidence_permille;
    snapshot->stable_frames = g_steelball.stable_frames;
    snapshot->target_count = g_steelball.target_count;
}
