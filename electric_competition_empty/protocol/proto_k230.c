#include "protocol/proto_k230.h"

#include <string.h>

static uint16_t proto_k230_read_u16_le(const uint8_t *data)
{
    return (uint16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8U));
}

static int16_t proto_k230_read_i16_le(const uint8_t *data)
{
    return (int16_t) proto_k230_read_u16_le(data);
}

static void proto_k230_reset_parser(k230_parser_t *parser)
{
    parser->index = 0U;
}

static bool proto_k230_parse_ball_report(k230_parser_t *parser, k230_frame_t *out_frame)
{
    const uint8_t *payload = &parser->raw[6];
    k230_frame_t *frame = &parser->latest_frame;

    if (parser->raw[3] != K230_PROTOCOL_TYPE_BALL_REPORT ||
        (parser->raw[5] != 5U && parser->raw[5] != 7U)) {
        return false;
    }

    frame->ball.status = payload[0];
    frame->ball.valid = (payload[0] & (K230_BALL_FLAG_VALID | K230_BALL_FLAG_IN_ROD)) ==
        (K230_BALL_FLAG_VALID | K230_BALL_FLAG_IN_ROD);
    frame->ball.stable = frame->ball.valid && (payload[0] & K230_BALL_FLAG_STABLE) != 0U;
    frame->ball.reference_ready = (payload[0] & K230_BALL_FLAG_REFERENCE_READY) != 0U;
    frame->ball.stop_requested = (payload[0] & K230_BALL_FLAG_STOP_REQUEST) != 0U;
    frame->ball.position_mm = proto_k230_read_i16_le(&payload[1]);
    frame->ball.confidence_permille = proto_k230_read_u16_le(&payload[3]);
    frame->ball.actual_position_present = parser->raw[5] == 7U;
    frame->ball.actual_position_valid = false;
    frame->ball.actual_position_mm = 0;
    if (frame->ball.actual_position_present && frame->ball.valid) {
        frame->ball.actual_position_mm = proto_k230_read_i16_le(&payload[5]);
        frame->ball.actual_position_valid =
            frame->ball.actual_position_mm != INT16_MIN;
    }
    frame->ball.ball_diameter_px = 0U;
    frame->ball.stable_frames = 0U;
    frame->ball.sequence = parser->raw[4];
    frame->ball.frame_count++;

    frame->type = K230_PROTOCOL_TYPE_BALL_REPORT;
    frame->task_flag = 0U;
    frame->valid = frame->ball.valid;
    frame->x_error = frame->ball.position_mm;
    frame->y_error = 0;
    frame->status = frame->ball.status;
    frame->frame_count++;

    if (out_frame != NULL) {
        *out_frame = *frame;
    }
    return true;
}

static bool proto_k230_parse_task_start(k230_parser_t *parser, k230_frame_t *out_frame)
{
    k230_frame_t frame;

    if (parser->raw[3] != K230_PROTOCOL_TYPE_TASK_START || parser->raw[5] != 1U ||
        (parser->raw[6] < 1U) || (parser->raw[6] > 6U)) {
        return false;
    }

    memset(&frame, 0, sizeof(frame));
    frame.type = K230_PROTOCOL_TYPE_TASK_START;
    frame.task_flag = parser->raw[6];
    if (out_frame != NULL) {
        *out_frame = frame;
    }
    return true;
}

uint16_t proto_k230_crc16_ccitt_false(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;

    for (index = 0U; index < length; index++) {
        uint8_t bit;
        crc ^= (uint16_t) data[index] << 8U;
        for (bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) != 0U ? (uint16_t) ((crc << 1U) ^ 0x1021U) : (uint16_t) (crc << 1U);
        }
    }
    return crc;
}

void proto_k230_init(k230_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

bool proto_k230_process_ringbuf(k230_parser_t *parser, ringbuf_t *ringbuf, k230_frame_t *out_frame)
{
    uint8_t byte;

    while (ringbuf_pop_byte(ringbuf, &byte)) {
        uint8_t payload_length;
        uint8_t expected_length;
        uint16_t received_crc;
        uint16_t calculated_crc;

        if (parser->index == 0U) {
            if (byte != 0xAAU) {
                continue;
            }
        } else if (parser->index == 1U && byte != 0x55U) {
            parser->index = byte == 0xAAU ? 1U : 0U;
            continue;
        }

        parser->raw[parser->index++] = byte;
        if (parser->index < 6U) {
            continue;
        }

        payload_length = parser->raw[5];
        if (payload_length > K230_PROTOCOL_MAX_PAYLOAD_LENGTH) {
            proto_k230_reset_parser(parser);
            continue;
        }

        expected_length = (uint8_t) (8U + payload_length);
        if (parser->index < expected_length) {
            continue;
        }

        received_crc = proto_k230_read_u16_le(&parser->raw[6U + payload_length]);
        calculated_crc = proto_k230_crc16_ccitt_false(&parser->raw[2], (uint16_t) (4U + payload_length));
        if (parser->raw[2] == K230_PROTOCOL_VERSION && received_crc == calculated_crc) {
            bool parsed;

            if (parser->raw[3] == K230_PROTOCOL_TYPE_BALL_REPORT) {
                parsed = proto_k230_parse_ball_report(parser, out_frame);
            } else if (parser->raw[3] == K230_PROTOCOL_TYPE_TASK_START) {
                parsed = proto_k230_parse_task_start(parser, out_frame);
            } else {
                parsed = false;
            }
            proto_k230_reset_parser(parser);
            if (parsed) {
                return true;
            }
            continue;
        }

        proto_k230_reset_parser(parser);
    }

    return false;
}

const k230_frame_t *proto_k230_get_latest(const k230_parser_t *parser)
{
    return &parser->latest_frame;
}
