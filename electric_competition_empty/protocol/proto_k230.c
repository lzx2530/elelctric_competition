#include "protocol/proto_k230.h"

#include <string.h>

static uint8_t proto_k230_checksum(const uint8_t *data, uint8_t length)
{
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

void proto_k230_init(k230_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

bool proto_k230_process_ringbuf(k230_parser_t *parser, ringbuf_t *ringbuf, k230_frame_t *out_frame)
{
    uint8_t byte;

    while (ringbuf_pop_byte(ringbuf, &byte)) {
        /* 先用帧头做同步，丢弃噪声字节。 */
        if ((parser->index == 0U) && (byte != 0xAAU)) {
            continue;
        }
        if ((parser->index == 1U) && (byte != 0x55U)) {
            parser->index = 0U;
            continue;
        }

        parser->raw[parser->index++] = byte;
        if (parser->index < sizeof(parser->raw)) {
            continue;
        }

        parser->index = 0U;
        /* 一帧收满后再统一验和并更新最新结果。 */
        if (proto_k230_checksum(parser->raw, 7U) != parser->raw[7]) {
            continue;
        }

        parser->latest_frame.x_error = (int16_t) ((parser->raw[2] << 8) | parser->raw[3]);
        parser->latest_frame.y_error = (int16_t) ((parser->raw[4] << 8) | parser->raw[5]);
        parser->latest_frame.valid = ((parser->raw[6] & 0x01U) != 0U);
        parser->latest_frame.status = parser->raw[6];
        parser->latest_frame.frame_count++;

        if (out_frame != NULL) {
            *out_frame = parser->latest_frame;
        }
        return true;
    }

    return false;
}

const k230_frame_t *proto_k230_get_latest(const k230_parser_t *parser)
{
    return &parser->latest_frame;
}
