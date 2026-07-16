#include "protocol/proto_k230.h"

#include <stdlib.h>
#include <string.h>

static bool proto_k230_parse_float_field(const char **cursor, float *value, char delimiter)
{
    char *endptr;

    if (**cursor == '\0') {
        return false;
    }

    *value = strtof(*cursor, &endptr);
    if (endptr == *cursor) {
        return false;
    }

    if (delimiter == '\0') {
        if (*endptr != '\0') {
            return false;
        }
    } else {
        if (*endptr != delimiter) {
            return false;
        }
        endptr++;
    }

    *cursor = endptr;
    return true;
}

static bool proto_k230_parse_line(const char *line, k230_frame_t *frame)
{
    const char *cursor;
    k230_frame_t parsed = *frame;

    if (strcmp(line, "AIM,SEARCH") == 0) {
        parsed.valid = false;
        parsed.dx_cm = 0.0f;
        parsed.dy_cm = 0.0f;
        parsed.distance_cm = 0.0f;
        parsed.angle_deg = 0.0f;
        *frame = parsed;
        return true;
    }

    if (strncmp(line, "AIM,", 4U) != 0) {
        return false;
    }

    cursor = line + 4U;
    if (!proto_k230_parse_float_field(&cursor, &parsed.dx_cm, ',')) {
        return false;
    }
    if (!proto_k230_parse_float_field(&cursor, &parsed.dy_cm, ',')) {
        return false;
    }
    if (!proto_k230_parse_float_field(&cursor, &parsed.distance_cm, ',')) {
        return false;
    }
    if (!proto_k230_parse_float_field(&cursor, &parsed.angle_deg, '\0')) {
        return false;
    }

    parsed.valid = true;
    *frame = parsed;
    return true;
}

void proto_k230_init(k230_parser_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

bool proto_k230_process_ringbuf(k230_parser_t *parser, ringbuf_t *ringbuf, k230_frame_t *out_frame)
{
    uint8_t byte;

    while (ringbuf_pop_byte(ringbuf, &byte)) {
        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            if ((parser->length == 0U) || parser->line_overflow) {
                parser->length = 0U;
                parser->line_overflow = false;
                continue;
            }

            parser->line[parser->length] = '\0';
            parser->length = 0U;
            if (!proto_k230_parse_line(parser->line, &parser->latest_frame)) {
                continue;
            }

            parser->latest_frame.frame_count++;
            if (out_frame != NULL) {
                *out_frame = parser->latest_frame;
            }
            return true;
        }

        if (parser->line_overflow) {
            continue;
        }

        if (parser->length >= (uint8_t) (K230_LINE_MAX_LENGTH - 1U)) {
            parser->length = 0U;
            parser->line_overflow = true;
            continue;
        }

        parser->line[parser->length++] = (char) byte;
    }

    return false;
}

const k230_frame_t *proto_k230_get_latest(const k230_parser_t *parser)
{
    return &parser->latest_frame;
}
