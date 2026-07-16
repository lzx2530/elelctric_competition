#ifndef PROTO_K230_H
#define PROTO_K230_H

#include "common/ringbuf.h"
#include "common/types.h"

#define K230_LINE_MAX_LENGTH    (80U)

typedef struct {
    bool valid;
    float dx_cm;
    float dy_cm;
    float distance_cm;
    float angle_deg;
    uint32_t frame_count;
} k230_frame_t;

typedef struct {
    char line[K230_LINE_MAX_LENGTH];
    uint8_t length;
    bool line_overflow;
    k230_frame_t latest_frame;
} k230_parser_t;

void proto_k230_init(k230_parser_t *parser);
/* Consume newline-terminated K230 text messages from UART ring buffer. */
bool proto_k230_process_ringbuf(k230_parser_t *parser, ringbuf_t *ringbuf, k230_frame_t *out_frame);
const k230_frame_t *proto_k230_get_latest(const k230_parser_t *parser);

#endif
