#ifndef PROTO_K230_H
#define PROTO_K230_H

#include "common/ringbuf.h"
#include "common/types.h"

typedef struct {
    bool valid;
    int16_t x_error;
    int16_t y_error;
    uint8_t status;
    uint32_t frame_count;
} k230_frame_t;

/* 当前协议假定固定 8 字节帧：帧头 + xy 偏差 + 状态 + 校验 */
typedef struct {
    uint8_t raw[8];
    uint8_t index;
    k230_frame_t latest_frame;
} k230_parser_t;

void proto_k230_init(k230_parser_t *parser);
/* 从 UART 环形缓冲区持续取字节，解析出完整帧时返回 true */
bool proto_k230_process_ringbuf(k230_parser_t *parser, ringbuf_t *ringbuf, k230_frame_t *out_frame);
const k230_frame_t *proto_k230_get_latest(const k230_parser_t *parser);

#endif
