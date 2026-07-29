#ifndef PROTO_K230_H
#define PROTO_K230_H

#include "common/ringbuf.h"
#include "common/types.h"

#define K230_PROTOCOL_VERSION                 (0x01U)
#define K230_PROTOCOL_MAX_PAYLOAD_LENGTH      (32U)
#define K230_PROTOCOL_MAX_FRAME_LENGTH        (40U)

#define K230_PROTOCOL_TYPE_BALL_REPORT        (0x12U)
#define K230_PROTOCOL_TYPE_TASK_START          (0x13U)

#define K230_BALL_FLAG_VALID                  (0x01U)
#define K230_BALL_FLAG_STABLE                 (0x02U)
#define K230_BALL_FLAG_IN_ROD                  (0x04U)
#define K230_BALL_FLAG_REFERENCE_READY         (0x08U)
#define K230_BALL_FLAG_STOP_REQUEST            (0x10U)

typedef enum {
    K230_MISSION_COMMAND_START = 1U,
    K230_MISSION_COMMAND_ABORT = 2U,
    K230_MISSION_COMMAND_RESET = 3U,
} k230_mission_command_t;

typedef struct {
    bool valid;
    bool stable;
    bool reference_ready;
    bool stop_requested;
    int16_t position_mm;
    uint16_t confidence_permille;
    uint16_t ball_diameter_px;
    uint8_t stable_frames;
    uint8_t status;
    uint8_t sequence;
    uint32_t frame_count;
} k230_ball_report_t;

typedef struct {
    uint8_t type;
    uint8_t task_flag;
    bool valid;
    int16_t x_error;
    int16_t y_error;
    uint8_t status;
    uint32_t frame_count;
    k230_ball_report_t ball;
} k230_frame_t;

typedef struct {
    uint8_t raw[K230_PROTOCOL_MAX_FRAME_LENGTH];
    uint8_t index;
    k230_frame_t latest_frame;
} k230_parser_t;

void proto_k230_init(k230_parser_t *parser);
bool proto_k230_process_ringbuf(k230_parser_t *parser, ringbuf_t *ringbuf, k230_frame_t *out_frame);
const k230_frame_t *proto_k230_get_latest(const k230_parser_t *parser);
uint16_t proto_k230_crc16_ccitt_false(const uint8_t *data, uint16_t length);
#endif
