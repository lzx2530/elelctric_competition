#ifndef PROTO_VOFA_FIREWATER_H
#define PROTO_VOFA_FIREWATER_H

#include "common/types.h"

typedef enum {
    PROTO_VOFA_FIREWATER_MODE_RAW = 0,
    PROTO_VOFA_FIREWATER_MODE_NAMED = 1,
} proto_vofa_firewater_mode_t;

typedef struct {
    proto_vofa_firewater_mode_t mode;
    const char *const *names;
    const float *data;
    uint16_t count;
} proto_vofa_firewater_packet_t;

/* Send one FireWater frame containing only numeric channel values. */
void proto_vofa_firewater_send(const float *data, uint16_t count);

/* Send one readable text frame in "name:=value" form for serial-monitor style debugging. */
void proto_vofa_firewater_send_named(const char *const *names, const float *data, uint16_t count);

/* Send one debug frame according to the selected output mode. */
void proto_vofa_firewater_send_packet(const proto_vofa_firewater_packet_t *packet);

#endif
