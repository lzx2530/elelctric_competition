#include "protocol/proto_vofa_firewater.h"

#include <stdio.h>
#include <string.h>

#include "bsp/bsp_uart.h"

static void proto_vofa_firewater_write_line(const char *line)
{
    bsp_uart_debug_write((const uint8_t *) line, (uint16_t) strlen(line));
}

void proto_vofa_firewater_send(const float *data, uint16_t count)
{
    char line[160];
    int offset = 0;
    uint16_t i;

    /* FireWater is plain text, so build one bounded CSV line and push it in one shot. */
    for (i = 0U; i < count; i++) {
        int written = snprintf(&line[offset], (size_t) (sizeof(line) - offset),
            (i == (count - 1U)) ? "%.3f\r\n" : "%.3f,", data[i]);
        if ((written <= 0) || ((offset + written) >= (int) sizeof(line))) {
            break;
        }
        offset += written;
    }

    if (offset > 0) {
        proto_vofa_firewater_write_line(line);
    }
}

void proto_vofa_firewater_send_named(const char *const *names, const float *data, uint16_t count)
{
    char line[256];
    int offset = 0;
    uint16_t i;

    /* Emit one key-value line so host-side reading does not depend on channel order memory. */
    for (i = 0U; i < count; i++) {
        const char *name = (names != NULL) ? names[i] : NULL;
        int written;

        if ((name == NULL) || (name[0] == '\0')) {
            name = "unnamed";
        }

        written = snprintf(&line[offset],
            (size_t) (sizeof(line) - offset),
            (i == (count - 1U)) ? "%s:=%.3f\r\n" : "%s:=%.3f, ",
            name,
            data[i]);
        if ((written <= 0) || ((offset + written) >= (int) sizeof(line))) {
            break;
        }
        offset += written;
    }

    if (offset > 0) {
        proto_vofa_firewater_write_line(line);
    }
}

void proto_vofa_firewater_send_packet(const proto_vofa_firewater_packet_t *packet)
{
    if ((packet == NULL) || (packet->data == NULL) || (packet->count == 0U)) {
        return;
    }

    /* Keep mode selection in one entry point so upper layers can switch formatting cheaply. */
    if (packet->mode == PROTO_VOFA_FIREWATER_MODE_NAMED) {
        proto_vofa_firewater_send_named(packet->names, packet->data, packet->count);
    } else {
        proto_vofa_firewater_send(packet->data, packet->count);
    }
}
