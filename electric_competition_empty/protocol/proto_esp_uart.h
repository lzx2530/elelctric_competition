#ifndef PROTO_ESP_UART_H
#define PROTO_ESP_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "common/ringbuf.h"

#define ESP_UART_PROTOCOL_VERSION 1U
#define ESP_UART_MAX_PAYLOAD_LENGTH 64U

typedef enum {
    ESP_UART_MESSAGE_HELLO = 0x01U,
    ESP_UART_MESSAGE_HEARTBEAT = 0x02U,
    ESP_UART_MESSAGE_TEXT = 0x10U,
    ESP_UART_MESSAGE_ACK = 0x7FU,
} esp_uart_message_type_t;

typedef struct {
    esp_uart_message_type_t type;
    uint8_t sequence;
    uint8_t payload_length;
    uint8_t payload[ESP_UART_MAX_PAYLOAD_LENGTH];
} esp_uart_frame_t;

typedef void (*esp_uart_frame_handler_t)(const esp_uart_frame_t *frame);

typedef struct {
    uint8_t parse_state;
    uint8_t payload_index;
    uint8_t calculated_crc;
    uint8_t next_sequence;
    esp_uart_frame_t frame;
    esp_uart_frame_handler_t frame_handler;
} esp_uart_protocol_t;

void proto_esp_uart_init(esp_uart_protocol_t *protocol, esp_uart_frame_handler_t frame_handler);
void proto_esp_uart_process_ringbuf(esp_uart_protocol_t *protocol, ringbuf_t *ringbuf);
bool proto_esp_uart_send(esp_uart_protocol_t *protocol, esp_uart_message_type_t type,
    const uint8_t *payload, uint8_t payload_length);

#endif
