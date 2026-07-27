#include "protocol/proto_esp_uart.h"

#include <string.h>

#include "bsp/bsp_uart.h"

#define ESP_UART_SOF_0 0xAAU
#define ESP_UART_SOF_1 0x55U

enum {
    ESP_UART_WAIT_SOF_0 = 0U,
    ESP_UART_WAIT_SOF_1,
    ESP_UART_WAIT_VERSION,
    ESP_UART_WAIT_TYPE,
    ESP_UART_WAIT_SEQUENCE,
    ESP_UART_WAIT_LENGTH,
    ESP_UART_WAIT_PAYLOAD,
    ESP_UART_WAIT_CRC,
};

static uint8_t proto_esp_uart_crc8_update(uint8_t crc, uint8_t value)
{
    uint8_t bit;

    crc ^= value;
    for (bit = 0U; bit < 8U; ++bit) {
        crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x07U) :
            (uint8_t)(crc << 1U);
    }
    return crc;
}

static void proto_esp_uart_reset(esp_uart_protocol_t *protocol)
{
    protocol->parse_state = ESP_UART_WAIT_SOF_0;
    protocol->payload_index = 0U;
    protocol->calculated_crc = 0U;
}

static void proto_esp_uart_restart_from_byte(esp_uart_protocol_t *protocol, uint8_t byte)
{
    proto_esp_uart_reset(protocol);
    if (byte == ESP_UART_SOF_0) {
        protocol->parse_state = ESP_UART_WAIT_SOF_1;
    }
}

static void proto_esp_uart_process_byte(esp_uart_protocol_t *protocol, uint8_t byte)
{
    switch (protocol->parse_state) {
        case ESP_UART_WAIT_SOF_0:
            if (byte == ESP_UART_SOF_0) {
                protocol->parse_state = ESP_UART_WAIT_SOF_1;
            }
            break;
        case ESP_UART_WAIT_SOF_1:
            if (byte == ESP_UART_SOF_1) {
                protocol->parse_state = ESP_UART_WAIT_VERSION;
            } else {
                proto_esp_uart_restart_from_byte(protocol, byte);
            }
            break;
        case ESP_UART_WAIT_VERSION:
            if (byte != ESP_UART_PROTOCOL_VERSION) {
                proto_esp_uart_restart_from_byte(protocol, byte);
                break;
            }
            protocol->calculated_crc = proto_esp_uart_crc8_update(0U, byte);
            protocol->parse_state = ESP_UART_WAIT_TYPE;
            break;
        case ESP_UART_WAIT_TYPE:
            protocol->frame.type = (esp_uart_message_type_t)byte;
            protocol->calculated_crc = proto_esp_uart_crc8_update(protocol->calculated_crc, byte);
            protocol->parse_state = ESP_UART_WAIT_SEQUENCE;
            break;
        case ESP_UART_WAIT_SEQUENCE:
            protocol->frame.sequence = byte;
            protocol->calculated_crc = proto_esp_uart_crc8_update(protocol->calculated_crc, byte);
            protocol->parse_state = ESP_UART_WAIT_LENGTH;
            break;
        case ESP_UART_WAIT_LENGTH:
            if (byte > ESP_UART_MAX_PAYLOAD_LENGTH) {
                proto_esp_uart_restart_from_byte(protocol, byte);
                break;
            }
            protocol->frame.payload_length = byte;
            protocol->payload_index = 0U;
            protocol->calculated_crc = proto_esp_uart_crc8_update(protocol->calculated_crc, byte);
            protocol->parse_state = (byte == 0U) ? ESP_UART_WAIT_CRC : ESP_UART_WAIT_PAYLOAD;
            break;
        case ESP_UART_WAIT_PAYLOAD:
            protocol->frame.payload[protocol->payload_index++] = byte;
            protocol->calculated_crc = proto_esp_uart_crc8_update(protocol->calculated_crc, byte);
            if (protocol->payload_index == protocol->frame.payload_length) {
                protocol->parse_state = ESP_UART_WAIT_CRC;
            }
            break;
        case ESP_UART_WAIT_CRC:
            if ((byte == protocol->calculated_crc) && (protocol->frame_handler != NULL)) {
                protocol->frame_handler(&protocol->frame);
            }
            proto_esp_uart_restart_from_byte(protocol, byte);
            break;
        default:
            proto_esp_uart_reset(protocol);
            break;
    }
}

void proto_esp_uart_init(esp_uart_protocol_t *protocol, esp_uart_frame_handler_t frame_handler)
{
    memset(protocol, 0, sizeof(*protocol));
    protocol->frame_handler = frame_handler;
    proto_esp_uart_reset(protocol);
}

void proto_esp_uart_process_ringbuf(esp_uart_protocol_t *protocol, ringbuf_t *ringbuf)
{
    uint8_t byte;

    while (ringbuf_pop_byte(ringbuf, &byte)) {
        proto_esp_uart_process_byte(protocol, byte);
    }
}

bool proto_esp_uart_send(esp_uart_protocol_t *protocol, esp_uart_message_type_t type,
    const uint8_t *payload, uint8_t payload_length)
{
    uint8_t frame[2U + 4U + ESP_UART_MAX_PAYLOAD_LENGTH + 1U];
    uint8_t index = 0U;
    uint8_t crc = 0U;
    uint8_t payload_index;

    if ((payload_length > ESP_UART_MAX_PAYLOAD_LENGTH) ||
        ((payload_length > 0U) && (payload == NULL))) {
        return false;
    }

    frame[index++] = ESP_UART_SOF_0;
    frame[index++] = ESP_UART_SOF_1;
    frame[index++] = ESP_UART_PROTOCOL_VERSION;
    frame[index++] = (uint8_t)type;
    frame[index++] = protocol->next_sequence++;
    frame[index++] = payload_length;
    for (payload_index = 2U; payload_index < index; ++payload_index) {
        crc = proto_esp_uart_crc8_update(crc, frame[payload_index]);
    }
    for (payload_index = 0U; payload_index < payload_length; ++payload_index) {
        frame[index++] = payload[payload_index];
        crc = proto_esp_uart_crc8_update(crc, payload[payload_index]);
    }
    frame[index++] = crc;
    return bsp_uart_esp_write(frame, index) == index;
}
