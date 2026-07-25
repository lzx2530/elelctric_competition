#ifndef TI_STEELBALL_PROTOCOL_H
#define TI_STEELBALL_PROTOCOL_H

#include <stdint.h>

#define SB_SOF_0 0xAAu
#define SB_SOF_1 0x55u
#define SB_PROTOCOL_VERSION 0x01u
#define SB_MAX_PAYLOAD_LENGTH 32u
#define SB_INVALID_RANGE_MM 0xFFFFu

#define SB_TYPE_VISION_REPORT 0x10u
#define SB_TYPE_TI_STATUS 0x11u
#define SB_TYPE_MISSION_COMMAND 0x20u
#define SB_TYPE_ACK 0x7Fu

#define SB_CMD_START 0x01u
#define SB_CMD_ABORT 0x02u
#define SB_CMD_RESET 0x03u

#define SB_ACK_OK 0x00u
#define SB_ACK_BAD_PAYLOAD 0x01u
#define SB_ACK_UNSUPPORTED 0x02u

#define SB_VISION_READY 0x00u
#define SB_VISION_NO_TARGET 0x01u
#define SB_VISION_CANDIDATE 0x02u
#define SB_VISION_TRACKING 0x03u
#define SB_VISION_NEAR_HANDOFF 0x04u
#define SB_VISION_LOST 0x05u
#define SB_VISION_FAULT 0x06u

#define SB_TI_IDLE 0x00u
#define SB_TI_FOLLOW_LINE 0x01u
#define SB_TI_VISUAL_APPROACH 0x02u
#define SB_TI_NEAR_CREEP 0x03u
#define SB_TI_MAGNET_HOLD 0x04u
#define SB_TI_SUCCESS_UNVERIFIED 0x05u
#define SB_TI_RETRY 0x06u
#define SB_TI_ABORTED 0x07u
#define SB_TI_FAULT 0x08u

#define SB_CAPTURE_NONE 0x00u
#define SB_CAPTURE_EXECUTED_UNVERIFIED 0x01u
#define SB_CAPTURE_FAILED 0x02u

#define SB_VISION_FLAG_TARGET_VALID 0x01u
#define SB_VISION_FLAG_TARGET_STABLE 0x02u
#define SB_VISION_FLAG_RANGE_VALID 0x04u
#define SB_VISION_FLAG_NEAR_LATCHED 0x08u
#define SB_VISION_FLAG_CAMERA_FAULT 0x10u
#define SB_VISION_FLAG_INFERENCE_FAULT 0x20u

#define SB_VISION_REPORT_LENGTH 15u
#define SB_TI_STATUS_LENGTH 6u
#define SB_MISSION_COMMAND_LENGTH 2u
#define SB_ACK_LENGTH 4u

static inline uint16_t sb_crc16_ccitt_false(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    uint16_t index;
    uint8_t bit;

    for (index = 0u; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                   : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static inline uint16_t sb_read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static inline void sb_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

#endif
