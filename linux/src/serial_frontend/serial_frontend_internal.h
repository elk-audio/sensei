/**
 * @brief Internal structs, definitions and helper functions for the serial frontend
 * @copyright MIND Music Labs AB, Stockholm
 */

#ifndef SERIAL_FRONTEND_INTERNAL_H
#define SERIAL_FRONTEND_INTERNAL_H

#include "../../../common/sensei_serial_protocol.h"

namespace sensei {
namespace serial_frontend {

const unsigned int READ_WRITE_TIMEOUT_MS = 100;

const PACKET_HEADER START_SIGNATURE = {0x12, 0x34, 0x56};
const PACKET_HEADER STOP_SIGNATURE = {0x12, 0x34, 0x56};

/*
 * Convenience function for comparing header signatures, same pattern as memcmp, strcmp
 */
int inline compare_packet_header(const PACKET_HEADER &lhv, const PACKET_HEADER &rhv)
{
    int diff = 0;
    for (unsigned int i = 0; i < sizeof(lhv.vByte); ++i)
    {
        diff += lhv.vByte[i] - rhv.vByte[i];
    }
    return diff;
}

/*
 * Calculate the checksum of a teensy packet
 */
uint16_t inline calculate_crc(const sSenseiDataPacket *packet)
{
    uint16_t sum = packet->cmd + packet->sub_cmd;
    for (unsigned int i = 0; i < SENSEI_PAYLOAD_LENGTH
                                 + sizeof(packet->continuation)
                                 + sizeof(packet->timestamp); ++i)
    {
        sum += *(packet->payload + i);
    }
    return sum;
}

// set 1 byte boundary for matching
#pragma pack(push, 1)

struct teensy_digital_value_msg
{
    uint16_t pin_id;
    uint8_t pin_type;
    uint8_t value;
};

struct teensy_analog_value_msg
{
    uint16_t pin_id;
    uint8_t pin_type;
    uint16_t value;
    uint16_t slider_value;
};

struct teensy_set_value_cmd
{
    uint16_t    pin_idx;
    uint8_t     value;
};

struct teensy_set_samplerate_cmd
{
    uint8_t     sample_rate_divisor;
};

#pragma pack(pop)


}; // end namespace sensei
}; // end namespace serial_frontend

#endif //SERIAL_FRONTEND_INTERNAL_H
