/**
 * @brief Internal structs and definitions for the serial frontend
 * @copyright MIND Music Labs AB, Stockholm
 */

#ifndef SERIAL_FRONTEND_INTERNAL_DEFINITIONS_H
#define SERIAL_FRONTEND_INTERNAL_DEFINITIONS_H

namespace sensei {
namespace serial_frontend {

const unsigned int READ_WRITE_TIMEOUT_MS = 100;

const PACKET_HEADER START_SIGNATURE = {0x12, 0x34, 0x56};
const PACKET_HEADER STOP_SIGNATURE = {0x12, 0x34, 0x56};

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

#pragma pack(pop)

#endif //SERIAL_FRONTEND_INTERNAL_DEFINITIONS_H

}; // end namespace sensei
}; // end namespace serial_frontend