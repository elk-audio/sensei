/**
 * @brief Internal structs, definitions and helper functions for the serial frontend
 * @copyright MIND Music Labs AB, Stockholm
 */

#ifndef SERIAL_FRONTEND_INTERNAL_H
#define SERIAL_FRONTEND_INTERNAL_H

#include <string>

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

/*
 * Convenience function for getting the uid of an ack packet
 */
uint64_t inline extract_uuid(const sSenseiACKPacket* ack)
{
    uint64_t uuid = ack->timestamp;
    uuid += (static_cast<uint64_t>(ack->cmd) << 32);
    uuid += (static_cast<uint64_t>(ack->sub_cmd) << 48);
    return uuid;
}


/*
 * Translate a teensy status code to a string for debugging and logging.
 */
const inline std::string& translate_teensy_status_code(int code)
{
    static std::string str;
    switch (code)
    {
        case SENSEI_ERROR_CODE::NO_EXTERNAL_PROCESSING_NECESSARY:
            return str = "NO_EXTERNAL_PROCESSING_NECESSARY";
        case SENSEI_ERROR_CODE::OK:
            return str ="OK";
        case SENSEI_ERROR_CODE::START_HEADER_NOT_PRESENT:
            return str ="START_HEADER_NOT_PRESENT";
        case SENSEI_ERROR_CODE::STOP_HEADER_NOT_PRESENT:
            return str ="STOP_HEADER_NOT_PRESENT";
        case SENSEI_ERROR_CODE::CRC_NOT_CORRECT:
            return str ="CRC_NOT_CORRECT";
        case SENSEI_ERROR_CODE::CMD_NOT_VALID:
            return str ="CMD_NOT_VALID";
        case SENSEI_ERROR_CODE::SUB_CMD_NOT_VALID:
            return str ="SUB_CMD_NOT_VALID";
        case SENSEI_ERROR_CODE::CMD_NOT_PROCESSED:
            return str ="CMD_NOT_PROCESSED";
        case SENSEI_ERROR_CODE::DIGITAL_OUTPUT_IDX_BANK_NOT_VALID:
            return str ="DIGITAL_OUTPUT_IDX_BANK_NOT_VALID";
        case SENSEI_ERROR_CODE::DIGITAL_OUTPUT_IDX_PIN_NOT_VALID:
            return str ="DIGITAL_OUTPUT_IDX_PIN_NOT_VALID";
        case SENSEI_ERROR_CODE::IDX_PIN_NOT_VALID:
            return str ="IDX_PIN_NOT_VALID";
        case SENSEI_ERROR_CODE::PIN_TYPE_NOT_VALID:
            return str ="PIN_TYPE_NOT_VALID";
        case SENSEI_ERROR_CODE::TIMEOUT_ON_RESPONSE:
            return str ="TIMEOUT_ON_RESPONSE";
        case SENSEI_ERROR_CODE::INCORRECT_PAYLOAD_SIZE:
            return str ="INCORRECT_PAYLOAD_SIZE";
        case SENSEI_ERROR_CODE::NO_AFFINITY_WITH_RESPONSE_PACKET:
            return str ="NO_AFFINITY_WITH_RESPONSE_PACKET";
        case SENSEI_ERROR_CODE::CMD_NOT_EXPECTED:
            return str ="CMD_NOT_EXPECTED";
        case SENSEI_ERROR_CODE::INCORRECT_PARAMETERS_NUMBER:
            return str ="INCORRECT_PARAMETERS_NUMBER";
        case SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE:
            return str ="INCORRECT_PARAMETER_TYPE";
        case SENSEI_ERROR_CODE::INCOMPLETE_PARAMETERS:
            return str ="INCOMPLETE_PARAMETERS";
        case SENSEI_ERROR_CODE::WRONG_NUMBER_EXPECTED_RESPONSE_PACKETS:
            return str ="WRONG_NUMBER_EXPECTED_RESPONSE_PACKETS";
        case SENSEI_ERROR_CODE::SERIAL_DEVICE_GENERIC_ERROR:
            return str ="SERIAL_DEVICE_GENERIC_ERROR";
        case SENSEI_ERROR_CODE::SERIAL_DEVICE_PORT_NOT_OPEN:
            return str ="SERIAL_DEVICE_PORT_NOT_OPEN";
        case SENSEI_ERROR_CODE::GENERIC_ERROR:
            return str ="GENERIC_ERROR";
    }

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
