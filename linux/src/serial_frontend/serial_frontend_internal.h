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


namespace teensy_command {
enum teensy_command
{
    HELLO = 1,
    SERIAL_DEBUG,
    SET_SAMPLING_RATE = 100,
    CONFIGURE_PIN,
    GET_VALUE,
    GET_ALL_VALUES,
    SET_VALUE,
    IMU_GET_SETTINGS = 200,
    IMU_SET_SETTINGS,
    IMU_GYROSCOPE_CALIBRATION,
    IMU_RESET_FILTER,
    IMU_GET_DATA,
    ACK = 255
};
}

namespace teensy_configure_pin_subcommand {
enum teensy_configure_pin_subcommand
{
    DISABLE = 0,
    DIGITAL_INPUT,
    DIGITAL_OUTPUT,
    ANALOG_INPUT,
};
}

namespace teensy_get_all_values_subcommand{
enum teensy_get_all_values_subbcommand
{
    ALL = 0,
    DIGITAL_INPUT,
    ANALOG_INPUT,
    DIGITAL_OUTPUT,
};
}

namespace teensy_set_value_subcommand{
enum teensy_set_value_subbcommand
{
    SET_SINGLE_PIN = 0,
    SET_BYTE,
    SET_WORD_16,
    SET_WORD_32,
};
}

namespace teensy_get_imu_data_subcommand {
enum teensy_get_imu_data_subcommand
{
    ALL = 0,
    COMPONENT_SENSOR,
    COMPONENT_SENSOR_NORMALIZED,
    QUATERNION,
    LINEARACCELERATION,
    QUATERNION_LINEARACCELERATION,
};
}

// Maybe not necessary
enum teensy_sampling_rate
{
    SAMPLING_RATE_0_HZ = 0,
    SAMPLING_RATE_1000_HZ,
    SAMPLING_RATE_500_HZ,
    SAMPLING_RATE_333_HZ,
    SAMPLING_RATE_250_HZ,
    SAMPLING_RATE_200_HZ,
    SAMPLING_RATE_125_HZ = 8,
    SAMPLING_RATE_100_HZ = 10,
    SAMPLING_RATE_50_HZ = 20,
    SAMPLING_RATE_40_HZ,
    SAMPLING_RATE_3_92_HZ,
};


namespace teensy_pin_type
{
enum teensy_pin_type
{
    PIN_DISABLE = 0,
    PIN_DIGITAL_INPUT,
    PIN_DIGITAL_OUTPUT,
    PIN_ANALOG_INPUT,
};
}

// set 1 byte boundary for matching
#pragma pack(push, 1)


struct imu_all_data_msg
{
    float q0;
    float q1;
    float q2;
    float q3;
    float lax;
    float lay;
    float laz;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
};

struct imu_component_data_msg
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
};

struct imu_linear_acceleration_data_msg
{
    float lax;
    float lay;
    float laz;
};

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

struct teensy_configuration_packet
{
    uint8_t     subcommand;
    uint16_t    pin_id;
    uint8_t     sending_mode;
    uint16_t    delta_ticks;
    uint8_t     bit_res;
    float       lowpass_cutoff;
    uint8_t     slider_mode;
    uint16_t    slider_threshold;
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


/*struct teensy_generic_packet
{
    uint8_t     start_sig_b1;
    uint8_t     start_sig_b2;
    uint8_t     start_sig_b3;
    uint8_t     command;
    uint8_t     subcommand;
    uint8_t     data[49];
    uint32_t    continuation;
    uint32_t    timestamp;
    uint16_t    crc;
    uint8_t     end_sig_b1;
    uint8_t     end_sig_b2;
    uint8_t     end_sig_b3;
};*/
#pragma pack(pop)

#endif //SERIAL_FRONTEND_INTERNAL_H

}; // end namespace sensei
}; // end namespace serial_frontend