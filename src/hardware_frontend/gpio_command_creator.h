/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Class which helps in creation of gpio protocol master packets
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_GPIO_COMMAND_CREATOR_H
#define SENSEI_GPIO_COMMAND_CREATOR_H

#include <array>
#include <string>

#include <sys/param.h>
#include <arpa/inet.h>

#include "gpio_protocol/gpio_protocol.h"

namespace sensei {
namespace hw_frontend {

/* THe Gpio Protocol is little endian, but the host might be big endian for some arm systems */
inline uint32_t to_gpio_protocol_byteord(uint32_t word)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return word;
#else
    return htole32(word);
#endif
}

inline uint32_t from_gpio_protocol_byteord(uint32_t word)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return word;
#else
    return le32toh(word);
#endif
}

std::string gpio_status_to_string(uint8_t);

std::string gpio_packet_to_string(const gpio::GpioPacket& packet);

struct Pinlist
{
    uint8_t pincount;
    std::array<uint8_t, 20> pins;
};

class GpioCommandCreator
{
public:
    gpio::GpioPacket make_reset_system_command();
    gpio::GpioPacket make_start_system_command();
    gpio::GpioPacket make_set_tick_rate_command(uint8_t tick_rate);
    gpio::GpioPacket make_get_board_info_command();
    gpio::GpioPacket make_reset_all_controllers_command();
    gpio::GpioPacket make_reset_controller_command(uint8_t controller_id);
    gpio::GpioPacket make_add_controller_command(uint8_t controller_id, uint8_t hw_type);
    gpio::GpioPacket make_add_controller_to_mux_command(uint8_t controller_id, uint8_t mux_id, uint8_t mux_pin);
    gpio::GpioPacket make_set_polarity_command(uint8_t controller_id, uint8_t polarity);
    gpio::GpioPacket make_set_controller_tick_rate_command(uint8_t controller_id, uint8_t tick_rate_divisor);
    gpio::GpioPacket make_set_notification_mode(uint8_t controller_id, uint8_t notif_mode);
    gpio::GpioPacket make_add_pins_to_controller_command(uint8_t controller_id, Pinlist& pins);
    gpio::GpioPacket make_mute_controller_command(uint8_t controller_id, uint8_t mute_status);
    gpio::GpioPacket make_set_range_command(uint8_t controller_id, uint32_t min, uint32_t max);
    gpio::GpioPacket make_set_debounce_mode_command(uint8_t controller_id, uint8_t debounce_mode);
    gpio::GpioPacket make_set_analog_resolution_command(uint8_t controller_id, uint8_t adc_bits);
    gpio::GpioPacket make_set_analog_time_constant_command(uint8_t controller_id, float time_constant);
    gpio::GpioPacket make_get_value_command(uint8_t controller_id);
    gpio::GpioPacket make_set_value_command(uint8_t controller_id, uint32_t value);

private:
    gpio::GpioPacket _prepare_packet();

    uint32_t _sequence_number() {return _sequence_count++;}

    uint32_t _sequence_count{1};
};

} // namespace hw_frontend
} // namespace sensei

#endif //SENSEI_GPIO_COMMAND_CREATOR_H