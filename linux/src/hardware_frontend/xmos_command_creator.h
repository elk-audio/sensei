#ifndef SENSEI_XMOS_COMMAND_CREATOR_H
#define SENSEI_XMOS_COMMAND_CREATOR_H

#include <array>
#include <string>

#include <sys/param.h>
#include <arpa/inet.h>

#include "xmos_gpio_protocol.h"

namespace sensei {
namespace hw_frontend {

/* Xmos is little endian, but the host might be big endian for some arm systems */
inline uint32_t to_xmos_byteord(uint32_t word)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return word;
#else
    return htole32(word);
#endif
}

inline uint32_t from_xmos_byteord(uint32_t word)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
    return word;
#else
    return le32toh(word);
#endif
}

std::string xmos_status_to_string(uint8_t);

struct Pinlist
{
    char pincount;
    std::array<uint8_t, 20> pins;
};

class XmosCommandCreator
{
public:
    xmos::XmosGpioPacket make_reset_system_command();
    xmos::XmosGpioPacket make_start_system_command();
    xmos::XmosGpioPacket make_stop_system_command();
    xmos::XmosGpioPacket make_set_tick_rate_command(uint8_t tick_rate);
    xmos::XmosGpioPacket make_get_board_info_command();
    xmos::XmosGpioPacket make_reset_all_controllers_command();
    xmos::XmosGpioPacket make_reset_controller_command(uint8_t controller_id);
    xmos::XmosGpioPacket make_add_controller_command(uint8_t controller_id, uint8_t hw_type);
    xmos::XmosGpioPacket make_add_controller_to_mux_command(uint8_t controller_id, uint8_t mux_id, uint8_t mux_pin);
    xmos::XmosGpioPacket make_set_polarity_command(uint8_t controller_id, uint8_t polarity);
    xmos::XmosGpioPacket make_set_controller_tick_rate_command(uint8_t controller_id, uint8_t tick_rate_divisor);
    xmos::XmosGpioPacket make_set_notification_mode(uint8_t controller_id, uint8_t notif_mode);
    xmos::XmosGpioPacket make_add_pins_to_controller_command(uint8_t controller_id, Pinlist& pins);
    xmos::XmosGpioPacket make_mute_controller_command(uint8_t controller_id, uint8_t mute_status);
    xmos::XmosGpioPacket make_remove_controller_command(uint8_t controller_id);
    xmos::XmosGpioPacket make_set_analog_resolution_command(uint8_t controller_id, uint8_t adc_bits);
    xmos::XmosGpioPacket make_get_value_command(uint8_t controller_id);
    xmos::XmosGpioPacket make_set_value_command(uint8_t controller_id, uint32_t value);

private:
    xmos::XmosGpioPacket _prepare_packet();

    uint32_t _sequence_number() {return _sequence_count++;}

    uint32_t _sequence_count{1};
};

}
}

#endif //SENSEI_XMOS_COMMAND_CREATOR_H
