#ifndef SENSEI_XMOS_COMMAND_CREATOR_H
#define SENSEI_XMOS_COMMAND_CREATOR_H

#include <array>

#include <arpa/inet.h>

#include "xmos_gpio_protocol.h"

namespace sensei {
namespace hw_frontend {

inline uint32_t to_xmos_byteord(uint32_t word)
{
    return htonl(word);
}

inline uint32_t from_xmos_byteord(uint32_t word)
{
    return ntohl(word);
}

struct Pinlist
{
    char pincount;
    std::array<uint8_t, 20> pins;
};

struct ControllerCache
{
    uint8_t hw_type;
    uint8_t mux_inv;
    uint8_t mux_ctrlr_id;
    uint8_t mux_ctrlr_pin;
    uint8_t tick_rate;
    uint8_t not_mode;
};


class XmosCommandCreator
{
public:
    XmosGpioPacket make_reset_system_command();
    XmosGpioPacket make_start_system_command();
    XmosGpioPacket make_stop_system_command();
    XmosGpioPacket make_set_tick_rate_command(uint8_t tick_rate);
    XmosGpioPacket make_get_board_info_command();
    XmosGpioPacket make_reset_all_controllers_command();
    XmosGpioPacket make_reset_controller_command(uint8_t controller_id);
    XmosGpioPacket make_add_controller_command(uint8_t controller_id, uint8_t hw_type);
    XmosGpioPacket make_add_controller_to_mux_command(uint8_t controller_id, uint8_t mux_id, uint8_t mux_pin);
    XmosGpioPacket make_set_polarity_command(uint8_t controller_id, uint8_t polarity);
    XmosGpioPacket make_set_controller_tick_rate_command(uint8_t controller_id, uint8_t tick_rate_divisor);
    XmosGpioPacket make_set_notification_mode(uint8_t controller_id, uint8_t notif_mode);
    XmosGpioPacket make_add_pins_to_controller_command(uint8_t controller_id, Pinlist& pins);
    XmosGpioPacket make_mute_controller_command(uint8_t controller_id, uint8_t mute_status);
    XmosGpioPacket make_remove_controller_command(uint8_t controller_id);
    XmosGpioPacket make_set_controller_range_command(uint8_t controller_id, uint32_t low_range, uint32_t high_range);
    XmosGpioPacket make_get_value_command(uint8_t controller_id);
    XmosGpioPacket make_set_value_command(uint8_t controller_id, uint32_t value);

private:
    XmosGpioPacket _prepare_packet();

    uint32_t _sequence_number() {return _sequence_count++;}

    uint32_t _sequence_count{1};
};

}
}

#endif //SENSEI_XMOS_COMMAND_CREATOR_H
