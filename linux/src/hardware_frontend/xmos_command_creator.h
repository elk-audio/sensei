#ifndef SENSEI_XMOS_COMMAND_CREATOR_H
#define SENSEI_XMOS_COMMAND_CREATOR_H

#include <array>

#include "xmos_control_protocol.h"

namespace sensei {
namespace hw_frontend {

struct Pinlist
{
    char pincount;
    std::array<uint8_t, 20> pins;
};

class XmosCommandCreator
{
public:
    XmosControlPacket make_reset_system_command();
    XmosControlPacket make_start_system_command();
    XmosControlPacket make_stop_system_command();
    XmosControlPacket make_set_tick_rate_command(int tick_rate);
    XmosControlPacket make_get_board_info_command();
    XmosControlPacket make_reset_all_controllers_command();
    XmosControlPacket make_reset_controller_command(int controller_id);
    XmosControlPacket make_add_digital_output_command(int controller_id, uint8_t hw_type, uint8_t mux_inv,
                                                      uint8_t mux_ctrl_id, uint8_t mux_ctrl_pin,
                                                      uint8_t tickrate, const Pinlist& pins);
    XmosControlPacket make_add_digital_input_command(int controller_id, uint8_t hw_type, uint8_t mux_inv,
                                                     uint8_t mux_ctrl_id, uint8_t mux_ctrl_pin,
                                                     uint8_t tickrate, uint8_t not_mode, const Pinlist& pins);
    XmosControlPacket make_add_analog_input_command(int controller_id, int pin);
    XmosControlPacket make_add_pins_to_controller_command(int controller_id, Pinlist& pins);
    XmosControlPacket make_mute_controller_command(int controller_id);
    XmosControlPacket make_remove_controller_command(int controller_id);
    XmosControlPacket make_set_controller_range_command();
    XmosControlPacket make_get_value_command(int controller_id);
    XmosControlPacket make_set_value_command(int controller_id, uint32_t value);

private:
    uint32_t _sequence_number() {return _sequence_count++;}

    uint32_t _sequence_count{2};
};

}
}

#endif //SENSEI_XMOS_COMMAND_CREATOR_H
