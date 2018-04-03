#include <cassert>
#include <cstring>

#include "xmos_command_creator.h"

namespace sensei {
namespace hw_frontend {

using namespace xmos;

xmos::XmosGpioPacket XmosCommandCreator::make_reset_system_command()
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_STOP_RESET_SYSTEM;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_start_system_command()
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_START_SYSTEM;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_stop_system_command()
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_STOP_SYSTEM;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_set_tick_rate_command(uint8_t tick_rate)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_SET_TICK_RATE;
    packet.payload.tick_rate_data.system_tick_rate = tick_rate;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_get_board_info_command()
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = 0; //XMOS_SUB_CMD_GET_BOARD_INFO;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_reset_all_controllers_command()
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_RESET_ALL_CNTRLRS;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_reset_controller_command(uint8_t controller_id)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_RESET_CNTRLR;
    packet.payload.reset_cntrlr_data.controller_id = controller_id;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_add_controller_command(uint8_t controller_id, uint8_t hw_type)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_ADD_CNTRLR;
    packet.payload.cntrlr_data.controller_id = controller_id;
    packet.payload.cntrlr_data.hw_type = hw_type;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_add_controller_to_mux_command(uint8_t controller_id, uint8_t mux_id, uint8_t mux_pin)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_ADD_CNTRLR_TO_MUX;
    packet.payload.cntrlr_to_mux_data.controller_id = controller_id;
    packet.payload.cntrlr_to_mux_data.mux_controller_id = mux_id;
    packet.payload.cntrlr_to_mux_data.mux_controller_pin = mux_pin;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_set_polarity_command(uint8_t controller_id, uint8_t polarity)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_SET_CNTRLR_POLARITY;
    packet.payload.cntrlr_polarity_data.controller_id = controller_id;
    packet.payload.cntrlr_polarity_data.polarity = polarity;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_set_controller_tick_rate_command(uint8_t controller_id,
                                                                         uint8_t tick_rate_divisor)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_SET_INPUT_CNTRLR_TICK_RATE;
    packet.payload.cntrlr_tick_rate.controller_id = controller_id;
    packet.payload.cntrlr_tick_rate.delta_tick_rate = tick_rate_divisor;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_set_notification_mode(uint8_t controller_id, uint8_t notif_mode)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_SET_INPUT_CNTRLR_NOTIF_MODE;
    packet.payload.notif_mode_data.controller_id = controller_id;
    packet.payload.notif_mode_data.notif_mode = notif_mode;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_add_pins_to_controller_command(uint8_t controller_id, Pinlist& pins)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR;
    auto& data = packet.payload.pin_data;
    data.controller_id = controller_id;
    data.num_pins = pins.pincount;
    assert(pins.pincount <= sizeof(data.pins));
    for (int i = 0; i < pins.pincount; ++i)
    {
        data.pins[i] = pins.pins[i];
    }
    return packet;}

xmos::XmosGpioPacket XmosCommandCreator::make_mute_controller_command(uint8_t controller_id, uint8_t mute_status)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_MUTE_UNMUTE_CNTRLR;
    packet.payload.mute_cmnd_data.controller_id = controller_id;
    packet.payload.mute_cmnd_data.mute_status = mute_status;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_remove_controller_command(uint8_t controller_id)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_REMOVE_CNTRLR;
    packet.payload.remove_cntrlr_data.controller_id = controller_id;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_set_controller_range_command(uint8_t controller_id, uint32_t min_value, uint32_t max_value)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_SET_CNTRLR_RANGE;
    packet.payload.set_cntrlr_range_data.controller_id = controller_id;
    packet.payload.set_cntrlr_range_data.min_value = to_xmos_byteord(min_value);
    packet.payload.set_cntrlr_range_data.max_value = to_xmos_byteord(max_value);
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_get_value_command(uint8_t controller_id)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_GET_VALUE;
    packet.payload.value_request_data.controller_id = controller_id;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_set_value_command(uint8_t controller_id, uint32_t value)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = xmos::XMOS_CMD_SET_VALUE;
    packet.payload.value_request_data.controller_id = controller_id;
    packet.payload.value_send_data.controller_val = to_xmos_byteord(value);
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::_prepare_packet()
{
    XmosGpioPacket packet;
    memset(&packet, 0, sizeof(packet));
    packet.sequence_no = to_xmos_byteord(_sequence_number());
    return packet;
}


}
}