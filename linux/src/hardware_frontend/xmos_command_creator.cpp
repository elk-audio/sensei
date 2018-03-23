#include <cassert>
#include <cstring>

#include "xmos_control_protocol.h"
#include "xmos_command_creator.h"

namespace sensei {
namespace hw_frontend {


XmosControlPacket XmosCommandCreator::make_reset_system_command()
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_STOP_RESET_SYSTEM;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_start_system_command()
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_START_SYSTEM;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_stop_system_command()
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_STOP_SYSTEM;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_set_tick_rate_command(int tick_rate)
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_SET_TICK_RATE;
    packet.payload[0] = tick_rate;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_get_board_info_command()
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_SYSTEM_CNTRL;
    packet.sub_command = XMOS_SUB_CMD_STOP_RESET_SYSTEM;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_reset_all_controllers_command()
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_RESET_ALL_CNTRLRS;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_reset_controller_command(int controller_id)
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_RESET_CNTRLR;
    packet.payload[0] = controller_id;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_add_digital_output_command(int controller_id, uint8_t hw_type,
                                                                         uint8_t mux_inv, uint8_t mux_ctrl_id,
                                                                         uint8_t mux_ctrl_pin, uint8_t tickrate,
                                                                         const Pinlist& pins)
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_ADD_DIGITAL_OUTPUT;
    auto data = reinterpret_cast<DigitalOutputData*>(packet.payload);
    data->controller_id = controller_id;
    data->output_hw_type = hw_type;
    data->mux_inverted = mux_inv;
    data->mux_controller_id = mux_ctrl_id;
    data->mux_controller_pin = mux_ctrl_pin;
    data->delta_tick_rate = tickrate;
    data->num_pins = pins.pincount;
    assert(pins.pincount <= sizeof(data->pins));
    for (int i = 0; i < pins.pincount; ++i)
    {
        data->pins[i] = pins.pins[i];
    }
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket
XmosCommandCreator::make_add_digital_input_command(int controller_id, uint8_t hw_type,
                                                   uint8_t mux_inv, uint8_t mux_ctrl_id,
                                                   uint8_t mux_ctrl_pin, uint8_t tickrate,
                                                   uint8_t not_mode, const Pinlist& pins)
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_ADD_DIGITAL_INPUT;
    auto data = reinterpret_cast<DigitalInputData*>(packet.payload);
    data->controller_id = controller_id;
    data->input_hw_type = hw_type;
    data->mux_inverted = mux_inv;
    data->mux_controller_id = mux_ctrl_id;
    data->mux_controller_pin = mux_ctrl_pin;
    data->delta_tick_rate = tickrate;
    data->notification_mode = not_mode;
    data->num_pins = pins.pincount;
    assert(pins.pincount <= sizeof(data->pins));
    for (int i = 0; i < pins.pincount; ++i)
    {
        data->pins[i] = pins.pins[i];
    }
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_add_analog_input_command(int controller_id, int pin)
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_ADD_ANALOG_INPUT;
    auto data = reinterpret_cast<AnalogInputData*>(packet.payload);
    data->controller_id = controller_id;
    data->pin_number = pin;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket
XmosCommandCreator::make_add_pins_to_controller_command(int controller_id, Pinlist& pins)
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR;
    auto data = reinterpret_cast<PinData*>(packet.payload);
    data->controller_id = controller_id;
    data->num_pins = pins.pincount;
    assert(pins.pincount <= sizeof(data->pins));
    for (int i = 0; i < pins.pincount; ++i)
    {
        data->pins[i] = pins.pins[i];
    }
    packet.sequence_no = _sequence_number();
    return packet;}

XmosControlPacket XmosCommandCreator::make_mute_controller_command(int controller_id)
{
    return XmosControlPacket();
}

XmosControlPacket XmosCommandCreator::make_remove_controller_command(int controller_id)
{
    return XmosControlPacket();
}

XmosControlPacket XmosCommandCreator::make_set_controller_range_command()
{
    return XmosControlPacket();
}

XmosControlPacket XmosCommandCreator::make_get_value_command(int controller_id)
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_GET_VALUE;
    auto data = reinterpret_cast<ValueData*>(packet.payload);
    data->controller_id = controller_id;
    packet.sequence_no = _sequence_number();
    return packet;
}

XmosControlPacket XmosCommandCreator::make_set_value_command(int controller_id, uint32_t value)
{
    XmosControlPacket packet;
    memset(&packet, sizeof(packet), 0);
    packet.command = XMOS_CMD_SET_VALUE;
    auto data = reinterpret_cast<ValueData*>(packet.payload);
    data->controller_id = controller_id;
    data->value = value;
    packet.sequence_no = _sequence_number();
    return packet;
}
}
}