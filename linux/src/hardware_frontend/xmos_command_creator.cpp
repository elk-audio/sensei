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
    packet.sub_command = XMOS_SUB_CMD_GET_BOARD_INFO;
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
    auto& data = packet.payload.pins_data;
    data.controller_id = controller_id;
    data.num_pins = static_cast<uint8_t>(pins.pincount);
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
    packet.payload.mute_cmd_data.controller_id = controller_id;
    packet.payload.mute_cmd_data.mute_status = mute_status;
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

xmos::XmosGpioPacket XmosCommandCreator::make_set_analog_resolution_command(uint8_t controller_id, uint8_t adc_bits)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_SET_ANALOG_CNTRLR_RES;
    packet.payload.analog_cntrlr_res_data.controller_id = controller_id;
    packet.payload.analog_cntrlr_res_data.resolution_in_bits = adc_bits;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_set_range_command(uint8_t controller_id, uint32_t min, uint32_t max)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_SET_CNTRLR_RANGE;
    packet.payload.cntrlr_range_data.controller_id = controller_id;
    packet.payload.cntrlr_range_data.min_val = min;
    packet.payload.cntrlr_range_data.max_val = max;
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::make_set_debounce_mode_command(uint8_t controller_id, uint8_t debounce_mode)
{
    XmosGpioPacket packet = _prepare_packet();
    packet.command = XMOS_CMD_CONFIGURE_CNTRLR;
    packet.sub_command = XMOS_SUB_CMD_SET_CNTRLR_DEBOUNCE_MODE;
    packet.payload.cntrlr_debounce_data.controller_id = controller_id;
    packet.payload.cntrlr_debounce_data.cntrlr_debounce_mode = debounce_mode;
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
    packet.payload.value_data.controller_val = to_xmos_byteord(value);
    return packet;
}

xmos::XmosGpioPacket XmosCommandCreator::_prepare_packet()
{
    XmosGpioPacket packet;
    memset(&packet, 0, sizeof(packet));
    packet.sequence_no = to_xmos_byteord(_sequence_number());
    return packet;
}


std::string xmos_status_to_string(uint8_t status)
{
    switch(status)
    {
        case XmosReturnStatus::OK:
            return "OK";
        case XmosReturnStatus::ERROR:
            return "ERROR";
        case XmosReturnStatus::INVALID_GPIO_CMD:
            return "INVALID_GPIO_CMD";
        case XmosReturnStatus::INVALID_GPIO_SUB_CMD:
            return "INVALID_GPIO_SUB_CMD";
        case XmosReturnStatus::NO_CNTRLRS_ADDED:
            return "NO_CNTRLRS_ADDED";
        case XmosReturnStatus::UNITIALIZED_CNTRLRS:
            return "UNITIALIZED_CNTRLRS";
        case XmosReturnStatus::INVALID_TICK_RATE:
            return "INVALID_TICK_RATE";
        case XmosReturnStatus::INVALID_CNTRLR_ID:
            return "INVALID_CNTRLR_ID";
        case XmosReturnStatus::INVALID_HW_TYPE:
            return "INVALID_HW_TYPE";
        case XmosReturnStatus::INVALID_MUX_CNTRLR:
            return "INVALID_MUX_CNTRLR";
        case XmosReturnStatus::INVALID_CNTRLR_POLARITY:
            return "INVALID_CNTRLR_POLARITY";
        case XmosReturnStatus::NO_PINS_AVAILABLE:
            return "NO_PINS_AVAILABLE";
        case XmosReturnStatus::INVALID_SHARING_OF_PINS:
            return "INVALID_SHARING_OF_PINS";
        case XmosReturnStatus::RES_OUT_OF_RANGE:
            return "RES_OUT_OF_RANGE";
        case XmosReturnStatus::PARAMETER_ERROR:
            return "PARAMETER_ERROR";
        case XmosReturnStatus::INVALID_COMMAND_FOR_CNTRLR:
            return "INVALID_COMMAND_FOR_CNTRLR";
        default:
            return "";
    }
}

std::string xmos_packet_to_string(const xmos::XmosGpioPacket& packet)
{
    switch (packet.command)
    {
        case XMOS_CMD_SYSTEM_CNTRL:
            switch(packet.sub_command)
            {
                case XMOS_SUB_CMD_STOP_RESET_SYSTEM:
                    return "XMOS_SUB_CMD_STOP_RESET_SYSTEM";
                case XMOS_SUB_CMD_START_SYSTEM:
                    return "XMOS_SUB_CMD_START_SYSTEM";
                case XMOS_SUB_CMD_STOP_SYSTEM:
                    return "XMOS_SUB_CMD_STOP_SYSTEM";
                case XMOS_SUB_CMD_SET_TICK_RATE:
                    return "XMOS_SUB_CMD_SET_TICK_RATE";
                case XMOS_SUB_CMD_GET_BOARD_INFO:
                    return "XMOS_SUB_CMD_GET_BOARD_INFO";
                default:
                    return "Unrecognised Xmos command";
            }
        case XMOS_CMD_CONFIGURE_CNTRLR:
            switch(packet.sub_command)
            {
                case XMOS_SUB_CMD_RESET_ALL_CNTRLRS:
                    return "XMOS_SUB_CMD_RESET_ALL_CNTRLRS";
                case XMOS_SUB_CMD_RESET_CNTRLR:
                    return "XMOS_SUB_CMD_RESET_CNTRLR";
                case XMOS_SUB_CMD_ADD_CNTRLR:
                    return "XMOS_SUB_CMD_ADD_CNTRLR";
                case XMOS_SUB_CMD_ADD_CNTRLR_TO_MUX:
                    return "XMOS_SUB_CMD_ADD_CNTRLR_TO_MUX";
                case XMOS_SUB_CMD_SET_CNTRLR_POLARITY:
                    return "XMOS_SUB_CMD_SET_CNTRLR_POLARITY";
                case XMOS_SUB_CMD_SET_INPUT_CNTRLR_TICK_RATE:
                    return "XMOS_SUB_CMD_SET_INPUT_CNTRLR_TICK_RATE";
                case XMOS_SUB_CMD_SET_INPUT_CNTRLR_NOTIF_MODE:
                    return "XMOS_SUB_CMD_SET_INPUT_CNTRLR_NOTIF_MODE";
                case XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR:
                    return "XMOS_SUB_CMD_ADD_PINS_TO_CNTRLR";
                case XMOS_SUB_CMD_MUTE_UNMUTE_CNTRLR:
                    return "XMOS_SUB_CMD_MUTE_UNMUTE_CNTRLR";
                case XMOS_SUB_CMD_REMOVE_CNTRLR:
                    return "XMOS_SUB_CMD_REMOVE_CNTRLR";
                case XMOS_SUB_CMD_SET_ANALOG_CNTRLR_RES:
                    return "XMOS_SUB_CMD_SET_ANALOG_CNTRLR_RES";
                case XMOS_SUB_CMD_SET_CNTRLR_RANGE:
                    return "XMOS_SUB_CMD_SET_CNTRLR_RANGE";
                default:
                    return "Unrecognised Xmos command";
            }
        case XMOS_CMD_GET_VALUE:
            return "XMOS_CMD_GET_VALUE";
        case XMOS_CMD_SET_VALUE:
            return "XMOS_CMD_SET_VALUE";
        case XMOS_ACK:
            return "XMOS_ACK";
        default:
            return "Unrecognised Xmos command";
    }
}


}
}