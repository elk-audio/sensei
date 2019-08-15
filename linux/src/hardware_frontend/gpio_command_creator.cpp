#include <cassert>
#include <cstring>

#include "gpio_command_creator.h"

namespace sensei {
namespace hw_frontend {

using namespace gpio;

gpio::GpioPacket GpioCommandCreator::make_reset_system_command()
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_SYSTEM_CONTROL;
    packet.sub_command = GPIO_SUB_CMD_STOP_RESET_SYSTEM;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_start_system_command()
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_SYSTEM_CONTROL;
    packet.sub_command = GPIO_SUB_CMD_START_SYSTEM;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_stop_system_command()
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_SYSTEM_CONTROL;
    packet.sub_command = GPIO_SUB_CMD_STOP_SYSTEM;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_set_tick_rate_command(uint8_t tick_rate)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_SYSTEM_CONTROL;
    packet.sub_command = GPIO_SUB_CMD_SET_SYSTEM_TICK_RATE;
    packet.payload.system_tick_rate_data.gpio_system_tick_rate = tick_rate;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_get_board_info_command()
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_SYSTEM_CONTROL;
    packet.sub_command = GPIO_SUB_CMD_GET_BOARD_INFO;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_reset_all_controllers_command()
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_RESET_ALL_CONTROLLERS;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_reset_controller_command(uint8_t controller_id)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_RESET_CONTROLLER;
    packet.payload.reset_controller_data.controller_id = controller_id;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_add_controller_command(uint8_t controller_id, uint8_t hw_type)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_ADD_CONTROLLER;
    packet.payload.add_controller_data.controller_id = controller_id;
    packet.payload.add_controller_data.gpio_hw_type = hw_type;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_add_controller_to_mux_command(uint8_t controller_id, uint8_t mux_id, uint8_t mux_pin)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_ATTACH_CONTROLLER_TO_MUX;
    packet.payload.controller_to_mux_data.controller_id = controller_id;
    packet.payload.controller_to_mux_data.mux_controller_id = mux_id;
    packet.payload.controller_to_mux_data.mux_controller_pin = mux_pin;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_set_polarity_command(uint8_t controller_id, uint8_t polarity)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_SET_CONTROLLER_POLARITY;
    packet.payload.controller_polarity_data.controller_id = controller_id;
    packet.payload.controller_polarity_data.polarity = polarity;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_set_controller_tick_rate_command(uint8_t controller_id,
                                                                         uint8_t tick_rate_divisor)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_SET_INPUT_CONTROLLER_TICK_RATE;
    packet.payload.controller_tick_rate.controller_id = controller_id;
    packet.payload.controller_tick_rate.delta_tick_rate = tick_rate_divisor;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_set_notification_mode(uint8_t controller_id, uint8_t notif_mode)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_SET_INPUT_CONTROLLER_NOTIF_MODE;
    packet.payload.controller_notif_data.controller_id = controller_id;
    packet.payload.controller_notif_data.notif_mode = notif_mode;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_add_pins_to_controller_command(uint8_t controller_id, Pinlist& pins)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_ADD_PINS_TO_CONTROLLER;
    auto& data = packet.payload.controller_pins_data;
    data.controller_id = controller_id;
    data.num_pins = static_cast<uint8_t>(pins.pincount);
    assert(pins.pincount <= sizeof(data.pins));
    for (int i = 0; i < pins.pincount; ++i)
    {
        data.pins[i] = pins.pins[i];
    }
    return packet;}

gpio::GpioPacket GpioCommandCreator::make_mute_controller_command(uint8_t controller_id, uint8_t mute_status)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_MUTE_UNMUTE_CONTROLLER;
    packet.payload.controller_mute_data.controller_id = controller_id;
    packet.payload.controller_mute_data.mute_status = mute_status;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_remove_controller_command(uint8_t controller_id)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_REMOVE_CONTROLLER;
    packet.payload.remove_controller_data.controller_id = controller_id;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_set_analog_resolution_command(uint8_t controller_id, uint8_t adc_bits)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_SET_ANALOG_CONTROLLER_RES;
    packet.payload.analog_controller_res_data.controller_id = controller_id;
    packet.payload.analog_controller_res_data.res_in_bits = adc_bits;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_set_range_command(uint8_t controller_id, uint32_t min, uint32_t max)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_SET_CONTROLLER_RANGE;
    packet.payload.controller_range_data.controller_id = controller_id;
    packet.payload.controller_range_data.min_val = min;
    packet.payload.controller_range_data.max_val = max;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_set_debounce_mode_command(uint8_t controller_id, uint8_t debounce_mode)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_CONFIG_CONTROLLER;
    packet.sub_command = GPIO_SUB_CMD_SET_CONTROLLER_DEBOUNCE_MODE;
    packet.payload.controller_debounce_data.controller_id = controller_id;
    packet.payload.controller_debounce_data.controller_debounce_mode = debounce_mode;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_get_value_command(uint8_t controller_id)
{
    GpioPacket packet = _prepare_packet();
    packet.command = GPIO_CMD_GET_VALUE;
    packet.payload.gpio_value_request.controller_id = controller_id;
    return packet;
}

gpio::GpioPacket GpioCommandCreator::make_set_value_command(uint8_t controller_id, uint32_t value)
{
    GpioPacket packet = _prepare_packet();
    packet.command = gpio::GPIO_CMD_SET_VALUE;
    packet.payload.gpio_value_data.controller_id = controller_id;
    packet.payload.gpio_value_data.controller_val = to_gpio_protocol_byteord(value);
    return packet;
}

gpio::GpioPacket GpioCommandCreator::_prepare_packet()
{
    GpioPacket packet;
    memset(&packet, 0, sizeof(packet));
    packet.sequence_no = to_gpio_protocol_byteord(_sequence_number());
    return packet;
}


std::string gpio_status_to_string(uint8_t status)
{
    switch(status)
    {
        case GpioReturnStatus::GPIO_OK:
            return "OK";
        case GpioReturnStatus::GPIO_ERROR:
            return "ERROR";
        case GpioReturnStatus::GPIO_INVALID_CMD:
            return "GPIO_INVALID_CMD";
        case GpioReturnStatus::GPIO_INVALID_SUB_CMD:
            return "GPIO_INVALID_SUB_CMD";
        case GpioReturnStatus::GPIO_NO_CONTROLLERS_ADDED:
            return "GPIO_NO_CONTROLLERS_ADDED";
        case GpioReturnStatus::GPIO_UNITIALIZED_CONTROLLERS:
            return "GPIO_UNITIALIZED_CONTROLLERS";
        case GpioReturnStatus::GPIO_INVALID_TICK_RATE:
            return "GPIO_INVALID_TICK_RATE";
        case GpioReturnStatus::GPIO_INVALID_CONTROLLER_ID:
            return "GPIO_INVALID_CONTROLLER_ID";
        case GpioReturnStatus::GPIO_INVALID_HW_TYPE:
            return "GPIO_INVALID_HW_TYPE";
        case GpioReturnStatus::GPIO_INVALID_MUX_CONTROLLER:
            return "GPIO_INVALID_MUX_CONTROLLER";
        case GpioReturnStatus::GPIO_INVALID_CONTROLLER_POLARITY:
            return "GPIO_INVALID_CONTROLLER_POLARITY";
        case GpioReturnStatus::GPIO_NO_PINS_AVAILABLE:
            return "GPIO_NO_PINS_AVAILABLE";
        case GpioReturnStatus::GPIO_INVALID_SHARING_OF_PINS:
            return "GPIO_INVALID_SHARING_OF_PINS";
        case GpioReturnStatus::GPIO_RES_OUT_OF_RANGE:
            return "GPIO_RES_OUT_OF_RANGE";
        case GpioReturnStatus::GPIO_UNRECOGNIZED_CMD:
            return "GPIO_UNRECOGNIZED_CMD";
        case GpioReturnStatus::GPIO_PARAMETER_ERROR:
            return "GPIO_PARAMETER_ERROR";
        case GpioReturnStatus::GPIO_INVALID_COMMAND_FOR_CONTROLLER:
            return "GPIO_INVALID_COMMAND_FOR_CONTROLLER";
        default:
            return "";
    }
}

std::string gpio_packet_to_string(const gpio::GpioPacket& packet)
{
    switch (packet.command)
    {
        case GPIO_CMD_SYSTEM_CONTROL:
            switch(packet.sub_command)
            {
                case GPIO_SUB_CMD_STOP_RESET_SYSTEM:
                    return "GPIO_SUB_CMD_STOP_RESET_SYSTEM";
                case GPIO_SUB_CMD_START_SYSTEM:
                    return "GPIO_SUB_CMD_START_SYSTEM";
                case GPIO_SUB_CMD_STOP_SYSTEM:
                    return "GPIO_SUB_CMD_STOP_SYSTEM";
                case GPIO_SUB_CMD_SET_SYSTEM_TICK_RATE:
                    return "GPIO_SUB_CMD_SET_TICK_RATE";
                case GPIO_SUB_CMD_GET_BOARD_INFO:
                    return "GPIO_SUB_CMD_GET_BOARD_INFO";
                default:
                    return "Unrecognised Gpio command";
            }
        case GPIO_CMD_CONFIG_CONTROLLER:
            switch(packet.sub_command)
            {
                case GPIO_SUB_CMD_RESET_ALL_CONTROLLERS:
                    return "GPIO_SUB_CMD_RESET_ALL_CONTROLLERS";
                case GPIO_SUB_CMD_RESET_CONTROLLER:
                    return "GPIO_SUB_CMD_RESET_CONTROLLER";
                case GPIO_SUB_CMD_ADD_CONTROLLER:
                    return "GPIO_SUB_CMD_ADD_CONTROLLER";
                case GPIO_SUB_CMD_ATTACH_CONTROLLER_TO_MUX:
                    return "GPIO_SUB_CMD_ATTACH_CONTROLLER_TO_MUX";
                case GPIO_SUB_CMD_SET_CONTROLLER_POLARITY:
                    return "GPIO_SUB_CMD_SET_CONTROLLER_POLARITY";
                case GPIO_SUB_CMD_SET_INPUT_CONTROLLER_TICK_RATE:
                    return "GPIO_SUB_CMD_SET_INPUT_CONTROLLER_TICK_RATE";
                case GPIO_SUB_CMD_SET_INPUT_CONTROLLER_NOTIF_MODE:
                    return "GPIO_SUB_CMD_SET_INPUT_CONTROLLER_NOTIF_MODE";
                case GPIO_SUB_CMD_ADD_PINS_TO_CONTROLLER:
                    return "GPIO_SUB_CMD_ADD_PINS_TO_CONTROLLER";
                case GPIO_SUB_CMD_MUTE_UNMUTE_CONTROLLER:
                    return "GPIO_SUB_CMD_MUTE_UNMUTE_CONTROLLER";
                case GPIO_SUB_CMD_REMOVE_CONTROLLER:
                    return "GPIO_SUB_CMD_REMOVE_CONTROLLER";
                case GPIO_SUB_CMD_SET_ANALOG_CONTROLLER_RES:
                    return "GPIO_SUB_CMD_SET_ANALOG_CONTROLLER_RES";
                case GPIO_SUB_CMD_SET_CONTROLLER_RANGE:
                    return "GPIO_SUB_CMD_SET_CONTROLLER_RANGE";
                default:
                    return "Unrecognised Gpio command";
            }
        case GPIO_CMD_GET_VALUE:
            return "GPIO_CMD_GET_VALUE";
        case GPIO_CMD_SET_VALUE:
            return "GPIO_CMD_SET_VALUE";
        case GPIO_ACK:
            return "GPIO_ACK";
        default:
            return "Unrecognised Gpio command";
    }
}


}
}