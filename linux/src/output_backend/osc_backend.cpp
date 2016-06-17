#include <sstream>
#include <algorithm>
#include <iostream>

#include "osc_backend.h"

#include "logging.h"

using namespace sensei;
using namespace sensei::output_backend;

namespace {

SENSEI_GET_LOGGER;

void trim_osc_path_components(std::string& s)
{
    // Trim whitespace
    s.erase(std::remove_if(s.begin(), s.end(), isspace), s.end());
    // Trim leading and trailing slashes /
    auto char_idx = s.find_first_not_of("/");
    s.erase(0, char_idx);
    char_idx = s.find_last_not_of("/");
    if (char_idx != std::string::npos)
    {
        s.erase(char_idx+1);
    }
}

std::string concatenate_osc_paths(std::string a, std::string b)
{
    std::stringstream stream;
    trim_osc_path_components(a);
    trim_osc_path_components(b);
    stream << "/" << a << "/" << b;
    return stream.str();
}

}; // anonymous namespace

OSCBackend::OSCBackend(const int max_n_input_pins) :
    OutputBackend(max_n_input_pins),
    _base_path("sensors"),
    _base_raw_path("raw_input"),
    _host("localhost"),
    _port(23023)

{
    _full_out_paths.resize(static_cast<size_t>(max_n_input_pins));
    _full_raw_paths.resize(static_cast<size_t>(max_n_input_pins));
    _compute_full_paths();
    _compute_address();
}

void OSCBackend::send(const OutputValue* transformed_value, const Value* raw_input_value)
{
    // TODO: see if it's worth checking errors in lo_send calls

    int sensor_index = transformed_value->index();

    SENSEI_LOG_INFO("OSC backend, got value to send");
    if (_send_output_active)
    {
        lo_send(_address, _full_out_paths[sensor_index].c_str(), "f", transformed_value->value());
    }

    if (_send_raw_input_active)
    {
        int input_val = -1;

        switch (raw_input_value->type())
        {
        case ValueType::ANALOG:
            {
                auto typed_val = static_cast<const AnalogValue *>(raw_input_value);
                input_val = static_cast<int>(typed_val->value());
            }
            break;

        case ValueType::DIGITAL:
            {
                auto typed_val = static_cast<const DigitalValue *>(raw_input_value);
                input_val = static_cast<int>(typed_val->value());
            }
            break;

        case ValueType::IMU:
            {
                auto typed_val = static_cast<const ImuValue *>(raw_input_value);
                input_val = static_cast<int>(typed_val->value());
            }
            break;

        default:
            break;
        }

        lo_send(_address, _full_raw_paths[sensor_index].c_str(), "i", input_val);
    }

}

CommandErrorCode OSCBackend::apply_command(const Command *cmd)
{
    CommandErrorCode status = CommandErrorCode::OK;
    auto pin_idx = cmd->index();

    switch(cmd->type())
    {

    case CommandType::SET_PIN_NAME:
        {
            const auto typed_cmd = static_cast<const SetPinNameCommand *>(cmd);
            _pin_names[pin_idx] = typed_cmd->data();
            _compute_full_paths();
        };
        break;

    case CommandType::SET_PIN_TYPE:
        {
            const auto typed_cmd = static_cast<const SetPinTypeCommand*>(cmd);
            _pin_types[pin_idx] = typed_cmd->data();
            _compute_full_paths();
        };
        break;

    case CommandType::SET_OSC_OUTPUT_BASE_PATH:
        {
            const auto typed_cmd = static_cast<const SetOSCOutputBasePathCommand*>(cmd);
            _base_path = typed_cmd->data();
            _compute_full_paths();
        };
        break;

    case CommandType::SET_OSC_OUTPUT_RAW_PATH:
        {
            const auto typed_cmd = static_cast<const SetOSCOutputRawPathCommand*>(cmd);
            _base_raw_path = typed_cmd->data();
            _compute_full_paths();
        };
        break;

    case CommandType::SET_OSC_OUTPUT_HOST:
        {
            const auto typed_cmd = static_cast<const SetOSCOutputHostCommand*>(cmd);
            _host = typed_cmd->data();
            status = _compute_address();
        };
        break;

    case CommandType::SET_OSC_OUTPUT_PORT:
        {
            const auto typed_cmd = static_cast<const SetOSCOutputPortCommand*>(cmd);
            _port = typed_cmd->data();
            if ((_port < 1000) || (_port > 65535))
            {
                status = CommandErrorCode::INVALID_PORT_NUMBER;
            }
            else
            {
                status = _compute_address();
            }
        };
        break;

    default:
        status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
        break;

    }

    // If command was not handled, try in the parent class
    if (status == CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE)
    {
        return OutputBackend::apply_command(cmd);
    }
    else
    {
        return status;
    }

}

void OSCBackend::_compute_full_paths()
{
    for (size_t i=0; i<_full_out_paths.size(); i++)
    {
        std::string cur_path = _base_path;
        std::string cur_raw_path = _base_raw_path;
        std::string cur_sensor_type;
        switch (_pin_types[i])
        {
        case PinType::ANALOG_INPUT:
            cur_sensor_type = std::string("analog");
            break;

        case PinType::DIGITAL_INPUT:
            cur_sensor_type = std::string("digital");
            break;

        case PinType::IMU_INPUT:
            cur_sensor_type = std::string("imu");
            break;

        default:
            break;
        }

        _full_out_paths[i] = concatenate_osc_paths(cur_path,
                                                   concatenate_osc_paths(cur_sensor_type, _pin_names[i]) );
        _full_raw_paths[i] = concatenate_osc_paths(cur_raw_path,
                                                   concatenate_osc_paths(cur_sensor_type, _pin_names[i]) );
    }


}

CommandErrorCode OSCBackend::_compute_address()
{
    std::stringstream port_stream;
    port_stream << _port;
    auto port_str = port_stream.str();
    _address = lo_address_new(_host.c_str(), port_str.c_str());

    if (_address == nullptr)
    {
        return CommandErrorCode::INVALID_URL;
    }

    return CommandErrorCode::OK;
}

