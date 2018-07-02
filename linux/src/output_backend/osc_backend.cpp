#include <sstream>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <lo/lo_types.h>

#include "osc_backend.h"

#include "logging.h"

using namespace sensei;
using namespace sensei::output_backend;

namespace {

SENSEI_GET_LOGGER_WITH_MODULE_NAME("osc_backend");

constexpr uint32_t US_TO_S = 1'000'000;
constexpr uint32_t OSC_TIME_FRAC = UINT32_MAX / US_TO_S;

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

lo_timetag to_osc_timestamp(uint32_t value_time)
{
    lo_timetag lo_time;
    lo_time.sec = value_time / 1'000'000;
    lo_time.frac = (value_time % 1'000'000) * OSC_TIME_FRAC;
    return lo_time;
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
        if (transformed_value->timestamp() == 0)
            lo_send(_address, _full_out_paths[sensor_index].c_str(), "f", transformed_value->value());
        else
            lo_send(_address, _full_out_paths[sensor_index].c_str(), "ft",
                    transformed_value->value(), to_osc_timestamp(transformed_value->timestamp()));
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

        case ValueType::CONTINUOUS:
            {
                auto typed_val = static_cast<const ContinuousValue *>(raw_input_value);
                input_val = static_cast<int>(typed_val->value());
            }
            break;

        default:
            break;
        }
        if (transformed_value->timestamp() == 0)
            lo_send(_address, _full_raw_paths[sensor_index].c_str(), "i", input_val);
        else
            lo_send(_address, _full_raw_paths[sensor_index].c_str(), "i", input_val, to_osc_timestamp(transformed_value->timestamp()));
    }

}

CommandErrorCode OSCBackend::apply_command(const Command *cmd)
{
    CommandErrorCode status = CommandErrorCode::OK;
    auto pin_idx = cmd->index();

    switch(cmd->type())
    {

    case CommandType::SET_SENSOR_NAME:
        {
            const auto typed_cmd = static_cast<const SetPinNameCommand *>(cmd);
            _sensor_names[pin_idx] = typed_cmd->data();
            _compute_full_paths();
        };
        break;

    case CommandType::SET_SENSOR_TYPE:
        {
            const auto typed_cmd = static_cast<const SetSensorTypeCommand*>(cmd);
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
        case SensorType::ANALOG_INPUT:
            cur_sensor_type = std::string("analog");
            break;

        case SensorType::DIGITAL_INPUT:
            cur_sensor_type = std::string("digital");
            break;

        case SensorType::RANGE_INPUT:
            cur_sensor_type = std::string("range");
            break;

        case SensorType::CONTINUOUS_INPUT:
            cur_sensor_type = std::string("continuous");
            break;

        default:
            break;
        }

        _full_out_paths[i] = concatenate_osc_paths(cur_path,
                                                   concatenate_osc_paths(cur_sensor_type, _sensor_names[i]) );
        _full_raw_paths[i] = concatenate_osc_paths(cur_raw_path,
                                                   concatenate_osc_paths(cur_sensor_type, _sensor_names[i]) );
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

