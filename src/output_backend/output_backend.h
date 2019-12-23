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
 * @brief Base class for output backends
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_OUTPUT_BACKEND_H_H
#define SENSEI_OUTPUT_BACKEND_H_H

#include "message/value_defs.h"
#include "message/command_defs.h"

namespace sensei {

namespace output_backend {

class OutputBackend
{
public:
    OutputBackend(const int max_n_input_pins = 64) :
            _max_n_pins(max_n_input_pins),
            _send_output_active(true),
            _send_raw_input_active(false)
    {
        _sensor_names.resize(static_cast<size_t>(_max_n_pins));
        _pin_types.resize(static_cast<size_t>(_max_n_pins));
        std::fill(_pin_types.begin(), _pin_types.end(), SensorType::UNDEFINED);
    }

    virtual ~OutputBackend()
    {}

    virtual CommandErrorCode apply_command(const Command *cmd)
    {
        CommandErrorCode status = CommandErrorCode::OK;
        auto pin_idx = cmd->index();

        switch(cmd->type())
        {
        case CommandType::SET_SENSOR_NAME:
            {
                const auto typed_cmd = static_cast<const SetPinNameCommand *>(cmd);
                _sensor_names[pin_idx] = typed_cmd->data();
            };
            break;

        case CommandType::SET_SENSOR_TYPE:
            {
                const auto typed_cmd = static_cast<const SetSensorTypeCommand*>(cmd);
                _pin_types[pin_idx] = typed_cmd->data();
            };
            break;

        case CommandType::SET_SEND_OUTPUT_ENABLED:
            {
                const auto typed_cmd = static_cast<const SetSendOutputEnabledCommand*>(cmd);
                _send_output_active = typed_cmd->data();
            };
            break;

        case CommandType::SET_SEND_RAW_INPUT_ENABLED:
            {
                const auto typed_cmd = static_cast<const SetSendRawInputEnabledCommand*>(cmd);
                _send_raw_input_active = typed_cmd->data();
            };
            break;

        default:
            status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
            break;
        }

        return status;
    }

    virtual void send(const OutputValue* transformed_value, const Value* raw_input_value) = 0;

protected:
    int _max_n_pins;
    bool _send_output_active;
    bool _send_raw_input_active;
    std::vector<std::string> _sensor_names;
    std::vector<SensorType> _pin_types;
};

} // namespace output_backend
} // namespace sensei

#endif //SENSEI_OUTPUT_BACKEND_H_H