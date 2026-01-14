/*
 * Copyright 2017-2026 Elk Audio AB
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
 * @brief Output backend with gRPC - forwards events to user frontend
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 */
#include "grpc_backend.h"
#include "message/command_defs.h"
#include "user_frontend/grpc_user_frontend.h"
#include "logging.h"

using namespace sensei;
using namespace sensei::output_backend;

namespace {

SENSEI_GET_LOGGER_WITH_MODULE_NAME("grpc_backend");

} // anonymous namespace

//==============================================================================
// GrpcBackend Implementation
//==============================================================================

GrpcBackend::GrpcBackend(const int max_n_input_pins) :
    OutputBackend(max_n_input_pins),
    _user_frontend(nullptr)
{
    SENSEI_LOG_INFO("GrpcBackend created");
}

GrpcBackend::~GrpcBackend()
{
    SENSEI_LOG_INFO("GrpcBackend destroyed");
}

void GrpcBackend::set_user_frontend(user_frontend::GrpcUserFrontend* frontend)
{
    _user_frontend = frontend;
    SENSEI_LOG_INFO("GrpcBackend linked to GrpcUserFrontend");
}

CommandErrorCode GrpcBackend::apply_command(const Command* cmd)
{
    auto status = OutputBackend::apply_command(cmd);

    if (_user_frontend)
    {
        // the frontend needs a map of IDs to names and types so we forward this info
        if (cmd->type() == CommandType::SET_SENSOR_NAME || cmd->type() == CommandType::SET_SENSOR_TYPE)
        {
            auto id = cmd->index();
            _user_frontend->update_controller(id, _sensor_names[id], _pin_types[id]);
        }
    }
    else
    {
        // this can happen in tests, otherwise shouldn't be possible
        SENSEI_LOG_ERROR("gRPC user frontend not set, skipping controller update");
    }

    return status;
}

void GrpcBackend::send(const OutputValue* transformed_value, const Value* /*raw_input_value*/)
{
    if (!_user_frontend)
    {
        SENSEI_LOG_ERROR("gRPC user frontend not set, skipping send");
        return;
    }

    int sensor_index = transformed_value->index();

    // Send transformed output value
    if (_send_output_active)
    {
        SensorType sensor_type = _pin_types[sensor_index];
        float value = transformed_value->value();
        uint32_t timestamp = transformed_value->timestamp();

        sensei_rpc::Event event = _create_proto_event(sensor_index, sensor_type, value, timestamp);

        // Forward event to user frontend for broadcasting
        _user_frontend->broadcast_event(event);

        SENSEI_LOG_DEBUG("Sending: sensor={} value={}", _sensor_names[sensor_index], value);
    }

    // Raw input value sending - not implemented yet
    if (_send_raw_input_active)
    {
        // TODO(andrew): Is this needed?
        SENSEI_LOG_ERROR("Raw input sending not implemented yet");
        assert(false);
    }
}

sensei_rpc::Event GrpcBackend::_create_proto_event(int sensor_index,
                                                  SensorType sensor_type,
                                                  float value,
                                                  uint32_t timestamp)
{
    sensei_rpc::Event event;
    event.set_controller_id(sensor_index);
    event.set_timestamp(static_cast<int64_t>(timestamp));

    // Map SENSEI sensor types to proto event types
    switch (sensor_type)
    {
        case SensorType::DIGITAL_INPUT:
        {
            // Digital sensor -> ToggleEvent
            auto* toggle_ev = event.mutable_toggle_ev();
            toggle_ev->set_value(value > 0.5f);  // Convert float to bool
            break;
        }

        case SensorType::RANGE_INPUT:
        {
            // Range sensor -> RangeEvent
            auto* range_ev = event.mutable_range_ev();
            range_ev->set_value(static_cast<int32_t>(value));  // Convert float to int32
            break;
        }

        case SensorType::ANALOG_INPUT:
        {
            // Analog/Continuous -> AnalogEvent
            auto* analog_ev = event.mutable_analog_ev();
            analog_ev->set_value(value);
            break;
        }

        // This will only be sent following a RefreshAllStates.
        case SensorType::DIGITAL_OUTPUT:
        {
            // Digital output -> LedEvent
            auto* led_ev = event.mutable_led_ev();
            led_ev->set_value(value > 0.5f);  // Convert float to bool
            break;
        }

        default:
        {
            SENSEI_LOG_ERROR("Trying to create event for unexpected sensor_type: {}", static_cast<int>(sensor_type));
            assert(false);
        }
    }

    return event;
}
