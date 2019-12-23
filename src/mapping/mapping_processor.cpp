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
 * @brief Main class for remapping sensors data into output format
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#include <memory>
#include <cassert>

#include "mapping_processor.h"
#include "logging.h"

using namespace sensei;
using namespace sensei::mapping;

SENSEI_GET_LOGGER_WITH_MODULE_NAME("mapper");

MappingProcessor::MappingProcessor(int max_no_sensors) :
    _max_no_sensors(max_no_sensors)
{
    _mappers.resize(_max_no_sensors);
    std::fill(_mappers.begin(), _mappers.end(), nullptr);
}

CommandErrorCode MappingProcessor::apply_command(const Command *cmd)
{
    // TODO: find a better way to manage changes of SensorType,
    //       possibly without requiring SetPinType to be the 1st command received

    int sensor_index = cmd->index();
    if ( (sensor_index < 0) || (sensor_index > (_max_no_sensors-1)) )
    {
        return CommandErrorCode::INVALID_SENSOR_INDEX;
    }

    if (cmd->type() == CommandType::SET_SENSOR_TYPE)
    {
        SENSEI_LOG_INFO("Setting up new mapper for sensor id: {}", sensor_index);
        CommandErrorCode status = CommandErrorCode::OK;
        const auto typed_cmd = static_cast<const SetSensorTypeCommand*>(cmd);
        auto pin_type = typed_cmd->data();
        switch(pin_type)
        {
        case SensorType::DIGITAL_INPUT:
        case SensorType::DIGITAL_OUTPUT:
        case SensorType::NO_OUTPUT:
            _mappers[sensor_index].reset(new DigitalSensorMapper(sensor_index));
            break;

        case SensorType::ANALOG_INPUT:
        case SensorType::ANALOG_OUTPUT:
            _mappers[sensor_index].reset(new AnalogSensorMapper(sensor_index));
            break;

        case SensorType::CONTINUOUS_INPUT:
        case SensorType::CONTINUOUS_OUTPUT:
            _mappers[sensor_index].reset(new ContinuousSensorMapper(sensor_index));
            break;

        case SensorType::RANGE_INPUT:
        case SensorType::RANGE_OUTPUT:
            _mappers[sensor_index].reset(new RangeSensorMapper(sensor_index));
            break;

        default:
            status = CommandErrorCode::INVALID_VALUE;

        }
        return status;
    }
    else
    {
        // Apply command only to already initialized pins
        if (_mappers[sensor_index] == nullptr)
        {
            return CommandErrorCode::UNINITIALIZED_SENSOR;
        }
        return _mappers[sensor_index]->apply_command(cmd);
    }

}

void MappingProcessor::put_config_commands_into(CommandIterator out_iterator)
{
    for (auto& mapper : _mappers)
    {
        if (mapper != nullptr)
        {
            mapper->put_config_commands_into(out_iterator);
        }
    }
}

void MappingProcessor::process(Value *value, output_backend::OutputBackend *backend)
{
    int sensor_index = value->index();
    if (_mappers[sensor_index] != nullptr)
    {
        _mappers[sensor_index]->process(value, backend);
    }
    else
    {
        SENSEI_LOG_ERROR("Got value message for uninitialized sensor {}", value->index());
    }
}

std::unique_ptr<Command> MappingProcessor::process_set(Value* value)
{
    int sensor_index = value->index();
    if (static_cast<unsigned int>(sensor_index) < _mappers.size() && _mappers[sensor_index] != nullptr)
    {
        return _mappers[sensor_index]->process_set_value(value);
    }
    else
    {
        SENSEI_LOG_ERROR("Got set value message for uninitialized sensor {}", value->index());
        return nullptr;
    }
}