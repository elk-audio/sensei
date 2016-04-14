/**
 * @brief Main class for remapping sensors data into output format
 * @copyright MIND Music Labs AB, Stockholm
 *
 */

#include <memory>
#include <cassert>

#include "mapping_processor.h"

using namespace sensei;
using namespace sensei::mapping;

MappingProcessor::MappingProcessor(const int max_n_sensors) :
    _max_n_sensors(max_n_sensors)
{
    _mappers.resize(_max_n_sensors);
    std::fill(_mappers.begin(), _mappers.end(), nullptr);
}

CommandErrorCode MappingProcessor::apply_command(const Command *cmd)
{
    // TODO: find a better way to manage changes of PinType,
    //       possibly without requiring SetPinType to be the 1st command received

    int sensor_index = cmd->sensor_index();
    if ( (sensor_index < 0) || (sensor_index > (_max_n_sensors-1)) )
    {
        return CommandErrorCode::INVALID_PIN_INDEX;
    }

    if (cmd->type() == CommandType::SET_PIN_TYPE)
    {
        CommandErrorCode status = CommandErrorCode::OK;
        const auto typed_cmd = static_cast<const SetPinTypeCommand*>(cmd);
        auto pin_type = typed_cmd->data();
        switch(pin_type)
        {
        case PinType::DIGITAL_INPUT:
            _mappers[sensor_index].reset(new DigitalSensorMapper(sensor_index));
            break;

        case PinType::ANALOG_INPUT:
            _mappers[sensor_index].reset(new AnalogSensorMapper(sensor_index));
            break;

        default:
            status = CommandErrorCode::INVALID_VALUE;

        }
        return status;
    }
    else
    {
        assert(_mappers[sensor_index] != nullptr);
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
    int sensor_index = value->sensor_index();
    if (_mappers[sensor_index] != nullptr)
    {
        _mappers[sensor_index]->process(value, backend);
    }
    else
    {
        // TODO: log an error instead
    }
}

