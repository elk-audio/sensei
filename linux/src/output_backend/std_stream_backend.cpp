#include <cstdio>
#include <cassert>

#include "std_stream_backend.h"

using namespace sensei;
using namespace sensei::output_backend;

StandardStreamBackend::StandardStreamBackend(const int max_n_sensors) :
        OutputBackend(max_n_sensors)
{
}

void StandardStreamBackend::send(const OutputValue* transformed_value, const Value* raw_input_value)
{
    int sensor_index = transformed_value->sensor_index();

    if (_send_output_active)
    {
        printf("Pin: %d, name: %s, value: %f\n", sensor_index,
                                                 _sensor_names[sensor_index].c_str(),
                                                 transformed_value->value());
    }

    if (_send_raw_input_active)
    {
        assert(raw_input_value != nullptr);
        switch (raw_input_value->type())
        {
        case ValueType::ANALOG:
            {
                auto typed_val = static_cast<const AnalogValue *>(raw_input_value);
                fprintf(stderr, "--RAW INPUT-- Pin: %d, name: %s, value: %d\n", sensor_index,
                                                                                _sensor_names[sensor_index].c_str(),
                                                                                typed_val->value());
            }
            break;

        case ValueType::DIGITAL:
            {
                auto typed_val = static_cast<const DigitalValue *>(raw_input_value);
                fprintf(stderr, "--RAW INPUT-- Pin: %d, name: %s, value: %d\n", sensor_index,
                                                                                _sensor_names[sensor_index].c_str(),
                                                                                typed_val->value());
            }
            break;

        default:
            break;
        }
    }

}

CommandErrorCode StandardStreamBackend::apply_command(const Command *cmd)
{
    // Try to handle generic cases in base class method
    CommandErrorCode status = OutputBackend::apply_command(cmd);

    // command was consumed by parent or some error has occurred while doing it
    if ((status == CommandErrorCode::OK) || (status != CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE) )
    {
        return status;
    }

    switch(cmd->type())
    {
    default:
        status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
        break;

    }

    return status;

}