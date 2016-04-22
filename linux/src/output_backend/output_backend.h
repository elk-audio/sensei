/**
 * @brief Base class for output backends
 * @copyright MIND Music Labs AB, Stockholm
 *
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
    OutputBackend(const int max_n_sensors = 64) :
            _max_n_sensors(max_n_sensors),
            _send_output_active(true),
            _send_raw_input_active(false)
    {
        _sensor_names.resize(static_cast<size_t>(_max_n_sensors));
        _pin_types.resize(static_cast<size_t>(_max_n_sensors));
        std::fill(_pin_types.begin(), _pin_types.end(), PinType::UNDEFINED);
    }

    virtual ~OutputBackend()
    {}

    void enable_send_raw_input(const bool enabled)
    {
        _send_raw_input_active = enabled;
    }

    virtual CommandErrorCode apply_command(const Command *cmd)
    {
        CommandErrorCode status = CommandErrorCode::OK;
        auto pin_idx = cmd->sensor_index();

        switch(cmd->type())
        {
        case CommandType::SET_PIN_NAME:
            {
                const auto typed_cmd = static_cast<const SetPinNameCommand *>(cmd);
                _sensor_names[pin_idx] = typed_cmd->data();
            };
            break;

        case CommandType::SET_PIN_TYPE:
            {
                const auto typed_cmd = static_cast<const SetPinTypeCommand*>(cmd);
                _pin_types[pin_idx] = typed_cmd->data();
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
    int _max_n_sensors;
    bool _send_output_active;
    bool _send_raw_input_active;
    std::vector<std::string> _sensor_names;
    std::vector<PinType> _pin_types;
};

}; // namespace output_backend
}; // namespace sensei

#endif //SENSEI_OUTPUT_BACKEND_H_H
