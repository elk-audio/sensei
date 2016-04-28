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
    OutputBackend(const int max_n_input_pins = 64) :
            _max_n_pins(max_n_input_pins),
            _send_output_active(true),
            _send_raw_input_active(false)
    {
        _pin_names.resize(static_cast<size_t>(_max_n_pins));
        _pin_types.resize(static_cast<size_t>(_max_n_pins));
        std::fill(_pin_types.begin(), _pin_types.end(), PinType::UNDEFINED);
    }

    virtual ~OutputBackend()
    {}

    virtual CommandErrorCode apply_command(const Command *cmd)
    {
        CommandErrorCode status = CommandErrorCode::OK;
        auto pin_idx = cmd->index();

        switch(cmd->type())
        {
        case CommandType::SET_PIN_NAME:
            {
                const auto typed_cmd = static_cast<const SetPinNameCommand *>(cmd);
                _pin_names[pin_idx] = typed_cmd->data();
            };
            break;

        case CommandType::SET_PIN_TYPE:
            {
                const auto typed_cmd = static_cast<const SetPinTypeCommand*>(cmd);
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
    std::vector<std::string> _pin_names;
    std::vector<PinType> _pin_types;
};

}; // namespace output_backend
}; // namespace sensei

#endif //SENSEI_OUTPUT_BACKEND_H_H
