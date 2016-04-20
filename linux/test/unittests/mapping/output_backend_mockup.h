#ifndef SENSEI_OUTPUT_BACKEND_MOCKUP_H_H
#define SENSEI_OUTPUT_BACKEND_MOCKUP_H_H

#include "output_backend/output_backend.h"

// just for testing and nowhere else, I swear ;)
using namespace sensei;
using namespace sensei::output_backend;

// Mockup class
class OutputBackendMockup : public OutputBackend
{
public:
    OutputBackendMockup() :
            OutputBackend(64),
            _last_output_value(0.0f),
            _last_raw_analogue_input(0),
            _last_raw_digital_input(false)
    {}

    ~OutputBackendMockup()
    {}

    CommandErrorCode apply_command(const Command* /*cmd*/) override
    {
        return CommandErrorCode::OK;
    }

    void send(const OutputValue* transformed_value, const Value* raw_input_value)
    {
        _last_output_value = transformed_value->value();
        _last_timestamp = transformed_value->timestamp();

        if (_send_raw_input_active)
        {

            switch (raw_input_value->type())
            {
            case ValueType::ANALOG:
            {
                auto typed_val = static_cast<const AnalogValue *>(raw_input_value);
                _last_raw_analogue_input = typed_val->value();
            }
                break;

            case ValueType::DIGITAL:
            {
                auto typed_val = static_cast<const DigitalValue *>(raw_input_value);
                _last_raw_digital_input = typed_val->value();
            }
                break;

            default:
                break;
            }
        }

    }

    uint32_t _last_timestamp;
    float _last_output_value;
    int   _last_raw_analogue_input;
    bool  _last_raw_digital_input;
};

#endif //SENSEI_OUTPUT_BACKEND_MOCKUP_H_H
