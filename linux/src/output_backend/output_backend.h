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
            _send_raw_input_active(false)
    {
        _sensor_names.resize(static_cast<size_t>(_max_n_sensors));
    }

    virtual ~OutputBackend()
    { }

    void enable_send_raw_input(const bool enabled)
    {
        _send_raw_input_active = enabled;
    }

    virtual void apply_cmd(const Command *cmd) = 0;

    virtual void send(const OutputValue* transformed_value, const Value* raw_input_value) = 0;

protected:
    int _max_n_sensors;
    bool _send_raw_input_active;
    std::vector<std::string> _sensor_names;
};

}; // namespace output_backend
}; // namespace sensei

#endif //SENSEI_OUTPUT_BACKEND_H_H
