/**
 * @brief Output backend with OSC
 * @copyright MIND Music Labs AB, Stockholm
 */
#ifndef SENSEI_OSC_BACKEND_H
#define SENSEI_OSC_BACKEND_H

#include <lo/lo.h>
#include "output_backend.h"

namespace sensei {
namespace output_backend {

class OSCBackend : public OutputBackend
{
public:
    OSCBackend(const int max_n_sensors=64);

    ~OSCBackend()
    {}

    CommandErrorCode apply_command(const Command *cmd) override;

    void send(const OutputValue* transformed_value, const Value* raw_input_value) override;

private:
    void _compute_full_paths();

    CommandErrorCode _compute_address();

    std::string _base_path;
    std::string _base_raw_path;
    std::string _host;
    int _port;
    lo_address  _address;

    std::vector<std::string> _full_out_paths;
    std::vector<std::string> _full_raw_paths;
};

}; // namespace output_backend
}; // namespace sensei

#endif //SENSEI_OSC_BACKEND_H
