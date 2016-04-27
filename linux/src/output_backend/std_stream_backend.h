/**
 * @brief Output backend using standard output/error streams
 * @copyright MIND Music Labs AB, Stockholm
 */
#ifndef SENSEI_STD_STREAM_BACKEND_H
#define SENSEI_STD_STREAM_BACKEND_H

#include "output_backend.h"

namespace sensei {

namespace output_backend {

class StandardStreamBackend : public OutputBackend
{
public:
    StandardStreamBackend(const int max_n_input_pins=64);

    ~StandardStreamBackend()
    {}

    CommandErrorCode apply_command(std::unique_ptr<Command> cmd) override;

    void send(std::unique_ptr<OutputValue> transformed_value, std::unique_ptr<Value> raw_input_value) override;
};

}; // namespace output_backend
}; // namespace sensei

#endif //SENSEI_STD_STREAM_BACKEND_H
