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

class OutputBackend
{
public:

    OutputBackend()
    {
    }

    virtual ~OutputBackend()
    {}

    virtual void apply_cmd(const Command* cmd);

    virtual void send(const OutputValue* value) = 0;

};


}; // namespace sensei

#endif //SENSEI_OUTPUT_BACKEND_H_H
