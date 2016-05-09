/**
 * @brief Base class for runtime user frontend
 * @copyright MIND Music Labs AB, Stockholm
 *
 * This module give run-time control from the user over some fast-changing configuration
 * parameters (e.g. sensors enabled/disabled) and access to digital output pins.
 */
#ifndef SENSEI_USER_FRONTEND_H
#define SENSEI_USER_FRONTEND_H

#include <message/message_factory.h>
#include "synchronized_queue.h"
#include "message/message_factory.h"

namespace sensei {
namespace user_frontend {

class UserFrontend
{
public:
    UserFrontend(SynchronizedQueue<std::unique_ptr<BaseMessage>> *queue,
                 const int max_n_input_pins,
                 const int max_n_digital_out_pins) :
            _queue(queue),
            _max_n_input_pins(max_n_input_pins),
            _max_n_out_pins(max_n_digital_out_pins)
    {}

    virtual ~UserFrontend()
    {}

    /**
     * @brief Modify internal configuration according to the given command.
     *
     * @param [in] cmd Configuration command.
     *
     * @return CommandErrorCode::OK if command was succesful,
     *         other codes in case of error.
     */
    virtual CommandErrorCode apply_command(const Command *cmd);

    /**
     * @brief Put an enabled message in the shared queue.
     *
     * This should be preferably called by derived classes in their
     * runtime thread.
     *
     * @param [in] pin_index    Configurable input pin index
     * @param [in] enabled      Enable status
     */
    void set_pin_enabled(const int pin_index, const bool enabled);

    /**
     * @brief Set a message to change the value of a digital output pin.
     *
     * This should be preferably called by derived classes in their
     * runtime thread.
     *
     * @param [in] out_pin_index    Digital output pin index
     * @param [in] value            New value for selected pin
     */
    void set_digital_output(const int out_pin_index, const bool value);

private:
    SynchronizedQueue<std::unique_ptr<BaseMessage>>* _queue;
    int _max_n_input_pins;
    int _max_n_out_pins;

    MessageFactory _factory;
};

}; // namespace user_frontend
}; // namespace sensei

#endif //SENSEI_USER_FRONTEND_H
