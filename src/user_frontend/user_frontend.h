/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Base class for runtime user frontend
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
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
     * @param [in] sensor_index Configurable input pin index
     * @param [in] enabled      Enable status
     */
    void set_enabled(int index, bool enabled);

    /**
     * @brief Set the value of a digital output.
     *
     * This should be preferably called by derived classes in their
     * runtime thread.
     *
     * @param [in] index         Digital output index
     * @param [in] value         New value for selected output
     */
    void set_digital_output(int index, bool value);

    /**
     * @brief Set the value of a continuous output
     *
     * This should be preferably called by derived classes in their
     * runtime thread.
     *
     * @param [in] index         Output index
     * @param [in] value         New value for selected sensor normalised to a [0, 1] range
     */
    void set_continuous_output(int index, float value);

    /**
     * @brief Set the value of a range output.
     *
     * This should be preferably called by derived classes in their
     * runtime thread.
     *
     * @param [in] index         Output index
     * @param [in] value         New value for selected sensor
     */
    void set_range_output(int index, int value);

private:
    SynchronizedQueue<std::unique_ptr<BaseMessage>>* _queue;
    int _max_n_input_pins;
    int _max_n_out_pins;

    MessageFactory _factory;
};

} // namespace user_frontend
} // namespace sensei

#endif //SENSEI_USER_FRONTEND_H