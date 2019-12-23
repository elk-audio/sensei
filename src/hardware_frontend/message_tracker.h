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
 * @brief Class for keeping track of sent messages and their respective acks.
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * A helper class to keep track of sent messages and acknowledgements to identify
 * message timeouts.
 */
#ifndef SENSEI_MESSAGE_TRACKER_H
#define SENSEI_MESSAGE_TRACKER_H

#include <cstdint>
#include <chrono>
#include <mutex>
#include <map>

#include <iostream>

#include "message/base_command.h"

namespace sensei {
namespace hw_frontend {

enum class timeout
{
    NO_MESSAGE,
    WAITING,
    TIMED_OUT,
    TIMED_OUT_PERMANENTLY,
};


class MessageTracker
{
public:
    MessageTracker(std::chrono::milliseconds timeout, int max_retries);

    ~MessageTracker();

    /**
     * @brief Log a new entry, i.e. call this when sending a packet and waiting for a response
     */
    void store(std::unique_ptr<Command>&& message, uint64_t uuid);

    /**
     * @brief Check the status of a received packet against un-acknowledged packets.
     * If it was logged, it will automatically be removed from the list after this call.
     */
    bool ack(uint64_t identifier);

    /**
     * @brief Returns timeout status
     */
    timeout timed_out();

    /**
    * @brief Returns the message_in_transit for resend or for destruction
     * Returns nullptr if there is no cached message.
    */
    std::unique_ptr<Command> get_cached_message();

private:
    void update_time();

    std::chrono::steady_clock::duration    _timeout;
    std::chrono::steady_clock::time_point  _current_time;
    std::chrono::steady_clock::time_point  _send_time;

    int                                    _max_retries;
    int                                    _retries;
    std::unique_ptr<Command>               _message_in_transit;

    uint64_t    _identifier;
    std::mutex  _mutex;

};

} // namespace hw_frontend
} // namespace sensei


#endif //SENSEI_MESSAGE_TRACKER_H