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
#include "message_tracker.h"

namespace sensei {
namespace hw_frontend {

MessageTracker::MessageTracker(std::chrono::milliseconds timeout, int max_retries) :
        _timeout(timeout),
        _max_retries(max_retries),
        _retries(0),
        _identifier(0)
{
}

MessageTracker::~MessageTracker()
{

}

void MessageTracker::store(std::unique_ptr<Command>&& message, uint64_t uuid)
{
    std::lock_guard<std::mutex> lock(_mutex);
    update_time();
    _message_in_transit = std::move(message);
    _send_time = _current_time;
    if (uuid == _identifier)
    {
        _retries--;
    }
    else
    {
        _identifier = uuid;
        _retries = _max_retries - 1;
    }
}

bool MessageTracker::ack(uint64_t identifier)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (identifier != _identifier)
    {
        return false;
    }
    _message_in_transit = nullptr;
    _identifier = 0;
    return true;
}

timeout MessageTracker::timed_out()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_identifier == 0 &&  _message_in_transit == nullptr)
    {
        return timeout::NO_MESSAGE;
    }
    update_time();
    if (_send_time + _timeout > _current_time)
    {
        return timeout::WAITING;
    }
    else
    {
        if (_retries > 0)
        {
            return timeout::TIMED_OUT;
        }
        else
        {
            return timeout::TIMED_OUT_PERMANENTLY;
        }
    }
}

std::unique_ptr<Command> MessageTracker::get_cached_message()
{
    std::lock_guard<std::mutex> lock(_mutex);
    update_time();
    if (_message_in_transit)
    {
        return std::move(_message_in_transit);
    }
    return nullptr;
}

void MessageTracker::update_time()
{
    _current_time = std::chrono::steady_clock::now();
};

}; // namespace hw_frontend
}; // namespace sensei