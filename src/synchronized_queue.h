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
 * @brief A templated uni-directional queue with build in condition_variable
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_SYNCHRONIZED_QUEUE_H
#define SENSEI_SYNCHRONIZED_QUEUE_H

#include <condition_variable>
#include <chrono>
#include "locked_queue.h"

template <class T> class SynchronizedQueue
{
public:
    void push(T const& message)
    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _queue.push_front(message);
        _notifier.notify_one();
    }

    void push(T&& message)
    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _queue.push_front(std::move(message));
        _notifier.notify_one();
    }

    T pop()
    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        T message = std::move(_queue.back());
        _queue.pop_back();
        return std::move(message);
    }

    void wait_for_data(const std::chrono::milliseconds& timeout)
    {
        if (_queue.empty())
        {
            std::unique_lock<std::mutex> lock(_wait_mutex);
            _notifier.wait_for(lock, timeout);
        }
    }

    bool empty()
    {
        return _queue.empty();
    }
private:
    std::deque<T>           _queue;
    std::mutex              _queue_mutex;
    std::mutex              _wait_mutex;
    std::condition_variable _notifier;
};

#endif //SENSEI_SYNCHRONIZED_QUEUE_H