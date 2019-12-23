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
 * @brief A simple locked queue implementation
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_LOCKEDQUEUE_H
#define SENSEI_LOCKEDQUEUE_H

#include <deque>
#include <memory>
#include <mutex>

template <class T> class LockedQueue
{
public:

    void push(T const& message)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.push_front(message);
    }

    void push(T&& message)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.push_front(std::move(message));
    }

    T pop()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        T message = std::move(_queue.back());
        _queue.pop_back();
        return std::move(message);
    }

    bool empty()
    {
        return _queue.empty();
    }
private:
    std::deque<T> _queue;
    std::mutex    _mutex;
};

#endif //SENSEI_LOCKEDQUEUE_H