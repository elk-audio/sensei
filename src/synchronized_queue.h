//! @file synchronized_queue.cpp
//! @brief A templated uni-directional queue with build in condition_variable
//! @author Gustav Andersson
//! @copyright MIND Music Labs AB, Stockholm
//! @date 2016-03-30

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
