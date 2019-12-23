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
