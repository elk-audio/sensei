#include "message_tracker.h"

namespace sensei {
namespace serial_frontend {




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
    return true;
}

timeout MessageTracker::timed_out()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_message_in_transit == nullptr)
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

}; // end namespace serial_frontend
}; // end namespace sensei