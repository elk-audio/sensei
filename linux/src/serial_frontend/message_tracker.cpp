#include "message_tracker.h"

namespace sensei {
namespace serial_frontend {

MessageTracker::MessageTracker(std::chrono::milliseconds timeout) :
_timeout(timeout)
{
}

MessageTracker::~MessageTracker()
{

}

void MessageTracker::store(uint64_t identifier)
{
    update_time();
    std::lock_guard<std::mutex> lock(_mutex);
    _entries.insert(std::pair<uint64_t, std::chrono::steady_clock::time_point>(identifier, _current_time));
}

ack_status MessageTracker::check_status(uint64_t identifier)
{
    update_time();
    ack_status status = ack_status::UNKNOWN_IDENTIFIER;
    std::lock_guard<std::mutex> lock(_mutex);
    auto entry = _entries.find(identifier);
    if (entry != _entries.end())
    {
        if (entry->second > _current_time - _timeout)
        {
            status = ack_status::ACKED_OK;
        }
        else
        {
            status = ack_status::TIMED_OUT;
        }
        _entries.erase(entry);
    }
    return status;
}

uint64_t MessageTracker::timed_out()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_entries.empty())
    {
        return 0;
    }
    for(const auto &entry : _entries)
    {
        if (entry.second < _current_time - _timeout)
        {
            uint64_t id = entry.first;
            _entries.erase(id);
            return id;
        }
    }
    return 0;
}

void MessageTracker::update_time()
{
    _current_time = std::chrono::steady_clock::now();
};

}; // end namespace serial_frontend
}; // end namespace sensei