/**
 * @brief Class for keeping track of sent messages and their respective acks.
 * @copyright MIND Music Labs AB, Stockholm
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

namespace sensei {
namespace serial_frontend {

enum class ack_status
{
    ACKED_OK,
    TIMED_OUT,
    UNKNOWN_IDENTIFIER,
};


class MessageTracker
{
public:
    MessageTracker(std::chrono::milliseconds timeout);

    ~MessageTracker();

    /**
     * @brief Log a new entry, i.e. call this when sending a packet and waiting for a response
     */
    void log(uint64_t identifier);

    /**
     * @brief Check the status of a received packet against the list of un-acknowledged packets.
     * If it was logged, it will automatically be removed from the list after this call.
     */
    ack_status check_status(uint64_t identifier);

    /**
     * @brief Returns the identifier of the earliest timed out message and removes it from the
     * list of un-acknowledged packets. Returns 0 if there are no timed out messages.
     */
    uint64_t timed_out();

private:
    void update_time();

    std::map<uint64_t, std::chrono::steady_clock::time_point> _entries;
    std::chrono::steady_clock::duration    _timeout;
    std::chrono::steady_clock::time_point  _current_time;
    std::mutex  _mutex;

};
}; // end namespace serial_frontend
}; // end namespace sensei


#endif //SENSEI_MESSAGE_TRACKER_H
