#ifndef SENSEI_TEST_UTILS_H
#define SENSEI_TEST_UTILS_H

#include <functional>
#include <thread>

#include "utils.h"
#include "message/message_factory.h"

#define CMD_UPTR(msg) static_unique_ptr_cast<Command, BaseMessage>(msg)
#define CMD_PTR(msg) CMD_UPTR(msg).get()

constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(2);

template<typename DerivedCommand>
std::unique_ptr<DerivedCommand> extract_cmd_from(std::vector<std::unique_ptr<sensei::BaseMessage>>& msg_queue)
{
    auto tmp_msg = static_unique_ptr_cast<DerivedCommand, sensei::BaseMessage>(std::move(msg_queue.back()));
    msg_queue.pop_back();

    return std::move(tmp_msg);
}

inline bool wait_for(std::function<bool()> condition, std::chrono::seconds timeout) {
    auto start = std::chrono::steady_clock::now();
    while (!condition() &&
        std::chrono::steady_clock::now() - start < std::chrono::seconds(timeout)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return condition();
}

/* Custom comparison operator for Range & MultiplexerData structs. This is only needed for testing */
namespace sensei {
inline bool operator==(const Range& lhs, const Range& rhs)
{
    return lhs.min == rhs.min && lhs.max == rhs.max;
}

inline bool operator==(const MultiplexerData& lhs, const MultiplexerData& rhs)
{
    return lhs.id == rhs.id && lhs.pin == rhs.pin;
}
}
#endif //SENSEI_TEST_UTILS_H
