#ifndef SENSEI_TEST_UTILS_H
#define SENSEI_TEST_UTILS_H

#include "utils.h"
#include "message/message_factory.h"

#define CMD_UPTR(msg) static_unique_ptr_cast<Command, BaseMessage>(msg)
#define CMD_PTR(msg) CMD_UPTR(msg).get()

template<typename DerivedCommand>
std::unique_ptr<DerivedCommand> extract_cmd_from(std::vector<std::unique_ptr<sensei::BaseMessage>>& msg_queue)
{
    auto tmp_msg = static_unique_ptr_cast<DerivedCommand, sensei::BaseMessage>(std::move(msg_queue.back()));
    msg_queue.pop_back();

    return std::move(tmp_msg);
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
