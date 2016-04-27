#ifndef SENSEI_TEST_UTILS_H
#define SENSEI_TEST_UTILS_H

#include "utils.h"
#include "message/message_factory.h"


#define CMD_UPTR(msg) std::move(static_unique_ptr_cast<Command, BaseMessage>(msg))

template<typename DerivedCommand>
std::unique_ptr<DerivedCommand> extract_cmd_from(std::vector<std::unique_ptr<sensei::BaseMessage>>& msg_queue)
{
    auto tmp_msg = static_unique_ptr_cast<DerivedCommand, sensei::BaseMessage>(std::move(msg_queue.back()));
    msg_queue.pop_back();

    return std::move(tmp_msg);
}

#endif //SENSEI_TEST_UTILS_H
