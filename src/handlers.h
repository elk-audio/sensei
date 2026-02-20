
#ifndef SENSEI_HANDLERS_H
#define SENSEI_HANDLERS_H

#include "synchronized_queue.h"
#include "message/base_command.h"

namespace sensei {

enum class ThreadingMode
{
    ASYNCHRONOUS,
    SYNCHRONOUS
};

class MessageHandler
{
public:
    virtual void handle_event(const BaseMessage* event) = 0;
    virtual void handle_command(const Command* command) = 0;
    virtual SynchronizedQueue<std::unique_ptr<BaseMessage>>* incoming_queue() = 0;
    virtual SynchronizedQueue<std::unique_ptr<Command>>* outgoing_queue() = 0;
};

}

#endif //SENSEI_HANDLERS_H
