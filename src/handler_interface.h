
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

/**
 * @brief Interface for handling messages and commands in the Sensei pipeline.
 */
class MessageHandler
{
public:
    virtual ~MessageHandler() = default;

    /**
     * @brief Process an event message immediately on the calling thread.
     * @param event Pointer to the event to process. Must not be null.
     */
    virtual void process_event(const BaseMessage* event) = 0;

    /**
     * @brief Post an event to be processed asynchronously.
     * @param event The event to enqueue. Ownership is transferred to the handler.
     */
    virtual void post_event(std::unique_ptr<BaseMessage> event) = 0;

    /**
     * @brief Returns the queue used for outgoing commands produced by this handler.
     * @return Pointer to the outgoing command queue. The caller does not take ownership.
     */
    virtual SynchronizedQueue<std::unique_ptr<Command>>* outgoing_queue() = 0;
};

} // namespace sensei

#endif //SENSEI_HANDLERS_H
