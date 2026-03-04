#ifndef SENSEI_TEST_UTILS_H
#define SENSEI_TEST_UTILS_H

#include <functional>
#include <thread>

#include "utils.h"
#include "message/message_factory.h"
#include "handler_interface.h"

#define CMD_UPTR(msg) static_unique_ptr_cast<Command, BaseMessage>(msg)
#define CMD_PTR(msg)  CMD_UPTR(msg).get()

constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(2);

template<typename DerivedCommand>
std::unique_ptr<DerivedCommand> extract_cmd_from(std::vector<std::unique_ptr<sensei::BaseMessage>>& msg_queue)
{
    auto tmp_msg = static_unique_ptr_cast<DerivedCommand, sensei::BaseMessage>(std::move(msg_queue.back()));
    msg_queue.pop_back();

    return std::move(tmp_msg);
}

inline bool wait_for(std::function<bool()> condition, std::chrono::seconds timeout)
{
    auto start = std::chrono::steady_clock::now();
    while (!condition() &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(timeout))
    {
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

class MessageHandlerMock : public MessageHandler
{
public:
    void process_event(const BaseMessage* event) override
    {
        // Events are stack objects and cant be saved, so just store their representation
        processed_events.push_back(event->representation());
    }

    virtual void post_event(std::unique_ptr<BaseMessage> event) override
    {
        event_queue.push(std::move(event));
    }

    SynchronizedQueue<std::unique_ptr<Command>>* outgoing_queue() override
    {
        return &to_frontend_queue;
    }

    SynchronizedQueue<std::unique_ptr<BaseMessage>> event_queue;
    SynchronizedQueue<std::unique_ptr<Command>>     to_frontend_queue;
    std::vector<std::string>                        processed_events;

    const BaseMessage* rec_event{nullptr};
    const Command*     rec_command{nullptr};
};


} // namespace sensei
#endif //SENSEI_TEST_UTILS_H
