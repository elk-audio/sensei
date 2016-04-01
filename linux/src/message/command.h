/**
 * @brief Command messages definition
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Classes for messages containing configurations commands to either
 * sensei or sensor board.
 *
 * Don't instantiate these object directly, use factory methods provided in
 * message/message_factory.h::MessageFactory
 */


#ifndef SENSEI_COMMAND_H
#define SENSEI_COMMAND_H

#include <memory>

#include "message/base_message.h"

namespace sensei {

class MessageFactory;

/**
 * @brief Abstract base class for values.
 */
class CommandMessage : public BaseMessage
{
public:
    virtual ~CommandMessage()
    {
    }

    CommandMessage(const CommandMessage&) = delete;

    void operator=(const CommandMessage &x) = delete;


    bool    is_cmd() override
    {
        return true;
    }
};

// TODO: IMUValue

}; // namespace sensei

#endif //SENSEI_COMMAND_H
