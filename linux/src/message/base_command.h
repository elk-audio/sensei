/**
 * @brief Command messages definition
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Base command class and macros for quick subclasses definition.
 * This is intended for internal module use, if you need to define special command sub-classes do it so in
 * message/command_defs.h
 *
 */

#ifndef SENSEI_BASE_COMMAND_H
#define SENSEI_BASE_COMMAND_H

#include "message/base_message.h"

namespace sensei {

class MessageFactory;

enum class CommandType;

/**
 * @brief Abstract base class for commands.
 */
class Command : public BaseMessage
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(Command)

    virtual ~Command()
    {
    }

    bool    is_cmd() override
    {
        return true;
    }

    /**
     * @brief Used to dispatch external command to the serial frontend.
     *        Derived classes may override the value in their constructor.
     */
    bool is_external()
    {
        return _is_external;
    }

    /**
     * @brief Tag type for RTTI-like dynamic dispatch.
     *        Tag definitions are defined in messages/command_defs.h
     */
    CommandType type()
    {
        return _type;
    }

    /**
     * @brief Unique identifier for command message
     *
     * @returns 64-bit unsigned integer constructed from sensor index, tag and timestamp
     */
    uint64_t uuid()
    {
        uint64_t result = 0u;
        result |= static_cast<uint64_t>(_sensor_index) << 48;
        result |= static_cast<uint64_t>(_type) << 32;
        result |= static_cast<uint64_t>(_timestamp);
        return result;
    }

protected:
    Command(const int sensor_index,
            bool is_external,
            CommandType type,
            const uint32_t timestamp=0) :
                BaseMessage(sensor_index, timestamp),
                _is_external(is_external),
                _type(type)
    {
    }

    bool    _is_external;
    CommandType _type;
};

//////////////////////////////////////////////////////////////////////////////////
// Concrete command classes
//////////////////////////////////////////////////////////////////////////////////

#define __SENSEI_DECLARE_COMMAND(ClassName, command_type, InternalType, representation_prefix, is_external) \
class ClassName : public Command \
{ \
public: \
    SENSEI_MESSAGE_CONCRETE_CLASS_PREAMBLE(ClassName) \
    std::string representation() override \
    {\
        return std::string(representation_prefix);\
    }\
    InternalType data()\
    {\
        return _data;\
    }\
private:\
    ClassName(const int sensor_index,\
              const InternalType data,\
              const uint32_t timestamp=0) :\
        Command(sensor_index, is_external, command_type, timestamp),\
        _data(data)\
    {\
    }\
    InternalType _data;\
}

#define SENSEI_DECLARE_EXTERNAL_COMMAND(ClassName, command_type, InternalType, representation_prefix) \
    __SENSEI_DECLARE_COMMAND(ClassName, command_type, InternalType, representation_prefix, true)

#define SENSEI_DECLARE_INTERNAL_COMMAND(ClassName, command_type, InternalType, representation_prefix) \
    __SENSEI_DECLARE_COMMAND(ClassName, command_type, InternalType, representation_prefix, false)

}; // namespace sensei

#endif //SENSEI_BASE_COMMAND_H
