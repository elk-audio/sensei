/**
 * @brief Base class for commands
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Base command class and macros for quick subclasses definition.
 * This is intended for internal module use, if you need to define special command sub-classes do it so in
 * message/command_defs.h
 *
 */

#ifndef SENSEI_BASE_COMMAND_H
#define SENSEI_BASE_COMMAND_H

#include <type_traits>

#include "message/base_message.h"

namespace sensei {

class MessageFactory;

enum class CommandType;

/**
 * @brief Target addresses for commands
 *
 * Use operator overloading and typetraits to emulate old-style C flags composition
 * within a type-safe enum
 */
enum class CommandDestination : int
{
    INTERNAL = 1 << 0,
    SERIAL_FRONTEND = 1 << 1,
    OUTPUT_BACKEND = 1 << 2,
    CONFIG_BACKEND = 1 << 3
};

using CommandDestinationType = std::underlying_type_t <CommandDestination>;

inline CommandDestination operator | (CommandDestination lhs, CommandDestination rhs)
{
    return static_cast<CommandDestination>(
            static_cast<CommandDestinationType>(lhs) | static_cast<CommandDestinationType>(rhs)
    );
}

inline CommandDestination& operator |= (CommandDestination& lhs, CommandDestination rhs)
{
    lhs = static_cast<CommandDestination>(static_cast<CommandDestinationType>(lhs) | static_cast<CommandDestinationType>(rhs));
    return lhs;
}

inline bool operator & (CommandDestination lhs, CommandDestination rhs)
{
    return static_cast<bool>(static_cast<CommandDestinationType>(lhs) & static_cast<CommandDestinationType>(rhs));
}


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

    /**
     * @brief Tag type for RTTI-like dynamic dispatch.
     *        Tag definitions are defined in messages/command_defs.h
     */
    CommandType type() const
    {
        return _type;
    }

    /**
     * @brief Identify the destination module of this command
     *
     */
    CommandDestination destination() const
    {
        return _destination;
    }

    /**
     * @brief Unique identifier for command message
     *
     * @returns 64-bit unsigned integer constructed from sensor index, tag and timestamp
     */
    uint64_t uuid() const
    {
        uint64_t result = 0u;
        result |= static_cast<uint64_t>(_index) << 48;
        result |= static_cast<uint64_t>(_type) << 32;
        result |= static_cast<uint64_t>(_timestamp);
        return result;
    }

protected:
    Command(const int sensor_index,
            const CommandType type,
            const CommandDestination destination,
            const uint32_t timestamp=0) :
                BaseMessage(sensor_index, timestamp, MessageType::COMMAND),
                _type(type),
                _destination(destination)
    {
    }

    CommandType _type;
    CommandDestination _destination;
};

//////////////////////////////////////////////////////////////////////////////////
// Concrete class definition macros
//////////////////////////////////////////////////////////////////////////////////

#define SENSEI_DECLARE_COMMAND(ClassName, command_type, InternalType, representation_prefix, destination ) \
class ClassName : public Command \
{ \
public: \
    SENSEI_MESSAGE_CONCRETE_CLASS_PREAMBLE(ClassName) \
    std::string representation() const override \
    {\
        return std::string(representation_prefix);\
    }\
    InternalType data() const\
    {\
        return _data;\
    }\
private:\
    ClassName(const int sensor_index,\
              const InternalType data,\
              const uint32_t timestamp=0) :\
        Command(sensor_index, command_type, destination, timestamp),\
        _data(data)\
    {\
    }\
    InternalType _data;\
}

}; // namespace sensei

#endif //SENSEI_BASE_COMMAND_H
