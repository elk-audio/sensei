/**
 * @brief Base class for errors
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Base error class and macros for quick subclasses definition.
 * This is intended for internal module use, if you need to define special command sub-classes do it so in
 * message/error_defs.h
 *
 */
#ifndef SENSEI_BASE_ERROR_H
#define SENSEI_BASE_ERROR_H

#include "message/base_message.h"

namespace sensei {

class MessageFactory;

enum class ErrorType;

/**
 * @brief Abstract base class for errors.
 */
class Error : public BaseMessage
{
public:
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(Error)

    virtual ~Error()
    {
    }

    ErrorType type() const
    {
        return _type;
    }

protected:
    Error(const int sensor_index,
          const ErrorType type,
          const uint32_t timestamp = 0) :
            BaseMessage(sensor_index, timestamp, MessageType::ERROR),
            _type(type)
    {
    }

    ErrorType _type;
};

//////////////////////////////////////////////////////////////////////////////////
// Concrete class definition macros
//////////////////////////////////////////////////////////////////////////////////

#define SENSEI_DECLARE_VOID_ERROR(ClassName, error_type, representation_prefix) \
class ClassName : public Error \
{ \
public: \
    SENSEI_MESSAGE_CONCRETE_CLASS_PREAMBLE(ClassName) \
    std::string representation() const override \
    {\
        return std::string(representation_prefix);\
    }\
private:\
    ClassName(const int sensor_index,\
              const uint32_t timestamp=0) :\
        Error(sensor_index, error_type, timestamp)\
    {\
    }\
}

#define SENSEI_DECLARE_ERROR(ClassName, error_type, InternalType, representation_prefix) \
class ClassName : public Error \
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
        Error(sensor_index, error_type, timestamp),\
        _data(data)\
    {\
    }\
    InternalType _data;\
}

}; // namespace sensei

#endif //SENSEI_BASE_ERROR_H
