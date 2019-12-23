/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Base class for errors
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * Base error class and macros for quick subclasses definition.
 * This is intended for internal module use, if you need to define special command sub-classes do it so in
 * message/error_defs.h
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