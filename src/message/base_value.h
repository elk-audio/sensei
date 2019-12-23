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
 * @brief Value messages definition
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * Base Value class and macros for quick subclasses definition.
 * This is intended for internal module use, if you need to define special command sub-classes do it so in
 * message/value_defs.h
 */

#ifndef SENSEI_BASE_VALUE_H
#define SENSEI_BASE_VALUE_H

#include "message/base_message.h"

namespace sensei {

class MessageFactory;

enum class ValueType;

/**
 * @brief Abstract base class for values.
 */
class Value : public BaseMessage
{
public:
    virtual ~Value()
    {
    }

    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(Value)

    /**
     * @brief Tag type for RTTI-like dynamic dispatch.
     *        Tag definitions are defined in messages/value_defs.h
     */
    ValueType type() const
    {
        return _type;
    }

protected:
    Value(const int sensor_index,
          const ValueType type,
          const uint32_t timestamp=0) :
        BaseMessage(sensor_index, timestamp, MessageType::VALUE),
        _type(type)
    {
    }

    ValueType _type;
};

#define SENSEI_DECLARE_VALUE(ClassName, value_type, InternalType, representation_prefix) \
class ClassName : public Value \
{ \
public: \
    SENSEI_MESSAGE_CONCRETE_CLASS_PREAMBLE(ClassName) \
    std::string representation() const override \
    {\
        return std::string(representation_prefix);\
    }\
    InternalType value() const\
    {\
        return _value;\
    }\
private:\
    ClassName(const int sensor_index,\
              const InternalType value,\
              const uint32_t timestamp=0) :\
        Value(sensor_index, value_type, timestamp),\
        _value(value)\
    {\
    }\
    InternalType _value;\
}

}; // namespace sensei

#endif // SENSEI_BASE_VALUE_H