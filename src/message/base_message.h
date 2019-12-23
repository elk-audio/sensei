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
 * @brief Internal Message base class
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * Abstract base class for internal messages exchanged among communication queues.
 * All messages are immutable: properties are defined at creation moment using factory methods and never changed.
 */
#ifndef SENSEI_MESSAGE_H
#define SENSEI_MESSAGE_H

#include <cstdint>
#include <memory>

/**
 * @brief Concrete classes tags used for RTTI emulation
 */
enum class MessageType
{
    VALUE,
    COMMAND,
    ERROR
};

/**
 * @brief Convenience macro for declaring non-copyable behaviour
 *        Put it at beginning of public: section.
 */
#define SENSEI_MESSAGE_DECLARE_NON_COPYABLE(ClassName) \
    ClassName (const ClassName&) = delete ;\
    ClassName& operator= (const ClassName&) = delete;

/**
 * @brief Convenience macro for declaring non-copyable behaviour, factory friend and empty destructor.
 *        Put it at beginning of public: section.
 */
#define SENSEI_MESSAGE_CONCRETE_CLASS_PREAMBLE(ClassName) \
    ~ClassName() \
    { \
    } \
    friend class MessageFactory; \
    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(ClassName)

namespace sensei {

/**
 * @brief Abstract base class for internal messages.
 *
 * Used mostly for polymorphic containers of different type of messages.
 * Pure virtual, not meant for direct instantiation.
 */
class BaseMessage
{

public:
    virtual ~BaseMessage()
    {
    }

    SENSEI_MESSAGE_DECLARE_NON_COPYABLE(BaseMessage)

    /**
     * @brief Get integer tag to identify the message in collections
     *
     * @return Index of the sensor
     */
    int index() const
    {   
        return _index;
    }

    /**
     * @brief Get timestamp attached at message creation.
     * For messages representing sensor values coming from the board, this is a number obtained from external board's timer.
     *
     * @return Timestamp in microseconds from start instant
     */
    uint32_t timestamp() const
    {
        return _timestamp;
    }

    /**
     * @brief Message representation
     *
     * @return Human readable string with message contents.
     */
    virtual std::string representation() const = 0;

    /**
     * @brief Return type tag
     */
    MessageType base_type() const
    {
        return _base_type;
    }

protected:
    BaseMessage(const int sensor_index,
                const unsigned int timestamp=0,
                const MessageType msg_type=MessageType::VALUE) :
            _index(sensor_index),
            _timestamp(timestamp),
            _base_type(msg_type)
    {
    }

    int _index;
    uint32_t _timestamp;
    MessageType _base_type;

};

}; // namespace sensei

#endif // SENSEI_MESSAGE_H