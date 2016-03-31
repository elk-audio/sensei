/**
 * @brief Internal Message base class
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Abstract base class for internal messages exchanged among communication queues.
 *
 */

#ifndef SENSEI_MESSAGE_H
#define SENSEI_MESSAGE_H

#include <cstdint>
#include <memory>

namespace sensei {

    /**
     * @brief Abstract base class for internal messages.
     *
     * Used mostly for polymorphic containers of different type of messages.
     * Pure virtual, not meant for direct instatiation.
     */
class BaseMessage
{

public:

    /**
     * @brief Base constructor
     *
     * @param sensor_index Index of sensor pin in the board
     * @param timestamp Timestamp of creation in microseconds from the beginning
     */
    BaseMessage(const int sensor_index, const unsigned int timestamp=0) :
            _sensor_index(sensor_index),
            _timestamp(timestamp)
    {
    }

    virtual ~BaseMessage()
    {
    }

    int sensor_index()
    {   
        return _sensor_index;
    }

    uint32_t timestamp()
    {
        return _timestamp;
    }

    /**
     * @brief Message representation
     *
     * @return Human readable string with message contents.
     */
    virtual std::string representation() = 0;

    /**
     * @brief Check if message is of SensorValue base type. Used for RTTI-like emulation.
     */
    virtual bool is_value()
    {
        return false;
    }

    /**
     * @brief Check if message is of Command base type. Used for RTTI-like emulation.
     */
    virtual bool is_cmd()
    {
        return false;
    }

private:

    int _sensor_index;
    uint32_t _timestamp;

};

}; // namespace sensei

#endif // SENSEI_MESSAGE_H
