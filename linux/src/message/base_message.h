/**
 * @brief Internal Message base class
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Abstract base class for internal messages exchanged among communication queues.
 * All messages are immutable: properties are defined at creation moment using factory methods and never changed.
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
 * Pure virtual, not meant for direct instantiation.
 */
class BaseMessage
{

public:
    virtual ~BaseMessage()
    {
    }

    BaseMessage(const BaseMessage&) = delete;
    void operator=(const BaseMessage &x) = delete;

    /**
     * @brief Get integer tag to identify the sensor among the pins available in the board.
     *
     * @return Index of the sensor
     */
    int sensor_index()
    {   
        return _sensor_index;
    }

    /**
     * @brief Get timestamp attached at message creation.
     * For messages representing sensor values coming from the board, this is a number obtained from external board's timer.
     *
     * @return Timestamp in microseconds from start instant
     */
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

protected:
    BaseMessage(const int sensor_index, const unsigned int timestamp=0) :
            _sensor_index(sensor_index),
            _timestamp(timestamp)
    {
    }

private:

    int _sensor_index;
    uint32_t _timestamp;

};

}; // namespace sensei

#endif // SENSEI_MESSAGE_H
