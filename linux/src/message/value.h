/**
 * @brief Value messages definition
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Classes for messages containing values received from board sensors
 * or obtained as result of transformations in processors.
 *
 * Don't instatiate directly objects, use factory methods provided in
 * message/message_factory.h::MessageFactory
 */

#ifndef SENSEI_VALUE_H
#define SENSEI_VALUE_H

#include <memory>

#include "message/base_message.h"

namespace sensei {

    /**
     * @brief Abstract base class for values.
     */
class SensorValue : public BaseMessage
{

public:
    SensorValue(const int sensor_index, const uint32_t timestamp=0) :
        BaseMessage(sensor_index, timestamp)
    {
    }

    virtual ~SensorValue()
    {
    }

    bool    is_value() override
    {
        return true;
    }

};

/**
 * @brief Internal abstraction for values sampled from an analog pin
 */
class AnalogValue : public SensorValue
{

public:
    AnalogValue(const int sensor_index, const int value, const uint32_t timestamp=0) :
        SensorValue(sensor_index, timestamp),
        _value(value)
    {
    }

    ~AnalogValue()
    {
    }

    int value()
    {
        return _value;
    }

    std::string representation() override
    {
        return std::string("Analog Value");
    }

private:
    int _value;

};

/**
 * @brief Internal abstractions for values read from a digital pin
 */
class DigitalValue : public SensorValue
{

public:
    DigitalValue(const int sensor_index, const bool value, const uint32_t timestamp=0) :
        SensorValue(sensor_index, timestamp),
        _value(value)
    {
    }

    ~DigitalValue()
    {
    }

    bool value()
    {
        return _value;
    }

    std::string representation() override
    {
        return std::string("Digital Value");
    }

private:
    bool _value;

};

/**
 * @brief Internal abstraction for values sampled from an analog pin
 */
class OutputValue : public SensorValue
{

public:
    OutputValue(const int sensor_index, const float value, const uint32_t timestamp=0) :
        SensorValue(sensor_index, timestamp),
        _value(value)
    {
    }

    ~OutputValue()
    {
    }

    float value()
    {
        return _value;
    }

    std::string representation() override
    {
        return std::string("Output Value");
    }

private:
    float _value;

};

// TODO: IMUValue

}; // namespace sensei

#endif // SENSEI_VALUE_H
