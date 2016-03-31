/**
 * @brief Value messages definition
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Classes for messages containing values received from board sensors
 * or obtained as result of transformations in processors.
 *
 * Don't instantiate directly objects, use factory methods provided in
 * message/message_factory.h::MessageFactory
 */

#ifndef SENSEI_VALUE_H
#define SENSEI_VALUE_H

#include <memory>

#include "message/base_message.h"

namespace sensei {

class MessageFactory;

/**
 * @brief Abstract base class for values.
 */
class SensorValue : public BaseMessage
{
public:
    virtual ~SensorValue()
    {
    }

    SensorValue(const SensorValue&) = delete;

    void operator=(const SensorValue &x) = delete;


    bool    is_value() override
    {
        return true;
    }

protected:
    SensorValue(const int sensor_index, const uint32_t timestamp=0) :
            BaseMessage(sensor_index, timestamp)
    {
    }

};

/**
 * @brief Internal abstraction for values sampled from an analog pin
 */
class AnalogValue : public SensorValue
{
    friend class MessageFactory;

public:

    ~AnalogValue()
    {
    }

    AnalogValue(const AnalogValue&) = delete;

    void operator=(const AnalogValue &x) = delete;

    int value()
    {
        return _value;
    }

    std::string representation() override
    {
        return std::string("Analog Value");
    }

private:
    AnalogValue(const int sensor_index, const int value, const uint32_t timestamp=0) :
            SensorValue(sensor_index, timestamp),
            _value(value)
    {
    }

    int _value;

};

/**
 * @brief Internal abstractions for values read from a digital pin
 */
class DigitalValue : public SensorValue
{
    friend class MessageFactory;

public:

    ~DigitalValue()
    {
    }

    DigitalValue(const DigitalValue&) = delete;

    void operator=(const DigitalValue &x) = delete;

    bool value()
    {
        return _value;
    }

    std::string representation() override
    {
        return std::string("Digital Value");
    }

private:

    DigitalValue(const int sensor_index, const bool value, const uint32_t timestamp=0) :
            SensorValue(sensor_index, timestamp),
            _value(value)
    {
    }

    bool _value;

};

/**
 * @brief Internal abstraction for values sampled from an analog pin
 */
class OutputValue : public SensorValue
{
public:
    friend class MessageFactory;

    ~OutputValue()
    {
    }

    OutputValue(const OutputValue&) = delete;

    void operator=(const OutputValue &x) = delete;

    float value()
    {
        return _value;
    }

    std::string representation() override
    {
        return std::string("Output Value");
    }

private:

    OutputValue(const int sensor_index, const float value, const uint32_t timestamp=0) :
            SensorValue(sensor_index, timestamp),
            _value(value)
    {
    }

    float _value;

};

// TODO: IMUValue

}; // namespace sensei

#endif // SENSEI_VALUE_H
