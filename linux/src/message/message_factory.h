/**
 * @brief Factory class for controlled message instatiation.
 * @copyright MIND Music Labs AB, Stockholm
 *
 * The factory defined here provides the suggested method to instatiate messages
 * of different types.
 */

#ifndef SENSEI_MESSAGE_FACTORY_H
#define SENSEI_MESSAGE_FACTORY_H

#include <memory>

#include "message/value.h"

namespace sensei {

// TODO:
//      add constructor parameters for e.g. maximum number of sensors, maximum analog value, etc.

    /**
     * @brief Message factory implemented with Singleton pattern.
     *
     * The provided methods always return unique pointers in the form of
     *      std::unique_ptr<BaseMessage>
     * nullptr is returned if creation fails.
     *
     * Safe for usage from different thread contexts.
     */
class MessageFactory {

public:

    MessageFactory()
    {
    }

    ~MessageFactory()
    {
    }

    std::unique_ptr<BaseMessage> make_analog_value(const int sensor_index,
                                                          const int value,
                                                          const uint32_t timestamp = 0)
    {
        return std::make_unique<AnalogValue>(sensor_index, value, timestamp);
    }

    std::unique_ptr<BaseMessage> make_digital_value(const int sensor_index,
                                                           const bool value,
                                                           const uint32_t timestamp = 0)
    {
        return std::make_unique<DigitalValue>(sensor_index, value, timestamp);
    }

    std::unique_ptr<BaseMessage> make_output_value(const int sensor_index,
                                                          const float value,
                                                          const uint32_t timestamp = 0)
    {
        return std::make_unique<OutputValue>(sensor_index, value, timestamp);
    }

};

}; // namespace sensei

#endif //SENSEI_MESSAGE_FACTORY_H
