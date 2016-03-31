#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "message/message_factory.h"

using namespace sensei;

TEST(MessagesTest, test_value_creation)
{
    MessageFactory factory;

    auto tmp_msg = factory.make_analog_value(1, 10, 100);
    auto analog_msg = static_cast<AnalogValue*>(tmp_msg.get());
    ASSERT_EQ(1, analog_msg->sensor_index());
    ASSERT_EQ(10, analog_msg->value());
    ASSERT_EQ(100u, analog_msg->timestamp());

    tmp_msg = factory.make_digital_value(1, true, 100);
    auto digital_msg = static_cast<DigitalValue*>(tmp_msg.get());
    ASSERT_EQ(1, digital_msg->sensor_index());
    ASSERT_EQ(true, digital_msg->value());
    ASSERT_EQ(100u, digital_msg->timestamp());

    tmp_msg = factory.make_output_value(1, -0.1f, 100);
    auto output_msg = static_cast<OutputValue*>(tmp_msg.get());
    ASSERT_EQ(1, output_msg->sensor_index());
    ASSERT_EQ(-0.1f, output_msg->value());
    ASSERT_EQ(100u, output_msg->timestamp());

}

