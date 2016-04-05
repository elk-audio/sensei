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

TEST(MessagesTest, test_external_command_creation)
{
    MessageFactory factory;

    // Test that commands have the right tag attached, declared as external
    // and type of data()
    // This also illustrates how to parse commands from queue
    // using internal tag for dispatching

    std::vector<std::unique_ptr<BaseMessage>> msg_queue;

    // Fill message queue with all types of commands
    msg_queue.push_back(factory.make_set_sampling_rate_command(1, 1000.0f));
    msg_queue.push_back(factory.make_set_pin_type_command(2, PinType::ANALOG_INPUT));
    msg_queue.push_back(factory.make_set_sending_mode_command(3, SendingMode::ON_VALUE_CHANGED));
    msg_queue.push_back(factory.make_set_sending_delta_ticks_command(4, 10));
    msg_queue.push_back(factory.make_set_adc_bit_resolution_command(5, 12));
    msg_queue.push_back(factory.make_set_lowpass_filter_order_command(6, 4));
    msg_queue.push_back(factory.make_set_lowpass_cutoff_command(7, 125.0f));
    msg_queue.push_back(factory.make_set_slider_threshold_command(8, 9));
    msg_queue.push_back(factory.make_send_digital_value_command(9, true));

    // Parse messages in queue
    for (auto const& msg : msg_queue)
    {
        ASSERT_TRUE(msg->is_cmd());
        ASSERT_FALSE(msg->is_value());
        auto cmd_msg = static_cast<Command*>(msg.get());
        ASSERT_TRUE(cmd_msg->is_external());

        CommandTag tag = cmd_msg->tag();
        switch(tag)
        {

        case CommandTag::SET_SAMPLING_RATE:
            {
                auto typed_cmd = static_cast<SetSamplingRateCommand *>(cmd_msg);
                ASSERT_EQ(1000.0f, typed_cmd->data());
            };
            break;

        case CommandTag::SET_PIN_TYPE:
            {
                auto typed_cmd = static_cast<SetPinTypeCommand *>(cmd_msg);
                ASSERT_EQ(PinType::ANALOG_INPUT, typed_cmd->data());
            };
            break;

        case CommandTag::SET_SENDING_MODE:
            {
                auto typed_cmd = static_cast<SetSendingModeCommand *>(cmd_msg);
                ASSERT_EQ(SendingMode::ON_VALUE_CHANGED, typed_cmd->data());
            };
            break;

        case CommandTag::SET_SENDING_DELTA_TICKS:
            {
                auto typed_cmd = static_cast<SetSendingDeltaTicksCommand *>(cmd_msg);
                ASSERT_EQ(10, typed_cmd->data());
            };
            break;

        case CommandTag::SET_ADC_BIT_RESOLUTION:
            {
                auto typed_cmd = static_cast<SetADCBitResolutionCommand *>(cmd_msg);
                ASSERT_EQ(12, typed_cmd->data());
            };
            break;

        case CommandTag::SET_LOWPASS_FILTER_ORDER:
            {
                auto typed_cmd = static_cast<SetLowpassFilterOrderCommand *>(cmd_msg);
                ASSERT_EQ(4, typed_cmd->data());
            };
                break;

        case CommandTag::SET_LOWPASS_CUTOFF:
            {
                auto typed_cmd = static_cast<SetLowpassCutoffCommand *>(cmd_msg);
                ASSERT_EQ(125.0f, typed_cmd->data());
            };
            break;

        case CommandTag::SET_SLIDER_THRESHOLD:
            {
                auto typed_cmd = static_cast<SetSliderThresholdCommand *>(cmd_msg);
                ASSERT_EQ(9, typed_cmd->data());
            };
            break;

        case CommandTag::SEND_DIGITAL_PIN_VALUE:
            {
                auto typed_cmd = static_cast<SendDigitalPinValueCommand *>(cmd_msg);
                ASSERT_EQ(true, typed_cmd->data());
            };
            break;

        default:
            ASSERT_TRUE(false);
            break;

        }

    }

}
