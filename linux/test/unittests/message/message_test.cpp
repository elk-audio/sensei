#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "message/message_factory.h"

using namespace sensei;

TEST(MessagesTest, test_value_creation)
{
    MessageFactory factory;

    auto tmp_msg = factory.make_analog_value(1, 10, 100);
    ASSERT_EQ(MessageType::VALUE, tmp_msg->base_type());
    auto analog_msg = static_cast<AnalogValue*>(tmp_msg.get());
    ASSERT_EQ(ValueType::ANALOG, analog_msg->type());
    ASSERT_EQ(1, analog_msg->sensor_index());
    ASSERT_EQ(10, analog_msg->value());
    ASSERT_EQ(100u, analog_msg->timestamp());

    tmp_msg = factory.make_digital_value(1, true, 100);
    ASSERT_EQ(MessageType::VALUE, tmp_msg->base_type());
    auto digital_msg = static_cast<DigitalValue*>(tmp_msg.get());
    ASSERT_EQ(ValueType::DIGITAL, digital_msg->type());
    ASSERT_EQ(1, digital_msg->sensor_index());
    ASSERT_EQ(true, digital_msg->value());
    ASSERT_EQ(100u, digital_msg->timestamp());

    tmp_msg = factory.make_output_value(1, -0.1f, 100);
    ASSERT_EQ(MessageType::VALUE, tmp_msg->base_type());
    auto output_msg = static_cast<OutputValue*>(tmp_msg.get());
    ASSERT_EQ(ValueType::OUTPUT, output_msg->type());
    ASSERT_EQ(1, output_msg->sensor_index());
    ASSERT_EQ(-0.1f, output_msg->value());
    ASSERT_EQ(100u, output_msg->timestamp());
}

TEST(MessagesTest, test_external_command_creation)
{
    MessageFactory factory;

    // Test that commands have the right cmd_type attached, declared as external
    // and type of data()
    // This also illustrates how to parse commands from queue
    // using internal cmd_type for dispatching

    std::vector<std::unique_ptr<BaseMessage>> msg_queue;

    // Fill message queue with all types of commands
    msg_queue.push_back(factory.make_set_sampling_rate_command(1, 1000.0f));
    msg_queue.push_back(factory.make_set_enabled_command(2, false));
    msg_queue.push_back(factory.make_set_pin_type_command(3, PinType::ANALOG_INPUT));
    msg_queue.push_back(factory.make_set_sending_mode_command(4, SendingMode::ON_VALUE_CHANGED));
    msg_queue.push_back(factory.make_set_sending_delta_ticks_command(5, 10));
    msg_queue.push_back(factory.make_set_adc_bit_resolution_command(6, 12));
    msg_queue.push_back(factory.make_set_lowpass_cutoff_command(7, 125.0f));
    msg_queue.push_back(factory.make_set_slider_mode_enabled_command(8, true));
    msg_queue.push_back(factory.make_set_slider_threshold_command(9, 9));
    msg_queue.push_back(factory.make_send_digital_value_command(10, true));

    // Parse messages in queue
    for (auto const& msg : msg_queue)
    {
        ASSERT_EQ(MessageType::COMMAND, msg->base_type());
        auto cmd_msg = static_cast<Command*>(msg.get());
        ASSERT_EQ(CommandDestination::SERIAL_FRONTEND, cmd_msg->destination());

        CommandType cmd_type = cmd_msg->type();
        switch(cmd_type)
        {

        case CommandType::SET_SAMPLING_RATE:
            {
                auto typed_cmd = static_cast<SetSamplingRateCommand *>(cmd_msg);
                ASSERT_EQ(1000.0f, typed_cmd->data());
            };
            break;

        case CommandType::SET_ENABLED:
            {
                auto typed_cmd = static_cast<SetEnabledCommand *>(cmd_msg);
                ASSERT_FALSE(typed_cmd->data());
            };
            break;

        case CommandType::SET_PIN_TYPE:
            {
                auto typed_cmd = static_cast<SetPinTypeCommand *>(cmd_msg);
                ASSERT_EQ(PinType::ANALOG_INPUT, typed_cmd->data());
            };
            break;

        case CommandType::SET_SENDING_MODE:
            {
                auto typed_cmd = static_cast<SetSendingModeCommand *>(cmd_msg);
                ASSERT_EQ(SendingMode::ON_VALUE_CHANGED, typed_cmd->data());
            };
            break;

        case CommandType::SET_SENDING_DELTA_TICKS:
            {
                auto typed_cmd = static_cast<SetSendingDeltaTicksCommand *>(cmd_msg);
                ASSERT_EQ(10, typed_cmd->data());
            };
            break;

        case CommandType::SET_ADC_BIT_RESOLUTION:
            {
                auto typed_cmd = static_cast<SetADCBitResolutionCommand *>(cmd_msg);
                ASSERT_EQ(12, typed_cmd->data());
            };
            break;

        case CommandType::SET_LOWPASS_CUTOFF:
            {
                auto typed_cmd = static_cast<SetLowpassCutoffCommand *>(cmd_msg);
                ASSERT_EQ(125.0f, typed_cmd->data());
            };
            break;

        case CommandType::SET_SLIDER_MODE_ENABLED:
            {
                auto typed_cmd = static_cast<SetSliderModeEnabledCommand *>(cmd_msg);
                ASSERT_TRUE(typed_cmd->data());
            };
            break;

        case CommandType::SET_SLIDER_THRESHOLD:
            {
                auto typed_cmd = static_cast<SetSliderThresholdCommand *>(cmd_msg);
                ASSERT_EQ(9, typed_cmd->data());
            };
            break;

        case CommandType::SEND_DIGITAL_PIN_VALUE:
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

TEST(MessagesTest, test_internal_command_creation)
{
    MessageFactory factory;

    std::vector<std::unique_ptr<BaseMessage>> msg_queue;

    // Fill message queue with all types of commands
    msg_queue.push_back(factory.make_set_invert_enabled_command(1, true));
    msg_queue.push_back(factory.make_set_input_scale_range_low(2, 20));
    msg_queue.push_back(factory.make_set_input_scale_range_high(3, 200));

    // Parse messages in queue
    for (auto const& msg : msg_queue)
    {

        ASSERT_EQ(MessageType::COMMAND, msg->base_type());
        auto cmd_msg = static_cast<Command*>(msg.get());
        ASSERT_EQ(CommandDestination::INTERNAL, cmd_msg->destination());

        CommandType cmd_type = cmd_msg->type();
        switch(cmd_type)
        {

        case CommandType::SET_INVERT_ENABLED:
            {
                auto typed_cmd = static_cast<SetInvertEnabledCommand *>(cmd_msg);
                ASSERT_TRUE(typed_cmd->data());
            };
            break;

        case CommandType::SET_INPUT_SCALE_RANGE_LOW:
            {
                auto typed_cmd = static_cast<SetInputScaleRangeLow *>(cmd_msg);
                ASSERT_EQ(20, typed_cmd->data());
            };
            break;

        case CommandType::SET_INPUT_SCALE_RANGE_HIGH:
            {
                auto typed_cmd = static_cast<SetInputScaleRangeHigh *>(cmd_msg);
                ASSERT_EQ(200, typed_cmd->data());
            };
            break;

        default:
            ASSERT_TRUE(false);
            break;

        }

    }

}


TEST(MessagesTest, test_error_creation)
{
    MessageFactory factory;

    auto tmp_msg = factory.make_bad_crc_error(0);
    auto bad_crc_msg = static_cast<BadCrcError*>(tmp_msg.get());
    ASSERT_EQ(ErrorType::BAD_CRC, bad_crc_msg->type() );

    tmp_msg = factory.make_too_many_timeouts_error(0);
    auto timeouts_msg = static_cast<TooManyTimeoutsError*>(tmp_msg.get());
    ASSERT_EQ(ErrorType::TOO_MANY_TIMEOUTS, timeouts_msg->type() );
}
