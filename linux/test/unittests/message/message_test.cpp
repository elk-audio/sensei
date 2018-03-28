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
    ASSERT_EQ(1, analog_msg->index());
    ASSERT_EQ(10, analog_msg->value());
    ASSERT_EQ(100u, analog_msg->timestamp());

    tmp_msg = factory.make_digital_value(1, true, 100);
    ASSERT_EQ(MessageType::VALUE, tmp_msg->base_type());
    auto digital_msg = static_cast<DigitalValue*>(tmp_msg.get());
    ASSERT_EQ(ValueType::DIGITAL, digital_msg->type());
    ASSERT_EQ(1, digital_msg->index());
    ASSERT_EQ(true, digital_msg->value());
    ASSERT_EQ(100u, digital_msg->timestamp());

    tmp_msg = factory.make_output_value(1, -0.1f, 100);
    ASSERT_EQ(MessageType::VALUE, tmp_msg->base_type());
    auto output_msg = static_cast<OutputValue*>(tmp_msg.get());
    ASSERT_EQ(ValueType::OUTPUT, output_msg->type());
    ASSERT_EQ(1, output_msg->index());
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
    msg_queue.push_back(factory.make_set_enabled_command(2, false));
    msg_queue.push_back(factory.make_set_sensor_type_command(3, SensorType::ANALOG_INPUT));
    msg_queue.push_back(factory.make_set_sending_mode_command(4, SendingMode::ON_VALUE_CHANGED));
    msg_queue.push_back(factory.make_set_sending_delta_ticks_command(5, 10));
    msg_queue.push_back(factory.make_set_adc_bit_resolution_command(6, 12));
    msg_queue.push_back(factory.make_set_lowpass_cutoff_command(7, 125.0f));
    msg_queue.push_back(factory.make_set_slider_threshold_command(9, 9));
    msg_queue.push_back(factory.make_send_digital_value_command(10, true));
    msg_queue.push_back(factory.make_enable_sending_packets_command(0, true));


    // Parse messages in queue
    for (auto const& msg : msg_queue)
    {
        ASSERT_EQ(MessageType::COMMAND, msg->base_type());
        auto cmd_msg = static_cast<Command*>(msg.get());
        ASSERT_TRUE(cmd_msg->destination() & CommandDestination::HARDWARE_FRONTEND);

        CommandType cmd_type = cmd_msg->type();
        switch(cmd_type)
        {
        case CommandType::SET_ENABLED:
            {
                auto typed_cmd = static_cast<SetEnabledCommand *>(cmd_msg);
                ASSERT_FALSE(typed_cmd->data());
            };
            break;

        case CommandType::SET_SENSOR_TYPE:
            {
                auto typed_cmd = static_cast<SetSensorTypeCommand *>(cmd_msg);
                ASSERT_EQ(SensorType::ANALOG_INPUT, typed_cmd->data());
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

        case CommandType::ENABLE_SENDING_PACKETS:
            {
                auto typed_cmd = static_cast<EnableSendingPacketsCommand *>(cmd_msg);
                ASSERT_EQ(true, typed_cmd->data());
            };
            break;

        default:
            FAIL();
        }

    }

}

TEST(MessagesTest, test_internal_command_creation)
{
    MessageFactory factory;

    std::vector<std::unique_ptr<BaseMessage>> msg_queue;

    // Fill message queue with all types of commands
    msg_queue.push_back(factory.make_set_invert_enabled_command(1, true));
    msg_queue.push_back(factory.make_set_input_scale_range_low_command(2, 20));
    msg_queue.push_back(factory.make_set_input_scale_range_high_command(3, 200));

    // Parse messages in queue
    for (auto const& msg : msg_queue)
    {

        ASSERT_EQ(MessageType::COMMAND, msg->base_type());
        auto cmd_msg = static_cast<Command*>(msg.get());
        ASSERT_TRUE(cmd_msg->destination() & CommandDestination::INTERNAL);

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
            FAIL();
        }

    }

}


TEST(MessagesTest, test_imu_specific_command_creation)
{
    MessageFactory factory;

    // Test that commands have the right cmd_type attached, declared as external
    // and type of data()
    // This also illustrates how to parse commands from queue
    // using internal cmd_type for dispatching

    std::vector<std::unique_ptr<BaseMessage>> msg_queue;

    // Fill message queue with all types of commands
    msg_queue.push_back(factory.make_enable_imu_command(true));
    msg_queue.push_back(factory.make_imu_set_filter_mode_command(1));
    msg_queue.push_back(factory.make_imu_set_acc_range_max_command(2));
    msg_queue.push_back(factory.make_imu_set_gyro_range_max_command(2));
    msg_queue.push_back(factory.make_imu_set_compass_range_max_command(3.5));
    msg_queue.push_back(factory.make_imu_enable_compass_command(true));
    msg_queue.push_back(factory.make_imu_set_sending_mode_command(SendingMode::CONTINUOUS));
    msg_queue.push_back(factory.make_imu_sending_delta_ticks_command(5));
    msg_queue.push_back(factory.make_imu_set_data_mode_command(2));
    msg_queue.push_back(factory.make_imu_acc_threshold_command(0.2));
    msg_queue.push_back(factory.make_imu_calibrate_command());
    msg_queue.push_back(factory.make_imu_factory_reset_command());
    msg_queue.push_back(factory.make_imu_reboot_command());
    msg_queue.push_back(factory.make_imu_get_temperature_command());
    msg_queue.push_back(factory.make_imu_commit_settings_command());


    // Parse messages in queue
    for (auto const& msg : msg_queue)
    {
        ASSERT_EQ(MessageType::COMMAND, msg->base_type());
        auto cmd_msg = static_cast<Command*>(msg.get());
        ASSERT_TRUE(cmd_msg->destination()& CommandDestination::HARDWARE_FRONTEND);
        CommandType cmd_type = cmd_msg->type();
        switch(cmd_type)
        {
        case CommandType::SET_IMU_ENABLED:
            {
                auto typed_cmd = static_cast<SetImuEnabledCommand*>(cmd_msg);
                ASSERT_TRUE(typed_cmd->data());
            };
            break;
        case CommandType::SET_IMU_FILTER_MODE:
            {
                auto typed_cmd = static_cast<SetImuFilterModeCommand*>(cmd_msg);
                ASSERT_EQ(1, typed_cmd->data());
            };
            break;

        case CommandType::SET_IMU_ACC_RANGE_MAX:
            {
                auto typed_cmd = static_cast<SetImuAccelerometerRangeMaxCommand*>(cmd_msg);
                ASSERT_EQ(2, typed_cmd->data());
            };
            break;

        case CommandType::SET_IMU_GYRO_RANGE_MAX:
            {
                auto typed_cmd = static_cast<SetImuGyroscopeRangeMaxCommand*>(cmd_msg);
                ASSERT_EQ(2, typed_cmd->data());
            };
            break;

        case CommandType::SET_IMU_COMPASS_RANGE_MAX:
            {
                auto typed_cmd = static_cast<SetImuCompassRangeMaxCommand*>(cmd_msg);
                ASSERT_FLOAT_EQ(3.5, typed_cmd->data());
            };
            break;

        case CommandType::SET_IMU_COMPASS_ENABLED:
            {
                auto typed_cmd = static_cast<SetImuCompassEnabledCommand*>(cmd_msg);
                ASSERT_TRUE(typed_cmd->data());
            };
            break;

        case CommandType::SET_IMU_SENDING_MODE:
            {
                auto typed_cmd = static_cast<SetImuSendingModeCommand*>(cmd_msg);
                ASSERT_EQ(SendingMode::CONTINUOUS, typed_cmd->data());
            };
            break;

        case CommandType::SET_IMU_SENDING_DELTA_TICKS:
            {
                auto typed_cmd = static_cast<SetImuSendingDeltaTicksCommand*>(cmd_msg);
                ASSERT_EQ(5, typed_cmd->data());
            };
            break;

        case CommandType::SET_IMU_DATA_MODE:
            {
                auto typed_cmd = static_cast<SetImuDataModeCommand*>(cmd_msg);
                ASSERT_EQ(2, typed_cmd->data());
            };
            break;

        case CommandType::SET_IMU_ACC_THRESHOLD:
            {
                auto typed_cmd = static_cast<SetImuAccThresholdCommand*>(cmd_msg);
                ASSERT_FLOAT_EQ(0.2, typed_cmd->data());
            };
            break;

        case CommandType::IMU_CALIBRATE:
            {
                auto typed_cmd = static_cast<ImuCalibrateCommand*>(cmd_msg);
                ASSERT_FALSE(typed_cmd->data());
            };
            break;

        case CommandType::IMU_FACTORY_RESET:
            {
                auto typed_cmd = static_cast<ImuFactoryResetCommand*>(cmd_msg);
                ASSERT_FALSE(typed_cmd->data());
            };
            break;

        case CommandType::IMU_REBOOT:
            {
                auto typed_cmd = static_cast<ImuRebootCommand*>(cmd_msg);
                ASSERT_FALSE(typed_cmd->data());
            };
            break;

        case CommandType::IMU_GET_TEMPERATURE:
            {
                auto typed_cmd = static_cast<ImuGetTemperatureCommand*>(cmd_msg);
                ASSERT_FALSE(typed_cmd->data());
            };
            break;

        case CommandType::IMU_COMMIT_SETTINGS:
            {
                auto typed_cmd = static_cast<ImuCommitSettingsCommand*>(cmd_msg);
                ASSERT_FALSE(typed_cmd->data());
            };
            break;
        default:
            FAIL();

        }
    }
}


TEST(MessagesTest, test_output_backend_command_creation)
{
    MessageFactory factory;

    std::vector<std::unique_ptr<BaseMessage>> msg_queue;

    msg_queue.push_back(factory.make_set_backend_type_command(0, BackendType::OSC));
    msg_queue.push_back(factory.make_set_sensor_name_command(0, std::string("pippo")));
    msg_queue.push_back(factory.make_set_send_output_enabled_command(0, false));
    msg_queue.push_back(factory.make_set_send_raw_input_enabled_command(0, true));
    msg_queue.push_back(factory.make_set_osc_output_base_path_command(0, "/sensors"));
    msg_queue.push_back(factory.make_set_osc_output_raw_path_command(0, "/raw_input"));
    msg_queue.push_back(factory.make_set_osc_output_host_command(0, "192.168.1.100"));
    msg_queue.push_back(factory.make_set_osc_output_port_command(0, 9999));


    for (auto const& msg : msg_queue)
    {

        ASSERT_EQ(MessageType::COMMAND, msg->base_type());
        auto cmd_msg = static_cast<Command*>(msg.get());
        ASSERT_TRUE(cmd_msg->destination() & CommandDestination::OUTPUT_BACKEND);

        CommandType cmd_type = cmd_msg->type();
        switch(cmd_type)
        {

        case CommandType::SET_BACKEND_TYPE:
            {
                auto typed_cmd = static_cast<SetBackendTypeCommand *>(cmd_msg);
                ASSERT_EQ(BackendType::OSC, typed_cmd->data());
            };
            break;

        case CommandType::SET_SENSOR_NAME:
            {
                auto typed_cmd = static_cast<SetPinNameCommand *>(cmd_msg);
                ASSERT_EQ(std::string("pippo"), typed_cmd->data());
            };
            break;

        case CommandType::SET_SEND_OUTPUT_ENABLED:
            {
                auto typed_cmd = static_cast<SetSendOutputEnabledCommand *>(cmd_msg);
                ASSERT_FALSE(typed_cmd->data());
            };
            break;

        case CommandType::SET_SEND_RAW_INPUT_ENABLED:
            {
                auto typed_cmd = static_cast<SetSendRawInputEnabledCommand *>(cmd_msg);
                ASSERT_TRUE(typed_cmd->data());
            };
            break;

        case CommandType::SET_OSC_OUTPUT_BASE_PATH:
            {
                auto typed_cmd = static_cast<SetOSCOutputBasePathCommand *>(cmd_msg);
                ASSERT_EQ("/sensors", typed_cmd->data());
            };
            break;

        case CommandType::SET_OSC_OUTPUT_RAW_PATH:
            {
                auto typed_cmd = static_cast<SetOSCOutputRawPathCommand *>(cmd_msg);
                ASSERT_EQ("/raw_input", typed_cmd->data());
            };
            break;

        case CommandType::SET_OSC_OUTPUT_HOST:
            {
                auto typed_cmd = static_cast<SetOSCOutputHostCommand *>(cmd_msg);
                ASSERT_EQ("192.168.1.100", typed_cmd->data());
            };
            break;

        case CommandType::SET_OSC_OUTPUT_PORT:
            {
                auto typed_cmd = static_cast<SetOSCOutputPortCommand *>(cmd_msg);
                ASSERT_EQ(9999, typed_cmd->data());
            };
            break;

        default:
            FAIL();
        }
    }

}

TEST(MessagesTest, test_user_frontend_message_creation)
{
    MessageFactory factory;

    std::vector<std::unique_ptr<BaseMessage>> msg_queue;

    msg_queue.push_back(factory.make_set_osc_input_port_command(0, 9999));

    for (auto const& msg : msg_queue)
    {

        ASSERT_EQ(MessageType::COMMAND, msg->base_type());
        auto cmd_msg = static_cast<Command*>(msg.get());
        ASSERT_TRUE(cmd_msg->destination() & CommandDestination::USER_FRONTEND);

        CommandType cmd_type = cmd_msg->type();
        switch(cmd_type)
        {
        case CommandType::SET_OSC_INPUT_PORT:
            {
                auto typed_cmd = static_cast<SetOSCInputPortCommand *>(cmd_msg);
                ASSERT_EQ(9999, typed_cmd->data());
            };
            break;

        default:
            FAIL();
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
