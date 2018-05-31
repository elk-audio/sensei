#include "gtest/gtest.h"

#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "user_frontend/user_frontend.cpp"
#include "user_frontend/osc_user_frontend.cpp"
#pragma GCC diagnostic pop

#include "../test_utils.h"

using namespace sensei;
using namespace sensei::user_frontend;

class TestOSCUserFrontend : public ::testing::Test
{
protected:
    TestOSCUserFrontend()
    {
    }

    ~TestOSCUserFrontend()
    {
    }

    void SetUp()
    {
        _user_frontend.reset(new user_frontend::OSCUserFrontend(&_event_queue, 64, 32));
        MessageFactory factory;

        auto cmd = CMD_UPTR(factory.make_set_osc_input_port_command(0, _server_port));
        auto status = _user_frontend->apply_command(cmd.get());
        ASSERT_EQ(CommandErrorCode::OK, status);

        std::stringstream port_stream;
        port_stream << _server_port;
        auto port_str = port_stream.str();
        _address = lo_address_new("localhost", port_str.c_str());
    }

    void TearDown()
    {
        lo_address_free(_address);
    }

    std::unique_ptr<OSCUserFrontend> _user_frontend;
    SynchronizedQueue<std::unique_ptr<BaseMessage>> _event_queue;
    int _server_port{25000};
    lo_address _address;
};

TEST_F(TestOSCUserFrontend, test_set_pin_enabled)
{
    lo_send(_address, "/set_enabled", "ii", 5, 1);
    _event_queue.wait_for_data(std::chrono::milliseconds(10));
    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    ASSERT_EQ(MessageType::COMMAND, event->base_type());
    auto cmd = static_unique_ptr_cast<SetEnabledCommand, BaseMessage>(std::move(event));

    ASSERT_EQ(5, cmd->index());
    ASSERT_EQ(true, cmd->data());
    ASSERT_TRUE(_event_queue.empty());
}

TEST_F(TestOSCUserFrontend, test_set_digital_output)
{
    lo_send(_address, "/set_digital_output", "ii", 3, 0);
    _event_queue.wait_for_data(std::chrono::milliseconds(10));
    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    ASSERT_EQ(MessageType::VALUE, event->base_type());
    auto val = static_unique_ptr_cast<IntegerSetValue, BaseMessage>(std::move(event));

    ASSERT_EQ(ValueType::INT_SET, val->type());
    ASSERT_EQ(3, val->index());
    ASSERT_EQ(0, val->value());
    ASSERT_TRUE(_event_queue.empty());
}

TEST_F(TestOSCUserFrontend, test_set_continuous_output)
{
    lo_send(_address, "/set_continuous_output", "if", 4, 0.4f);
    _event_queue.wait_for_data(std::chrono::milliseconds(10));
    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    ASSERT_EQ(MessageType::VALUE, event->base_type());
    auto val = static_unique_ptr_cast<FloatSetValue, BaseMessage>(std::move(event));

    ASSERT_EQ(ValueType::FLOAT_SET, val->type());
    ASSERT_EQ(4, val->index());
    ASSERT_FLOAT_EQ(0.4f, val->value());
    ASSERT_TRUE(_event_queue.empty());
}

TEST_F(TestOSCUserFrontend, test_set_range_output)
{
    lo_send(_address, "/set_range_output", "ii", 5, 12);
    _event_queue.wait_for_data(std::chrono::milliseconds(10));
    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    ASSERT_EQ(MessageType::VALUE, event->base_type());
    auto val = static_unique_ptr_cast<IntegerSetValue, BaseMessage>(std::move(event));

    ASSERT_EQ(ValueType::INT_SET, val->type());
    ASSERT_EQ(5, val->index());
    ASSERT_EQ(12, val->value());
    ASSERT_TRUE(_event_queue.empty());
}

TEST_F(TestOSCUserFrontend, test_imu_calibrate)
{
    lo_send(_address, "/imu_calibrate", "");
    _event_queue.wait_for_data(std::chrono::milliseconds(10));
    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> cal_event = _event_queue.pop();
    ASSERT_EQ(MessageType::COMMAND, cal_event->base_type());
    auto cal_cmd = static_unique_ptr_cast<ImuCalibrateCommand, BaseMessage>(std::move(cal_event));

    ASSERT_EQ(CommandType::IMU_CALIBRATE, cal_cmd->type());

    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> save_event = _event_queue.pop();
    ASSERT_EQ(MessageType::COMMAND, save_event->base_type());
    auto save_cmd = static_unique_ptr_cast<ImuCommitSettingsCommand, BaseMessage>(std::move(save_event));

    ASSERT_EQ(CommandType::IMU_COMMIT_SETTINGS, save_cmd->type());
    ASSERT_TRUE(_event_queue.empty());
}

TEST_F(TestOSCUserFrontend, test_imu_factory_reset)
{
    lo_send(_address, "/imu_reset", "");
    _event_queue.wait_for_data(std::chrono::milliseconds(10));
    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    ASSERT_EQ(MessageType::COMMAND, event->base_type());
    auto cmd = static_unique_ptr_cast<ImuFactoryResetCommand, BaseMessage>(std::move(event));

    ASSERT_EQ(CommandType::IMU_FACTORY_RESET, cmd->type());
    ASSERT_TRUE(_event_queue.empty());
}

TEST_F(TestOSCUserFrontend, test_imu_reboot)
{
    lo_send(_address, "/imu_reboot", "");
    _event_queue.wait_for_data(std::chrono::milliseconds(10));
    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    ASSERT_EQ(MessageType::COMMAND, event->base_type());
    auto cmd = static_unique_ptr_cast<ImuRebootCommand, BaseMessage>(std::move(event));

    ASSERT_EQ(CommandType::IMU_REBOOT, cmd->type());
    ASSERT_TRUE(_event_queue.empty());
}


