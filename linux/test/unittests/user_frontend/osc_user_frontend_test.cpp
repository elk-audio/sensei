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

/*TEST_F(TestOSCUserFrontend, test_set_output)
{
    lo_send(_address, "/set_output", "if", 5, 12.0f);
    _event_queue.wait_for_data(std::chrono::milliseconds(10));
    ASSERT_FALSE(_event_queue.empty());
    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    ASSERT_EQ(MessageType::VALUE, event->base_type());
    auto val = static_unique_ptr_cast<FloatSetValue, BaseMessage>(std::move(event));

    ASSERT_EQ(ValueType::FLOAT_SET, val->type());
    ASSERT_EQ(5, val->index());
    ASSERT_FLOAT_EQ(12, val->value());
    ASSERT_TRUE(_event_queue.empty());
}*/