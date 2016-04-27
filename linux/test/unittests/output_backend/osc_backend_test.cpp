#include "gtest/gtest.h"

#define private public
#define protected public
#include "output_backend/osc_backend.cpp"
#include "message/message_factory.h"

#include <atomic>
#include <chrono>
#include <thread>

#include "../test_utils.h"

using namespace sensei;
using namespace sensei::output_backend;

void osc_error_handler(int /*num*/, const char* /*msg*/, const char* /*path*/)
{
}

int alice_values_handler(const char *path, const char *types, lo_arg **argv,
                         int argc, void *data, void *user_data);

int bob_values_handler(const char *path, const char *types, lo_arg ** argv,
                       int argc, void *data, void *user_data);

int alice_raw_handler(const char *path, const char *types, lo_arg **argv,
                      int argc, void *data, void *user_data);

int bob_raw_handler(const char *path, const char *types, lo_arg **argv,
                    int argc, void *data, void *user_data);

class TestOscBackend : public ::testing::Test
{
protected:
    TestOscBackend()
    {
    }

    ~TestOscBackend()
    {
    }

    void SetUp()
    {
        // Configure backend
        MessageFactory factory;
        std::vector<std::unique_ptr<Command>> config_cmds;

        // global config messages
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_osc_output_base_path_command(0, _base_path))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_osc_output_raw_path_command(0, _base_raw_path))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_osc_output_host_command(0, _host))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_osc_output_port_command(0, _port))));

        // Some per-pin configs
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_pin_type_command(0, PinType::DIGITAL_INPUT))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_pin_name_command(0, "alice"))));

        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_pin_type_command(1, PinType::ANALOG_INPUT))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_pin_name_command(1, "bob"))));

        for (auto& cmd : config_cmds)
        {
            auto status = _backend.apply_command(std::move(cmd));
            ASSERT_EQ(CommandErrorCode::OK, status);
        }

        // Create OSC threaded server
        std::stringstream port_stream;
        port_stream << _port;
        auto port_str = port_stream.str();
        _osc_server = lo_server_new(port_str.c_str(), osc_error_handler);
        lo_server_add_method(_osc_server, "/test_sensors/digital/alice", "f", alice_values_handler, this);
        lo_server_add_method(_osc_server, "/test_sensors/analog/bob", "f", bob_values_handler, this);
        lo_server_add_method(_osc_server, "/test_input_raw/digital/alice", "i", alice_raw_handler, this);
        lo_server_add_method(_osc_server, "/test_input_raw/analog/bob", "i", bob_raw_handler, this);
    }

    void TearDown()
    {
        lo_server_free(_osc_server);
    }

    int _max_n_sensors{64};
    OSCBackend _backend{_max_n_sensors};

    std::string _base_path{"test_sensors"};
    std::string _base_raw_path{"test_input_raw"};
    std::string _host{"127.0.0.1"};
    int _port{22022};

    float _last_alice_received{0.0f};
    float _last_bob_received{0.0f};
    int _last_raw_alice_received{0};
    int _last_raw_bob_received{0};

    lo_server _osc_server{nullptr};

};

int alice_values_handler(const char* /*path*/, const char* /*types*/, lo_arg **argv,
                         int /*argc*/, void* /*data*/, void *user_data)
{
    auto backend = static_cast<TestOscBackend*>(user_data);
    backend->_last_alice_received = argv[0]->f;

    return 1;
}

int alice_raw_handler(const char* /*path*/, const char* /*types*/, lo_arg **argv,
                      int /*argc*/, void* /*data*/, void *user_data)
{
    auto backend = static_cast<TestOscBackend*>(user_data);
    backend->_last_raw_alice_received = argv[0]->i;

    return 1;
}

int bob_values_handler(const char* /*path*/, const char* /*types*/, lo_arg **argv,
                       int /*argc*/, void* /*data*/, void *user_data)
{
    auto backend = static_cast<TestOscBackend*>(user_data);
    backend->_last_bob_received = argv[0]->f;

    return 1;
}

int bob_raw_handler(const char* /*path*/, const char* /*types*/, lo_arg **argv,
                    int /*argc*/, void* /*data*/, void *user_data)
{
    auto backend = static_cast<TestOscBackend*>(user_data);
    backend->_last_raw_bob_received = argv[0]->i;

    return 1;
}

TEST_F(TestOscBackend, test_config)
{
    ASSERT_EQ(_base_path, _backend._base_path);
    ASSERT_EQ(_base_raw_path, _backend._base_raw_path);
    ASSERT_EQ(_host, _backend._host);
    ASSERT_EQ(_port, _backend._port);
    ASSERT_EQ(PinType::DIGITAL_INPUT, _backend._pin_types[0]);
    ASSERT_EQ("alice", _backend._pin_names[0]);
    ASSERT_EQ(PinType::ANALOG_INPUT, _backend._pin_types[1]);
    ASSERT_EQ("bob", _backend._pin_names[1]);
}

TEST_F(TestOscBackend, test_path_creation)
{
    ASSERT_EQ("/test_sensors/digital/alice", _backend._full_out_paths[0]);
    ASSERT_EQ("/test_input_raw/digital/alice", _backend._full_raw_paths[0]);
    ASSERT_EQ("/test_sensors/analog/bob", _backend._full_out_paths[1]);
    ASSERT_EQ("/test_input_raw/analog/bob", _backend._full_raw_paths[1]);
}

TEST_F(TestOscBackend, test_send_output)
{
    MessageFactory factory;

    // Enable output and disable raw input
    std::vector<std::unique_ptr<Command>> config_cmds;
    config_cmds.push_back(CMD_UPTR(factory.make_set_send_output_enabled_command(0, true)));
    config_cmds.push_back(CMD_UPTR(factory.make_set_send_raw_input_enabled_command(0, false)));
    for (auto& cmd : config_cmds)
    {
        auto status = _backend.apply_command(std::move(cmd));
        ASSERT_EQ(CommandErrorCode::OK, status);
    }

    auto value_msg = factory.make_output_value(0, 1.0f);
    auto value = static_unique_ptr_cast<OutputValue, BaseMessage>(std::move(value_msg));
    _backend.send(std::move(value), nullptr);
    lo_server_recv(_osc_server);
    ASSERT_EQ(_last_alice_received, 1.0f);

    value_msg = factory.make_output_value(1, 0.12345f);
    value = static_unique_ptr_cast<OutputValue, BaseMessage>(std::move(value_msg));
    _backend.send(std::move(value), nullptr);
    lo_server_recv(_osc_server);
    ASSERT_EQ(_last_bob_received, 0.12345f);
}

TEST_F(TestOscBackend, test_send_raw_input)
{
    MessageFactory factory;

    // Enable raw input
    std::vector<std::unique_ptr<Command>> config_cmds;
    config_cmds.push_back(CMD_UPTR(factory.make_set_send_output_enabled_command(0, false)));
    config_cmds.push_back(CMD_UPTR(factory.make_set_send_raw_input_enabled_command(0, true)));
    for (auto & cmd : config_cmds)
    {
        auto status = _backend.apply_command(std::move(cmd));
        ASSERT_EQ(CommandErrorCode::OK, status);
    }

    auto value_msg_alice = factory.make_output_value(0, 0.5f);
    auto value_msg_bob = factory.make_output_value(1, 5.6789f);
    auto digital_input_msg = factory.make_digital_value(0, true);
    auto analog_input_msg = factory.make_analog_value(1, 176);

    auto value_alice = static_unique_ptr_cast<OutputValue, BaseMessage>(std::move(value_msg_alice));
    auto value_bob = static_unique_ptr_cast<OutputValue, BaseMessage>(std::move(value_msg_bob));
    auto analog_input = static_unique_ptr_cast<AnalogValue, BaseMessage>(std::move(analog_input_msg));
    auto digital_input = static_unique_ptr_cast<DigitalValue, BaseMessage>(std::move(digital_input_msg));

    _backend.send(std::move(value_alice), std::move(digital_input));
    lo_server_recv(_osc_server);
    ASSERT_EQ(1, _last_raw_alice_received);

    _backend.send(std::move(value_bob), std::move(analog_input));
    lo_server_recv(_osc_server);
    ASSERT_EQ(176, _last_raw_bob_received);
}
