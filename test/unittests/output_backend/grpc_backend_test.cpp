#include "gtest/gtest.h"

#define private public
#define protected public
#include "output_backend/grpc_backend.cpp"
#include "user_frontend/grpc_user_frontend.h"
#include "message/message_factory.h"

#include <atomic>
#include <chrono>
#include <thread>
#include <grpcpp/grpcpp.h>
#include "sensei-grpc-api/sensei_rpc.pb.h"
#include "sensei-grpc-api/sensei_rpc.grpc.pb.h"

#include "../test_utils.h"

using namespace sensei;
using namespace sensei::output_backend;
using namespace sensei::user_frontend;

class TestGrpcBackend : public ::testing::Test
{
protected:
    TestGrpcBackend()
    {
    }

    ~TestGrpcBackend()
    {
    }

    void SetUp()
    {
        // Configure backend
        MessageFactory factory;
        std::vector<std::unique_ptr<Command>> config_cmds;

        _user_frontend.reset(new GrpcUserFrontend(&_event_queue, 64, 32));
        _backend.set_user_frontend(_user_frontend.get());

        // Some per-pin configs
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_type_command(0, SensorType::DIGITAL_INPUT))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_name_command(0, "SW1"))));

        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_type_command(1, SensorType::ANALOG_INPUT))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_name_command(1, "POT1"))));

        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_type_command(2, SensorType::RANGE_INPUT))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_name_command(2, "RANGE1"))));

        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_type_command(3, SensorType::DIGITAL_OUTPUT))));
        config_cmds.push_back(std::move(CMD_UPTR(factory.make_set_sensor_name_command(3, "LED1"))));

        for (auto const& cmd : config_cmds)
        {
            auto status = _backend.apply_command(cmd.get());
            ASSERT_EQ(CommandErrorCode::OK, status);
        }

        // Create gRPC client
        std::stringstream ss;
        ss << "localhost:" << _server_port;
        _channel = grpc::CreateChannel(ss.str(), grpc::InsecureChannelCredentials());
        _stub = sensei_rpc::PinProxyService::NewStub(_channel);
    }

    void TearDown()
    {
        _user_frontend.reset();
    }

    int _max_n_sensors{64};
    GrpcBackend _backend{_max_n_sensors};

    std::unique_ptr<GrpcUserFrontend> _user_frontend;
    SynchronizedQueue<std::unique_ptr<BaseMessage>> _event_queue;
    int _server_port{50051};

    std::shared_ptr<grpc::Channel> _channel;
    std::unique_ptr<sensei_rpc::PinProxyService::Stub> _stub;

    std::vector<sensei_rpc::Event> _received_events;
};

TEST_F(TestGrpcBackend, test_config)
{
    ASSERT_EQ(SensorType::DIGITAL_INPUT, _backend._pin_types[0]);
    ASSERT_EQ("SW1", _backend._sensor_names[0]);
    ASSERT_EQ(SensorType::ANALOG_INPUT, _backend._pin_types[1]);
    ASSERT_EQ("POT1", _backend._sensor_names[1]);
    ASSERT_EQ(SensorType::RANGE_INPUT, _backend._pin_types[2]);
    ASSERT_EQ("RANGE1", _backend._sensor_names[2]);
}

TEST_F(TestGrpcBackend, test_send_without_frontend)
{
    // Create a backend without linked frontend and confirm that sending doesn't crash
    GrpcBackend standalone_backend{64};
    MessageFactory factory;

    auto cmd = CMD_UPTR(factory.make_set_sensor_type_command(0, SensorType::DIGITAL_INPUT));
    standalone_backend.apply_command(cmd.get());

    auto enable_cmd = CMD_UPTR(factory.make_set_send_output_enabled_command(0, true));
    standalone_backend.apply_command(enable_cmd.get());

    auto value_msg = factory.make_output_value(0, 1.0f);
    auto value = static_cast<OutputValue*>(value_msg.get());
    standalone_backend.send(value, nullptr);

    // Test passes if no crash occurs
}

TEST_F(TestGrpcBackend, test_send_with_frontend)
{
    MessageFactory factory;

    // Enable output
    auto enable_cmd = CMD_UPTR(factory.make_set_send_output_enabled_command(0, true));
    auto status = _backend.apply_command(enable_cmd.get());
    ASSERT_EQ(CommandErrorCode::OK, status);

    // Start subscription in separate thread
    std::atomic<bool> subscribed{false};
    std::atomic<bool> received{false};
    sensei_rpc::Event received_event;
    std::thread subscriber_thread([this, &subscribed, &received, &received_event]() {
        grpc::ClientContext context;
        sensei_rpc::SubscribeRequest request;
        std::unique_ptr<grpc::ClientReader<sensei_rpc::Event>> reader(
            _stub->SubscribeToEvents(&context, request));
        subscribed.store(true);

        sensei_rpc::Event event;
        if (reader->Read(&event)) {
            received_event = event;
            received.store(true);
        }
    });

    // Wait for subscription before sending the event
    ASSERT_TRUE(wait_for([&subscribed]() { return subscribed.load(); }, DEFAULT_TIMEOUT));

    // Send value through backend
    auto value_msg = factory.make_output_value(0, 1.0f);
    auto value = static_cast<OutputValue*>(value_msg.get());
    _backend.send(value, nullptr);

    // Wait for it to be received
    ASSERT_TRUE(wait_for([&received]() { return received.load(); }, DEFAULT_TIMEOUT));

    subscriber_thread.join();

    ASSERT_TRUE(received.load());
    ASSERT_TRUE(received_event.has_toggle_ev());
    ASSERT_EQ(0, received_event.controller_id());
    ASSERT_TRUE(received_event.toggle_ev().value());
}

TEST_F(TestGrpcBackend, test_event_type_mapping)
{
    MessageFactory factory;

    // Enable output
    auto enable_cmd = CMD_UPTR(factory.make_set_send_output_enabled_command(0, true));
    _backend.apply_command(enable_cmd.get());

    // Start subscription
    std::atomic<bool> subscribed{false};
    std::vector<sensei_rpc::Event> received_events;
    std::mutex events_mutex;
    std::atomic<int> event_count{0};

    std::thread subscriber_thread([this, &subscribed, &received_events, &events_mutex, &event_count]() {
        grpc::ClientContext context;
        sensei_rpc::SubscribeRequest request;
        std::unique_ptr<grpc::ClientReader<sensei_rpc::Event>> reader(
            _stub->SubscribeToEvents(&context, request));
        subscribed.store(true);

        sensei_rpc::Event event;
        while (event_count < 3 && reader->Read(&event)) {
            {
                std::lock_guard<std::mutex> lock(events_mutex);
                received_events.push_back(event);
            }
            event_count++;
        }
    });

    ASSERT_TRUE(wait_for([&subscribed]() { return subscribed.load(); }, DEFAULT_TIMEOUT));

    // Test digital sensor -> ToggleEvent
    auto digital_msg = factory.make_output_value(0, 1.0f);
    auto digital_value = static_cast<OutputValue*>(digital_msg.get());
    _backend.send(digital_value, nullptr);
    ASSERT_TRUE(wait_for([&event_count]() { return event_count == 1; }, DEFAULT_TIMEOUT));

    // Test analog sensor -> AnalogEvent
    auto analog_msg = factory.make_output_value(1, 0.75f);
    auto analog_value = static_cast<OutputValue*>(analog_msg.get());
    _backend.send(analog_value, nullptr);
    ASSERT_TRUE(wait_for([&event_count]() { return event_count == 2; }, DEFAULT_TIMEOUT));

    // Test range sensor -> RangeEvent
    auto range_msg = factory.make_output_value(2, 42.0f);
    auto range_value = static_cast<OutputValue*>(range_msg.get());
    _backend.send(range_value, nullptr);
    ASSERT_TRUE(wait_for([&event_count]() { return event_count == 3; }, DEFAULT_TIMEOUT));

    subscriber_thread.join();

    std::lock_guard<std::mutex> lock(events_mutex);
    ASSERT_EQ(3, received_events.size());

    // Verify digital -> ToggleEvent
    ASSERT_TRUE(received_events[0].has_toggle_ev());
    ASSERT_EQ(0, received_events[0].controller_id());
    ASSERT_TRUE(received_events[0].toggle_ev().value());

    // Verify analog -> AnalogEvent
    ASSERT_TRUE(received_events[1].has_analog_ev());
    ASSERT_EQ(1, received_events[1].controller_id());
    ASSERT_FLOAT_EQ(0.75f, received_events[1].analog_ev().value());

    // Verify range -> RangeEvent
    ASSERT_TRUE(received_events[2].has_range_ev());
    ASSERT_EQ(2, received_events[2].controller_id());
    ASSERT_EQ(42, received_events[2].range_ev().value());
}

TEST_F(TestGrpcBackend, test_value_conversion)
{
    MessageFactory factory;

    // Test digital value conversion: 0.0 -> false, 1.0 -> true, 0.6 -> true
    auto event_low = _backend._create_proto_event(0, SensorType::DIGITAL_INPUT, 0.0f, 12345);
    ASSERT_TRUE(event_low.has_toggle_ev());
    ASSERT_FALSE(event_low.toggle_ev().value());

    auto event_high = _backend._create_proto_event(0, SensorType::DIGITAL_INPUT, 1.0f, 12345);
    ASSERT_TRUE(event_high.has_toggle_ev());
    ASSERT_TRUE(event_high.toggle_ev().value());

    auto event_threshold = _backend._create_proto_event(0, SensorType::DIGITAL_INPUT, 0.6f, 12345);
    ASSERT_TRUE(event_threshold.has_toggle_ev());
    ASSERT_TRUE(event_threshold.toggle_ev().value());

    // Test range value conversion: float -> int32
    auto event_range = _backend._create_proto_event(1, SensorType::RANGE_INPUT, 127.8f, 12345);
    ASSERT_TRUE(event_range.has_range_ev());
    ASSERT_EQ(127, event_range.range_ev().value());

    // Test analog value conversion: float preserved
    auto event_analog = _backend._create_proto_event(2, SensorType::ANALOG_INPUT, 0.12345f, 12345);
    ASSERT_TRUE(event_analog.has_analog_ev());
    ASSERT_FLOAT_EQ(0.12345f, event_analog.analog_ev().value());
}

TEST_F(TestGrpcBackend, test_send_flags)
{
    MessageFactory factory;

    // Disable output, enable raw input (not implemented yet)
    auto disable_output_cmd = CMD_UPTR(factory.make_set_send_output_enabled_command(0, false));
    _backend.apply_command(disable_output_cmd.get());

    ASSERT_FALSE(_backend._send_output_active);

    // Send should not broadcast when disabled
    std::atomic<bool> subscribed{false};
    std::atomic<bool> received{false};
    std::thread subscriber_thread([this, &subscribed, &received]() {
        grpc::ClientContext context;
        sensei_rpc::SubscribeRequest request;
        auto deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(200);
        context.set_deadline(deadline);

        std::unique_ptr<grpc::ClientReader<sensei_rpc::Event>> reader(
            _stub->SubscribeToEvents(&context, request));
        subscribed.store(true);

        sensei_rpc::Event event;
        if (reader->Read(&event)) {
            received.store(true);
        }
    });

    ASSERT_TRUE(wait_for([&subscribed]() { return subscribed.load(); }, DEFAULT_TIMEOUT));

    auto value_msg = factory.make_output_value(0, 1.0f);
    auto value = static_cast<OutputValue*>(value_msg.get());
    _backend.send(value, nullptr);

    // should timeout as there should be no broadcast, use a shorter timeout
    ASSERT_FALSE(wait_for([&received]() { return received.load(); }, std::chrono::seconds(1)));

    subscriber_thread.join();

    ASSERT_FALSE(received.load());

    // Re-enable output
    auto enable_output_cmd = CMD_UPTR(factory.make_set_send_output_enabled_command(0, true));
    _backend.apply_command(enable_output_cmd.get());
    ASSERT_TRUE(_backend._send_output_active);
}

TEST_F(TestGrpcBackend, test_timestamp_conversion)
{
    uint32_t timestamp_in = 123456;
    auto event = _backend._create_proto_event(0, SensorType::ANALOG_INPUT, 1.0f, timestamp_in);

    ASSERT_TRUE(event.has_analog_ev());
    ASSERT_EQ(static_cast<int64_t>(timestamp_in), event.timestamp());
}

TEST_F(TestGrpcBackend, test_populate_controller_map)
{
    // this map should be auto-populated by backend events, so need to test here and not in frontend tests
    sensei_rpc::GetControllerMapResponse response;
    _user_frontend->populate_controller_map(&response);

    ASSERT_EQ(response.switches_size(), 1);
    ASSERT_EQ(response.switches().Get(0).id(), 0);
    ASSERT_EQ(response.switches().Get(0).name(), "SW1");
    ASSERT_EQ(response.pots_size(), 1);
    ASSERT_EQ(response.pots().Get(0).id(), 1);
    ASSERT_EQ(response.pots().Get(0).name(), "POT1");
    ASSERT_EQ(response.leds_size(), 1);
    ASSERT_EQ(response.leds().Get(0).id(), 3);
    // ASSERT_EQ(response.leds().Get(0).name(), "LED1");
}
