#include "gtest/gtest.h"
#include <grpcpp/support/status.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "user_frontend/async_call_handlers.cpp"
#include "user_frontend/grpc_user_frontend.cpp"
#pragma GCC diagnostic pop

#include <grpcpp/grpcpp.h>
#include "pin-proxy/pin_events.pb.h"
#include "pin-proxy/pin_events.grpc.pb.h"

#include "../test_utils.h"

using namespace sensei;
using namespace sensei::user_frontend;

class TestGrpcUserFrontend : public ::testing::Test
{
protected:
    TestGrpcUserFrontend()
    {
    }

    ~TestGrpcUserFrontend()
    {
    }

    void SetUp()
    {
        // Create user frontend (starts server on default port 50051)
        _user_frontend.reset(new GrpcUserFrontend(&_event_queue, 64, 32));

        // Create gRPC client
        std::stringstream ss;
        ss << "localhost:" << _server_port;
        _server_address = ss.str();

        _channel = grpc::CreateChannel(_server_address, grpc::InsecureChannelCredentials());
        _stub = pin_proxy::PinProxyService::NewStub(_channel);
    }

    void TearDown()
    {
        _user_frontend.reset();
    }

    std::unique_ptr<GrpcUserFrontend> _user_frontend;
    SynchronizedQueue<std::unique_ptr<BaseMessage>> _event_queue;
    int _server_port{50051};
    std::string _server_address;

    std::shared_ptr<grpc::Channel> _channel;
    std::unique_ptr<pin_proxy::PinProxyService::Stub> _stub;
};

TEST_F(TestGrpcUserFrontend, test_server_startup)
{
    // Server should be running after SetUp
    // Verify by checking channel state
    auto state = _channel->GetState(true);
    ASSERT_TRUE(state == GRPC_CHANNEL_IDLE);
}

TEST_F(TestGrpcUserFrontend, test_subscribe_single_client)
{
    grpc::ClientContext context;
    pin_proxy::SubscribeRequest request;

    std::unique_ptr<grpc::ClientReader<pin_proxy::Event>> reader(
        _stub->SubscribeToEvents(&context, request));

    // Give subscription time to establish
    ASSERT_TRUE(wait_for([this]() { return _user_frontend->num_subscribers() == 1; }, DEFAULT_TIMEOUT));

    // Create and broadcast an event
    pin_proxy::Event event;
    auto* toggle_ev = event.mutable_toggle_ev();
    toggle_ev->set_controller_id(5);
    toggle_ev->set_timestamp(12345);
    toggle_ev->set_value(true);

    _user_frontend->broadcast_event(event);

    // Read event from stream with timeout
    pin_proxy::Event received_event;
    std::atomic<bool> read_complete{false};
    std::thread read_thread([&reader, &received_event, &read_complete]() {
        if (reader->Read(&received_event)) {
            read_complete.store(true);
        }
    });

    ASSERT_TRUE(wait_for([&read_complete]() {
        return read_complete.load(); }, DEFAULT_TIMEOUT));

    context.TryCancel();
    read_thread.join();

    ASSERT_TRUE(read_complete.load());
    ASSERT_TRUE(received_event.has_toggle_ev());
    ASSERT_EQ(5, received_event.toggle_ev().controller_id());
    ASSERT_EQ(12345, received_event.toggle_ev().timestamp());
    ASSERT_TRUE(received_event.toggle_ev().value());
}

TEST_F(TestGrpcUserFrontend, test_subscribe_multiple_clients)
{
    // Create two client subscriptions
    grpc::ClientContext context1;
    grpc::ClientContext context2;
    pin_proxy::SubscribeRequest request;

    std::unique_ptr<grpc::ClientReader<pin_proxy::Event>> reader1(
        _stub->SubscribeToEvents(&context1, request));
    std::unique_ptr<grpc::ClientReader<pin_proxy::Event>> reader2(
        _stub->SubscribeToEvents(&context2, request));

    ASSERT_TRUE(wait_for([this]() {
        return _user_frontend->num_subscribers() == 2; }, DEFAULT_TIMEOUT));

    // Broadcast event
    pin_proxy::Event event;
    auto* analog_ev = event.mutable_analog_ev();
    analog_ev->set_controller_id(7);
    analog_ev->set_timestamp(54321);
    analog_ev->set_value(0.75f);

    _user_frontend->broadcast_event(event);

    // Read from both clients
    pin_proxy::Event received_event1, received_event2;
    std::atomic<int> reads_complete{0};

    std::thread read_thread1([&reader1, &received_event1, &reads_complete]() {
        if (reader1->Read(&received_event1)) {
            reads_complete++;
        }
    });

    std::thread read_thread2([&reader2, &received_event2, &reads_complete]() {
        if (reader2->Read(&received_event2)) {
            reads_complete++;
        }
    });

    ASSERT_TRUE(wait_for([&reads_complete]() { return reads_complete == 2; }, DEFAULT_TIMEOUT));

    context1.TryCancel();
    context2.TryCancel();
    read_thread1.join();
    read_thread2.join();

    ASSERT_EQ(2, reads_complete.load());

    // Verify both clients received the same event
    ASSERT_TRUE(received_event1.has_analog_ev());
    ASSERT_EQ(7, received_event1.analog_ev().controller_id());
    ASSERT_FLOAT_EQ(0.75f, received_event1.analog_ev().value());

    ASSERT_TRUE(received_event2.has_analog_ev());
    ASSERT_EQ(7, received_event2.analog_ev().controller_id());
    ASSERT_FLOAT_EQ(0.75f, received_event2.analog_ev().value());
}

TEST_F(TestGrpcUserFrontend, test_client_disconnect)
{
    // Subscribe and then disconnect
    {
        grpc::ClientContext context;
        pin_proxy::SubscribeRequest request;

        std::unique_ptr<grpc::ClientReader<pin_proxy::Event>> reader(
            _stub->SubscribeToEvents(&context, request));

        ASSERT_TRUE(wait_for([this]() {
            return _user_frontend->num_subscribers() == 1; }, DEFAULT_TIMEOUT));

        context.TryCancel();

        // TryCancel() isn't guaranteed to cancel, so we can't check that subscribers are back to 0 at exactly this point
    }

    // Broadcast event - should not crash even though client disconnected
    pin_proxy::Event event;
    auto* toggle_ev = event.mutable_toggle_ev();
    toggle_ev->set_controller_id(1);
    toggle_ev->set_timestamp(999);
    toggle_ev->set_value(false);

    _user_frontend->broadcast_event(event);
}

TEST_F(TestGrpcUserFrontend, test_event_type_streaming)
{
    grpc::ClientContext context;
    pin_proxy::SubscribeRequest request;

    std::unique_ptr<grpc::ClientReader<pin_proxy::Event>> reader(
        _stub->SubscribeToEvents(&context, request));

    ASSERT_TRUE(wait_for([this]() {
        return _user_frontend->num_subscribers() == 1; }, DEFAULT_TIMEOUT));

    std::vector<pin_proxy::Event> received_events;
    std::mutex events_mutex;
    std::atomic<int> event_count{0};

    std::thread read_thread([&reader, &received_events, &events_mutex, &event_count]() {
        pin_proxy::Event event;
        while (reader->Read(&event) && event_count < 3) {
            {
                std::lock_guard<std::mutex> lock(events_mutex);
                received_events.push_back(event);
            }
            event_count++;
        }
    });

    // Broadcast ToggleEvent
    pin_proxy::Event toggle_event;
    auto* toggle_ev = toggle_event.mutable_toggle_ev();
    toggle_ev->set_controller_id(0);
    toggle_ev->set_timestamp(111);
    toggle_ev->set_value(true);
    _user_frontend->broadcast_event(toggle_event);
    ASSERT_TRUE(wait_for([&event_count]() { return event_count == 1; }, DEFAULT_TIMEOUT));

    // Broadcast AnalogEvent
    pin_proxy::Event analog_event;
    auto* analog_ev = analog_event.mutable_analog_ev();
    analog_ev->set_controller_id(1);
    analog_ev->set_timestamp(222);
    analog_ev->set_value(0.5f);
    _user_frontend->broadcast_event(analog_event);
    ASSERT_TRUE(wait_for([&event_count]() { return event_count == 2; }, DEFAULT_TIMEOUT));

    // Broadcast RangeEvent
    pin_proxy::Event range_event;
    auto* range_ev = range_event.mutable_range_ev();
    range_ev->set_controller_id(2);
    range_ev->set_timestamp(333);
    range_ev->set_value(42);
    _user_frontend->broadcast_event(range_event);
    ASSERT_TRUE(wait_for([&event_count]() { return event_count == 3; }, DEFAULT_TIMEOUT));

    context.TryCancel();
    read_thread.join();

    std::lock_guard<std::mutex> lock(events_mutex);
    ASSERT_EQ(3, received_events.size());

    // Verify ToggleEvent
    ASSERT_TRUE(received_events[0].has_toggle_ev());
    ASSERT_EQ(0, received_events[0].toggle_ev().controller_id());
    ASSERT_TRUE(received_events[0].toggle_ev().value());

    // Verify AnalogEvent
    ASSERT_TRUE(received_events[1].has_analog_ev());
    ASSERT_EQ(1, received_events[1].analog_ev().controller_id());
    ASSERT_FLOAT_EQ(0.5f, received_events[1].analog_ev().value());

    // Verify RangeEvent
    ASSERT_TRUE(received_events[2].has_range_ev());
    ASSERT_EQ(2, received_events[2].range_ev().controller_id());
    ASSERT_EQ(42, received_events[2].range_ev().value());
}

TEST_F(TestGrpcUserFrontend, test_update_led_rpc)
{
    grpc::ClientContext context;
    pin_proxy::UpdateLedRequest request;
    pin_proxy::GenericVoidValue response;

    request.set_led_id(5);
    request.set_active(true);

    grpc::Status status = _stub->UpdateLed(&context, request, &response);

    ASSERT_TRUE(status.ok());

    // Wait for message to be queued
    _event_queue.wait_for_data(std::chrono::milliseconds(50));
    ASSERT_FALSE(_event_queue.empty());

    // Verify correct message was created
    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    ASSERT_EQ(MessageType::VALUE, event->base_type());

    auto val = static_unique_ptr_cast<IntegerSetValue, BaseMessage>(std::move(event));
    ASSERT_EQ(ValueType::INT_SET, val->type());
    ASSERT_EQ(5, val->index());
    ASSERT_EQ(1, val->value());
}

TEST_F(TestGrpcUserFrontend, test_update_led_rpc_off)
{
    grpc::ClientContext context;
    pin_proxy::UpdateLedRequest request;
    pin_proxy::GenericVoidValue response;

    request.set_led_id(3);
    request.set_active(false);

    grpc::Status status = _stub->UpdateLed(&context, request, &response);

    ASSERT_TRUE(status.ok());

    _event_queue.wait_for_data(std::chrono::milliseconds(50));
    ASSERT_FALSE(_event_queue.empty());

    std::unique_ptr<BaseMessage> event = _event_queue.pop();
    auto val = static_unique_ptr_cast<IntegerSetValue, BaseMessage>(std::move(event));

    ASSERT_EQ(3, val->index());
    ASSERT_EQ(0, val->value());
}

TEST_F(TestGrpcUserFrontend, test_refresh_all_states_rpc)
{
    grpc::ClientContext context;
    pin_proxy::RefreshAllStatesRequest request;
    pin_proxy::RefreshAllStatesResponse response;

    grpc::Status status = _stub->RefreshAllStates(&context, request, &response);
    ASSERT_EQ(status.error_code(), grpc::StatusCode::UNIMPLEMENTED);
}

TEST_F(TestGrpcUserFrontend, test_concurrent_operations)
{
    // Start multiple subscriptions
    std::vector<std::unique_ptr<grpc::ClientContext>> contexts;
    std::vector<std::unique_ptr<grpc::ClientReader<pin_proxy::Event>>> readers;

    for (int i = 0; i < 3; i++) {
        contexts.push_back(std::make_unique<grpc::ClientContext>());
        pin_proxy::SubscribeRequest request;
        readers.push_back(_stub->SubscribeToEvents(contexts.back().get(), request));
    }

    ASSERT_TRUE(wait_for([this]() {
        return _user_frontend->num_subscribers() == 3; }, DEFAULT_TIMEOUT));

    // Broadcast multiple events rapidly
    std::atomic<int> total_received{0};
    std::vector<std::thread> read_threads;

    for (size_t i = 0; i < readers.size(); i++) {
        read_threads.emplace_back([&readers, &total_received, i]() {
            pin_proxy::Event event;
            int count = 0;
            while (readers[i]->Read(&event) && count < 5) {
                total_received++;
                count++;
            }
        });
    }

    // Broadcast 5 events
    for (int i = 0; i < 5; i++) {
        pin_proxy::Event event;
        auto* analog_ev = event.mutable_analog_ev();
        analog_ev->set_controller_id(i);
        analog_ev->set_timestamp(i * 1000);
        analog_ev->set_value(i * 0.1f);
        _user_frontend->broadcast_event(event);
        ASSERT_TRUE(wait_for([&total_received, &readers, i]() {
            return total_received == (i+1)*readers.size(); }, DEFAULT_TIMEOUT));
    }

    // Cancel all contexts
    for (auto& ctx : contexts) {
        ctx->TryCancel();
    }

    for (auto& thread : read_threads) {
        thread.join();
    }

    // All 3 clients should have received all 5 events (15 total)
    ASSERT_EQ(15, total_received.load());
}

TEST_F(TestGrpcUserFrontend, test_only_subscribed_controllers)
{
    grpc::ClientContext context;
    pin_proxy::SubscribeRequest request;

    // only get events from controller_id 2 and 3
    request.add_controller_ids(2);
    request.add_controller_ids(3);

    std::unique_ptr<grpc::ClientReader<pin_proxy::Event>> reader(
        _stub->SubscribeToEvents(&context, request));

    ASSERT_TRUE(wait_for([this]() {
        return _user_frontend->num_subscribers() == 1; }, DEFAULT_TIMEOUT));

    // Create and broadcast events for different controllers
    for (int i=0; i<4; ++i)
    {
        pin_proxy::Event event;
        auto* toggle_ev = event.mutable_toggle_ev();
        toggle_ev->set_controller_id(i);
        toggle_ev->set_timestamp(i);
        toggle_ev->set_value(true);
        _user_frontend->broadcast_event(event);
    }

    // Read event from stream with timeout
    pin_proxy::Event event1, event2;
    std::atomic<bool> received{false};
    std::thread read_thread([&reader, &event1, &event2, &received]() {
        ASSERT_TRUE(reader->Read(&event1));
        ASSERT_TRUE(reader->Read(&event2));
        received = true;
    });

    ASSERT_TRUE(wait_for([&received]() {
        return received.load(); }, DEFAULT_TIMEOUT));

    context.TryCancel();
    read_thread.join();

    // double check that the final events have expected controller ids
    ASSERT_EQ(2, event1.toggle_ev().controller_id());
    ASSERT_EQ(3, event2.toggle_ev().controller_id());
}

