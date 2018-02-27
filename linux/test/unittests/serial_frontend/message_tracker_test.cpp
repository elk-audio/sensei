#include "gtest/gtest.h"
#define private public

#include "hardware_frontend/message_tracker.cpp"
#include "message/message_factory.h"
#include "hardware_frontend/serial_frontend_internal.h"

using namespace sensei;
using namespace hw_frontend;

const int MAX_TIMEOUT = 1000;
const int MAX_RESENDS = 2;

class TestMessageTracker : public ::testing::Test
{
protected:
    TestMessageTracker() :
            _module_under_test(std::chrono::milliseconds(MAX_TIMEOUT), MAX_RESENDS)
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
    MessageTracker _module_under_test;
    MessageFactory _factory;
};

/*
 * Test the initial conditions
 */
TEST_F (TestMessageTracker, test_initial_conditions)
{
    EXPECT_EQ(std::chrono::milliseconds(MAX_TIMEOUT), _module_under_test._timeout);
    EXPECT_EQ(MAX_RESENDS, _module_under_test._max_retries);
    EXPECT_EQ(nullptr, _module_under_test._message_in_transit);
}

/*
 * Test tracking and acking a packet
 */
TEST_F (TestMessageTracker, test_simple_tracking)
{
    auto message = std::unique_ptr<Command>(static_cast<Command*>(_factory.make_set_sending_delta_ticks_command(1, 1, 1).release()));
    _module_under_test.store(std::move(message), 1234);

    /* Test that it hasn't timed out */
    EXPECT_EQ(timeout::WAITING, _module_under_test.timed_out());

    /* First, test acking with an incorrect uuid */
    EXPECT_TRUE(_module_under_test.ack(1234));

    /* Test that an ack will correctly remove this msg */
    EXPECT_TRUE(_module_under_test.ack(1234));
    EXPECT_EQ(std::unique_ptr<Command>(nullptr), _module_under_test.get_cached_message());
}

/*
 * Test timeouts on entries
 */
TEST_F (TestMessageTracker, test_timeout)
{
    auto message = std::unique_ptr<Command>(static_cast<Command*>(_factory.make_set_sending_delta_ticks_command(1, 1, 1).release()));
    _module_under_test.store(std::move(message), 1234);

    /* Rewind the stored timestamp */
    _module_under_test._send_time -= std::chrono::milliseconds(1200);
    EXPECT_EQ(timeout::TIMED_OUT, _module_under_test.timed_out());

    /* Get the message and pretend to send it again */
    auto m2 = _module_under_test.get_cached_message();
    _module_under_test.store(std::move(m2), 1234);
    _module_under_test._send_time -= std::chrono::milliseconds(1200);

    EXPECT_EQ(timeout::TIMED_OUT_PERMANENTLY, _module_under_test.timed_out());
    EXPECT_NE(std::unique_ptr<Command>(nullptr), _module_under_test.get_cached_message());
}