//
// Created by gustav on 3/17/16.
//

#include "gtest/gtest.h"
#define private public

#include "serial_frontend/message_tracker.cpp"

using namespace sensei;
using namespace serial_frontend;


class TestMessageTracker : public ::testing::Test
{
protected:
    TestMessageTracker() :
            _module_under_test(std::chrono::milliseconds(1000))
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
    MessageTracker _module_under_test;
};

/*
 * Test the initial conditions
 */
TEST_F (TestMessageTracker, test_initial_conditions)
{
    ASSERT_EQ(ack_status::UNKNOWN_IDENTIFIER, _module_under_test.check_status(123456));
    ASSERT_TRUE(_module_under_test._entries.empty());
}

/*
 * Test time handling
 */
TEST_F (TestMessageTracker, test_time_handling)
{
    ASSERT_EQ(std::chrono::milliseconds(1000), _module_under_test._timeout);
}

/*
 * Test tracking a few simple packets
 */
TEST_F (TestMessageTracker, test_simple_tracking)
{
    _module_under_test.log(1);
    ASSERT_FALSE(_module_under_test._entries.empty());
    _module_under_test.log(2);
    _module_under_test.log(3);
    /*
     * Check that all entries return ok and that no has timed out.
     */
    ASSERT_EQ(ack_status::ACKED_OK, _module_under_test.check_status(2));
    ASSERT_EQ(0u, _module_under_test.timed_out());
    ASSERT_EQ(ack_status::ACKED_OK, _module_under_test.check_status(3));
    ASSERT_EQ(ack_status::ACKED_OK, _module_under_test.check_status(1));
}

/*
 * Test timeouts on entries
 */
TEST_F (TestMessageTracker, test_late_ack)
{
    _module_under_test.log(2);
    _module_under_test.log(3);
    /*
     * Hack the list by rewinding the stored timestamps
     */
    auto entry = _module_under_test._entries.find(2);
    entry->second -= std::chrono::milliseconds(1000);
    entry = _module_under_test._entries.find(3);
    entry->second -= std::chrono::milliseconds(1000);
    ASSERT_EQ(ack_status::TIMED_OUT, _module_under_test.check_status(2));
    ASSERT_EQ(3u, _module_under_test.timed_out());
    ASSERT_EQ(0u, _module_under_test.timed_out());

}