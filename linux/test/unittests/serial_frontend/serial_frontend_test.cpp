//
// Created by gustav on 3/17/16.
//

#include "gtest/gtest.h"
#include "serial_frontend.cpp"
#define private public

using namespace sensei;
using namespace serial_frontend;


uint8_t test_msg[] =    { 0x12, 0x34, 0x56, 0x12, 0x25, 0x24, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12,
                          0x34, 0x56, 0x78, 0x6F, 0x01, 0x12, 0x34, 0x56 };

// Test standalone functions
TEST (TestHelperFunctions, test_compare_packet_header) {
    sensei::PACKET_HEADER hdr1 = {1, 2, 3};
    sensei::PACKET_HEADER hdr2 = {2, 3, 4};
    ASSERT_EQ(0, compare_packet_header(hdr1, hdr1));
    ASSERT_NE(0, compare_packet_header(hdr1, hdr2));
}

TEST (TestHelperFunctions, test_calculate_crc)
{
    uint16_t crc = calculate_crc(reinterpret_cast<sensei::sSenseiDataPacket*>(test_msg));
    ASSERT_EQ(0x16F, crc);
}

TEST (TestHelperFunctions, test_verify_message)
{
    ASSERT_TRUE(verify_message(reinterpret_cast<sensei::sSenseiDataPacket*>(test_msg)));
}

TEST (TestSerialFrontend, test_instanciation)
{
    SynchronizedQueue<std::unique_ptr<BaseMessage>>  out_queue;
    SynchronizedQueue<std::unique_ptr<CommandMessage>>  in_queue;
    SerialFrontend module_under_test("/dev/ttyS011", &in_queue, &out_queue);
    ASSERT_TRUE(module_under_test.connected());
    module_under_test.run();
    module_under_test.stop();
}

// A more complex test case where tests can be grouped
// And setup and teardown functions added.
class SerialFrontendTest : public ::testing::Test
{
protected:
    SerialFrontendTest()
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
};

TEST_F(SerialFrontendTest, TestSetup)
{
EXPECT_FALSE(0);
}
