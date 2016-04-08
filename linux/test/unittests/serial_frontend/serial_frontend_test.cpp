//
// Created by gustav on 3/17/16.
//

#include "gtest/gtest.h"
#define private public
#include "serial_frontend/serial_frontend.cpp"

using namespace sensei;
using namespace serial_frontend;


static uint8_t test_msg[] =    { 0x12, 0x34, 0x56, 0x12, 0x25, 0x24, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12,
                          0x34, 0x56, 0x78, 0x6F, 0x01, 0x12, 0x34, 0x56 };

// Test standalone functions

TEST (TestHelperFunctions, test_verify_message)
{
    ASSERT_TRUE(verify_message(reinterpret_cast<sensei::sSenseiDataPacket*>(test_msg)));
}

class SerialFrontendTest : public ::testing::Test
{
protected:
    SerialFrontendTest() :
            _module_under_test("/dev/ttyS011", &_in_queue, &_out_queue)
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
    SynchronizedQueue<std::unique_ptr<BaseMessage>>  _out_queue;
    SynchronizedQueue<std::unique_ptr<Command>>      _in_queue;
    SerialFrontend _module_under_test;
};

/*
 * Verify that serial packets are correctly created from Command messages
 */
TEST_F(SerialFrontendTest, test_create_serial_message)
{
    MessageFactory factory;
    auto command = static_cast<Command*>(factory.make_set_sampling_rate_command(3, 500.0, 100u).release());
    const sSenseiDataPacket* packet =_module_under_test.create_send_command(command);
    ASSERT_EQ(SENSEI_CMD::SET_SAMPLING_RATE, packet->cmd);
    auto payload = reinterpret_cast<const teensy_set_samplerate_cmd*>(packet->payload);
    ASSERT_EQ(2, payload->sample_rate_divisor);

    command = static_cast<Command*>(factory.make_set_lowpass_cutoff_command(4, 1234.0, 100u).release());
    packet =_module_under_test.create_send_command(command);
    ASSERT_EQ(SENSEI_CMD::CONFIGURE_PIN, packet->cmd);
    auto payload_cfg = reinterpret_cast<const sPinConfiguration*>(packet->payload);
    ASSERT_FLOAT_EQ(1234, payload_cfg->lowPassCutOffFilter);
}

TEST_F(SerialFrontendTest, test_mute_function)
{
    ASSERT_FALSE(_module_under_test._muted);
    _module_under_test.mute(true);
    ASSERT_TRUE(_module_under_test._muted);
}


TEST_F(SerialFrontendTest, test_instanciation_again)
{
    EXPECT_TRUE(_module_under_test.connected());
}
