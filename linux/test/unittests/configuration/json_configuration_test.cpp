#include <json/value.h>
#include "gtest/gtest.h"

#define private public
#include "config_backend/json_configuration.cpp"

using namespace sensei;
using namespace config;

static const std::string test_file = "../../../test/unittests/configuration/test_configuration.json";

TEST(JsonConfigurationTestInternal, test_read_configuration)
{
    Json::Value config = read_configuration(test_file);
    EXPECT_FALSE(config.isNull());
    EXPECT_TRUE(config.isObject());
}



class JsonConfigurationTest : public ::testing::Test
{
protected:
    JsonConfigurationTest() :
            _module_under_test(&_queue, test_file)
    {
    }
    void SetUp()
    {
    }

    void TearDown()
    {
    }
    SynchronizedQueue<std::unique_ptr<BaseMessage>>  _queue;
    JsonConfiguration _module_under_test;
};

/*
 * Parse a json config file and verify that all commands were created correctly
 */
TEST_F(JsonConfigurationTest, test_apply_configuration)
{
    EXPECT_TRUE(_queue.empty());
    _module_under_test.read();
    ASSERT_FALSE(_queue.empty());

    /* Noe verify the commands one by one */
    /* First we should receive the backend related commands */
    std::unique_ptr<BaseMessage> m = std::move(_queue.pop());
    Command* c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_OSC_OUTPUT_HOST, c->type());
    EXPECT_EQ(0, c->index());
    EXPECT_EQ("localhost", static_cast<SetOSCOutputHostCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_OSC_OUTPUT_PORT, c->type());
    EXPECT_EQ(0, c->index());
    EXPECT_EQ(23023, static_cast<SetOSCOutputPortCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_OSC_OUTPUT_BASE_PATH, c->type());
    EXPECT_EQ(0, c->index());
    EXPECT_EQ("/sensei/sensors", static_cast<SetOSCOutputBasePathCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_OSC_OUTPUT_RAW_PATH, c->type());
    EXPECT_EQ(0 , c->index());
    EXPECT_EQ("/sensei/raw_input", static_cast<SetOSCOutputRawPathCommand*>(c)->data());


    /* And now the sensors, first an analog input configuration */
    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_PIN_TYPE, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_EQ(PinType::ANALOG_INPUT, static_cast<SetPinTypeCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_ENABLED, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_EQ(true, static_cast<SetEnabledCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_SENDING_MODE, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_EQ(SendingMode::CONTINUOUS, static_cast<SetSendingModeCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_SENDING_DELTA_TICKS, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_EQ(4, static_cast<SetSendingDeltaTicksCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_ADC_BIT_RESOLUTION, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_EQ(8, static_cast<SetADCBitResolutionCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_LOWPASS_CUTOFF, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_FLOAT_EQ(300, static_cast<SetLowpassCutoffCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_LOWPASS_FILTER_ORDER, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_EQ(4, static_cast<SetLowpassFilterOrderCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_SLIDER_MODE_ENABLED, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_FALSE(static_cast<SetSliderModeEnabledCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_SLIDER_THRESHOLD, c->type());
    EXPECT_EQ(1, c->index());
    EXPECT_EQ(10, static_cast<SetSliderThresholdCommand*>(c)->data());

    /* Then a pin configured as a digital input */
    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_PIN_TYPE, c->type());
    EXPECT_EQ(0, c->index());
    EXPECT_EQ(PinType::DIGITAL_INPUT, static_cast<SetPinTypeCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_ENABLED, c->type());
    EXPECT_EQ(0, c->index());
    EXPECT_EQ(true, static_cast<SetEnabledCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_SENDING_MODE, c->type());
    EXPECT_EQ(0, c->index());
    EXPECT_EQ(SendingMode::ON_VALUE_CHANGED, static_cast<SetSendingModeCommand*>(c)->data());

    m = std::move(_queue.pop());
    c = static_cast<Command*>(m.get());
    ASSERT_EQ(CommandType::SET_SENDING_DELTA_TICKS, c->type());
    EXPECT_EQ(0, c->index());
    EXPECT_EQ(4, static_cast<SetSendingDeltaTicksCommand*>(c)->data());
}
