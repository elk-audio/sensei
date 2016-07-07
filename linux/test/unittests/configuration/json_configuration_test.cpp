#include <json/value.h>
#include <sensei_serial_protocol.h>
#include "gtest/gtest.h"

#define private public
#include "config_backend/json_configuration.cpp"

using namespace sensei;
using namespace config;

static const std::string test_file = "../../../test/unittests/configuration/test_configuration.json";

/* Macro to reduce the footprint when verifying a single command */
#define EXPECT_COMMAND(message, commandtype, commandclass, id, expected_value) { \
    Command* c = static_cast<Command *>(message.get()); \
    ASSERT_EQ(commandtype, c->type()); \
    EXPECT_EQ(id, c->index());  \
    EXPECT_EQ(expected_value, static_cast<commandclass *>(c)->data()); }


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

TEST_F(JsonConfigurationTest, test_invalid_file)
{
    JsonConfiguration test_module(&_queue, "/non/existing/file.json");
    EXPECT_EQ(ConfigStatus::IO_ERROR, test_module.read());
}

/*
 * Parse a json config file and verify that all commands were created correctly
 */
TEST_F(JsonConfigurationTest, test_read_configuration)
{
    EXPECT_TRUE(_queue.empty());
    ConfigStatus status = _module_under_test.read();
    EXPECT_EQ(ConfigStatus::OK, status);
    EXPECT_FALSE(_queue.empty());

    /* Now verify the commands one by one */
    /* First we should receive the backend related commands */
    std::unique_ptr<BaseMessage> m = std::move(_queue.pop());
    int index = 0;
    EXPECT_COMMAND(m, CommandType::ENABLE_SENDING_PACKETS, EnableSendingPacketsCommand, index, (int)false);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_OUTPUT_ENABLED, SetSendOutputEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_RAW_INPUT_ENABLED, SetSendRawInputEnabledCommand, index, (int)false);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_OSC_OUTPUT_HOST, SetOSCOutputHostCommand, index, "localhost");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_OSC_OUTPUT_PORT, SetOSCOutputPortCommand, index, 23023);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_OSC_OUTPUT_BASE_PATH, SetOSCOutputBasePathCommand, index, "/sensei/sensors");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_OSC_OUTPUT_RAW_PATH, SetOSCOutputRawPathCommand, index, "/sensei/raw_input");

    /* stdout backend */
    index = 1;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_OUTPUT_ENABLED, SetSendOutputEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_RAW_INPUT_ENABLED, SetSendRawInputEnabledCommand, index, (int)true);


    /* And now the sensors, first an analog input configuration */
    /* HW setup */
    index = 1;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_PIN_NAME, SetPinNameCommand, index, "ribbon_1");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_PIN_TYPE, SetPinTypeCommand, index, PinType::ANALOG_INPUT);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_MODE, SetSendingModeCommand, index, SendingMode::CONTINUOUS);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_DELTA_TICKS, SetSendingDeltaTicksCommand, index, 4);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ADC_BIT_RESOLUTION, SetADCBitResolutionCommand, index, 8);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_LOWPASS_CUTOFF, SetLowpassCutoffCommand, index, 300);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_LOWPASS_FILTER_ORDER, SetLowpassFilterOrderCommand, index, 4);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SLIDER_MODE_ENABLED, SetSliderModeEnabledCommand, index, (int)false)
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SLIDER_THRESHOLD, SetSliderThresholdCommand, index, 10);

    /* Mapping configuration */
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INVERT_ENABLED, SetInvertEnabledCommand, index, (int)false);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_SCALE_RANGE_LOW, SetInputScaleRangeLow, index, 10);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_SCALE_RANGE_HIGH, SetInputScaleRangeHigh, index, 240);
    m = std::move(_queue.pop());

    /* Then a pin configured as a digital input */
    /* HW setup */
    index = 0;
    EXPECT_COMMAND(m, CommandType::SET_PIN_NAME, SetPinNameCommand, index, "button_0");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_PIN_TYPE, SetPinTypeCommand, index, PinType::DIGITAL_INPUT);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_MODE, SetSendingModeCommand, index, SendingMode::ON_VALUE_CHANGED);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_DELTA_TICKS, SetSendingDeltaTicksCommand, index, 4);

    /* The a virtual pin to use for IMU data */
    index = 61;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_PIN_NAME, SetPinNameCommand, index, "yaw");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_PIN_TYPE, SetPinTypeCommand, index, PinType::IMU_INPUT);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_VIRTUAL_PIN, SetVirtualPinCommand, index, ImuIndex::YAW);

    /* Now we should be receiving the IMU related commands */
    index = 0;
     m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_FILTER_MODE, SetImuFilterModeCommand, index, 1);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_ACC_RANGE_MAX, SetImuAccelerometerRangeMaxCommand, index, 4);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_GYRO_RANGE_MAX, SetImuGyroscopeRangeMaxCommand, index, 500);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_COMPASS_RANGE_MAX, SetImuCompassRangeMaxCommand, index, 5);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_COMPASS_ENABLED, SetImuCompassEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_SENDING_MODE, SetImuSendingModeCommand, index, SendingMode::ON_VALUE_CHANGED);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_SENDING_DELTA_TICKS, SetImuSendingDeltaTicksCommand, index, 5);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_DATA_MODE, SetImuDataModeCommand, index, IMU_GET_QUATERNIONS);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_ACC_THRESHOLD, SetImuAccThresholdCommand, index, 1);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_IMU_ENABLED, SetImuEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::IMU_CALIBRATE, ImuCalibrateCommand, index, 0);

/* Lastly we should have an EnableSendingPackets command to turn on all pins */
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::ENABLE_SENDING_PACKETS, EnableSendingPacketsCommand, index, (int)true);
}
