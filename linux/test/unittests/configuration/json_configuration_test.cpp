#include <json/value.h>
#include <iostream>
#include "gtest/gtest.h"

#include "gpio_protocol/gpio_protocol.h"

#define private public
#include "config_backend/json_configuration.cpp"
#include "../test_utils.h"

using namespace sensei;
using namespace config;

static const std::string test_file = "../../../test/unittests/configuration/test_configuration.json";

/* Macro to reduce the footprint when verifying a single command */
#define EXPECT_COMMAND(message, commandtype, commandclass, id, expected_value) { \
    Command* c = static_cast<Command *>(message.get()); \
    ASSERT_EQ(commandtype, c->type()); \
    EXPECT_EQ(id, c->index()); \
    EXPECT_EQ(expected_value, static_cast<commandclass *>(c)->data()); }

#define EXPECT_COMMAND_FLOAT(message, commandtype, commandclass, id, expected_value) { \
    Command* c = static_cast<Command *>(message.get()); \
    ASSERT_EQ(commandtype, c->type()); \
    EXPECT_FLOAT_EQ(id, c->index()); \
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
    HwFrontendConfig hw_config;
    EXPECT_EQ(ConfigStatus::IO_ERROR, test_module.read(hw_config));
}

/*
 * Parse a json config file and verify that all commands were created correctly
 */
TEST_F(JsonConfigurationTest, test_read_configuration)
{
    EXPECT_TRUE(_queue.empty());
    HwFrontendConfig hw_frontend;
    ConfigStatus status = _module_under_test.read(hw_frontend);
    EXPECT_EQ(ConfigStatus::OK, status);
    EXPECT_FALSE(_queue.empty());

    EXPECT_EQ(HwFrontendType::RASPA_GPIO, hw_frontend.type);

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

    /**
     * And now the sensors, first a digital output configuration. An LED connected to pin 3
     */
    index = 1;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "led");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::DIGITAL_OUTPUT);

    // HW specific configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::DIGITAL_OUTPUT_PIN);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, std::vector<int>{3});

    // Mapping configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int)false);

    /**
     * A mux controller to control two led rings.
     */
    index = 2;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "led_ring_mux");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::NO_OUTPUT);

    // HW specific configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::MULTIPLEXER);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{24, 25}));

    // Mapping configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int)false);

    /**
     * LED Ring one connected to pins 8,9,10,11,12,13 controlled by mux with pin 24
     */
    index = 3;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "led_ring_1");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::ANALOG_OUTPUT);

    // HW specific configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::STEPPED_OUTPUT);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{8, 9, 10, 11, 12, 13}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_MULTIPLEXED, SetMultiplexedSensorCommand, index, (MultiplexerData{2, 24}));

    // Mapping configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_RANGE, SetInputRangeCommand, index, (Range{0.0, 6.0}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int)false);

    /**
     * LED Ring two connected to pins 8,9,10,11,12,13 controlled by mux with pin 25
     */
    index = 4;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "led_ring_2");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::ANALOG_OUTPUT);

    // HW specific configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::STEPPED_OUTPUT);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{8, 9, 10, 11, 12, 13}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_MULTIPLEXED, SetMultiplexedSensorCommand, index, (MultiplexerData{2, 25}));

    // Mapping configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_RANGE, SetInputRangeCommand, index, (Range{0.0, 6.0}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int)false);

    /**
     * A rotary encoder connectected to pins 23 and 22 (in that order).
     */
    index = 5;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "rot_enc");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::ANALOG_INPUT);

    // HW specific configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::ENCODER);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{23, 22}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_DELTA_TICKS, SetSendingDeltaTicksCommand, index, 1);

    // Mapping configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_MODE, SetSendingModeCommand, index, SendingMode::CONTINUOUS);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_RANGE, SetInputRangeCommand, index, (Range{0.0, 15.0}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int)false);

    /**
     * A four way switch connectected to pins 12,13,14,15.
     */
    index = 6;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "rot_sw");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::RANGE_INPUT);

    // HW specific configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::N_WAY_SWITCH);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{12, 13, 14, 15}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_DELTA_TICKS, SetSendingDeltaTicksCommand, index, 1);

    // Mapping configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_MODE, SetSendingModeCommand, index, SendingMode::ON_VALUE_CHANGED);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INVERT_ENABLED, SetInvertEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_RANGE, SetInputRangeCommand, index, (Range{1.0, 4.0}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int)false);

    /**
     * An active low button connected to pin 21.
     */
    index = 7;
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "button");
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::DIGITAL_INPUT);

    // HW specific configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::DIGITAL_INPUT_PIN);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{21}));
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_DELTA_TICKS, SetSendingDeltaTicksCommand, index, 1);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_POLARITY, SetSensorHwPolarityCommand, index, HwPolarity::ACTIVE_LOW);

    // Mapping configuration
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_MODE, SetSendingModeCommand, index, SendingMode::ON_VALUE_CHANGED);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INVERT_ENABLED, SetInvertEnabledCommand, index, (int)true);
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int)true);

/*  Lastly we should have an EnableSendingPackets command to turn on all pins */
    m = std::move(_queue.pop());
    EXPECT_COMMAND(m, CommandType::ENABLE_SENDING_PACKETS, EnableSendingPacketsCommand, 0, (int)true);
}
