#include <iostream>
#include "gtest/gtest.h"


#include "gpio_protocol/gpio_protocol.h"
#include "logging.h"

#define private public
#include "config_backend/json_configuration.cpp"
#include "../test_utils.h"

using namespace sensei;
using namespace config;

static const char* test_config_json =
#include "test_configuration.json"
        ;

/* Macro to reduce the footprint when verifying a single command */
#define EXPECT_COMMAND(message, commandtype, commandclass, id, expected_value) \
    { \
        Command* c = static_cast<Command*>(message.get()); \
        ASSERT_EQ(commandtype, c->type()); \
        EXPECT_EQ(id, c->index()); \
        EXPECT_EQ(expected_value, static_cast<commandclass*>(c)->data()); \
    }

#define EXPECT_COMMAND_FLOAT(message, commandtype, commandclass, id, expected_value) \
    { \
        Command* c = static_cast<Command*>(message.get()); \
        ASSERT_EQ(commandtype, c->type()); \
        EXPECT_FLOAT_EQ(id, c->index()); \
        EXPECT_EQ(expected_value, static_cast<commandclass*>(c)->data()); \
    }


class JsonConfigurationTest : public ::testing::Test
{
protected:
    JsonConfigurationTest()
        : _module_under_test(&_handler, "")
    {
    }
    void SetUp() override
    {
    }
    MessageHandlerMock _handler;
    JsonConfiguration  _module_under_test;
};

TEST_F(JsonConfigurationTest, test_invalid_file)
{
    JsonConfiguration test_module(&_handler, "/non/existing/file.json");
    Config            config;
    EXPECT_EQ(ConfigStatus::IO_ERROR, test_module.read(config));
}

/*
 * Parse a json config file and verify that all commands were created correctly
 */
TEST_F(JsonConfigurationTest, test_read_configuration)
{
    SENSEI_LOG_INFO("Starting test_read_configuration");
    EXPECT_TRUE(_handler.event_queue.empty());
    Config       config;
    ConfigStatus status = _module_under_test.read_from_string(config, test_config_json);
    ASSERT_EQ(ConfigStatus::OK, status);
    ASSERT_FALSE(_handler.event_queue.empty());

    EXPECT_EQ(HwFrontendType::RASPA_GPIO, config.hw_config.type);

    /* Now verify the commands one by one */
    /* First we should receive the backend related commands */
    std::unique_ptr<BaseMessage> m = std::move(_handler.event_queue.pop());
    int                          index = 0;
    EXPECT_COMMAND(m, CommandType::ENABLE_SENDING_PACKETS, EnableSendingPacketsCommand, index, (int) false);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_OUTPUT_ENABLED, SetSendOutputEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_RAW_INPUT_ENABLED, SetSendRawInputEnabledCommand, index, (int) false);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_OSC_OUTPUT_HOST, SetOSCOutputHostCommand, index, "localhost");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_OSC_OUTPUT_PORT, SetOSCOutputPortCommand, index, 23023);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_OSC_OUTPUT_BASE_PATH, SetOSCOutputBasePathCommand, index, "/sensei/sensors");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_OSC_OUTPUT_RAW_PATH, SetOSCOutputRawPathCommand, index, "/sensei/raw_input");

    /* Backend enabled */
    index = 1;
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_OUTPUT_ENABLED, SetSendOutputEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_RAW_INPUT_ENABLED, SetSendRawInputEnabledCommand, index, (int) true);

    /**
     * And now the sensors, first a digital output configuration. An LED connected to pin 3
     */
    index = 1;
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "led");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::DIGITAL_OUTPUT);

    // HW specific configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::DIGITAL_OUTPUT_PIN);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, std::vector<int>{3});
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_POLARITY, SetSensorHwPolarityCommand, index, HwPolarity::ACTIVE_HIGH);

    // Sensor configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INVERT_ENABLED, SetInvertEnabledCommand, index, (int) false);

    /**
     * A mux controller to control two led rings.
     */
    index = 2;
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "led_ring_mux");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::NO_OUTPUT);

    // HW specific configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::MULTIPLEXER);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{24, 25}));

    // Sensor configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int) true);
    // NO_OUTPUT has no type-specific commands

    /**
     * LED Ring one connected to pins 8,9,10,11,12,13 controlled by mux with pin 24
     */
    index = 3;
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "led_ring_1");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::ANALOG_OUTPUT);

    // HW specific configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::STEPPED_OUTPUT);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{8, 9, 10, 11, 12, 13}));
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_MULTIPLEXED, SetMultiplexedSensorCommand, index, (MultiplexerData{2, 24}));

    // Sensor configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_RANGE, SetInputRangeCommand, index, (Range{0.0, 6.0}));

    /**
     * LED Ring two connected to pins 8,9,10,11,12,13 controlled by mux with pin 25
     */
    index = 4;
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "led_ring_2");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::ANALOG_OUTPUT);

    // HW specific configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::STEPPED_OUTPUT);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{8, 9, 10, 11, 12, 13}));
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_MULTIPLEXED, SetMultiplexedSensorCommand, index, (MultiplexerData{2, 25}));

    // Sensor configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_RANGE, SetInputRangeCommand, index, (Range{0.0, 6.0}));

    /**
     * A rotary encoder connectected to pins 23 and 22 (in that order).
     */
    index = 5;
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "rot_enc");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::ANALOG_INPUT);

    // HW specific configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::ENCODER);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{23, 22}));
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_POLARITY, SetSensorHwPolarityCommand, index, HwPolarity::ACTIVE_LOW);

    // Sensor configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_MODE, SetSendingModeCommand, index, SendingMode::CONTINUOUS);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INVERT_ENABLED, SetInvertEnabledCommand, index, (int) false);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int) false);

    /**
     * A four way switch connectected to pins 12,13,14,15.
     */
    index = 6;
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "rot_sw");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::RANGE_INPUT);

    // HW specific configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::N_WAY_SWITCH);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{12, 13, 14, 15}));
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_DELTA_TICKS, SetSendingDeltaTicksCommand, index, 1);

    // Mapping configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_MODE, SetSendingModeCommand, index, SendingMode::ON_VALUE_CHANGED);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INVERT_ENABLED, SetInvertEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INPUT_RANGE, SetInputRangeCommand, index, (Range{1.0, 4.0}));
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int) false);

    /**
     * An active low button connected to pin 21.
     */
    index = 7;
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_NAME, SetPinNameCommand, index, "button");
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_TYPE, SetSensorTypeCommand, index, SensorType::DIGITAL_INPUT);

    // HW specific configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENSOR_HW_TYPE, SetSensorHwTypeCommand, index, SensorHwType::DIGITAL_INPUT_PIN);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_PINS, SetHwPinsCommand, index, (std::vector<int>{21}));
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_DELTA_TICKS, SetSendingDeltaTicksCommand, index, 1);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_HW_POLARITY, SetSensorHwPolarityCommand, index, HwPolarity::ACTIVE_LOW);

    // Mapping configuration
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_ENABLED, SetEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SENDING_MODE, SetSendingModeCommand, index, SendingMode::ON_VALUE_CHANGED);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_INVERT_ENABLED, SetInvertEnabledCommand, index, (int) true);
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::SET_SEND_TIMESTAMP_ENABLED, SetSendTimestampEnabledCommand, index, (int) true);

    /*  Lastly we should have an EnableSendingPackets command to turn on all pins */
    m = std::move(_handler.event_queue.pop());
    EXPECT_COMMAND(m, CommandType::ENABLE_SENDING_PACKETS, EnableSendingPacketsCommand, 0, (int) true);
}

/*
 * Test that RapidJSON schema validation catches constraint violations
 */
TEST_F(JsonConfigurationTest, test_schema_validation)
{
    Config config;

    // Test 1: Empty sensors array (must have minItems >= 1) - schema validation catches this
    const char* empty_sensors = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": []
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, empty_sensors));

    // Test 2: ADC resolution = 0 (must be >= 1) - schema validation catches this
    const char* invalid_adc = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test",
            "config": {
                "sensor_type": "analog_input",
                "mode": "continuous",
                "range": [0, 100],
                "inverted": false,
                "timestamp": false
            },
            "hardware": {
                "hardware_type": "analog_input_pin",
                "pins": [1],
                "delta_ticks": 1,
                "adc_resolution": 0,
                "filter_time_constant": 0.5
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, invalid_adc));

    // Test 3: Filter time constant negative (must be >= 0.0) - schema validation catches this
    const char* invalid_filter = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test",
            "config": {
                "sensor_type": "analog_input",
                "mode": "continuous",
                "range": [0, 100],
                "inverted": false,
                "timestamp": false
            },
            "hardware": {
                "hardware_type": "analog_input_pin",
                "pins": [1],
                "delta_ticks": 1,
                "adc_resolution": 12,
                "filter_time_constant": -1.0
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, invalid_filter));

    // Test 4: Delta ticks = 0 (must be >= 1) - schema validation catches this
    const char* invalid_delta = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test",
            "config": {
                "sensor_type": "digital_input",
                "mode": "on_value_changed",
                "inverted": false,
                "timestamp": false
            },
            "hardware": {
                "hardware_type": "digital_input_pin",
                "polarity": "active_high",
                "pins": [1],
                "delta_ticks": 0
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, invalid_delta));

    // Test 5: Empty pins array (must have length >= 1) - schema validation catches this
    const char* empty_pins = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test",
            "config": {
                "sensor_type": "digital_output",
                "inverted": false
            },
            "hardware": {
                "hardware_type": "digital_output_pin",
                "polarity": "active_high",
                "pins": []
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, empty_pins));

    // Test 6: Encoder with wrong number of pins (must be exactly 2) - schema validation catches this
    const char* wrong_pin_count = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test",
            "config": {
                "sensor_type": "analog_input",
                "mode": "continuous",
                "range": [0, 100],
                "inverted": false,
                "timestamp": false
            },
            "hardware": {
                "hardware_type": "encoder",
                "polarity": "active_low",
                "pins": [1, 2, 3]
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, wrong_pin_count));

    // Test 7: Empty sensor name (must have length >= 1) - schema validation catches this
    const char* empty_name = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "",
            "config": {
                "sensor_type": "digital_output",
                "inverted": false
            },
            "hardware": {
                "hardware_type": "digital_output_pin",
                "polarity": "active_high",
                "pins": [1]
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, empty_name));

    // Test 8: Unknown sensor type - schema validation catches this
    const char* unknown_sensor_type = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test_sensor",
            "config": {
                "sensor_type": "unknown_type",
                "inverted": false
            },
            "hardware": {
                "hardware_type": "digital_output_pin",
                "polarity": "active_high",
                "pins": [1]
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, unknown_sensor_type));

    // Test 9: Unknown hardware type - schema validation catches this
    const char* unknown_hardware_type = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test_sensor",
            "config": {
                "sensor_type": "digital_output",
                "inverted": false
            },
            "hardware": {
                "hardware_type": "unknown_hardware",
                "polarity": "active_high",
                "pins": [1]
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, unknown_hardware_type));

    // Test 10: Empty backends array (must have minItems >= 1)
    const char* empty_backends = R"({
        "backends": [],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test_sensor",
            "config": {
                "sensor_type": "digital_output",
                "inverted": false
            },
            "hardware": {
                "hardware_type": "digital_output_pin",
                "polarity": "active_high",
                "pins": [1]
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, empty_backends));

    // Test 11: Invalid mode value
    const char* invalid_mode = R"({
        "backends": [{
            "id": 0,
            "enabled": true,
            "raw_input_enabled": false,
            "config": { "type": "stdout" }
        }],
        "hw_frontend": { "type": "raspa_gpio" },
        "sensors": [{
            "id": 1,
            "enabled": true,
            "name": "test_sensor",
            "config": {
                "sensor_type": "digital_input",
                "mode": "invalid_mode",
                "inverted": false,
                "timestamp": false
            },
            "hardware": {
                "hardware_type": "digital_input_pin",
                "polarity": "active_high",
                "pins": [1],
                "delta_ticks": 1
            }
        }]
    })";
    EXPECT_EQ(ConfigStatus::SCHEMA_VALIDATION_ERROR, _module_under_test.read_from_string(config, invalid_mode));


}
