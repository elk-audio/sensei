/*
 * Copyright 2017-2026 Elk Audio AB
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief Configuration Class for importing configuration from a JSON file
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 */
#include <cstdint>
#include <fstream>
#include <string_view>
#include <array>

#include "config_backend/base_configuration.h"
#include "elk-warning-suppressor/warning_suppressor.hpp"
#include "message/command_defs.h"

ELK_PUSH_WARNING
ELK_DISABLE_NAN_INFINITY_DISABLED
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/schema.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/pointer.h"
ELK_POP_WARNING

#include "json_configuration.h"
#include "logging.h"


namespace sensei {
namespace config {

SENSEI_GET_LOGGER_WITH_MODULE_NAME("config");

bool is_config_valid_for_schema(rapidjson::Document& config)
{
    static const char* schema_json =
#include "sensei_config_schema.json"
            ;

    rapidjson::Document schema;
    schema.Parse(schema_json);
    if (schema.HasParseError())
    {
        SENSEI_LOG_ERROR("Error parsing embedded schema at offset {}: {}",
                         schema.GetErrorOffset(),
                         rapidjson::GetParseError_En(schema.GetParseError()));
        return false;
    }

    rapidjson::SchemaDocument  schema_document(schema);
    rapidjson::SchemaValidator schema_validator(schema_document);

    /* Validate config against schema */
    if (!config.Accept(schema_validator))
    {
        rapidjson::Pointer      invalid_pointer = schema_validator.GetInvalidDocumentPointer();
        rapidjson::StringBuffer string_buffer;
        invalid_pointer.Stringify(string_buffer);
        std::string error_location = string_buffer.GetString();

        rapidjson::StringBuffer schema_buffer;
        schema_validator.GetInvalidSchemaPointer().Stringify(schema_buffer);
        std::string schema_location = schema_buffer.GetString();

        SENSEI_LOG_ERROR(
                "Schema validation failure:\n"
                "  Location in config: {}\n"
                "  Schema location: {}\n"
                "  Schema keyword: {}",
                error_location,
                schema_location,
                schema_validator.GetInvalidSchemaKeyword());

        return false;
    }

    SENSEI_LOG_INFO("Configuration validated successfully against schema");
    return true;
}


/*
 * Open _source as a file and parse is as a json file
 */
ConfigStatus JsonConfiguration::read(Config& config)
{
    SENSEI_LOG_INFO("Reading configuration file");
    std::ifstream file(_source);
    if (!file.good())
    {
        SENSEI_LOG_ERROR("Couldn't open JSON configuration file: {}", _source);
        return ConfigStatus::IO_ERROR;
    }

    std::string json_string((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
    return read_from_string(config, json_string.c_str());
}

/**
 * @throws std::invalid_argument if hardware frontend type is not recognized
 */
HwFrontendType hw_frontend_from_string(std::string_view type_str)
{
    static constexpr auto hw_frontend_map = std::to_array<std::pair<std::string_view, HwFrontendType>>(
            {{"raspa_xmos", HwFrontendType::RASPA_GPIO},
             {"raspa_gpio", HwFrontendType::RASPA_GPIO},
             {"elk_pi", HwFrontendType::ELK_PI_GPIO}});

    for (const auto& [name, value] : hw_frontend_map)
    {
        if (name == type_str)
            return value;
    }
    throw std::invalid_argument("Unknown hardware frontend type: " + std::string(type_str));
}

/*
 * Parse JSON from string (for testing with embedded configs)
 */
ConfigStatus JsonConfiguration::read_from_string(Config& config, const char* json_string)
{

    rapidjson::Document json_config;
    json_config.Parse(json_string);
    if (json_config.HasParseError())
    {
        SENSEI_LOG_ERROR("Error parsing JSON configuration at offset {}: {}",
                         json_config.GetErrorOffset(),
                         rapidjson::GetParseError_En(json_config.GetParseError()));
        return ConfigStatus::PARSING_ERROR;
    }


    if (!is_config_valid_for_schema(json_config))
    {
        SENSEI_LOG_ERROR("Configuration failed schema validation");
        return ConfigStatus::SCHEMA_VALIDATION_ERROR;
    }

    /* Start by disabling all pins to mute the board while sending the configuration commands */
    _handler->post_event(_message_factory.make_enable_sending_packets_command(0, false));

    /* Extract fields from the JSON config and generate commands - schema
     * guarantees all required fields exist with correct types. */
    try
    {

        config.hw_config.type = hw_frontend_from_string(json_config["hw_frontend"]["type"].GetString());
        const rapidjson::Value& backends = json_config["backends"];
        for (const auto& backend : backends.GetArray())
        {
            handle_backend_config(backend, config.backend_config);
        }

        const rapidjson::Value& sensors = json_config["sensors"];
        for (const auto& sensor : sensors.GetArray())
        {
            handle_sensor(sensor);
        }
    } catch (const std::exception& e)
    {
        SENSEI_LOG_ERROR("Parsing error: {}", e.what());
        return ConfigStatus::PARSING_ERROR;
    }

    /* The last commands enables sending of packets */
    _handler->post_event(_message_factory.make_enable_sending_packets_command(0, true));
    return ConfigStatus::OK;
}


/**
 * @throws std::invalid_argument if sensor type is not recognized
 */
SensorType sensor_type_from_string(std::string_view sensor_type_str)
{
    static constexpr auto sensor_type_map = std::to_array<std::pair<std::string_view, SensorType>>(
            {{"analog_input", SensorType::ANALOG_INPUT},
             {"analog_output", SensorType::ANALOG_OUTPUT},
             {"digital_input", SensorType::DIGITAL_INPUT},
             {"discrete_input", SensorType::DISCRETE_INPUT},
             {"continuous_input", SensorType::CONTINUOUS_INPUT},
             {"continuous_output", SensorType::CONTINUOUS_OUTPUT},
             {"digital_output", SensorType::DIGITAL_OUTPUT},
             {"range_input", SensorType::RANGE_INPUT},
             {"range_output", SensorType::RANGE_OUTPUT},
             {"no_output", SensorType::NO_OUTPUT},
             {"relative_input", SensorType::RELATIVE_INPUT}});

    for (auto const& [name, value] : sensor_type_map)
    {
        if (name == sensor_type_str)
            return value;
    }
    throw std::invalid_argument("Unrecognized sensor type: \"" + std::string(sensor_type_str) + "\"");
}

/**
 * @throws std::invalid_argument if sending mode is not recognized
 */
SendingMode mode_from_string(std::string_view mode_str)
{
    static constexpr auto sending_mode_map = std::to_array<std::pair<std::string_view, SendingMode>>(
            {{"off", SendingMode::OFF},
             {"continuous", SendingMode::CONTINUOUS},
             {"on_value_changed", SendingMode::ON_VALUE_CHANGED},
             {"on_request", SendingMode::ON_REQUEST},
             {"on_press", SendingMode::ON_PRESS},
             {"on_release", SendingMode::ON_RELEASE}});

    for (auto const& [name, value] : sending_mode_map)
    {
        if (name == mode_str)
            return value;
    }
    throw std::invalid_argument("Unknown sending mode: " + std::string(mode_str));
}


/*
 * Parse sensor config and generate commands - schema guarantees the fields
 * exist, additional validation is performed according to the schema
 * @throws std::invalid_argument if sensor type is unknown
 */
void handle_sensor_config(const rapidjson::Value& config, SensorType sensor_type,
                          int sensor_id, MessageFactory& factory, MessageHandler* handler)
{
    switch (sensor_type)
    {
        case SensorType::DIGITAL_OUTPUT:
        {
            // DigitalOutput { inverted }
            handler->post_event(factory.make_set_invert_enabled_command(sensor_id, config["inverted"].GetBool()));
            break;
        }

        case SensorType::DIGITAL_INPUT:
        {
            // DigitalInput { mode, inverted, timestamp }
            handler->post_event(factory.make_set_sending_mode_command(sensor_id, mode_from_string(config["mode"].GetString())));
            handler->post_event(factory.make_set_invert_enabled_command(sensor_id, config["inverted"].GetBool()));
            handler->post_event(factory.make_set_send_timestamp_enabled(sensor_id, config["timestamp"].GetBool()));
            break;
        }

        case SensorType::DISCRETE_INPUT:
        {
            // DiscreteInput { mode, discrete_ranges, inverted, timestamp }
            const rapidjson::Value& discrete_ranges = config["discrete_ranges"];
            std::vector<Range>      ranges;
            for (const auto& range_array : discrete_ranges.GetArray())
            {
                ranges.push_back({range_array[0].GetFloat(), range_array[1].GetFloat()});
            }
            handler->post_event(factory.make_set_sending_mode_command(sensor_id, mode_from_string(config["mode"].GetString())));
            handler->post_event(factory.make_set_discrete_ranges_command(sensor_id, ranges));
            handler->post_event(factory.make_set_invert_enabled_command(sensor_id, config["inverted"].GetBool()));
            handler->post_event(factory.make_set_send_timestamp_enabled(sensor_id, config["timestamp"].GetBool()));
            break;
        }

        case SensorType::ANALOG_INPUT:
        {
            // AnalogInput { mode, inverted, timestamp }
            handler->post_event(factory.make_set_sending_mode_command(sensor_id, mode_from_string(config["mode"].GetString())));
            handler->post_event(factory.make_set_invert_enabled_command(sensor_id, config["inverted"].GetBool()));
            handler->post_event(factory.make_set_send_timestamp_enabled(sensor_id, config["timestamp"].GetBool()));
            break;
        }

        case SensorType::ANALOG_OUTPUT:
        {
            // AnalogOutput { range }
            const rapidjson::Value& range = config["range"];
            handler->post_event(factory.make_set_input_range_command(sensor_id, range[0].GetFloat(), range[1].GetFloat()));
            break;
        }

        case SensorType::CONTINUOUS_INPUT:
        {
            // ContinuousInput { mode, range, inverted, timestamp }
            const rapidjson::Value& range = config["range"];
            handler->post_event(factory.make_set_sending_mode_command(sensor_id, mode_from_string(config["mode"].GetString())));
            handler->post_event(factory.make_set_invert_enabled_command(sensor_id, config["inverted"].GetBool()));
            handler->post_event(factory.make_set_input_range_command(sensor_id, range[0].GetFloat(), range[1].GetFloat()));
            handler->post_event(factory.make_set_send_timestamp_enabled(sensor_id, config["timestamp"].GetBool()));
            break;
        }

        case SensorType::CONTINUOUS_OUTPUT:
        {
            // ContinuousOutput { range }
            const rapidjson::Value& range = config["range"];
            handler->post_event(factory.make_set_input_range_command(sensor_id, range[0].GetFloat(), range[1].GetFloat()));
            break;
        }

        case SensorType::RANGE_INPUT:
        {
            // RangeInput { mode, range, inverted, timestamp }
            const rapidjson::Value& range = config["range"];
            handler->post_event(factory.make_set_sending_mode_command(sensor_id, mode_from_string(config["mode"].GetString())));
            handler->post_event(factory.make_set_invert_enabled_command(sensor_id, config["inverted"].GetBool()));
            handler->post_event(factory.make_set_input_range_command(sensor_id, range[0].GetFloat(), range[1].GetFloat()));
            handler->post_event(factory.make_set_send_timestamp_enabled(sensor_id, config["timestamp"].GetBool()));
            break;
        }

        case SensorType::RANGE_OUTPUT:
        {
            // RangeOutput { range }
            const rapidjson::Value& range = config["range"];
            handler->post_event(factory.make_set_input_range_command(sensor_id, range[0].GetFloat(), range[1].GetFloat()));
            break;
        }

        case SensorType::NO_OUTPUT:
            // NoOutput - no additional commands
            break;

        case SensorType::RELATIVE_INPUT:
        {
            // RelativeInput { mode, inverted, timestamp }
            handler->post_event(factory.make_set_sending_mode_command(sensor_id, mode_from_string(config["mode"].GetString())));
            handler->post_event(factory.make_set_invert_enabled_command(sensor_id, config["inverted"].GetBool()));
            handler->post_event(factory.make_set_send_timestamp_enabled(sensor_id, config["timestamp"].GetBool()));
            break;
        }

        default:
            // Should be unreachable
            throw std::invalid_argument("Unknown sensor type: " + std::to_string(static_cast<int>(sensor_type)));
    }
}

/*
 * Read all existing configuration keys for a single sensor. And create and
 * queue configuration commands from them. 
 * 
 * Each sensor type may have its own specific configuration keys which are parsed in
 * handle_sensor_config_commands. The schema guarantees that all required fields
 * exist.
 *
 * @throws std::invalid_argument if sensor name is empty or configuration
 * validation fails
 */
void JsonConfiguration::handle_sensor(const rapidjson::Value& sensor)
{
    int sensor_id = sensor["id"].GetInt();

    /* read and validate sensor name */
    std::string sensor_name = sensor["name"].GetString();
    auto        msg_name_command = _message_factory.make_set_sensor_name_command(sensor_id, sensor_name);
    _handler->post_event(std::move(msg_name_command));

    /* Read sensor type configuration from nested config object */
    const rapidjson::Value&      config = sensor["config"];
    std::unique_ptr<BaseMessage> msg = nullptr;
    const std::string            sensor_type_str = config["sensor_type"].GetString();
    const SensorType             sensor_type = sensor_type_from_string(sensor_type_str);
    msg = _message_factory.make_set_sensor_type_command(sensor_id, sensor_type);
    _handler->post_event(std::move(msg));

    const rapidjson::Value& hw_config = sensor["hardware"];
    handle_sensor_hw_config(hw_config, sensor_id);
    auto msg_enabled_command = _message_factory.make_set_enabled_command(sensor_id, sensor["enabled"].GetBool());
    _handler->post_event(std::move(msg_enabled_command));

    /* Parse sensor config and generate type-specific commands */
    handle_sensor_config(config, sensor_type, sensor_id, _message_factory, _handler);
}


/**
 * @throws std::invalid_argument if sensor hardware type is not recognized
 */
SensorHwType parse_hw_type_string(std::string_view hw_type_str)
{
    static constexpr auto hw_sensor_type_map = std::to_array<std::pair<std::string_view, SensorHwType>>(
            {{"analog_input_pin", SensorHwType::ANALOG_INPUT_PIN},
             {"digital_input_pin", SensorHwType::DIGITAL_INPUT_PIN},
             {"digital_output_pin", SensorHwType::DIGITAL_OUTPUT_PIN},
             {"ribbon", SensorHwType::RIBBON},
             {"button", SensorHwType::BUTTON},
             {"encoder", SensorHwType::ENCODER},
             {"n_way_switch", SensorHwType::N_WAY_SWITCH},
             {"stepped_output", SensorHwType::STEPPED_OUTPUT},
             {"multiplexer", SensorHwType::MULTIPLEXER}});

    for (auto const& [name, value] : hw_sensor_type_map)
    {
        if (name == hw_type_str)
            return value;
    }

    throw std::invalid_argument("Unrecognized sensor hardware type: " + std::string(hw_type_str));
}

/**
 * @throws std::invalid_argument if polarity is not recognized
 */
HwPolarity parse_polarity_str(std::string_view polarity_str)
{
    static constexpr auto polarity_map = std::to_array<std::pair<std::string_view, HwPolarity>>(
            {{"active_high", HwPolarity::ACTIVE_HIGH},
             {"active_low", HwPolarity::ACTIVE_LOW}});

    for (auto const& [name, value] : polarity_map)
    {
        if (name == polarity_str)
            return value;
    }
    throw std::invalid_argument("Unrecognized polarity: " + std::string(polarity_str));
}

/**
 * @throws std::invalid_argument if filter_type is not recognized
 */
AnalogFilterType parse_filter_type_str(std::string_view filter_type_str)
{
    static constexpr auto filter_type_map = std::to_array<std::pair<std::string_view, AnalogFilterType>>(
            {{"iir", AnalogFilterType::IIR},
             {"ma", AnalogFilterType::MOVING_AVERAGE}});

    for (auto const& [name, value] : filter_type_map)
    {
        if (name == filter_type_str)
            return value;
    }
    throw std::invalid_argument("Unrecognized filter type: " + std::string(filter_type_str));
}

std::vector<int> parse_pins(const rapidjson::Value& pins_json)
{
    std::vector<int> pins;
    for (const auto& pin : pins_json.GetArray())
    {
        pins.push_back(pin.GetInt64());
    }
    return pins;
}

/**
 * Parse hardware config and generate commands - validates according to schema
 * @throws std::invalid_argument if hardware validation fails (e.g., invalid pin counts, value ranges) or hardware type is unknown
 */
void handle_hardware_config_commands(const rapidjson::Value& hardware, SensorHwType hw_type,
                                     int sensor_id, MessageFactory& factory, MessageHandler* handler)
{
    switch (hw_type)
    {
        case SensorHwType::DIGITAL_OUTPUT_PIN:
        {
            // DigitalOutputPin { polarity, pins }
            auto pins = parse_pins(hardware["pins"]);
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            handler->post_event(factory.make_set_sensor_hw_polarity_command(sensor_id, parse_polarity_str(hardware["polarity"].GetString())));
            break;
        }

        case SensorHwType::DIGITAL_INPUT_PIN:
        {
            // DigitalInputPin { polarity, pins, delta_ticks }
            auto pins = parse_pins(hardware["pins"]);
            int  delta_ticks = hardware["delta_ticks"].GetInt();
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            handler->post_event(factory.make_set_sending_delta_ticks_command(sensor_id, delta_ticks));
            handler->post_event(factory.make_set_sensor_hw_polarity_command(sensor_id, parse_polarity_str(hardware["polarity"].GetString())));
            break;
        }

        case SensorHwType::ANALOG_INPUT_PIN:
        {
            // AnalogInputPin { pins, delta_ticks, adc_resolution, filter_time_constant }
            auto  pins = parse_pins(hardware["pins"]);
            int   delta_ticks = hardware["delta_ticks"].GetInt();
            int   adc_resolution = hardware["adc_resolution"].GetInt();
            float filter_time_constant = hardware["filter_time_constant"].GetFloat();
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            handler->post_event(factory.make_set_sending_delta_ticks_command(sensor_id, delta_ticks));
            handler->post_event(factory.make_set_adc_bit_resolution_command(sensor_id, adc_resolution));
            handler->post_event(factory.make_set_analog_time_constant_command(sensor_id, filter_time_constant));

            constexpr auto HYSTERESIS = "hysteresis";
            if (hardware.HasMember(HYSTERESIS))
            {
                int hysteresis = hardware[HYSTERESIS].GetInt();
                handler->post_event(factory.make_set_analog_hysteresis_command(sensor_id, hysteresis));
            }

            constexpr auto STABILIZATION_PERIOD = "stabilization_period";
            if (hardware.HasMember(STABILIZATION_PERIOD))
            {
                float stabilization_period = hardware[STABILIZATION_PERIOD].GetFloat();
                handler->post_event(factory.make_set_analog_stabilization_period_command(sensor_id, stabilization_period));
            }

            constexpr auto FILTER_TYPE = "filter_type";
            if (hardware.HasMember(FILTER_TYPE))
            {
                AnalogFilterType filter_type = parse_filter_type_str(hardware[FILTER_TYPE].GetString());
                handler->post_event(factory.make_set_analog_filter_type_command(sensor_id, filter_type));
            }

            break;
        }

        case SensorHwType::ENCODER:
        {
            // Encoder { polarity, pins[2] }
            auto pins = parse_pins(hardware["pins"]);
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            handler->post_event(factory.make_set_sensor_hw_polarity_command(sensor_id, parse_polarity_str(hardware["polarity"].GetString())));
            break;
        }

        case SensorHwType::RIBBON:
        {
            // Ribbon { pins, delta_ticks }
            auto pins = parse_pins(hardware["pins"]);
            int  delta_ticks = hardware["delta_ticks"].GetInt();
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            handler->post_event(factory.make_set_sending_delta_ticks_command(sensor_id, delta_ticks));
            break;
        }

        case SensorHwType::BUTTON:
        {
            // Button { pins, polarity, delta_ticks }
            auto pins = parse_pins(hardware["pins"]);
            int  delta_ticks = hardware["delta_ticks"].GetInt();
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            handler->post_event(factory.make_set_sending_delta_ticks_command(sensor_id, delta_ticks));
            handler->post_event(factory.make_set_sensor_hw_polarity_command(sensor_id, parse_polarity_str(hardware["polarity"].GetString())));
            break;
        }

        case SensorHwType::N_WAY_SWITCH:
        {
            // NWaySwitch { pins, delta_ticks }
            auto pins = parse_pins(hardware["pins"]);
            int  delta_ticks = hardware["delta_ticks"].GetInt();
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            handler->post_event(factory.make_set_sending_delta_ticks_command(sensor_id, delta_ticks));
            break;
        }

        case SensorHwType::STEPPED_OUTPUT:
        {
            // SteppedOutput { pins, multiplexed }
            auto                    pins = parse_pins(hardware["pins"]);
            const rapidjson::Value& mux_json = hardware["multiplexed"];
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            handler->post_event(factory.make_set_multiplexed_sensor_command(sensor_id, mux_json["multiplexer_id"].GetInt(), mux_json["multiplexer_pin"].GetInt()));
            break;
        }

        case SensorHwType::MULTIPLEXER:
        {
            // Multiplexer { pins }
            auto pins = parse_pins(hardware["pins"]);
            handler->post_event(factory.make_set_hw_pins_command(sensor_id, pins));
            break;
        }

        default:
            throw std::invalid_argument("Unknown hardware type: " + std::to_string(static_cast<int>(hw_type)));
    }
}

/*
 * Handle the hardware specific parts of a sensor
 * @throws std::invalid_argument if hardware configuration validation fails
 */
void JsonConfiguration::handle_sensor_hw_config(const rapidjson::Value& hardware, int sensor_id)
{
    const std::string  hw_type_str = hardware["hardware_type"].GetString();
    const SensorHwType hw_type = parse_hw_type_string(hw_type_str);

    _handler->post_event(_message_factory.make_set_sensor_hw_type_command(sensor_id, hw_type));

    /* Parse hardware config and generate type-specific commands */
    handle_hardware_config_commands(hardware, hw_type, sensor_id, _message_factory, _handler);
}

/*
 * Handle a backend configuration
 */
void JsonConfiguration::handle_backend_config(const rapidjson::Value& backend, BackendConfig& backend_config)
{
    int backend_id = backend["id"].GetInt();

    auto m = _message_factory.make_set_send_output_enabled_command(backend_id, backend["enabled"].GetBool());
    _handler->post_event(std::move(m));

    auto m2 = _message_factory.make_set_send_raw_input_enabled_command(backend_id, backend["raw_input_enabled"].GetBool());
    _handler->post_event(std::move(m2));

    const rapidjson::Value& config = backend["config"];
    const std::string       backend_type_str = config["type"].GetString();
    if (backend_type_str == "osc")
    {
        backend_config.type = BackendType::OSC;
        _handler->post_event(_message_factory.make_set_osc_output_host_command(backend_id, config["host"].GetString()));
        _handler->post_event(_message_factory.make_set_osc_output_port_command(backend_id, config["port"].GetInt()));
        _handler->post_event(_message_factory.make_set_osc_output_base_path_command(backend_id, config["base_path"].GetString()));
        _handler->post_event(_message_factory.make_set_osc_output_raw_path_command(backend_id, config["base_raw_input_path"].GetString()));
    }
    else if (backend_type_str == "grpc")
    {
        backend_config.type = BackendType::GRPC;
        _handler->post_event(_message_factory.make_set_grpc_listen_address_command(backend_id, config["host"].GetString()));
        _handler->post_event(_message_factory.make_set_grpc_listen_port_command(backend_id, config["port"].GetInt()));
    }
}


} // namespace config
} // namespace sensei
