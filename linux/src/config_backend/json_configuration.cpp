/**
 * @brief Configuration Class for importing configuration from a JSON file
 * @copyright MIND Music Labs AB, Stockholm
 *
 *
 */

#include <fstream>
#include <iostream>

#include "json_configuration.h"
#include "logging.h"

namespace sensei {
namespace config {

SENSEI_GET_LOGGER_WITH_MODULE_NAME("config");

Json::Value read_configuration(std::ifstream& file)
{
    Json::Value config;
    Json::Reader reader;
    bool parse_ok = reader.parse(file, config, false);
    if (!parse_ok)
    {
        SENSEI_LOG_ERROR("Error parsing JSON configuration file, {}", reader.getFormattedErrorMessages());
    }
    return config;
}


/*
 * Open _source as a file and parse is as a json file
 */
ConfigStatus JsonConfiguration::read(HwFrontendConfig& hw_config)
{
    SENSEI_LOG_INFO("Reading configuration file");
    std::ifstream file(_source);
    if (!file.good())
    {
        SENSEI_LOG_ERROR("Couldn't open JSON configuration file: {}", _source);
        return ConfigStatus::IO_ERROR;
    }
    Json::Value config = std::move(read_configuration(file));
    if (config.isNull())
    {
        return ConfigStatus::PARSING_ERROR;
    }

    /* Start by disabling all pins to mute the board while sending the configuration commands */
    _queue->push(std::move(_message_factory.make_enable_sending_packets_command(0, false)));
    const Json::Value& hw_frontend = config["hw_frontend"];
    const Json::Value& backends = config["backends"];
    const Json::Value& sensors = config["sensors"];

    /* Read the hw status, needs to be returned directly and not as an event */
    ConfigStatus status = handle_hw_config(hw_frontend, hw_config);
    if (status != ConfigStatus::OK)
    {
        return status;
    }
    /* Read the rest of the configuration */
    if (backends.isArray())
    {
        for(const Json::Value& backend : backends)
        {
            status = handle_backend(backend);
            if (status != ConfigStatus::OK)
            {
                return status;
            }
        }
    }
    if (sensors.isArray())
    {
        for(const Json::Value& sensor : sensors)
        {
            status = handle_sensor(sensor);
            if (status != ConfigStatus::OK)
            {
                return status;
            }
        }
    }

    /* The last commands enables sending of packets */
    _queue->push(std::move(_message_factory.make_enable_sending_packets_command(0, true)));
    return ConfigStatus::OK;
}

ConfigStatus JsonConfiguration::handle_hw_config(const Json::Value& frontend, HwFrontendConfig& config)
{
    const Json::Value& type = frontend["type"];
    if (type.isString())
    {
        if ((type == "raspa_xmos") || (type == "raspa_gpio"))
        {
            config.type = HwFrontendType::RASPA_GPIO;
        } else
        {
            SENSEI_LOG_WARNING("\"{}\" is not a recognized hardware frontend type", type.asString());
            return ConfigStatus::PARSING_ERROR;
        }
    }
    return ConfigStatus::OK;
}


/*
 * Read all existing configuration keys for a single pin. And create
 * and queue configuration commands from them.
 * "id" is the only required key, also note that pin type must be
 * handled as the first key.
 */
ConfigStatus JsonConfiguration::handle_sensor(const Json::Value& sensor)
{
    int sensor_id;
    const Json::Value& id = sensor["id"];
    if (id.isInt())
    {
        sensor_id = id.asInt();
    }
    else
    {
        /* id is a mandatory parameter */
        SENSEI_LOG_WARNING("Sensor id not found in configuration");
        return ConfigStatus::PARAMETER_ERROR;
    }

    /* read sensor name */
    const Json::Value& name = sensor["name"];
    if (name.isString())
    {
        auto m = _message_factory.make_set_sensor_name_command(sensor_id, name.asString());
        _queue->push(std::move(m));
    }

    /* read sensor type configuration */
    const Json::Value& sensor_type = sensor["sensor_type"];
    if (sensor_type.isString())
    {
        std::unique_ptr<BaseMessage> m = nullptr;
        const std::string& sensor_type_str = sensor_type.asString();
        if (sensor_type_str == "analog_input")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::ANALOG_INPUT);
        }
        else if (sensor_type_str == "analog_output")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::ANALOG_OUTPUT);
        }
        else if (sensor_type_str == "digital_input")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::DIGITAL_INPUT);
        }
        else if (sensor_type_str == "continuous_input")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::CONTINUOUS_INPUT);
        }
        else if (sensor_type_str == "continuous_output")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::CONTINUOUS_OUTPUT);
        }
        else if (sensor_type_str == "digital_output")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::DIGITAL_OUTPUT);
        }
        else if (sensor_type_str == "range_input")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::RANGE_INPUT);
        }
        else if (sensor_type_str == "range_output")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::RANGE_OUTPUT);
        }
        else if (sensor_type_str == "no_output")
        {
            m = _message_factory.make_set_sensor_type_command(sensor_id, SensorType::NO_OUTPUT);
        }
        else
        {
            SENSEI_LOG_WARNING("\"{}\" is not a recognized sensor type", sensor_type_str);
            return ConfigStatus::PARAMETER_ERROR;
        }
        _queue->push(std::move(m));
    }

    const Json::Value& hw_config = sensor["hardware"];
    if (!hw_config.empty())
    {
        auto status = handle_sensor_hw(hw_config, sensor_id);
        if (status != ConfigStatus::OK)
        {
            return status;
        }
    }

    /* read sensor enabled/disabled configuration */
    const Json::Value& enabled = sensor["enabled"];
    if (enabled.isBool())
    {
        auto m = _message_factory.make_set_enabled_command(sensor_id, enabled.asBool());
        _queue->push(std::move(m));
    }

    /* read sending mode configuration */
    const Json::Value& mode = sensor["mode"];
    if (mode.isString())
    {
        const std::string& mode_str = mode.asString();
        if (mode_str == "continuous")
        {
            auto m = _message_factory.make_set_sending_mode_command(sensor_id, SendingMode::CONTINUOUS);
            _queue->push(std::move(m));
        }
        else if (mode_str == "on_value_changed")
        {
            auto m = _message_factory.make_set_sending_mode_command(sensor_id, SendingMode::ON_VALUE_CHANGED);
            _queue->push(std::move(m));
        }
        else
        {
            SENSEI_LOG_WARNING("\"{}\" is not a recognized sending mode", mode_str);
            return ConfigStatus::PARAMETER_ERROR;
        }
    }

    /* read inverted configuration */
    const Json::Value& inverted = sensor["inverted"];
    if (inverted.isBool())
    {
        auto m = _message_factory.make_set_invert_enabled_command(sensor_id, inverted.asBool());
        _queue->push(std::move(m));
    }

    /* read range configuration */
    const Json::Value& range = sensor["range"];
    if (range.isArray() && range.size() >= 2)
    {
        auto m = _message_factory.make_set_input_range_command(sensor_id, range[0].asFloat(), range[1].asFloat());
        _queue->push(std::move(m));
    }

    /* read sensor timestamp output configuration */
    const Json::Value& timestamped = sensor["timestamp"];
    if (enabled.isBool())
    {
        auto m = _message_factory.make_set_send_timestamp_enabled(sensor_id, timestamped.asBool());
        _queue->push(std::move(m));
    }

    return ConfigStatus::OK ;
}

/*
 * Handle the hardware specific parts of a sensor
 */
ConfigStatus JsonConfiguration::handle_sensor_hw(const Json::Value& hardware, int sensor_id)
{
    /* Read sensor type configuration */
    const Json::Value& sensor_type = hardware["hardware_type"];
    if (sensor_type.isString())
    {
        std::unique_ptr<BaseMessage> m = nullptr;
        const std::string& hw_type_str = sensor_type.asString();
        if (hw_type_str == "analog_input_pin")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::ANALOG_INPUT_PIN);
        }
        else if (hw_type_str == "digital_input_pin")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::DIGITAL_INPUT_PIN);
        }
        else if (hw_type_str == "digital_output_pin")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::DIGITAL_OUTPUT_PIN);
        }
        else if (hw_type_str == "ribbon")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::RIBBON);
        }
        else if (hw_type_str == "button")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::BUTTON);
        }
        else if (hw_type_str == "encoder")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::ENCODER);
        }
        else if (hw_type_str == "n_way_switch")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::N_WAY_SWITCH);
        }
        else if (hw_type_str == "stepped_output")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::STEPPED_OUTPUT);
        }
        else if (hw_type_str == "multiplexer")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::MULTIPLEXER);
        }
        else if (hw_type_str == "audio_mute_button")
        {
            m = _message_factory.make_set_sensor_hw_type_command(sensor_id, SensorHwType::AUDIO_MUTE_BUTTON);
        }
        else
        {
            SENSEI_LOG_WARNING("\"{}\" is not a recognized sensor hardware type", hw_type_str);
            return ConfigStatus::PARAMETER_ERROR;
        }
        _queue->push(std::move(m));
    }

    /* For gpio protocol compliant configs, we need to set the hw type before assigning pins to it */
    auto read_pin_status = read_pins(hardware["pins"], sensor_id);
    if (read_pin_status != ConfigStatus::OK)
    {
        return read_pin_status;
    }

    /* read multiplexer configuration if sensor is multiplexed */
    const Json::Value& multiplexed = hardware["multiplexed"];
    if (multiplexed.isObject())
    {
        int id;
        int pin;
        const Json::Value& multiplexer_id = multiplexed["multiplexer_id"];
        if (multiplexer_id.isIntegral())
        {
            id = multiplexer_id.asInt();
        }
        else
        {
            SENSEI_LOG_WARNING("Multiplexer id is required");
            return ConfigStatus::PARAMETER_ERROR;
        }
        const Json::Value& multiplexer_pin = multiplexed["multiplexer_pin"];
        if (multiplexer_pin.isIntegral())
        {
            pin = multiplexer_pin.asInt();
        }
        else
        {
            SENSEI_LOG_WARNING("Multiplexer pin is required");
            return ConfigStatus::PARAMETER_ERROR;
        }
        _queue->push(_message_factory.make_set_multiplexed_sensor_command(sensor_id, id, pin));
    }

    /* read tick divisor configuration */
    const Json::Value& ticks = hardware["delta_ticks"];
    if (ticks.isInt())
    {
        auto m = _message_factory.make_set_sending_delta_ticks_command(sensor_id, ticks.asInt());
        _queue->push(std::move(m));
    }

    /* read adc bit resolution configuration */
    const Json::Value& res = hardware["adc_resolution"];
    if (res.isInt())
    {
        auto m = _message_factory.make_set_adc_bit_resolution_command(sensor_id, res.asInt());
        _queue->push(std::move(m));
    }

    /* read polarity configuration */
    const Json::Value& polarity = hardware["polarity"];
    if (polarity.isString())
    {
        const std::string& pol_str = polarity.asString();
        std::unique_ptr<BaseMessage> m = nullptr;
        if (pol_str == "active_high")
        {
            m = _message_factory.make_set_sensor_hw_polarity_command(sensor_id, HwPolarity::ACTIVE_HIGH);
        }
        else if (pol_str == "active_low")
        {
            m = _message_factory.make_set_sensor_hw_polarity_command(sensor_id, HwPolarity::ACTIVE_LOW);
        }
        else
        {
            SENSEI_LOG_WARNING("Unrecognised polarity: \"{}\"", pol_str);
            return ConfigStatus::PARAMETER_ERROR;
        }
        _queue->push(std::move(m));
    }

    /* read sensor lowpass filter cutoff configuration */
    const Json::Value& cutoff = hardware["lowpass_cutoff"];
    if (cutoff.isNumeric())
    {
        auto m = _message_factory.make_set_lowpass_cutoff_command(sensor_id, cutoff.asFloat());
        _queue->push(std::move(m));
    }

    /* read lowpass filter order configuration */
    const Json::Value& order = hardware["lowpass_order"];
    if (order.isInt())
    {
        auto m = _message_factory.make_set_lowpass_filter_order_command(sensor_id, order.asInt());
        _queue->push(std::move(m));
    }

    /* read slider threshold configuration */
    const Json::Value& threshold = hardware["slider_threshold"];
    if (threshold.isInt())
    {
        auto m = _message_factory.make_set_slider_threshold_command(sensor_id, threshold.asInt());
        _queue->push(std::move(m));
    }

    /* read fast mode configuration */
    const Json::Value& fast_mode = hardware["fast_mode"];
    if (fast_mode.isBool())
    {
        auto m = _message_factory.make_set_fast_mode_command(sensor_id, fast_mode.asBool());
        _queue->push(std::move(m));
    }

    return ConfigStatus::OK;
}

/*
 * Handle a backend configuration
 */
ConfigStatus JsonConfiguration::handle_backend(const Json::Value& backend)
{
    int backend_id;
    const Json::Value& id = backend["id"];
    if (id.isInt())
    {
        backend_id = id.asInt();
    }
    else
    {
        /* id is a mandatory parameter */
        SENSEI_LOG_WARNING("Backend id not found in configuration");
        return ConfigStatus::PARAMETER_ERROR;
    }

    const Json::Value& enabled = backend["enabled"];
    if (enabled.isBool())
    {
        auto m = _message_factory.make_set_send_output_enabled_command(backend_id, enabled.asBool());
        _queue->push(std::move(m));
    }
    const Json::Value& raw_input_enabled = backend["raw_input_enabled"];
    if (raw_input_enabled.isBool())
    {
        auto m = _message_factory.make_set_send_raw_input_enabled_command(backend_id, raw_input_enabled.asBool());
        _queue->push(std::move(m));
    }

    /* Type specific configuration */
    const Json::Value& backend_type = backend["type"];
    if (backend_type.isString())
    {
        const std::string& backend_type_str = backend_type.asString();
        if (backend_type_str == "osc")
        {
            return handle_osc_backend(backend, backend_id);
        }
    }
    return ConfigStatus::OK ;
}


/*
 * Handle configuration specific to OSC backend
 */
ConfigStatus JsonConfiguration::handle_osc_backend(const Json::Value& backend, int id)
{
    /* read host configuration */
    const Json::Value& host = backend["host"];
    if (host.isString())
    {
        auto m = _message_factory.make_set_osc_output_host_command(id, host.asString());
        _queue->push(std::move(m));
    }

    /* read port number configuration */
    const Json::Value& port = backend["port"];
    if (port.isInt())
    {
        auto m = _message_factory.make_set_osc_output_port_command(id, port.asInt());
        _queue->push(std::move(m));
    }
    /* read base path configuration */
    const Json::Value& path = backend["base_path"];
    if (path.isString())
    {
        auto m = _message_factory.make_set_osc_output_base_path_command(id, path.asString());
        _queue->push(std::move(m));
    }
    /* read base path for raw inputs  */
    const Json::Value& raw_path = backend["base_raw_input_path"];
    if (raw_path.isString())
    {
        auto m = _message_factory.make_set_osc_output_raw_path_command(id, raw_path.asString());
        _queue->push(std::move(m));
    }
    return ConfigStatus::OK;
}

ConfigStatus JsonConfiguration::read_pins(const Json::Value& pin_list, int sensor_id)
{
    if (pin_list.isArray())
    {
        std::vector<int> pins;
        for (const Json::Value& p : pin_list)
        {
            pins.push_back(p.asInt());
        }
        _queue->push(_message_factory.make_set_hw_pins_command(sensor_id, pins));
    }
    /* Pins is not a mandatory configuration parameter */
    return ConfigStatus::OK;
}


} // end namespace config
} // end namespace sensei