/**
 * @brief Configuration Class for importing configuration from a JSON file
 * @copyright MIND Music Labs AB, Stockholm
 *
 *
 */

#include <fstream>
#include <iostream>

#include "json_configuration.h"


namespace sensei {
namespace config {


Json::Value read_configuration(std::ifstream& file)
{
    Json::Value config;
    Json::Reader reader;
    bool parse_ok = reader.parse(file, config, false);
    if (!parse_ok)
    {
        // TODO - Log the error  << reader.getFormattedErrorMessages() << std::endl;
    }
    return config;
}

/*
 * Open _source as a file and parse is as a json file
 */
ConfigStatus JsonConfiguration::read()
{
    std::ifstream file(_source);
    if (!file.good())
    {
        return ConfigStatus::IO_ERROR;
    }
    Json::Value config = std::move(read_configuration(file));
    if (config.isNull())
    {
        return ConfigStatus::PARSING_ERROR;
    }

    /* Start by disabling all pins to mute the board while sending the configuration commands */
    _queue->push(std::move(_message_factory.make_enable_sending_packets_command(0, false)));
    const Json::Value& backends = config["backends"];
    const Json::Value& sensors = config["sensors"];
    ConfigStatus status;

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
            status = handle_pin(sensor);
            if (status != ConfigStatus::OK)
            {
                return status;
            }
        }
    }
    /* The last commands enables sending of packets */
    _queue->push(std::move(_message_factory.make_enable_sending_packets_command(0, false)));
    return ConfigStatus::OK;
}

/*
 * Read all existing configuration keys for a single pin. And create
 * and queue configuration commands from them.
 * "id" is the only required key, also note that pin type must be
 * handled as the first key.
 */
ConfigStatus JsonConfiguration::handle_pin(const Json::Value &pin)
{
    int pin_id;
    const Json::Value& id = pin["id"];
    if (id.isInt())
    {
        pin_id = id.asInt();
    }
    else
    {
        /* id is a mandatory parameter */
        return ConfigStatus::PARAMETER_ERROR;
    }

    /* read pin name */
    const Json::Value& name = pin["name"];
    if (name.isString())
    {
        auto m = _message_factory.make_set_pin_name_command(pin_id,
                                                            name.asString(),
                                                            0);
        _queue->push(std::move(m));
    }

    /* read pin type configuration */
    const Json::Value& pin_type = pin["pin_type"];
    if (pin_type.isString())
    {
        std::unique_ptr<BaseMessage> m = nullptr;
        const std::string& pin_type_str = pin_type.asString();
        if (pin_type_str == "analog_input")
        {
            m = _message_factory.make_set_pin_type_command(pin_id,
                                                           PinType::ANALOG_INPUT,
                                                           0);
        }
        else if (pin_type_str == "digital_input")
        {
            m = _message_factory.make_set_pin_type_command(pin_id,
                                                           PinType::DIGITAL_INPUT,
                                                           0);
        }
        else if (pin_type_str == "digital_output")
        {
            m = _message_factory.make_set_pin_type_command(pin_id,
                                                           PinType::DIGITAL_OUTPUT,
                                                           0);
        }
        else
        {
            return ConfigStatus::PARAMETER_ERROR;
        }
        _queue->push(std::move(m));
    }

    /* read pin enabled/disabled configuration */
    const Json::Value& enabled = pin["enabled"];
    if (enabled.isBool())
    {
        auto m = _message_factory.make_set_enabled_command(pin_id,
                                                           enabled.asBool(),
                                                           0);
        _queue->push(std::move(m));
    }

    /* read sending mode configuration */
    const Json::Value& mode = pin["mode"];
    if (mode.isString())
    {
        const std::string& mode_str = mode.asString();
        if (mode_str == "continuous")
        {
            auto m = _message_factory.make_set_sending_mode_command(pin_id,
                                                                    SendingMode::CONTINUOUS,
                                                                    0);
            _queue->push(std::move(m));
        }
        else if (mode_str == "on_value_changed")
        {
            auto m = _message_factory.make_set_sending_mode_command(pin_id,
                                                                    SendingMode::ON_VALUE_CHANGED,
                                                                    0);
            _queue->push(std::move(m));
        }
        else
        {
            return ConfigStatus::PARAMETER_ERROR;
        }
    }

    /* read tick divisor configuration */
    const Json::Value& ticks = pin["delta_ticks"];
    if (ticks.isInt())
    {
        auto m = _message_factory.make_set_sending_delta_ticks_command(pin_id,
                                                                       ticks.asInt(),
                                                                       0);
        _queue->push(std::move(m));
    }

    /* read adc bit resolution configuration */
    const Json::Value& res = pin["adc_resolution"];
    if (res.isInt())
    {
        auto m = _message_factory.make_set_adc_bit_resolution_command(pin_id,
                                                                      res.asInt(),
                                                                      0);
        _queue->push(std::move(m));
    }

    /* read pin lowpass filter cutoff configuration */
    const Json::Value& cutoff = pin["lowpass_cutoff"];
    if (cutoff.isNumeric())
    {
        auto m = _message_factory.make_set_lowpass_cutoff_command(pin_id,
                                                                  cutoff.asFloat(),
                                                                  0);
        _queue->push(std::move(m));
    }

    /* read lowpass filter order configuration */
    const Json::Value& order = pin["lowpass_order"];
    if (order.isInt())
    {
        auto m = _message_factory.make_set_lowpass_filter_order_command(pin_id,
                                                                        order.asInt(),
                                                                        0);
        _queue->push(std::move(m));
    }

    /* read slider mode configuration */
    const Json::Value& slider_mode = pin["slider_mode"];
    if (slider_mode.isBool())
    {
        auto m = _message_factory.make_set_slider_mode_enabled_command(pin_id,
                                                                       slider_mode.asBool(),
                                                                       0);
        _queue->push(std::move(m));
    }

    /* read slider threshold configuration */
    const Json::Value& threshold = pin["slider_threshold"];
    if (threshold.isInt())
    {
        auto m = _message_factory.make_set_slider_threshold_command(pin_id,
                                                                    threshold.asInt(),
                                                                    0);
        _queue->push(std::move(m));
    }

    /* read inverted configuration */
    const Json::Value& inverted = pin["inverted"];
    if (inverted.isBool())
    {
        auto m = _message_factory.make_set_invert_enabled_command(pin_id,
                                                                  inverted.asBool(),
                                                                  0);
        _queue->push(std::move(m));
    }

    /* read range configuration */
    const Json::Value& range = pin["range"];
    if (range.isArray() && range.size() >= 2)
    {
        auto low = _message_factory.make_set_input_scale_range_low_command(pin_id,
                                                                           range[0].asInt(),
                                                                           0);
        auto high = _message_factory.make_set_input_scale_range_high_command(pin_id,
                                                                             range[1].asInt(),
                                                                             0);
        _queue->push(std::move(low));
        _queue->push(std::move(high));

    }
    return ConfigStatus::OK ;
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
        return ConfigStatus::PARAMETER_ERROR;
    }

    const Json::Value& enabled = backend["enabled"];
    if (enabled.isBool())
    {
        auto m = _message_factory.make_set_send_output_enabled_command(backend_id,
                                                                       enabled.asBool(),
                                                                       0);
        _queue->push(std::move(m));
    }
    const Json::Value& raw_input_enabled = backend["raw_input_enabled"];
    if (raw_input_enabled.isBool())
    {
        auto m = _message_factory.make_set_send_raw_input_enabled_command(backend_id,
                                                                         raw_input_enabled.asBool(),
                                                                         0);
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
        auto m = _message_factory.make_set_osc_output_host_command(id,
                                                                   host.asString(),
                                                                   0);
        _queue->push(std::move(m));
    }

    /* read porn number configuration */
    const Json::Value& port = backend["port"];
    if (port.isInt())
    {
        auto m = _message_factory.make_set_osc_output_port_command(id,
                                                                   port.asInt(),
                                                                   0);
        _queue->push(std::move(m));
    }
    /* read base path configuration */
    const Json::Value& path = backend["base_path"];
    if (path.isString())
    {
        auto m = _message_factory.make_set_osc_output_base_path_command(id,
                                                                        path.asString(),
                                                                        0);
        _queue->push(std::move(m));
    }
    /* read base path for raw inputs  */
    const Json::Value& raw_path = backend["base_raw_input_path"];
    if (raw_path.isString())
    {
        auto m = _message_factory.make_set_osc_output_raw_path_command(id,
                                                                       raw_path.asString(),
                                                                       0);
        _queue->push(std::move(m));
    }
    return ConfigStatus::OK;
}

} // end namespace config
} // end namespace sensei