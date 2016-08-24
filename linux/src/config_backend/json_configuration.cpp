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

SENSEI_GET_LOGGER;

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
ConfigStatus JsonConfiguration::read()
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
    const Json::Value& backends = config["backends"];
    const Json::Value& sensors = config["sensors"];
    const Json::Value& imu = config["imu"];
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
    status = handle_imu(imu);
    if (status != ConfigStatus::OK)
    {
        return status;
    }
    /* The last commands enables sending of packets */
    _queue->push(std::move(_message_factory.make_enable_sending_packets_command(0, true)));
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
        SENSEI_LOG_WARNING("Pin id not found in configuration");
        return ConfigStatus::PARAMETER_ERROR;
    }

    /* read pin name */
    const Json::Value& name = pin["name"];
    if (name.isString())
    {
        auto m = _message_factory.make_set_pin_name_command(pin_id, name.asString());
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
            m = _message_factory.make_set_pin_type_command(pin_id, PinType::ANALOG_INPUT);
        }
        else if (pin_type_str == "digital_input")
        {
            m = _message_factory.make_set_pin_type_command(pin_id, PinType::DIGITAL_INPUT);
        }
        else if (pin_type_str == "imu_input")
        {
            m = _message_factory.make_set_pin_type_command(pin_id, PinType::IMU_INPUT);
        }
        else if (pin_type_str == "digital_output")
        {
            m = _message_factory.make_set_pin_type_command(pin_id, PinType::DIGITAL_OUTPUT);
        }
        else
        {
            SENSEI_LOG_WARNING("\"{}\" is not a recognized pin type", pin_type_str);
            return ConfigStatus::PARAMETER_ERROR;
        }
        _queue->push(std::move(m));
    }
    /* Read Imu parameter to map to this pin */
    const Json::Value& param = pin["parameter"];
    if (param.isString())
    {
        const std::string parameter = param.asString();
        std::unique_ptr<BaseMessage> m;
        if (parameter == "yaw")
        {
            m = _message_factory.make_set_virtual_pin_command(pin_id, ImuIndex::YAW);
        }
        else if (parameter == "pitch")
        {
            m = _message_factory.make_set_virtual_pin_command(pin_id, ImuIndex::PITCH);
        }
        else if (parameter == "roll")
        {
            m = _message_factory.make_set_virtual_pin_command(pin_id, ImuIndex::ROLL);
        }
        if (m)
        {
            _queue->push(std::move(m));
        }
    }

    /* read pin enabled/disabled configuration */
    const Json::Value& enabled = pin["enabled"];
    if (enabled.isBool())
    {
        auto m = _message_factory.make_set_enabled_command(pin_id, enabled.asBool());
        _queue->push(std::move(m));
    }

    /* read sending mode configuration */
    const Json::Value& mode = pin["mode"];
    if (mode.isString())
    {
        const std::string& mode_str = mode.asString();
        if (mode_str == "continuous")
        {
            auto m = _message_factory.make_set_sending_mode_command(pin_id, SendingMode::CONTINUOUS);
            _queue->push(std::move(m));
        }
        else if (mode_str == "on_value_changed")
        {
            auto m = _message_factory.make_set_sending_mode_command(pin_id, SendingMode::ON_VALUE_CHANGED);
            _queue->push(std::move(m));
        }
        else
        {
            SENSEI_LOG_WARNING("\"{}\" is not a recognized sending mode", mode_str);
            return ConfigStatus::PARAMETER_ERROR;
        }
    }

    /* read tick divisor configuration */
    const Json::Value& ticks = pin["delta_ticks"];
    if (ticks.isInt())
    {
        auto m = _message_factory.make_set_sending_delta_ticks_command(pin_id, ticks.asInt());
        _queue->push(std::move(m));
    }

    /* read adc bit resolution configuration */
    const Json::Value& res = pin["adc_resolution"];
    if (res.isInt())
    {
        auto m = _message_factory.make_set_adc_bit_resolution_command(pin_id, res.asInt());
        _queue->push(std::move(m));
    }

    /* read pin lowpass filter cutoff configuration */
    const Json::Value& cutoff = pin["lowpass_cutoff"];
    if (cutoff.isNumeric())
    {
        auto m = _message_factory.make_set_lowpass_cutoff_command(pin_id, cutoff.asFloat());
        _queue->push(std::move(m));
    }

    /* read lowpass filter order configuration */
    const Json::Value& order = pin["lowpass_order"];
    if (order.isInt())
    {
        auto m = _message_factory.make_set_lowpass_filter_order_command(pin_id, order.asInt());
        _queue->push(std::move(m));
    }

    /* read slider mode configuration */
    const Json::Value& slider_mode = pin["slider_mode"];
    if (slider_mode.isBool())
    {
        auto m = _message_factory.make_set_slider_mode_enabled_command(pin_id, slider_mode.asBool());
        _queue->push(std::move(m));
    }

    /* read slider threshold configuration */
    const Json::Value& threshold = pin["slider_threshold"];
    if (threshold.isInt())
    {
        auto m = _message_factory.make_set_slider_threshold_command(pin_id, threshold.asInt());
        _queue->push(std::move(m));
    }

    /* read inverted configuration */
    const Json::Value& inverted = pin["inverted"];
    if (inverted.isBool())
    {
        auto m = _message_factory.make_set_invert_enabled_command(pin_id, inverted.asBool());
        _queue->push(std::move(m));
    }

    /* read range configuration */
    const Json::Value& range = pin["range"];
    if (range.isArray() && range.size() >= 2)
    {
        auto low = _message_factory.make_set_input_scale_range_low_command(pin_id, range[0].asFloat());
        auto high = _message_factory.make_set_input_scale_range_high_command(pin_id, range[1].asFloat());
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

/*
 * Handle IMU configuration
 */
ConfigStatus JsonConfiguration::handle_imu(const Json::Value& imu)
{
    if (imu.empty())
    {
        return ConfigStatus::OK; /* Imu config is empty or not present, do nothing */
    }
    /* Read filter mode configuration */
    const Json::Value& mode = imu["filter_mode"];
    if (mode.isString())
    {
        std::string filter_str = mode.asString();
        int filter = 0;
        if (filter_str == "no_orientation")
        {
            filter = 0;
        }
        else if (filter_str == "kalman")
        {
            filter = 1;
        }
        else if (filter_str == "q_comp")
        {
            filter = 2;
        }
        else if (filter_str == "q_grad")
        {
            filter = 3;
        }
        auto m = _message_factory.make_imu_set_filter_mode_command(filter);
        _queue->push(std::move(m));
    }
    /* Read accelerometer range max value configuration */
    const Json::Value& acc_range = imu["accelerometer_range_max"];
    if (acc_range.isNumeric())
    {
        auto m = _message_factory.make_imu_set_acc_range_max_command(acc_range.asInt());
        _queue->push(std::move(m));
    }
    /* Read gyroscope range max value configuration */
    const Json::Value& gyro_range = imu["gyroscope_range_max"];
    if (gyro_range.isNumeric())
    {
        auto m = _message_factory.make_imu_set_gyro_range_max_command(gyro_range.asInt());
        _queue->push(std::move(m));
    }
    /* Read compass range max value configuration */
    const Json::Value& comp_range = imu["compass_range_max"];
    if (comp_range.isNumeric())
    {
        auto m = _message_factory.make_imu_set_compass_range_max_command(comp_range.asInt());
        _queue->push(std::move(m));
    }
    /* Read compass enabled configuration */
    const Json::Value& compass = imu["compass_enabled"];
    if (compass.isBool())
    {
        auto m = _message_factory.make_imu_enable_compass_command(compass.asBool());
        _queue->push(std::move(m));
    }
    /* Read sending mode configuration */
    const Json::Value& sending_mode = imu["mode"];
    if (sending_mode.isString())
    {
        if (sending_mode.isString())
        {
            const std::string& mode_str = sending_mode.asString();
            if (mode_str == "continuous")
            {
                auto m = _message_factory.make_imu_set_sending_mode_command(SendingMode::CONTINUOUS);
                _queue->push(std::move(m));
            }
            else if (mode_str == "on_value_changed")
            {
                auto m = _message_factory.make_imu_set_sending_mode_command(SendingMode::ON_VALUE_CHANGED);
                _queue->push(std::move(m));
            }
            else
            {
                SENSEI_LOG_WARNING("\"{}\" is not a recognized sending mode", mode_str);
                return ConfigStatus::PARAMETER_ERROR;
            }
        }
    }
    /* Read delta ticks configuration */
    const Json::Value& ticks = imu["delta_ticks"];
    if (ticks.isInt())
    {
        auto m = _message_factory.make_imu_sending_delta_ticks_command(ticks.asInt());
        _queue->push(std::move(m));
    }
    /* Read data mode configuration */
    const Json::Value& data_mode = imu["data"];
    if (data_mode.isString())
    {
        int mode;
        if (data_mode.asString() == "quaternions")
        {
            mode = 2;
        }
        else
        {
            SENSEI_LOG_ERROR("{} was not a recognized data mode", data_mode.asString());
            mode = 0;
        }
        auto m = _message_factory.make_imu_set_data_mode_command(mode);
        _queue->push(std::move(m));
    }
    /* Read threshold for sending data configuration */
    const Json::Value& threshold = imu["acc_norm_threshold"];
    if (threshold.isNumeric())
    {
        auto m = _message_factory.make_imu_acc_threshold_command(threshold.asFloat());
        _queue->push(std::move(m));
    }
    /* Read IMU enabled configuration */
    const Json::Value& enabled = imu["enabled"];
    if (enabled.isBool())
    {
        auto m = _message_factory.make_enable_imu_command(enabled.asBool());
        _queue->push(std::move(m));
        if (enabled.asBool())
        {
            /* If IMU is turning on, trigger an automatic calibration */
            auto c = _message_factory.make_imu_calibrate_command();
            _queue->push(std::move(c));
        }
    }

    return ConfigStatus::OK;
}


} // end namespace config
} // end namespace sensei