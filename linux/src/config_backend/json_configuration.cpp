/**
 * @brief Configuration Class for importing configuration from a JSON file
 * @copyright MIND Music Labs AB, Stockholm
 *
 *
 */

#include <fstream>
#include <iostream>
#include <json/json.h>

#include "json_configuration.h"


namespace sensei {
namespace config {


Json::Value read_configuration(const std::string& file_name)
{
    Json::Value config;
    Json::Reader reader;
    std::ifstream file(file_name);
    if (file.good())
    {
        bool parse_ok = reader.parse(file, config, false);
        if (!parse_ok)
        {
            // TODO - Log the error instead of just printing it.
            std::cout << "error parsing file" << reader.getFormattedErrorMessages() << std::endl;
        }
    }
    else
    {
        std::cout << "error opening file!!" << std::endl;
    }
    return config;
}



JsonConfiguration::JsonConfiguration(SynchronizedQueue<std::unique_ptr<BaseMessage>>* queue,
                                     const std::string& file) :
                        base_configuration(queue),
                        _file_name(file)
{

}

JsonConfiguration::~JsonConfiguration()
{

}

void JsonConfiguration::apply_configuration(const std::string& file_name)
{
    Json::Value config = std::move(read_configuration(file_name));
    const Json::Value& backends = config["backends"];
    const Json::Value& sensors = config["sensors"];
    if (backends.isArray())
    {
        for(const Json::Value& i : backends)
        {
            handle_backend(i);
        }
    }
    if (sensors.isArray())
    {
        for(const Json::Value& i : sensors)
        {
            handle_sensor(i);
        }
    }
}

int JsonConfiguration::handle_sensor(const Json::Value& sensor)
{
    int sensor_id;
    const Json::Value& id = sensor["id"];
    if (id.isInt())
    {
        sensor_id = id.asInt();
    }
    else
    {
        return -1;
    }

    const Json::Value& pin_type = sensor["pin_type"];
    if (pin_type.isInt())
    {
        auto m = _message_factory.make_set_pin_type_command(sensor_id,
                                                            static_cast<PinType>(pin_type.asInt()),
                                                            0);
        _queue->push(std::move(m));
    }

    const Json::Value& enabled = sensor["enabled"];
    if (pin_type.isBool())
    {
        auto m = _message_factory.make_set_enabled_command(sensor_id,
                                                           enabled.asBool(),
                                                           0);
        _queue->push(std::move(m));
    }

    const Json::Value& mode = sensor["mode"];
    if (mode.isInt())
    {
        auto m = _message_factory.make_set_sending_mode_command(sensor_id,
                                                               static_cast<SendingMode>(mode.asInt()),
                                                               0);
        _queue->push(std::move(m));
    }

    const Json::Value& ticks = sensor["delta_tics"];
    if (ticks.isInt())
    {
        auto m = _message_factory.make_set_sending_delta_ticks_command(sensor_id,
                                                                       ticks.asInt(),
                                                                       0);
        _queue->push(std::move(m));
    }

    const Json::Value& res = sensor["adc_resolution"];
    if (res.isInt())
    {
        auto m = _message_factory.make_set_adc_bit_resolution_command(sensor_id,
                                                                      res.asInt(),
                                                                      0);
        _queue->push(std::move(m));
    }

    const Json::Value& cutoff = sensor["lowpass_cutoff"];
    if (cutoff.isNumeric())
    {
        auto m = _message_factory.make_set_lowpass_cutoff_command(sensor_id,
                                                                  cutoff.asFloat(),
                                                                  0);
        _queue->push(std::move(m));
    }

    const Json::Value& order = sensor["lowpass_order"];
    if (order.isInt())
    {
        auto m = _message_factory.make_set_lowpass_filter_order_command(sensor_id,
                                                                        order.asInt(),
                                                                        0);
        _queue->push(std::move(m));
    }

    const Json::Value& slider_mode = sensor["slider_mode"];
    if (slider_mode.isBool())
    {
        auto m = _message_factory.make_set_slider_mode_enabled_command(sensor_id,
                                                                       slider_mode.asBool(),
                                                                       0);
        _queue->push(std::move(m));
    }

    const Json::Value& threshold = sensor["slider_threshold"];
    if (threshold.isInt())
    {
        auto m = _message_factory.make_set_slider_threshold_command(sensor_id,
                                                                    threshold.asInt(),
                                                                    0);
        _queue->push(std::move(m));
    }

}

int JsonConfiguration::handle_backend(const Json::Value& backend)
{


}


} // end namespace config
} // end namespace sensei