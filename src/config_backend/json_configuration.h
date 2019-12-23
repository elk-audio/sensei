/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
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
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_JSONCONFIGURATION_H
#define SENSEI_JSONCONFIGURATION_H

#include "message/message_factory.h"
#include "base_configuration.h"
#include <json/json.h>

namespace sensei {
namespace config {

class JsonConfiguration : public BaseConfiguration
{
public:
    JsonConfiguration(SynchronizedQueue<std::unique_ptr<BaseMessage>>* queue, const std::string& file) :
            BaseConfiguration(queue, file)
    {}

    ~JsonConfiguration() = default;

    /*
     * Open file, parse json and put commands in queue
     */
    ConfigStatus read(HwFrontendConfig& hw_config) override;

private:
    ConfigStatus handle_hw_config(const Json::Value& frontend, HwFrontendConfig& config);
    ConfigStatus handle_sensor(const Json::Value& sensor);
    ConfigStatus handle_sensor_hw(const Json::Value& hardware, int sensor_id);
    ConfigStatus handle_backend(const Json::Value& backend);
    ConfigStatus handle_osc_backend(const Json::Value& backend, int id);
    ConfigStatus read_pins(const Json::Value& pins, int sensor_id);

    MessageFactory _message_factory;
};


} // namespace config
} // namespace sensei

#endif //SENSEI_JSONCONFIGURATION_H