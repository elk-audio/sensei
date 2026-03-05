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
#ifndef SENSEI_JSONCONFIGURATION_H
#define SENSEI_JSONCONFIGURATION_H

#include "message/message_factory.h"
#include "base_configuration.h"
#include "rapidjson/document.h"

namespace sensei {
namespace config {

class JsonConfiguration : public BaseConfiguration
{
public:
    JsonConfiguration(MessageHandler* handler, const std::string& file,
                      ThreadingMode threading_mode = ThreadingMode::ASYNCHRONOUS)
        : BaseConfiguration(handler, file, threading_mode)
    {}

    ~JsonConfiguration() = default;

    /*
     * Open file, parse json and put commands in queue
     */
    ConfigStatus read(Config& config) override;

    /*
     * Parse json from string and put commands in queue 
     */
    ConfigStatus read_from_string(Config& config, const char* json_string);

private:
    void handle_sensor(const rapidjson::Value& sensor);
    void handle_sensor_hw_config(const rapidjson::Value& hardware, int sensor_id);
    void handle_backend_config(const rapidjson::Value& backend, BackendConfig& backend_config);

    MessageFactory _message_factory;
};


} // namespace config
} // namespace sensei

#endif //SENSEI_JSONCONFIGURATION_H
