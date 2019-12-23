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
 * @brief Class which is responsible for creation and handling of all events
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_EVENT_HANDLER_H
#define SENSEI_EVENT_HANDLER_H

#include <memory>
#include <string>
#include <chrono>

#include "synchronized_queue.h"
#include "mapping/mapping_processor.h"
#include "message/message_factory.h"
#include "hardware_frontend/base_hw_frontend.h"
#include "hardware_backend/base_hw_backend.h"
#include "output_backend/output_backend.h"
#include "config_backend/base_configuration.h"
#include "user_frontend/user_frontend.h"

namespace sensei {

class EventHandler
{
public:
    EventHandler() = default;

    ~EventHandler() = default;

    bool init(int max_n_input_pins, int max_n_digital_out_pins, const std::string& config_file);

    void handle_events(std::chrono::milliseconds wait_period);

    void deinit();

    void reload_config()
    {
        config::HwFrontendConfig hwc;
        _config_backend->read(hwc);
    }

private:
    void _handle_value(std::unique_ptr<Value> value);
    void _handle_command(std::unique_ptr<Command> cmd);
    void _handle_error(std::unique_ptr<Error> error);

    // Inter-modules communication queues
    SynchronizedQueue<std::unique_ptr<Command>> _to_frontend_queue;
    SynchronizedQueue<std::unique_ptr<BaseMessage>> _event_queue;

    // Sub-components instances
    std::unique_ptr<hw_frontend::BaseHwFrontend> _hw_frontend;
    std::unique_ptr<hw_backend::BaseHwBackend> _hw_backend;
    std::unique_ptr<mapping::MappingProcessor> _processor;
    std::unique_ptr<output_backend::OutputBackend> _output_backend;
    std::unique_ptr<config::BaseConfiguration> _config_backend;
    std::unique_ptr<user_frontend::UserFrontend> _user_frontend;
};

} // namespace sensei

#endif //SENSEI_EVENT_HANDLER_H