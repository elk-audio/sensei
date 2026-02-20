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
 * @brief Class which is responsible for creation and handling of all events
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 */
#include <chrono>

#include "event_handler.h"
#include "config_backend/json_configuration.h"
#include "hardware_frontend/hw_frontend.h"
#include "hardware_backend/gpio_hw_socket.h"
#include "output_backend/osc_backend.h"
#include "shiftreg_gpio/shiftreg_gpio.h"
#include "user_frontend/osc_user_frontend.h"
#include "utils.h"
#include "logging.h"

#ifdef SENSEI_WITH_GRPC
#include "output_backend/grpc_backend.h"
#include "user_frontend/grpc_user_frontend.h"
#endif

using namespace sensei;

constexpr auto HWBACKEND_TIMEOUT = std::chrono::milliseconds(250);

ELK_PUSH_WARNING
ELK_DISABLE_UNUSED_CONST_VARIABLE
SENSEI_GET_LOGGER_WITH_MODULE_NAME("eventhandler");
ELK_POP_WARNING

bool EventHandler::init(int max_n_input_pins,
                        int max_n_digital_out_pins,
                        const std::string& config_file,
                        ThreadingMode threading_mode)
{
    config::Config config;
    _config_backend.reset(new config::JsonConfiguration(this, config_file));
    auto ret = _config_backend->read(config);
    if (ret != config::ConfigStatus::OK)
    {
        switch (ret)
        {
        case config::ConfigStatus::IO_ERROR:
            SENSEI_LOG_ERROR("I/O error while reading config file");
            break;

        case config::ConfigStatus::PARSING_ERROR:
            SENSEI_LOG_ERROR("Couldn't parse config file");
            break;

        case config::ConfigStatus::PARAMETER_ERROR:
            SENSEI_LOG_ERROR("Wrong parameter in config file");

        default:
            break;
        }
    }

    // hw_frontend initialization
    switch (config.hw_config.type)
    {
    case HwFrontendType::RASPA_GPIO:
        SENSEI_LOG_INFO("Initializing Gpio Hw Frontend with socket hw backend");
        _hw_backend = std::make_unique<hw_backend::GpioHwSocket>("/tmp/raspa", HWBACKEND_TIMEOUT);
        _hw_frontend = std::make_unique<hw_frontend::HwFrontend>(_hw_backend.get(), this);
        break;

    case HwFrontendType::ELK_PI_GPIO:
        SENSEI_LOG_INFO("Initializing Gpio Frontend with Elk Pi hw backend");
        _hw_backend = std::make_unique<hw_backend::shiftregister_gpio::ShiftregGpio>(HWBACKEND_TIMEOUT);
        _hw_frontend = std::make_unique<hw_frontend::HwFrontend>(_hw_backend.get(), this);
        break;

    default:
        _hw_backend = std::make_unique<hw_backend::NoOpHwBackend>(HWBACKEND_TIMEOUT);
        _hw_frontend = std::make_unique<hw_frontend::NoOpFrontend>(this);
        SENSEI_LOG_ERROR("No HW Frontend configured");
        break;
    }

    if(!_hw_backend->init())
    {
        SENSEI_LOG_ERROR("Failed to initialize hw backend");
        return false;
    }

    _processor = std::make_unique<mapping::MappingProcessor>(max_n_input_pins);

    switch (config.backend_config.type)
    {
#ifdef SENSEI_WITH_GRPC
    case BackendType::GRPC:
    {
        SENSEI_LOG_INFO("Initializing gRPC backend");

        // The frontend runs the gRPC server where subscriptions are made so the
        // backend needs to forward events to the frontend. If we decide to fully
        // remove OSC support then this could be refactored.
        auto grpc_backend = std::make_unique<output_backend::GrpcBackend>(max_n_input_pins);
        auto grpc_frontend = std::make_unique<user_frontend::GrpcUserFrontend>(this, max_n_input_pins, max_n_digital_out_pins);
        grpc_backend->set_user_frontend(grpc_frontend.get());

        _output_backend = std::move(grpc_backend);
        _user_frontend = std::move(grpc_frontend);
        break;
    }
#endif

    case BackendType::OSC:
    default:
        SENSEI_LOG_INFO("Initializing OSC backend");
        _output_backend = std::make_unique<output_backend::OSCBackend>(max_n_input_pins);
        _user_frontend = std::make_unique<user_frontend::OSCUserFrontend>(this, max_n_input_pins, max_n_digital_out_pins);
        break;
    }

    _hw_frontend->verify_acks(true);
    _hw_frontend->run();
    return true;
}

void EventHandler::deinit()
{
    _hw_frontend->stop();
    _hw_frontend.reset(nullptr);
    _hw_backend->deinit();
    _hw_backend.reset(nullptr);
    _processor.reset(nullptr);
    _output_backend.reset(nullptr);
    _config_backend.reset(nullptr);
}

void EventHandler::_handle_event(BaseMessage* event)
{
    switch(event->base_type())
    {
        case MessageType::VALUE:
        {
            auto value = static_cast<Value*>(event);
            _handle_value(value);
        }
        break;

        case MessageType::COMMAND:
        {
            auto cmd = static_cast<Command*>(event);
            _handle_command(std::move(cmd));
        }
        break;

        case MessageType::ERROR:
        {
            auto error = static_cast<Error*>(event);
            _handle_error(std::move(error));
        }
        break;
    }
}

void EventHandler::handle_events(std::chrono::milliseconds wait_period)
{
    _event_queue.wait_for_data(wait_period);
    while (! _event_queue.empty())
    {
        std::unique_ptr<BaseMessage> event = _event_queue.pop();
        _handle_event(event.get());
        // At last handle the case where the message might be passed on to another queue
        _handle_hw_frontend_message(std::move(event));
    }
}

void EventHandler::_handle_value(Value* value) // value is not passed on
{
    if (is_output_value(value))
    {
        _processor->process(value, _output_backend.get());
    }
    else if (is_set_value(value))
    {
        auto cmd = _processor->process_set(value);
        if (cmd != nullptr)
        {
            _to_frontend_queue.push(std::move(cmd));
        }
    }
}

// This is the only handling function that takes ownership as it's the only one that might pass on the message
void EventHandler::_handle_hw_frontend_message(std::unique_ptr<BaseMessage> cmd)
{
    if (cmd->base_type() == MessageType::COMMAND)
    {
        auto typed_cmd = static_unique_ptr_cast<Command, BaseMessage>(std::move(cmd));
        if (typed_cmd->destination() & CommandDestination::HARDWARE_FRONTEND)
        {
            _to_frontend_queue.push(std::move(typed_cmd));
        }
    }
}

void EventHandler::_handle_command(Command* cmd) // passed on only if addressed to hw frontend
{
    CommandDestination address = cmd->destination();

    // Process first non-sink destinations
    // using non-owning raw pointer
    if (address & CommandDestination::MAPPING_PROCESSOR)
    {
        CommandErrorCode ret = _processor->apply_command(cmd);
        if (ret != CommandErrorCode::OK)
        {
            switch (ret)
            {
            case CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE:
                SENSEI_LOG_ERROR("Internal Mapping, Unhandled command: {}, sensor: {}", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::INVALID_SENSOR_INDEX:
                SENSEI_LOG_ERROR("Invalid pin index {} for command: {}", cmd->index(), cmd->representation());
                break;

            case CommandErrorCode::INVALID_RANGE:
                SENSEI_LOG_ERROR("Invalid range for command: {}, sensor: {}", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::INVALID_VALUE:
                SENSEI_LOG_ERROR("Invalid range for command: {}, sensor: {}", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::CLIP_WARNING:
                SENSEI_LOG_WARNING("Clipped value for command: {}, sensor: {}", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::UNINITIALIZED_SENSOR:
                SENSEI_LOG_WARNING("Dropping command {} for uninitialized sensor {}", cmd->representation(), cmd->index());
                break;

            default:
                break;
            }
        }
    }

    if (address & CommandDestination::OUTPUT_BACKEND)
    {
        CommandErrorCode ret = _output_backend->apply_command(cmd);
        if (ret != CommandErrorCode::OK)
        {
            switch (ret)
            {
            case CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE:
                SENSEI_LOG_ERROR("Output Backend, Unhandled command: {}, index: ", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::INVALID_URL:
                SENSEI_LOG_ERROR("Invalid OSC Backend URL");
                break;

            case CommandErrorCode::INVALID_PORT_NUMBER:
                SENSEI_LOG_ERROR("Invalid OSC output port number");
                break;

            default:
                break;
            }
        }
    }

    if (address & CommandDestination::CONFIG_BACKEND)
    {
        // TODO: implement me
    }

    if (address & CommandDestination::USER_FRONTEND)
    {
        CommandErrorCode ret = _user_frontend->apply_command(cmd);
        if (ret != CommandErrorCode::OK)
        {
            switch (ret)
            {
            case CommandErrorCode::INVALID_PORT_NUMBER:
                SENSEI_LOG_ERROR("Invalid OSC input port number");
                break;

            default:
                break;
            }
        }
    }
}

void EventHandler::_handle_error(Error* error)
{
    SENSEI_LOG_ERROR("Hardware Error: {}", error->representation());
}

void EventHandler::handle_event(const BaseMessage* event)
{
    std::scoped_lock lock(_sync_mutex);
    // TODO - Fix without casting away constness
    _handle_event(const_cast<BaseMessage*>(event));
    // Todo - handle passed on messages directly
}

SynchronizedQueue<std::unique_ptr<Command>>* EventHandler::outgoing_queue()
{
    return &_to_frontend_queue;
}

void EventHandler::handle_command(const Command* command)
{
}

SynchronizedQueue<std::unique_ptr<BaseMessage>>* EventHandler::incoming_queue()
{
    return &_event_queue;
}