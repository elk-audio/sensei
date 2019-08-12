#include <iostream>
#include <chrono>

#include "event_handler.h"
#include "output_backend/osc_backend.h"
#include "config_backend/json_configuration.h"
#include "user_frontend/osc_user_frontend.h"
#include "hardware_frontend/serial_frontend.h"
#include "hardware_frontend/raspa_frontend.h"
#include "utils.h"
#include "logging.h"

using namespace sensei;

SENSEI_GET_LOGGER_WITH_MODULE_NAME("eventhandler");

void EventHandler::init(int max_n_input_pins,
                        int max_n_digital_out_pins,
                        const std::string& config_file)
{
    _processor.reset(new mapping::MappingProcessor(max_n_input_pins));
    _output_backend.reset(new output_backend::OSCBackend(max_n_input_pins));
    _config_backend.reset(new config::JsonConfiguration(&_event_queue, config_file));
    _user_frontend.reset(new user_frontend::OSCUserFrontend(&_event_queue, max_n_input_pins, max_n_digital_out_pins));
    config::HwFrontendConfig hw_config;
    auto ret = _config_backend->read(hw_config);
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

    if (hw_config.type == HwFrontendType::SERIAL_TEENSY)
    {
        SENSEI_LOG_INFO("Initialising a Serial Teensy Frontend");
        _hw_frontend.reset(new hw_frontend::SerialFrontend(hw_config.port, &_to_frontend_queue, &_event_queue));
    }
    else if (hw_config.type == HwFrontendType::RASPA_GPIO)
    {
        SENSEI_LOG_INFO("Initializing a Raspa GPIO Frontend");
        _hw_frontend.reset(new hw_frontend::RaspaFrontend(&_to_frontend_queue, &_event_queue));
    }
    else
    {
        _hw_frontend.reset(new hw_frontend::NoOpFrontend(&_to_frontend_queue, &_event_queue));
        SENSEI_LOG_ERROR("No HW Frontend configured");
    }

    if (!_hw_frontend->connected())
    {
        SENSEI_LOG_ERROR("Hardware connection failed.");
    }
    _hw_frontend->verify_acks(true);
    _hw_frontend->run();
}

void EventHandler::deinit()
{
    _hw_frontend->stop();
    _hw_frontend.reset(nullptr);
    _processor.reset(nullptr);
    _output_backend.reset(nullptr);
    _config_backend.reset(nullptr);
}

void EventHandler::handle_events(std::chrono::milliseconds wait_period)
{
    _event_queue.wait_for_data(wait_period);
    while (! _event_queue.empty())
    {
        std::unique_ptr<BaseMessage> event = _event_queue.pop();
        switch(event->base_type())
        {
        case MessageType::VALUE:
            {
                auto value = static_unique_ptr_cast<Value, BaseMessage>(std::move(event));
                _handle_value(std::move(value));
            }
            break;

        case MessageType::COMMAND:
            {
                auto cmd = static_unique_ptr_cast<Command, BaseMessage>(std::move(event));
                _handle_command(std::move(cmd));
            }
            break;

        case MessageType::ERROR:
            {
                auto error = static_unique_ptr_cast<Error, BaseMessage>(std::move(event));
                _handle_error(std::move(error));
            }
            break;
        }
    }
}

void EventHandler::_handle_value(std::unique_ptr<Value> value)
{
    if (is_output_value(value.get()))
    {
        _processor->process(value.get(), _output_backend.get());
    }
    else if (is_set_value(value.get()))
    {
        auto cmd = _processor->process_set(value.get());
        if (cmd != nullptr)
        {
            _to_frontend_queue.push(std::move(cmd));
        }
    }
}

void EventHandler::_handle_command(std::unique_ptr<Command> cmd)
{
    CommandDestination address = cmd->destination();

    // Process first non-sink destinations
    // using non-owning raw pointer
    if (address & CommandDestination::MAPPING_PROCESSOR)
    {
        CommandErrorCode ret = _processor->apply_command(cmd.get());
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
        CommandErrorCode ret = _output_backend->apply_command(cmd.get());
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
        CommandErrorCode ret = _user_frontend->apply_command(cmd.get());
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

    // At last, pass the message to serial receiver queue
    // which is a owning sink
    if (address & CommandDestination::HARDWARE_FRONTEND)
    {
        _to_frontend_queue.push(std::move(cmd));
    }

}

void EventHandler::_handle_error(std::unique_ptr<Error> error)
{
    SENSEI_LOG_ERROR("Hardware Error: {}", error->representation());
}
