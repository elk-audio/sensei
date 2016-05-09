#include <iostream>
#include <chrono>

#include "event_handler.h"
#include "output_backend/osc_backend.h"
#include "config_backend/json_configuration.h"
#include "user_frontend/osc_user_frontend.h"
#include "utils.h"
#include "logging.h"

using namespace sensei;

SENSEI_GET_LOGGER;

void EventHandler::init(const std::string port_name,
                        const int max_n_input_pins,
                        const int max_n_digital_out_pins,
                        const std::string config_file)

{
    _processor.reset(new mapping::MappingProcessor(max_n_input_pins));
    _output_backend.reset(new output_backend::OSCBackend(max_n_input_pins));
    _frontend.reset(new serial_frontend::SerialFrontend(port_name, &_to_frontend_queue, &_event_queue));
    _config_backend.reset(new config::JsonConfiguration(&_event_queue, config_file));
    _user_frontend.reset(new user_frontend::OSCUserFrontend(&_event_queue, max_n_digital_out_pins, max_n_input_pins));

    auto ret = _config_backend->read();
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

    if (!_frontend->connected())
    {
        SENSEI_LOG_ERROR("Serial connection failed.");
    }
    _frontend->verify_acks(true);
    _frontend->run();

}

void EventHandler::deinit()
{
    _frontend.reset(nullptr);
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
            _processor->process(static_cast<Value*>(event.get()), _output_backend.get());
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


void EventHandler::_handle_command(std::unique_ptr<Command> cmd)
{
    CommandDestination address = cmd->destination();

    // Process first non-sink destinations
    // using non-owning raw pointer
    if (address & CommandDestination::INTERNAL)
    {
        CommandErrorCode ret = _processor->apply_command(cmd.get());
        if (ret != CommandErrorCode::OK)
        {
            switch (ret)
            {
            case CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE:
                SENSEI_LOG_ERROR("Internal Mapping, Unhandled command: {}, pin: {}", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::INVALID_PIN_INDEX:
                SENSEI_LOG_ERROR("Invalid pin index {} for command: {}", cmd->index(), cmd->representation());
                break;

            case CommandErrorCode::INVALID_RANGE:
                SENSEI_LOG_ERROR("Invalid range for command: {}, pin: {}", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::INVALID_VALUE:
                SENSEI_LOG_ERROR("Invalid range for command: {}, pin: {}", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::CLIP_WARNING:
                SENSEI_LOG_WARNING("Clipped value for command: {}, pin: {}", cmd->representation(), cmd->index());
                break;

            case CommandErrorCode::UNINITIALIZED_PIN:
                SENSEI_LOG_WARNING("Dropping command {} for uninitialized pin {}", cmd->representation(), cmd->index());
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

            default:
                break;
            }
        }
    }

    if (address & CommandDestination::CONFIG_BACKEND)
    {
        // TODO: implement me
    }

    // At last, pass the message to serial receiver queue
    // which is a owning sink
    if (address & CommandDestination::SERIAL_FRONTEND)
    {
        _to_frontend_queue.push(std::move(cmd));
    }

}

void EventHandler::_handle_error(std::unique_ptr<Error> error)
{
    SENSEI_LOG_ERROR("Hardware Error: {}", error->representation());
}
