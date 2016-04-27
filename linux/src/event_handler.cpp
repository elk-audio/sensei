#include <iostream>
#include <chrono>

#include "event_handler.h"
#include "output_backend/osc_backend.h"
#include "config_backend/json_configuration.h"
#include "utils.h"


using namespace sensei;


void EventHandler::init(const std::string port_name,
                        const int max_n_pins,
                        const std::string config_file)

{
    _processor.reset(new mapping::MappingProcessor(max_n_pins));
    _output_backend.reset(new output_backend::OSCBackend(max_n_pins));
    _frontend.reset(new serial_frontend::SerialFrontend(port_name, &_to_frontend_queue, &_event_queue));
    _config_backend.reset(new config::JsonConfiguration(&_event_queue, config_file));

    _config_backend->read();

    // TODO: use TBI logger system
    if (!_frontend->connected())
    {
        std::cerr << "Error: serial connection failed." << std::endl;
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
            {
                auto value = static_unique_ptr_cast<Value, BaseMessage>(std::move(event));
                _processor->process(std::move(value), _output_backend.get());
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


void EventHandler::_handle_command(std::unique_ptr<Command> cmd)
{
    CommandDestination address = cmd->destination();

    // TODO: check returning error code from apply_command calls
    if (address & CommandDestination::INTERNAL)
    {
        _processor->apply_command(std::move(cmd));
    }

    if (address & CommandDestination::SERIAL_FRONTEND)
    {
        _to_frontend_queue.push(std::move(cmd));
    }

    if (address & CommandDestination::CONFIG_BACKEND)
    {
        // TODO: implement me
    }

    if (address & CommandDestination::OUTPUT_BACKEND)
    {
        _output_backend->apply_command(std::move(cmd));
    }

}

void EventHandler::_handle_error(std::unique_ptr<Error> /* error */)
{
    // TODO: implement me
}
