#define SENSEI_TEST_WITHOUT_CONFIG_BACKEND

#ifdef SENSEI_TEST_WITHOUT_CONFIG_BACKEND
    #include <cstdio>
#endif

#include <chrono>

#include "event_handler.h"
#include "output_backend/osc_backend.h"


using namespace sensei;


void EventHandler::init(const std::string port_name,
                        const int max_n_sensors)
{
    _processor.reset(new mapping::MappingProcessor(max_n_sensors));
    _output_backend.reset(new output_backend::OSCBackend(max_n_sensors));
    _frontend.reset(new serial_frontend::SerialFrontend(port_name, &_to_frontend_queue, &_event_queue));

    // TODO: use TBI logger system
    if (!_frontend->connected())
    {
        fprintf(stderr, "Error: serial connection failed.\n");
    }

    // Start serial reading thread
    _frontend->run();

#ifdef SENSEI_TEST_WITHOUT_CONFIG_BACKEND
    _frontend->verify_acks(false);

    // temp hack: fill some configuration into event queue
    //            to test system with provided dumps
    MessageFactory factory;

    // Pin 0-15 : digital input
    for (int pin_idx = 0; pin_idx < 16; pin_idx++)
    {
        _event_queue.push(factory.make_set_pin_type_command(pin_idx, PinType::DIGITAL_INPUT));
        _event_queue.push(factory.make_set_pin_name_command(pin_idx, std::string("digitaLINO")));
        _event_queue.push(factory.make_set_enabled_command(pin_idx, true));
    }

    // Pin 32-55 : analog input
    for (int pin_idx = 32; pin_idx < 56; pin_idx++)
    {
        _event_queue.push(factory.make_set_pin_type_command(pin_idx, PinType::ANALOG_INPUT));
        _event_queue.push(factory.make_set_pin_name_command(pin_idx, std::string("analoGINO")));
        _event_queue.push(factory.make_set_invert_enabled_command(pin_idx, false));
        _event_queue.push(factory.make_set_input_scale_range_low_command(pin_idx, 35));
        _event_queue.push(factory.make_set_input_scale_range_high_command(pin_idx, 2300));
        _event_queue.push(factory.make_set_enabled_command(pin_idx, true));
    }
#endif

}

void EventHandler::deinit()
{
    _frontend.reset(nullptr);
    _processor.reset(nullptr);
    _output_backend.reset(nullptr);
}

void EventHandler::handle_events()
{
    _event_queue.wait_for_data(std::chrono::milliseconds(0));
    if (! _event_queue.empty())
    {
        std::unique_ptr<BaseMessage> event = _event_queue.pop();
        switch(event->base_type())
        {
        case MessageType::VALUE:
            _processor->process(static_cast<Value*>(event.release()), _output_backend.get());
            break;

        case MessageType::COMMAND:
            _handle_command(static_cast<Command*>(event.release()));
            break;

        case MessageType::ERROR:
            _handle_error(static_cast<Error*>(event.release()));
            break;
        }
    }
}


void EventHandler::_handle_command(Command* cmd)
{
    CommandDestination address = cmd->destination();

    // TODO: check returning error code from apply_command calls
    if (address & CommandDestination::INTERNAL)
    {
        _processor->apply_command(cmd);
    }

    if (address & CommandDestination::SERIAL_FRONTEND)
    {
        _to_frontend_queue.push(std::unique_ptr<Command>(cmd));
    }

    if (address & CommandDestination::CONFIG_BACKEND)
    {
        // TODO: implement me
    }

    if (address & CommandDestination::OUTPUT_BACKEND)
    {
        _output_backend->apply_command(cmd);
    }

}

void EventHandler::_handle_error(Error* /* error */)
{
    // TODO: implement me
}
