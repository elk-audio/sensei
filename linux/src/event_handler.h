#ifndef SENSEI_EVENT_HANDLER_H
#define SENSEI_EVENT_HANDLER_H

#include <memory>
#include <string>
#include <chrono>

#include "synchronized_queue.h"
#include "mapping/mapping_processor.h"
#include "message/message_factory.h"
#include "serial_frontend/serial_frontend.h"
#include "output_backend/output_backend.h"
#include "config_backend/base_configuration.h"

namespace sensei {

class EventHandler
{
public:
    EventHandler()
    {}

    ~EventHandler()
    {}

    void init(const std::string port_name,
              const int max_n_pins,
              const std::string config_file);

    void handle_events(std::chrono::milliseconds wait_period);

    void deinit();

    void reload_config()
    {
        _config_backend->read();
    }

private:
    void _handle_command(std::unique_ptr<Command> cmd);
    void _handle_error(std::unique_ptr<Error> error);

    // Inter-modules communication queues
    SynchronizedQueue<std::unique_ptr<Command>> _to_frontend_queue;
    SynchronizedQueue<std::unique_ptr<BaseMessage>> _event_queue;

    // Sub-components instances
    std::unique_ptr<serial_frontend::SerialFrontend> _frontend;
    std::unique_ptr<mapping::MappingProcessor> _processor;
    std::unique_ptr<output_backend::OutputBackend> _output_backend;
    std::unique_ptr<config::BaseConfiguration> _config_backend;
};

}; // namespace sensei

#endif //SENSEI_EVENT_HANDLER_H
