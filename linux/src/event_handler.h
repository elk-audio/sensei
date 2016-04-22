#ifndef SENSEI_EVENT_HANDLER_H
#define SENSEI_EVENT_HANDLER_H

#include <memory>
#include <string>

#include "synchronized_queue.h"
#include "mapping/mapping_processor.h"
#include "message/message_factory.h"
#include "serial_frontend/serial_frontend.h"
#include <output_backend/output_backend.h>

namespace sensei {

class EventHandler
{
public:
    EventHandler()
    {}

    ~EventHandler()
    {}

    void init(const std::string port_name,
              const int max_n_sensors);

    void handle_events();

    void deinit();

private:
    void _handle_command(Command* cmd);
    void _handle_error(Error* error);

    // Inter-modules communication queues
    SynchronizedQueue<std::unique_ptr<Command>> _to_frontend_queue;
    SynchronizedQueue<std::unique_ptr<BaseMessage>> _event_queue;

    // Sub-components instances
    std::unique_ptr<serial_frontend::SerialFrontend> _frontend;
    std::unique_ptr<mapping::MappingProcessor> _processor;
    std::unique_ptr<output_backend::OutputBackend> _output_backend;
};

}; // namespace sensei

#endif //SENSEI_EVENT_HANDLER_H
