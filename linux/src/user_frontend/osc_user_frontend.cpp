/**
 * @brief OSC runtime user frontend
 * @copyright MIND Music Labs AB, Stockholm
 */

#include "osc_user_frontend.h"
#include "logging.h"

#include <sstream>

using namespace sensei;
using namespace sensei::user_frontend;

SENSEI_GET_LOGGER;

namespace
{
static const int DEFAULT_SERVER_PORT = 24024;

static void osc_error(int num, const char *msg, const char *path)
{
    SENSEI_LOG_ERROR("liblo server error {} in path {}: {}", num, path, msg);
}

static int osc_set_pin_enabled(const char* /*path*/, const char* /*types*/, lo_arg ** argv, int /*argc*/, void* /*data*/, void *user_data)
{
    OSCUserFrontend *self = static_cast<OSCUserFrontend*>(user_data);
    int pin_idx = argv[0]->i;
    bool enabled = static_cast<bool>(argv[1]->i);
    self->set_pin_enabled(pin_idx, enabled);
    SENSEI_LOG_DEBUG("Setting pin {} to enabled status {}", pin_idx, enabled);

    return 0;
}

static int osc_set_digital_output(const char* /*path*/, const char* /*types*/, lo_arg ** argv, int /*argc*/, void* /*data*/, void *user_data)
{
    OSCUserFrontend *self = static_cast<OSCUserFrontend*>(user_data);
    int pin_idx = argv[0]->i;
    bool value = static_cast<bool>(argv[1]->i);
    self->set_digital_output(pin_idx, value);
    SENSEI_LOG_DEBUG("Sending value {} to digital pin {}", value, pin_idx);

    return 0;
}

}; // anonymous namespace

OSCUserFrontend::OSCUserFrontend(SynchronizedQueue<std::unique_ptr<BaseMessage>> *queue,
                                 const int max_n_digital_out_pins,
                                 const int max_n_input_pins) :
        UserFrontend(queue, max_n_digital_out_pins, max_n_input_pins),
            _osc_server(nullptr),
            _server_port(DEFAULT_SERVER_PORT)
{
    _start_server();
}

CommandErrorCode OSCUserFrontend::apply_command(const Command* /*cmd*/)
{
    // TODO:
    // implement me (at least set port command)
    return CommandErrorCode::OK;
}

void OSCUserFrontend::_start_server()
{
    std::stringstream port_stream;
    port_stream << _server_port;

    _osc_server = lo_server_thread_new(port_stream.str().c_str(), osc_error);
    lo_server_thread_add_method(_osc_server, "/set_pin_enabled", "ii", osc_set_pin_enabled, this);
    lo_server_thread_add_method(_osc_server, "/set_digital_output", "ii", osc_set_digital_output, this);
    int ret = lo_server_thread_start(_osc_server);
    if (ret < 0)
    {
        SENSEI_LOG_ERROR("Error {} while starting OSC server thread", ret);
    }
}

void OSCUserFrontend::_stop_server()
{
    int ret = lo_server_thread_stop(_osc_server);
    if (ret < 0)
    {
        SENSEI_LOG_ERROR("Error {} while stopping OSC server thread", ret);
    }
    lo_server_thread_free(_osc_server);
}

