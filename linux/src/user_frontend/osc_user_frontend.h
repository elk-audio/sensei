/**
 * @brief OSC runtime user frontend
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Starts a thread listening for OSC commands at the given port
 * (configurable with proper command sent with apply_command.
 *
 * OSC paths and arguments:
 *
 *  /set_pin_enabled    ii
 *  /set_digital_output ii
 *
 */
#ifndef SENSEI_OSC_USER_FRONTEND_H_H
#define SENSEI_OSC_USER_FRONTEND_H_H

#include "user_frontend.h"
#include "lo/lo.h"

namespace sensei {
namespace user_frontend {

class OSCUserFrontend : public UserFrontend
{
public:
    OSCUserFrontend(SynchronizedQueue<std::unique_ptr<BaseMessage>> *queue,
                        const int max_n_digital_out_pins, const int max_n_input_pins);

    ~OSCUserFrontend()
    {
        _stop_server();
    }

    CommandErrorCode apply_command(const Command *cmd) override;

private:
    void _start_server();

    void _stop_server();

    lo_server_thread _osc_server;
    int _server_port;
};

}; // namespace user_frontend
}; // namespace sensei

#endif //SENSEI_OSC_USER_FRONTEND_H_H
