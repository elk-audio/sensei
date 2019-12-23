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
 * @brief OSC runtime user frontend
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * Starts a thread listening for OSC commands at the given port
 * (configurable with proper command sent with apply_command.
 *
 * OSC paths and arguments:
 *
 *  /set_pin_enabled    ii
 *  /set_digital_output ii
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
    OSCUserFrontend(SynchronizedQueue<std::unique_ptr<BaseMessage>> *queue, const int max_n_input_pins,
                        const int max_n_digital_out_pins);

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

} // namespace user_frontend
} // namespace sensei

#endif //SENSEI_OSC_USER_FRONTEND_H_H