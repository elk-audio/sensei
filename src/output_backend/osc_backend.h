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
 * @brief Output backend with OSC
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_OSC_BACKEND_H
#define SENSEI_OSC_BACKEND_H

#include <lo/lo.h>
#include "output_backend.h"

namespace sensei {
namespace output_backend {

class OSCBackend : public OutputBackend
{
public:
    OSCBackend(const int max_n_input_pins=64);

    ~OSCBackend()
    {}

    CommandErrorCode apply_command(const Command *cmd) override;

    void send(const OutputValue* transformed_value, const Value* raw_input_value) override;

private:
    void _compute_full_paths();

    CommandErrorCode _compute_address();

    std::string _base_path;
    std::string _base_raw_path;
    std::string _host;
    int _port;
    lo_address  _address;

    std::vector<std::string> _full_out_paths;
    std::vector<std::string> _full_raw_paths;
};

} // namespace output_backend
} // namespace sensei

#endif //SENSEI_OSC_BACKEND_H