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
 * @brief Output backend using standard output/error streams
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_STD_STREAM_BACKEND_H
#define SENSEI_STD_STREAM_BACKEND_H

#include "output_backend.h"

namespace sensei {
namespace output_backend {

class StandardStreamBackend : public OutputBackend
{
public:
    StandardStreamBackend(const int max_n_input_pins=64);

    ~StandardStreamBackend()
    {}

    CommandErrorCode apply_command(const Command *cmd) override;

    void send(const OutputValue* transformed_value, const Value* raw_input_value) override;
};

} // namespace output_backend
} // namespace sensei

#endif //SENSEI_STD_STREAM_BACKEND_H
