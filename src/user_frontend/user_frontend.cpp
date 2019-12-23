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
 * @brief Base class for runtime user frontend
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * This module give run-time control from the user over some fast-changing configuration
 * parameters (e.g. sensors enabled/disabled) and access to digital output pins.
 */
#include "user_frontend.h"

using namespace sensei;
using namespace sensei::user_frontend;

CommandErrorCode UserFrontend::apply_command(const Command* /*cmd*/)
{
    auto status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
    return status;
}

void UserFrontend::set_enabled(int sensor_index, bool enabled)
{
    auto msg = _factory.make_set_enabled_command(sensor_index, enabled);
    _queue->push(std::move(msg));
}

void UserFrontend::set_digital_output(int index, bool value)
{
    auto msg = _factory.make_integer_set_value(index, value? 1:0);
    _queue->push(std::move(msg));
}

void UserFrontend::set_continuous_output(int index, float value)
{
    auto msg = _factory.make_float_set_value(index, value);
    _queue->push(std::move(msg));
}

void UserFrontend::set_range_output(int index, int value)
{
    auto msg = _factory.make_integer_set_value(index, value);
    _queue->push(std::move(msg));
}