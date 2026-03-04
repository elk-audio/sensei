/*
 * Copyright 2017-2026 Elk Audio AB
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
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 *
 * This module give run-time control from the user over some fast-changing configuration
 * parameters (e.g. sensors enabled/disabled) and access to output sensors.
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
    if (_threading_mode == ThreadingMode::ASYNCHRONOUS)
    {
        _handler->post_event(_factory.make_set_enabled_command(sensor_index, enabled));
    }
    else
    {
        auto m = SetEnabledCommand(sensor_index, enabled);
        _handler->process_event(&m);
    }
}

void UserFrontend::set_digital_output(int index, bool value)
{
    if (_threading_mode == ThreadingMode::ASYNCHRONOUS)
    {
        _handler->post_event(_factory.make_integer_set_value(index, value ? 1 : 0));
    }
    else
    {
        auto e = IntegerSetValue(index, value ? 1 : 0);
        _handler->process_event(&e);
    }
}

void UserFrontend::set_continuous_output(int index, float value)
{
    if (_threading_mode == ThreadingMode::ASYNCHRONOUS)
    {
        _handler->post_event(_factory.make_float_set_value(index, value));
    }
    else
    {
        auto e = FloatSetValue(index, value);
        _handler->process_event(&e);
    }
}

void UserFrontend::set_range_output(int index, int value)
{
    if (_threading_mode == ThreadingMode::ASYNCHRONOUS)
    {
        _handler->post_event(_factory.make_integer_set_value(index, value));
    }
    else
    {
        auto e = IntegerSetValue(index, value);
        _handler->process_event(&e);
    }
}
