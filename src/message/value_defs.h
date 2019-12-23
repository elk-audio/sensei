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
 * @brief Value messages definition
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 *
 * Declaration of Value Message types using macro facilities in message/value_base.h
 *
 * Define in this file all concrete value types, possibly using the macro in the form
 *
 * SENSEI_DECLARE_VALUE(ClassName, InternalType, representation_prefix)
 *
 * Classes defined here have to be instantiated with factory methods provide by message_factory.h
 */
#ifndef SENSEI_VALUE_DEFS_H
#define SENSEI_VALUE_DEFS_H

#include <vector>

#include "base_value.h"

namespace sensei {

/**
 * @brief Tags used for RTTI emulation.
 */
enum class ValueType
{
    ANALOG,
    DIGITAL,
    CONTINUOUS,
    OUTPUT,
    INT_SET,
    FLOAT_SET
};

inline bool is_output_value(const Value* value)
{
    return (value->type() >= ValueType::ANALOG && value->type() <= ValueType::CONTINUOUS);
}

inline bool is_set_value(const Value* value)
{
    return (value->type() >= ValueType::INT_SET && value->type() <= ValueType::FLOAT_SET);
}

SENSEI_DECLARE_VALUE(AnalogValue, ValueType::ANALOG, int, "Analog Value");

SENSEI_DECLARE_VALUE(DigitalValue, ValueType::DIGITAL, bool, "Digital Value");

SENSEI_DECLARE_VALUE(ContinuousValue, ValueType::CONTINUOUS, float, "Continuous Value");

SENSEI_DECLARE_VALUE(OutputValue, ValueType::OUTPUT, float, "Output Value");

SENSEI_DECLARE_VALUE(IntegerSetValue, ValueType::INT_SET, int, "Analog SetValue");

SENSEI_DECLARE_VALUE(FloatSetValue, ValueType::FLOAT_SET, float, "Continuous SetValue");


} // namespace sensei

#endif //SENSEI_VALUE_DEFS_H