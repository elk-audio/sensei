/**
 * @brief Value messages definition
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Declaration of Value Message types using macro facilities in message/value_base.h
 *
 * Define in this file all concrete value types, possibly using the macro in the form
 *
 * SENSEI_DECLARE_VALUE(ClassName, InternalType, representation_prefix)
 *
 * Classes defined here have to be instantiated with factory methods provide by message_factory.h
 *
 */

#ifndef SENSEI_VALUE_DEFS_H
#define SENSEI_VALUE_DEFS_H

#include "base_value.h"

namespace sensei {

SENSEI_DECLARE_VALUE(AnalogValue, int, "Analog Value");

SENSEI_DECLARE_VALUE(DigitalValue, bool, "Digital Value");

SENSEI_DECLARE_VALUE(OutputValue, float, "Output Value");


}; // namespace sensei

#endif //SENSEI_VALUE_DEFS_H
