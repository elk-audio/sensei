/**
 * @brief Error Messages definition
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Errors are used to communicate *exceptional* failure conditions (e.g. hw-related)
 * between the modules. They derive from Error defined in message/base_error.h
 *
 * Define in this file all concrete message types, possibly using the macros
 *
 * SENSEI_DECLARE_VOID_ERROR(ClassName, error_type, representation_prefix)
 *
 * SENSEI_DECLARE_ERROR(ClassName, error_type, InternalType, representation_prefix)
 *
 * Use the second one if you need to attach a payload to the error message,
 * accessible via the data() member function.
 *
 */

#include "message/base_error.h"

#ifndef SENSEI_ERROR_DEFS_H
#define SENSEI_ERROR_DEFS_H

namespace sensei {

enum class ErrorType
{
    BAD_CRC,
    TOO_MANY_TIMEOUTS,
    N_ERROR_TYPES
};

SENSEI_DECLARE_VOID_ERROR(BadCrcError,
                          ErrorType::BAD_CRC,
                          "Bad CRC in external message");

SENSEI_DECLARE_VOID_ERROR(TooManyTimeoutsError,
                          ErrorType::TOO_MANY_TIMEOUTS,
                          "Too many timeouts while trying to send external messages");

}; // namespace sensei

#endif //SENSEI_ERROR_DEFS_H
