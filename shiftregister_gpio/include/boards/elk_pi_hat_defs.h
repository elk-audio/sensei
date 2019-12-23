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
 * @brief Contains the Hardware definitions of the gpios on the elk pi hat.
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef ELK_PI_HAT_DEFS_H_
#define ELK_PI_HAT_DEFS_H_

namespace sensei {
namespace hw_backend {
namespace shiftregister_gpio {

// Board definitions
constexpr int NUM_DIGITAL_OUTPUTS = 32;
constexpr int NUM_DIGITAL_INPUTS = 32;
constexpr int NUM_ANALOG_INPUTS = 16;
constexpr int ADC_RES_IN_BITS = 10;

} // namespace shiftregister_gpio
} // namespace hw_backend
} // namespace sensei

#endif