/**
 * @brief Contains the Hardware definitions of the gpios on the elk pi hat.
 * @copyright Modern Ancient Instruments Networked AB, Stockholm
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

} // shiftregister_gpio
} // hw_backend
} // sensei

#endif