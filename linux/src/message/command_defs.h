/**
 * @brief Value messages definition
 * @copyright MIND Music Labs AB, Stockholm
 *
 * Declaration of Command Message types using macro facilities in message/command_base.h
 *
 * Define in this file all concrete message types, possibly using the macros
 *
 * SENSEI_DECLARE_EXTERNAL_MESSAGE(ClassName, command_tag, InternalType, representation_prefix)
 *
 * SENSEI_DECLARE_INTERNAL_MESSAGE(ClassName, command_tag, InternalType, representation_prefix)
 *
 * In case of External Messages,
 * Classes defined here have to be instantiated with factory methods provide by message_factory.h
 *
 */
#ifndef SENSEI_COMMAND_DEFS_H
#define SENSEI_COMMAND_DEFS_H

#include "base_command.h"

namespace sensei {

/**
 * @brief Tags used for RTTI emulation.
 */
enum class CommandTag
{
    INTERNAL,
    SET_SAMPLING_RATE,
    SET_PIN_TYPE,
    SET_ENABLED,
    SET_SENDING_MODE,
    SET_SENDING_DELTA_TICKS,
    SET_ADC_BIT_RESOLUTION,
    SET_LOWPASS_CUTOFF,
    SET_SLIDER_MODE_ENABLED,
    SET_SLIDER_THRESHOLD,
    SEND_DIGITAL_PIN_VALUE,
    N_COMMAND_TAGS
};

/**
 * @brief Pin type configuration
 */
enum class PinType
{
    DISABLED,
    DIGITAL_INPUT,
    DIGITAL_OUTPUT,
    ANALOG_INPUT,
    N_PIN_TYPES
};

/**
 * @brief Sending behaviours
 */
enum class SendingMode
{
    OFF,
    CONTINUOUS,
    ON_VALUE_CHANGED,
    ON_REQUEST,
    N_SENDING_MODES
};

////////////////////////////////////////////////////////////////////////////////
// Concrete command subclasses definitions
////////////////////////////////////////////////////////////////////////////////

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetSamplingRateCommand,
                                CommandTag::SET_SAMPLING_RATE,
                                float,
                                "Set Sampling Rate");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetEnabledCommand,
                                CommandTag::SET_ENABLED,
                                bool,
                                "Set Enabled");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetPinTypeCommand,
                                CommandTag::SET_PIN_TYPE,
                                PinType,
                                "Set Pin Type");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetSendingModeCommand,
                                CommandTag::SET_SENDING_MODE,
                                SendingMode ,
                                "Set Sending Mode");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetSendingDeltaTicksCommand,
                                CommandTag::SET_SENDING_DELTA_TICKS,
                                int,
                                "Set Sending Delta Ticks");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetADCBitResolutionCommand,
                                CommandTag::SET_ADC_BIT_RESOLUTION,
                                int,
                                "Set ADC Bit Resolution");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetLowpassCutoffCommand,
                                CommandTag::SET_LOWPASS_CUTOFF,
                                float,
                                "Set Lowpass Cutoff");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetSliderModeEnabledCommand,
                                CommandTag::SET_SLIDER_MODE_ENABLED,
                                bool,
                                "Set Slider Mode Enabled");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SetSliderThresholdCommand,
                                CommandTag::SET_SLIDER_THRESHOLD,
                                int,
                                "Set Slider Threshold");

SENSEI_DECLARE_EXTERNAL_MESSAGE(SendDigitalPinValueCommand,
                                CommandTag::SEND_DIGITAL_PIN_VALUE,
                                bool,
                                "Send Digital Pin Value");

}; // namespace sensei

#endif //SENSEI_COMMAND_DEFS_H
