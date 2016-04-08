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

#include <memory>
#include <vector>

#include "base_command.h"

namespace sensei {

/**
 * @brief Tags used for RTTI emulation.
 */
enum class CommandType
{
    // External Commands
    SET_SAMPLING_RATE,
    SET_PIN_TYPE,
    SET_ENABLED,
    SET_SENDING_MODE,
    SET_SENDING_DELTA_TICKS,
    SET_ADC_BIT_RESOLUTION,
    SET_LOWPASS_FILTER_ORDER,
    SET_LOWPASS_CUTOFF,
    SET_SLIDER_MODE_ENABLED,
    SET_SLIDER_THRESHOLD,
    SEND_DIGITAL_PIN_VALUE,
    // Internal Commands
    SET_INVERT_ENABLED,
    SET_INPUT_SCALE_RANGE_LOW,
    SET_INPUT_SCALE_RANGE_HIGH,
    N_COMMAND_TAGS
};

/**
 * @brief Pin type configuration
 */
enum class PinType
{
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

enum class CommandErrorCode
{
    OK,
    UNHANDLED_COMMAND_FOR_SENSOR_TYPE,
    INVALID_RANGE,
    CLIP_WARNING,
    N_COMMAND_ERROR_CODES
};

////////////////////////////////////////////////////////////////////////////////
// Concrete command subclasses definitions
////////////////////////////////////////////////////////////////////////////////

// External commands

SENSEI_DECLARE_EXTERNAL_COMMAND(SetSamplingRateCommand,
                                CommandType::SET_SAMPLING_RATE,
                                float,
                                "Set Sampling Rate");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetEnabledCommand,
                                CommandType::SET_ENABLED,
                                bool,
                                "Set Enabled");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetPinTypeCommand,
                                CommandType::SET_PIN_TYPE,
                                PinType,
                                "Set Pin Type");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetSendingModeCommand,
                                CommandType::SET_SENDING_MODE,
                                SendingMode ,
                                "Set Sending Mode");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetSendingDeltaTicksCommand,
                                CommandType::SET_SENDING_DELTA_TICKS,
                                int,
                                "Set Sending Delta Ticks");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetADCBitResolutionCommand,
                                CommandType::SET_ADC_BIT_RESOLUTION,
                                int,
                                "Set ADC Bit Resolution");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetLowpassFilterOrderCommand,
                                CommandType::SET_LOWPASS_FILTER_ORDER,
                                int,
                                "Set Lowpass Filter order");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetLowpassCutoffCommand,
                                CommandType::SET_LOWPASS_CUTOFF,
                                float,
                                "Set Lowpass Cutoff");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetSliderModeEnabledCommand,
                                CommandType::SET_SLIDER_MODE_ENABLED,
                                bool,
                                "Set Slider Mode Enabled");

SENSEI_DECLARE_EXTERNAL_COMMAND(SetSliderThresholdCommand,
                                CommandType::SET_SLIDER_THRESHOLD,
                                int,
                                "Set Slider Threshold");

SENSEI_DECLARE_EXTERNAL_COMMAND(SendDigitalPinValueCommand,
                                CommandType::SEND_DIGITAL_PIN_VALUE,
                                bool,
                                "Send Digital Pin Value");

// Internal commands

SENSEI_DECLARE_INTERNAL_COMMAND(SetInvertEnabledCommand,
                                CommandType::SET_INVERT_ENABLED,
                                bool,
                                "Set Invert Enabled");

SENSEI_DECLARE_INTERNAL_COMMAND(SetInputScaleRangeLow,
                                CommandType::SET_INPUT_SCALE_RANGE_LOW,
                                int,
                                "Set Input Scale Range Low");

SENSEI_DECLARE_INTERNAL_COMMAND(SetInputScaleRangeHigh,
                                CommandType::SET_INPUT_SCALE_RANGE_HIGH,
                                int,
                                "Set Input Scale Range Low");


}; // namespace sensei

#endif //SENSEI_COMMAND_DEFS_H
