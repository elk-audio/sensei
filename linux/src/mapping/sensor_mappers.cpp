/**
 * @brief Classes that perform transformation on received sensors data
 * @copyright MIND Music Labs AB, Stockholm
 *
 * These are instantiated internally as components of MappingProcessor
 */

#include <algorithm>
#include <cassert>

#include "mapping/sensor_mappers.h"
#include "message/message_factory.h"

using namespace sensei;

BaseSensorMapper::BaseSensorMapper(const PinType pin_type, const int sensor_index) :
    _pin_type(pin_type),
    _sensor_index(sensor_index),
    _sensor_enabled(false),
    _sending_mode(SendingMode::OFF),
    _invert_value(false)
{
}

BaseSensorMapper::~BaseSensorMapper()
{
}

CommandErrorCode BaseSensorMapper::apply_command(const Command *cmd)
{
    CommandErrorCode status = CommandErrorCode::OK;

    // Handle here per-sensor configuration common between sensor types
    switch(cmd->type())
    {
    case CommandType::SET_ENABLED:
        {
            const auto typed_cmd = static_cast<const SetEnabledCommand*>(cmd);
            _sensor_enabled = typed_cmd->data();
        };
        break;

    case CommandType::SET_SENDING_MODE:
        {
            const auto typed_cmd = static_cast<const SetSendingModeCommand*>(cmd);
            _sending_mode = typed_cmd->data();
        };
        break;

    case CommandType::SET_INVERT_ENABLED:
        {
            const auto typed_cmd = static_cast<const SetInvertEnabledCommand*>(cmd);
            _invert_value = typed_cmd->data();
        };
        break;

    default:
        status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
        break;

    }

    return status;

}

void BaseSensorMapper::put_config_commands_into(CommandIterator iterator)
{
    MessageFactory factory;

    *iterator = factory.make_set_enabled_command(_sensor_index, _sensor_enabled);
    *iterator = factory.make_set_sending_mode_command(_sensor_index, _sending_mode);
    *iterator = factory.make_set_invert_enabled_command(_sensor_index, _invert_value);
}

DigitalSensorMapper::DigitalSensorMapper(const int sensor_index) :
    BaseSensorMapper(PinType::DIGITAL_INPUT, sensor_index)
{
}

DigitalSensorMapper::~DigitalSensorMapper()
{
}

CommandErrorCode DigitalSensorMapper::apply_command(const Command *cmd)
{
    // Try to handle generic cases in base class method
    CommandErrorCode status = BaseSensorMapper::apply_command(cmd);

    // command was consumed by parent or some error has occurred while doing it
    if ((status == CommandErrorCode::OK) || (status != CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE) )
    {
        return status;
    }

    // Handle digital-specific configurations, and fail if CommandType is not appropriate for this kind of sensor
    status = CommandErrorCode::OK;
    switch(cmd->type())
    {
    case CommandType::SET_PIN_TYPE:
        {
            const auto typed_cmd = static_cast<const SetPinTypeCommand*>(cmd);
            assert(typed_cmd->data() == PinType::DIGITAL_INPUT);
        };
        break;

    default:
        status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
        break;

    }

    return status;

}

void DigitalSensorMapper::put_config_commands_into(CommandIterator iterator)
{
    BaseSensorMapper::put_config_commands_into(iterator);

    MessageFactory factory;
    *iterator = factory.make_set_pin_type_command(_sensor_index, PinType::DIGITAL_INPUT);
}

AnalogSensorMapper::AnalogSensorMapper(const int sensor_index) :
    BaseSensorMapper(PinType::ANALOG_INPUT, sensor_index),
    _delta_ticks_sending(1),
    _lowpass_filter_order(4),
    _lowpass_cutoff(DEFAULT_LOWPASS_CUTOFF),
    _slider_mode_enabled(false),
    _slider_threshold(0),
    _input_scale_range_low(0),
    _input_scale_range_high((1<<DEFAULT_ADC_BIT_RESOLUTION)-1)
{
    _set_adc_bit_resolution(DEFAULT_ADC_BIT_RESOLUTION);
}

AnalogSensorMapper::~AnalogSensorMapper()
{
}

CommandErrorCode AnalogSensorMapper::apply_command(const Command *cmd)
{
    // Try to handle generic cases in base class method
    CommandErrorCode status = BaseSensorMapper::apply_command(cmd);

    // command was consumed by parent or some error has occurred while doing it
    if ((status == CommandErrorCode::OK) || (status != CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE) )
    {
        return status;
    }

    // Handle analog-specific configurations, and fail if CommandType is not appropriate for this kind of sensor
    status = CommandErrorCode::OK;
    switch(cmd->type())
    {
    // External

    case CommandType::SET_SENDING_DELTA_TICKS:
        {
            const auto typed_cmd = static_cast<const SetSendingDeltaTicksCommand*>(cmd);
            _delta_ticks_sending = typed_cmd->data();
        };
        break;

    case CommandType::SET_ADC_BIT_RESOLUTION:
        {
            const auto typed_cmd = static_cast<const SetADCBitResolutionCommand*>(cmd);
            _set_adc_bit_resolution(typed_cmd->data());
        };
        break;

    case CommandType::SET_LOWPASS_FILTER_ORDER:
        {
            const auto typed_cmd = static_cast<const SetLowpassFilterOrderCommand*>(cmd);
            _lowpass_filter_order = typed_cmd->data();
        };
        break;

    case CommandType::SET_LOWPASS_CUTOFF:
        {
            const auto typed_cmd = static_cast<const SetLowpassCutoffCommand*>(cmd);
            _lowpass_cutoff = typed_cmd->data();
        };
        break;

    case CommandType::SET_SLIDER_MODE_ENABLED:
        {
            const auto typed_cmd = static_cast<const SetSliderModeEnabledCommand*>(cmd);
            _slider_mode_enabled = typed_cmd->data();
        };
        break;

    case CommandType::SET_SLIDER_THRESHOLD:
        {
            const auto typed_cmd = static_cast<const SetSliderThresholdCommand*>(cmd);
            _slider_threshold = typed_cmd->data();
        };
        break;

    // Internal

    case CommandType::SET_INPUT_SCALE_RANGE_LOW:
        {
            const auto typed_cmd = static_cast<const SetInputScaleRangeLow*>(cmd);
            _set_input_scale_range_low(typed_cmd->data());
        };
        break;

    case CommandType::SET_INPUT_SCALE_RANGE_HIGH:
        {
            const auto typed_cmd = static_cast<const SetInputScaleRangeHigh*>(cmd);
            _set_input_scale_range_high(typed_cmd->data());
        };
        break;

    default:
        status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
        break;

    }

    return status;

}

void AnalogSensorMapper::put_config_commands_into(CommandIterator iterator)
{
    BaseSensorMapper::put_config_commands_into(iterator);

    MessageFactory factory;
    *iterator = factory.make_set_pin_type_command(_sensor_index, PinType::ANALOG_INPUT);
    *iterator = factory.make_set_sending_delta_ticks_command(_sensor_index, _delta_ticks_sending);
    *iterator = factory.make_set_adc_bit_resolution_command(_sensor_index, _adc_bit_resolution);
    *iterator = factory.make_set_lowpass_filter_order_command(_sensor_index, _lowpass_filter_order);
    *iterator = factory.make_set_lowpass_cutoff_command(_sensor_index, _lowpass_cutoff);
    *iterator = factory.make_set_slider_mode_enabled_command(_sensor_index, _slider_mode_enabled);
    *iterator = factory.make_set_slider_threshold_command(_sensor_index, _slider_threshold);
    *iterator = factory.make_set_input_scale_range_low(_sensor_index, _input_scale_range_low);
    *iterator = factory.make_set_input_scale_range_high(_sensor_index, _input_scale_range_high);
}

CommandErrorCode AnalogSensorMapper::_set_adc_bit_resolution(const int resolution)
{
    if ((resolution < 1) || (resolution > MAX_ADC_BIT_RESOLUTION))
    {
        return CommandErrorCode::INVALID_RANGE;
    }

    _adc_bit_resolution = resolution;
    _max_allowed_input = (1 << _adc_bit_resolution) - 1;

    _input_scale_range_low = std::min(_input_scale_range_low, _max_allowed_input);
    _input_scale_range_high = std::min(_input_scale_range_high, _max_allowed_input);
    return CommandErrorCode::OK;
}

CommandErrorCode AnalogSensorMapper::_set_input_scale_range_low(const int value)
{
    if ( (value < 0) || (value > (_max_allowed_input-1)) )
    {
        return CommandErrorCode::INVALID_RANGE;
    }

    if (value >= _input_scale_range_high)
    {
        _input_scale_range_low = _input_scale_range_high - 1;
        return CommandErrorCode::CLIP_WARNING;
    }

    _input_scale_range_low = value;
    return CommandErrorCode::OK;
}

CommandErrorCode AnalogSensorMapper::_set_input_scale_range_high(const int value)
{
    if ( (value < 0) || (value > (_max_allowed_input-1)) )
    {
        return CommandErrorCode::INVALID_RANGE;
    }

    if (value <= _input_scale_range_low)
    {
        _input_scale_range_high = _input_scale_range_low + 1;
        return CommandErrorCode::CLIP_WARNING;
    }

    _input_scale_range_high = value;
    return CommandErrorCode::OK;
}
