/**
 * @brief Classes that perform transformation on received sensors data
 * @copyright MIND Music Labs AB, Stockholm
 *
 * These are instantiated internally as components of MappingProcessor
 */

#include <cassert>
#include <cmath>

#include "mapping/sensor_mappers.h"
#include "message/message_factory.h"
#include "utils.h"
#include "logging.h"

namespace {

static const int MAX_ADC_BIT_RESOLUTION = 16;
static const int DEFAULT_ADC_BIT_RESOLUTION = 12;
static const float DEFAULT_LOWPASS_CUTOFF = 100.0f;
static const int MAX_LOWPASS_FILTER_ORDER = 8;


}; // Anonymous namespace

SENSEI_GET_LOGGER;

using namespace sensei;
using namespace sensei::mapping;


////////////////////////////////////////////////////////////////////////////////
// BaseSensorMapper
////////////////////////////////////////////////////////////////////////////////

BaseSensorMapper::BaseSensorMapper(const PinType pin_type, const int pin_index) :
    _pin_type(pin_type),
    _pin_index(pin_index),
    _pin_enabled(false),
    _sending_mode(SendingMode::OFF),
    _invert_value(false)
{
}

BaseSensorMapper::~BaseSensorMapper()
{
}

CommandErrorCode BaseSensorMapper::apply_command(const Command *cmd)
{
    assert(cmd->index() == _pin_index);

    CommandErrorCode status = CommandErrorCode::OK;

    // Handle here per-pin configuration common between sensor types
    switch(cmd->type())
    {
    case CommandType::SET_ENABLED:
        {
            const auto typed_cmd = static_cast<const SetEnabledCommand*>(cmd);
            _pin_enabled = typed_cmd->data();
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

void BaseSensorMapper::put_config_commands_into(CommandIterator out_iterator)
{
    MessageFactory factory;

    *out_iterator = factory.make_set_enabled_command(_pin_index, _pin_enabled);
    *out_iterator = factory.make_set_sending_mode_command(_pin_index, _sending_mode);
    *out_iterator = factory.make_set_invert_enabled_command(_pin_index, _invert_value);
}

////////////////////////////////////////////////////////////////////////////////
// DigitalSensorMapper
////////////////////////////////////////////////////////////////////////////////

DigitalSensorMapper::DigitalSensorMapper(const int pin_index) :
    BaseSensorMapper(PinType::DIGITAL_INPUT, pin_index)
{
}

DigitalSensorMapper::~DigitalSensorMapper()
{
}

CommandErrorCode DigitalSensorMapper::apply_command(const Command *cmd)
{
    // Handle digital-specific configurations
    CommandErrorCode status = CommandErrorCode::OK;
    switch(cmd->type())
    {
    case CommandType::SET_PIN_TYPE:
        {
            // This is set internally and not from the user, so just assert for bugs
            assert(static_cast<const SetPinTypeCommand*>(cmd)->data() == PinType::DIGITAL_INPUT);
        };
        break;

    default:
        status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
        break;

    }

    // If command was not handled, try to handle it in parent
    if (status == CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE)
    {
        return BaseSensorMapper::apply_command(cmd);
    }
    else
    {
        return status;
    }

}

void DigitalSensorMapper::put_config_commands_into(CommandIterator out_iterator)
{
    BaseSensorMapper::put_config_commands_into(out_iterator);

    MessageFactory factory;
    *out_iterator = factory.make_set_pin_type_command(_pin_index, PinType::DIGITAL_INPUT);
}

void DigitalSensorMapper::process(Value *value, output_backend::OutputBackend *backend)
{
    if (! _pin_enabled)
    {
        return;
    }
    assert(value->type() == ValueType::DIGITAL);

    auto digital_val = static_cast<DigitalValue*>(value);
    float out_val = digital_val->value() ? 1.0f : 0.0f;
    if (_invert_value)
    {
        out_val = 1.0f - out_val;
    }

    MessageFactory factory;
    // Use temporary variable here, since if the factory method is created inside the temporary rvalue expression
    // it gets optimized away by the compiler in release mode
    auto temp_msg = factory.make_output_value(_pin_index,
                                              out_val,
                                              value->timestamp());
    auto transformed_value = static_cast<OutputValue*>(temp_msg.get());
    backend->send(transformed_value, value);
}

////////////////////////////////////////////////////////////////////////////////
// AnalogSensorMapper
////////////////////////////////////////////////////////////////////////////////

AnalogSensorMapper::AnalogSensorMapper(const int pin_index,
                                       const float adc_sampling_rate) :
    BaseSensorMapper(PinType::ANALOG_INPUT, pin_index),
    _delta_ticks_sending(1),
    _lowpass_filter_order(4),
    _lowpass_cutoff(DEFAULT_LOWPASS_CUTOFF),
    _slider_mode_enabled(false),
    _slider_threshold(0),
    _input_scale_range_low(0),
    _input_scale_range_high((1<<DEFAULT_ADC_BIT_RESOLUTION)-1),
    _adc_sampling_rate(adc_sampling_rate)
{
    _set_adc_bit_resolution(DEFAULT_ADC_BIT_RESOLUTION);
}

AnalogSensorMapper::~AnalogSensorMapper()
{
}

CommandErrorCode AnalogSensorMapper::apply_command(const Command *cmd)
{

    // Handle analog-specific configurations, and fail if CommandType is not appropriate for this kind of sensor
    CommandErrorCode  status = CommandErrorCode::OK;
    switch(cmd->type())
    {
    // External
    case CommandType::SET_PIN_TYPE:
        {
            // This is set internally and not from the user, so just assert for bugs
            assert(static_cast<const SetPinTypeCommand*>(cmd)->data() == PinType::ANALOG_INPUT);
        };
        break;

    case CommandType::SET_SENDING_DELTA_TICKS:
        {
            const auto typed_cmd = static_cast<const SetSendingDeltaTicksCommand*>(cmd);
            status = _set_delta_ticks_sending(typed_cmd->data());
        };
        break;

    case CommandType::SET_ADC_BIT_RESOLUTION:
        {
            const auto typed_cmd = static_cast<const SetADCBitResolutionCommand*>(cmd);
            status = _set_adc_bit_resolution(typed_cmd->data());
        };
        break;

    case CommandType::SET_LOWPASS_FILTER_ORDER:
        {
            const auto typed_cmd = static_cast<const SetLowpassFilterOrderCommand*>(cmd);
            status = _set_lowpass_filter_order(typed_cmd->data());
        };
        break;

    case CommandType::SET_LOWPASS_CUTOFF:
        {
            const auto typed_cmd = static_cast<const SetLowpassCutoffCommand*>(cmd);
            status = _set_lowpass_cutoff(typed_cmd->data());
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
            status = _set_slider_threshold(typed_cmd->data());
        };
        break;

    // Internal

    case CommandType::SET_INPUT_SCALE_RANGE_LOW:
        {
            const auto typed_cmd = static_cast<const SetInputScaleRangeLow*>(cmd);
            status = _set_input_scale_range_low(static_cast<int>(std::round(typed_cmd->data())));
        };
        break;

    case CommandType::SET_INPUT_SCALE_RANGE_HIGH:
        {
            const auto typed_cmd = static_cast<const SetInputScaleRangeHigh*>(cmd);
            status = _set_input_scale_range_high(static_cast<int>(std::round(typed_cmd->data())));
        };
        break;

    default:
        status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
        break;

    }

    // If command was not handled, try to handle it in parent
    if (status == CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE)
    {
        return BaseSensorMapper::apply_command(cmd);
    }
    else
    {
        return status;
    }

}

void AnalogSensorMapper::put_config_commands_into(CommandIterator out_iterator)
{
    BaseSensorMapper::put_config_commands_into(out_iterator);

    MessageFactory factory;
    *out_iterator = factory.make_set_pin_type_command(_pin_index, PinType::ANALOG_INPUT);
    *out_iterator = factory.make_set_sending_delta_ticks_command(_pin_index, _delta_ticks_sending);
    *out_iterator = factory.make_set_adc_bit_resolution_command(_pin_index, _adc_bit_resolution);
    *out_iterator = factory.make_set_lowpass_filter_order_command(_pin_index, _lowpass_filter_order);
    *out_iterator = factory.make_set_lowpass_cutoff_command(_pin_index, _lowpass_cutoff);
    *out_iterator = factory.make_set_slider_mode_enabled_command(_pin_index, _slider_mode_enabled);
    *out_iterator = factory.make_set_slider_threshold_command(_pin_index, _slider_threshold);
    *out_iterator = factory.make_set_input_scale_range_low_command(_pin_index, _input_scale_range_low);
    *out_iterator = factory.make_set_input_scale_range_high_command(_pin_index, _input_scale_range_high);
}

void AnalogSensorMapper::process(Value* value, output_backend::OutputBackend* backend)
{
    if (! _pin_enabled)
    {
        return;
    }
    assert(value->type() == ValueType::ANALOG);

    auto analog_val = static_cast<AnalogValue*>(value);
    int clipped_val = clip<int>(analog_val->value(), _input_scale_range_low, _input_scale_range_high);
    float out_val =   static_cast<float>(clipped_val - _input_scale_range_low)
                    / static_cast<float>(_input_scale_range_high - _input_scale_range_low);
    if (_invert_value)
    {
        out_val = 1.0f - out_val;
    }

    MessageFactory factory;
    // Use temporary variable here, since if the factory method is created inside the temporary rvalue expression
    // it gets optimized away by the compiler in release mode
    auto temp_msg = factory.make_output_value(_pin_index,
                                              out_val,
                                              value->timestamp());
    auto transformed_value = static_cast<OutputValue*>(temp_msg.get());
    backend->send(transformed_value, value);
}

CommandErrorCode AnalogSensorMapper::_set_adc_bit_resolution(const int resolution)
{
    if ((resolution < 1) || (resolution > MAX_ADC_BIT_RESOLUTION))
    {
        return CommandErrorCode::INVALID_VALUE;
    }

    _adc_bit_resolution = resolution;
    _max_allowed_input = (1 << _adc_bit_resolution) - 1;

    _input_scale_range_low = std::min(_input_scale_range_low, _max_allowed_input);
    _input_scale_range_high = std::min(_input_scale_range_high, _max_allowed_input);
    return CommandErrorCode::OK;
}

CommandErrorCode AnalogSensorMapper::_set_delta_ticks_sending(const int value)
{
    if (value < 1)
    {
        return CommandErrorCode::INVALID_VALUE;
    }

    _delta_ticks_sending = value;
    return CommandErrorCode::OK;
}

CommandErrorCode AnalogSensorMapper::_set_lowpass_filter_order(const int value)
{
    if ((value < 1) || (value > MAX_LOWPASS_FILTER_ORDER))
    {
        return CommandErrorCode::INVALID_VALUE;
    }

    _lowpass_filter_order = value;
    return CommandErrorCode::OK;
}

CommandErrorCode AnalogSensorMapper::_set_lowpass_cutoff(const float value)
{
    if ( (value <= 0.0f) || (value >= (0.5f * _adc_sampling_rate)) )
    {
        return CommandErrorCode::INVALID_VALUE;
    }

    _lowpass_cutoff = value;
    return CommandErrorCode::OK;
}

CommandErrorCode AnalogSensorMapper::_set_slider_threshold(const int value)
{
    if ( (value < 0) || (value > (_max_allowed_input-1)) )
    {
        return CommandErrorCode::INVALID_VALUE;
    }

    _slider_threshold = value;
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

////////////////////////////////////////////////////////////////////////////////
// ImuMapper
////////////////////////////////////////////////////////////////////////////////

ImuMapper::ImuMapper(const int pin_index) :
        BaseSensorMapper(PinType::IMU_INPUT, pin_index),
        _input_scale_range_low(-M_PI),
        _input_scale_range_high(M_PI),
        _max_allowed_input(M_PI * 2)
{
}

ImuMapper::~ImuMapper()
{
}

CommandErrorCode ImuMapper::apply_command(const Command *cmd)
{
    // Handle digital-specific configurations
    CommandErrorCode status = CommandErrorCode::OK;
    SENSEI_LOG_INFO("ImuMapper: Got a set pintype command {}!", (int)cmd->type());
    switch(cmd->type())
    {
        case CommandType::SET_PIN_TYPE:
        {
            // This is set internally and not from the user, so just assert for bugs
            assert(static_cast<const SetPinTypeCommand*>(cmd)->data() == PinType::IMU_INPUT);
            break;
        }
        case CommandType::SET_INPUT_SCALE_RANGE_LOW:
        {
            const auto typed_cmd = static_cast<const SetInputScaleRangeLow*>(cmd);
            status = _set_input_scale_range_low(typed_cmd->data());
            break;
        }
        case CommandType::SET_INPUT_SCALE_RANGE_HIGH:
        {
            const auto typed_cmd = static_cast<const SetInputScaleRangeHigh*>(cmd);
            status = _set_input_scale_range_high(typed_cmd->data());
            break;
        }
        default:
            status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
            break;

    }

    // If command was not handled, try to handle it in parent
    if (status == CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE)
    {
        return BaseSensorMapper::apply_command(cmd);
    }
    else
    {
        return status;
    }

}

void ImuMapper::put_config_commands_into(CommandIterator out_iterator)
{
    BaseSensorMapper::put_config_commands_into(out_iterator);

    MessageFactory factory;
    *out_iterator = factory.make_set_pin_type_command(_pin_index, PinType::IMU_INPUT);
    *out_iterator = factory.make_set_input_scale_range_low_command(_pin_index, _input_scale_range_low);
    *out_iterator = factory.make_set_input_scale_range_high_command(_pin_index, _input_scale_range_high);
}

void ImuMapper::process(Value *value, output_backend::OutputBackend *backend)
{
    if (! _pin_enabled)
    {
        return;
    }
    assert(value->type() == ValueType::IMU);

    auto imu_val = static_cast<ImuValue*>(value);
    float clipped_val = clip<float>(imu_val->value(), _input_scale_range_low, _input_scale_range_high);
    float out_val = (clipped_val - _input_scale_range_low) / (_input_scale_range_high - _input_scale_range_low);

    if (_invert_value)
    {
        out_val = 1.0f - out_val;
    }

    MessageFactory factory;
    // Use temporary variable here, since if the factory method is created inside the temporary rvalue expression
    // it gets optimized away by the compiler in release mode
    auto temp_msg = factory.make_output_value(_pin_index,
                                              out_val,
                                              value->timestamp());
    auto transformed_value = static_cast<OutputValue*>(temp_msg.get());
    backend->send(transformed_value, value);
}

CommandErrorCode ImuMapper::_set_input_scale_range_low(const float value)
{
    if ( std::abs(value) > _max_allowed_input )
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

CommandErrorCode ImuMapper::_set_input_scale_range_high(const float value)
{
    if ( std::abs(value) > _max_allowed_input )
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