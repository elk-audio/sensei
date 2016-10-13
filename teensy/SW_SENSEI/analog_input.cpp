#include "analog_input.h"
#include <cmath>

using namespace sensei;

AnalogInput::AnalogInput(SetupPin* setupPin) : PIN(setupPin)
{
	//SerialDebug.println("AnalogInput");
	_type = ePinType::PIN_ANALOG_INPUT;
	_ADCBitResolution = setupPin->ADCBitResolution;
	_filter.setFilter(setupPin->filterOrder,setupPin->filterCoeff_a,setupPin->filterCoeff_b);
	_sliderMode = setupPin->sliderMode;
	_sliderThreshold = setupPin->sliderThreshold;
	_sliderState = WAITING;
  _jumpedSliderValue = _maxValue;
	_maxValue = USHRT_MAX >> (_ADCBitResolution + 4);
}

void AnalogInput::setPinValue(uint16_t value)
{
	uint16_t filtered_value = static_cast<uint16_t>(_filter.processFilter(value))  >> _ADCBitResolution;
	if (_sliderMode)
        // Slider mode bypass filter during jumps in sensor value.
        // This is used mostly for ribbon sensors in order to don't send intermediate filtered
        // values when the sensor is pressed or released
        //
        // FSM states are:
        //      WAITING :
        //          no activity on the sensor, wait for jumping and discard values under threshold
        //      JUMPING :
        //          a jump has detected from WAITING state. Send the unfiltered value and discard
        //          further values if their distance from the first one is less than threshold.
        //          If it is greater, go to READING mode
        //      READING :
        //          normal operating mode. Send out filtered values unless they are below the
        //          dead zone threshold, in which case send directly the maximum value and go
        //          to WAITING state
		{
			_sliderValue = value >> _ADCBitResolution;
            switch (_sliderState)
            {
            case WAITING:
				if ((_maxValue - _sliderValue) >= _sliderThreshold)
				{
					_sliderState = JUMPING;
                    _jumpedSliderValue = _sliderValue;
                    _setPinValue(_jumpedSliderValue);
				}
                else
                {
                    _setPinValue(_maxValue);
                }
                break;

            case JUMPING:
                // Discard values if they are close to the first pressed position,
                // as they are likely to come from crosstalk noise or filter ramping
                if (   ((_sliderValue > _jumpedSliderValue) && ((_sliderValue - _jumpedSliderValue) <= _sliderThreshold))
                    || ((_sliderValue <= _jumpedSliderValue) && ((_jumpedSliderValue - _sliderValue) <= _sliderThreshold))
                   )
                {
                    _setPinValue(_jumpedSliderValue);
                }
                else if ((_maxValue - _sliderValue) < _sliderThreshold)
				{
					_sliderState = WAITING;
                    _setPinValue(_maxValue);
				}
                else
                {
                    _setPinValue(filtered_value);
                    _sliderState = READING;
                }
                break;

            case READING:
                // If value is in the dead zone range, the finger has been released:
                // send maximum value and go to WAITING state
				if ((_maxValue - _sliderValue) < _sliderThreshold)
				{
					_sliderState = WAITING;
                    _setPinValue(_maxValue);
				}
				else
				{
					_setPinValue(filtered_value);
				}
                break;
            }
		}
	else
    // no slider mode, just send out the filtered value
	{
		_setPinValue(filtered_value);
	}
}

AnalogInput::~AnalogInput()
{
	//SerialDebug.println("~AnalogInput");
}
