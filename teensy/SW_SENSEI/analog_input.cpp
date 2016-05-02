#include "analog_input.h"

AnalogInput::AnalogInput(SetupPin* setupPin) : PIN(setupPin)
{
	//SerialDebug.println("AnalogInput");
	_type = ePinType::PIN_ANALOG_INPUT;
	_ADCBitResolution = setupPin->ADCBitResolution;
	_filter.setFilter(setupPin->filterOrder,setupPin->filterCoeff_a,setupPin->filterCoeff_b);
	_sliderMode = setupPin->sliderMode;
	_sliderThreshold = setupPin->sliderThreshold;
	_sliderState = READING;
	_maxValue = USHRT_MAX >> (_ADCBitResolution + 4);
}

void AnalogInput::setPinValue(uint16_t value)
{
	if (_sliderMode)
		{
			_sliderValue = value >> _ADCBitResolution;

	        if ( (_sliderValue == _maxValue) && (_precSliderValue == _maxValue) && (_sliderState == JUMPING))
	        {
	            _sliderState = WAITING;
	        }
	        else if ( (_sliderValue == _maxValue) || (_sliderValue - _precSliderValue)>_sliderThreshold )
	        {
	            _sliderState = JUMPING;
				_pinValueChanged=false;
	        }
	        else
	        {
	            _sliderState = READING;
	        }

			if (_sliderState == READING)
			{
				_setPinValue(_sliderValue);
			}
			_precSliderValue=_sliderValue;
		}
	else
	{
		_setPinValue(static_cast<uint16_t>(_filter.processFilter(value)) >> _ADCBitResolution);
	}
}

AnalogInput::~AnalogInput()
{
	//SerialDebug.println("~AnalogInput");
}
