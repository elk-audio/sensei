#include "analog_input.h"

AnalogInput::AnalogInput(SetupPin* setupPin) : PIN(setupPin)
{
	//SerialDebug.println("AnalogInput");
	_type = ePinType::PIN_ANALOG_INPUT;
	_ADCBitResolution = setupPin->ADCBitResolution;
	_filter.setFilter(setupPin->filterOrder,setupPin->filterCoeff_a,setupPin->filterCoeff_b);
	_sliderMode = setupPin->sliderMode;
	_sliderThreshold = setupPin->sliderThreshold;

}

void AnalogInput::setPinValue(uint16_t value)
{
	_value=static_cast<uint16_t>(_filter.processFilter(value))>>_ADCBitResolution;
	_checkPinChange();
}

AnalogInput::~AnalogInput()
{
	//SerialDebug.println("~AnalogInput");
}
