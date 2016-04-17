#include "analog_input.h"

AnalogInput::AnalogInput(SetupPin* _setupPin) : DigitalInput(_setupPin)
{
	//SerialDebug.println("AnalogInput");

	type = ePinType::PIN_ANALOG_INPUT;

	ADCBitResolution = _setupPin->ADCBitResolution;

	filter.set_filter(_setupPin->filterOrder,_setupPin->filterCoeff_a,_setupPin->filterCoeff_b);

	sliderMode = _setupPin->sliderMode;
	sliderThreshold = _setupPin->sliderThreshold;

}

void AnalogInput::setPinValue(uint16_t _value)
{

	value=static_cast<uint16_t>(filter.process_filter(_value))>>ADCBitResolution;

	pinValueChanged = !(value==precValue); //TODO INCORPORARE
	precValue=value;
}

AnalogInput::~AnalogInput()
{
	//SerialDebug.println("~AnalogInput");
}
