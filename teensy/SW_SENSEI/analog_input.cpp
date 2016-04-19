#include "analog_input.h"

AnalogInput::AnalogInput(SetupPin* setupPin) : PIN(setupPin)
{
	//SerialDebug.println("AnalogInput");

	type = ePinType::PIN_ANALOG_INPUT;

	ADCBitResolution = setupPin->ADCBitResolution;

	filter.setFilter(setupPin->filterOrder,setupPin->filterCoeff_a,setupPin->filterCoeff_b);

	sliderMode = setupPin->sliderMode;
	sliderThreshold = setupPin->sliderThreshold;

}

void AnalogInput::setPinValue(uint16_t _value)
{

	value=static_cast<uint16_t>(filter.processFilter(_value))>>ADCBitResolution;
	checkPinChange();
}

AnalogInput::~AnalogInput()
{
	//SerialDebug.println("~AnalogInput");
}
