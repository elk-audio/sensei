#include "AnalogInput.h"

AnalogInput::AnalogInput(SetupPin* _setupPin) : DigitalInput(_setupPin)
{
	//SerialDebug.println("AnalogInput");

	type = ePinType::PIN_ANALOG_INPUT;

	ADCBitResolution = _setupPin->ADCBitResolution;

	filter.setFilter(_setupPin->filterOrder,_setupPin->filterCoeff_a,_setupPin->filterCoeff_b);

	sliderMode = _setupPin->sliderMode;
	sliderThreshold = _setupPin->sliderThreshold;

	/*SerialDebug.println("PIN_ANALOG_INPUT()");
	SerialDebug.println("sendingMode=" + String(sendingMode));
	SerialDebug.println("deltaTicksContinuousMode=" + String(deltaTicksContinuousMode));
	SerialDebug.println("ADCBitResolution=" + String(ADCBitResolution));
	SerialDebug.println("filterOrder=" + String(filterOrder));
	SerialDebug.println("sliderMode=" + String(sliderMode));
	SerialDebug.println("sliderThreshold=" + String(sliderThreshold));

	for (int idx = 0; idx < filterOrder + 1; idx++)
	{
		SerialDebug.println(String(filterCoeff_a[idx]) + " " + String(filterCoeff_b[idx]));
	}
	SerialDebug.println("------------------------------------");*/
}

void AnalogInput::setPinValue(uint16_t _value)
{

	value=static_cast<uint16_t>(filter.processFilter(_value))>>ADCBitResolution;

	pinValueChanged = !(value==precValue); //TODO INCORPORARE
	precValue=value;
}

AnalogInput::~AnalogInput()
{
	//SerialDebug.println("~AnalogInput");
}
