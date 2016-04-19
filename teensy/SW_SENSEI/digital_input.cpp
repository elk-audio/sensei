#include "digital_input.h"

DigitalInput::DigitalInput(SetupPin* setupPin) : PIN(setupPin)
{
	_type = ePinType::PIN_DIGITAL_INPUT;
	_sendingMode = setupPin->sendingMode;
	_deltaTicksContinuousMode = setupPin->deltaTicksContinuousMode;
}

DigitalInput::~DigitalInput()
{
	//SerialDebug.println("~DigitalInput");
}
