#include "digital_input.h"

DigitalInput::DigitalInput(SetupPin* setupPin) : PIN(setupPin)
{

	type = ePinType::PIN_DIGITAL_INPUT;
	sendingMode = setupPin->sendingMode;
	deltaTicksContinuousMode = setupPin->deltaTicksContinuousMode;
}


DigitalInput::~DigitalInput()
{
	//SerialDebug.println("~DigitalInput");
}
