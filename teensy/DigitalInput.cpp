#include "DigitalInput.h"

DigitalInput::DigitalInput(SetupPin* _setupPin) : DigitalOutput()
{

	type = ePinType::PIN_DIGITAL_INPUT;
	sendingMode = _setupPin->sendingMode;
	deltaTicksContinuousMode = _setupPin->deltaTicksContinuousMode;
}


DigitalInput::~DigitalInput()
{
	//SerialDebug.println("~DigitalInput");
}
