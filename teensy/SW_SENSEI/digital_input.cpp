#include "digital_input.h"

DigitalInput::DigitalInput(SetupPin* setupPin) : PIN(setupPin)
{
    //SerialDebug.println("DigitalInput");
	_type = ePinType::PIN_DIGITAL_INPUT;
}

DigitalInput::~DigitalInput()
{
	//SerialDebug.println("~DigitalInput");
}
