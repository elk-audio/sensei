#include "digital_input.h"

using namespace sensei;

DigitalInput::DigitalInput(SetupPin* setupPin) : PIN(setupPin)
{
    //SerialDebug.println("DigitalInput");
	_type = ePinType::PIN_DIGITAL_INPUT;
}

DigitalInput::~DigitalInput()
{
	//SerialDebug.println("~DigitalInput");
}
