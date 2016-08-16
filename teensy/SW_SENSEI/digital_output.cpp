#include "digital_output.h"

DigitalOutput::DigitalOutput() : PIN()
{
	//SerialDebug.println("DigitalOutput()");
	_type = ePinType::PIN_DIGITAL_OUTPUT;
}

DigitalOutput::~DigitalOutput()
{
	//SerialDebug.println("~DigitalOutput");
}
