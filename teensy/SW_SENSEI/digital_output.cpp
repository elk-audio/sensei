#include "digital_output.h"

using namespace sensei;

DigitalOutput::DigitalOutput() : PIN()
{
	//SerialDebug.println("DigitalOutput()");
	_type = ePinType::PIN_DIGITAL_OUTPUT;
}

DigitalOutput::~DigitalOutput()
{
	//SerialDebug.println("~DigitalOutput");
}
