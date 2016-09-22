#ifndef DIGITALINPUT_H
#define DIGITALINPUT_H

#include "digital_output.h"

class DigitalInput : public PIN 
{
public:
	DigitalInput(SetupPin* setupPin);
	~DigitalInput();

private:

};

#endif // DIGITALINPUT_H
