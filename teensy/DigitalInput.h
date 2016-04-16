#ifndef DIGITALINPUT_H
#define DIGITALINPUT_H

#include "DigitalOutput.h"

class DigitalInput : public DigitalOutput {
public:
	DigitalInput(SetupPin* _setupPin);
	~DigitalInput();

	uint8_t sendingMode;
	uint16_t deltaTicksContinuousMode;

private:

};


#endif
