#ifndef ANALOGINPUT_H
#define ANALOGINPUT_H

#include "common.h"
#include "digital_input.h"
#include "filter.h"

class AnalogInput : public DigitalInput  {
public:
	AnalogInput(SetupPin* _setup);
	~AnalogInput();

	void setPinValue(uint16_t _value);

	Filter filter;
private:

	uint8_t ADCBitResolution;

	uint8_t sliderMode;
	uint16_t sliderThreshold;

};

#endif // ANALOGINPUT_H
