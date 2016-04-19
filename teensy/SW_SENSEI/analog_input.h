#ifndef ANALOGINPUT_H
#define ANALOGINPUT_H

#include "common.h"
#include "digital_input.h"
#include "filter.h"

class AnalogInput : public PIN  {
public:
	AnalogInput(SetupPin* setupPin);
	~AnalogInput();
	void setPinValue(uint16_t value);

private:
	Filter _filter;
	uint8_t _ADCBitResolution;
	uint8_t _sliderMode;
	uint16_t _sliderThreshold;
};

#endif // ANALOGINPUT_H
