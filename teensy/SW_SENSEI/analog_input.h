#ifndef ANALOGINPUT_H
#define ANALOGINPUT_H

#include "common.h"
#include "digital_input.h"
#include "filter.h"

typedef enum SliderState
{
	JUMPING,
	WAITING,
	READING
} SliderState;

class AnalogInput : public PIN  {
public:
	AnalogInput(sensei::SetupPin* setupPin);
	~AnalogInput();
	void setPinValue(uint16_t value);

private:
	Filter _filter;
	uint8_t _ADCBitResolution;
	uint8_t _sliderMode;
	uint16_t _sliderThreshold;
	uint16_t _maxValue;

	uint8_t _sliderState;
	uint16_t _sliderValue;
	uint16_t _jumpedSliderValue;
};

#endif // ANALOGINPUT_H
