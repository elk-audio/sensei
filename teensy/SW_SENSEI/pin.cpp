#include "pin.h"

PIN::PIN()
{
	type = ePinType::PIN_DISABLE;
	value = 0;
	precValue = 0;
	pinValueChanged=false;
}

PIN::~PIN()
{

}

bool PIN::isPinValueChanged()
{
	return pinValueChanged;
}

void PIN::setPinValue(uint16_t _value)
{
    value = _value;
	pinValueChanged = !(value==precValue);
	precValue=value;
}

uint16_t PIN::getPinValue()
{
    return value;
}
