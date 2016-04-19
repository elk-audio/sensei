#include "pin.h"

PIN::PIN()
{
	type = ePinType::PIN_DISABLE;
	value = 0;
	precValue = 0;
	sendingMode = eSendingMode::SENDING_MODE_ON_REQUEST;
    deltaTicksContinuousMode = 0;
	pinValueChanged=false;
}

PIN::PIN(SetupPin* setupPin)
{
	PIN();
	sendingMode = setupPin->sendingMode;
    deltaTicksContinuousMode = setupPin->deltaTicksContinuousMode;
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
	checkPinChange();
}

void PIN::checkPinChange()
{
	pinValueChanged = !(value==precValue);
	precValue=value;
}

uint16_t PIN::getPinValue()
{
    return value;
}
