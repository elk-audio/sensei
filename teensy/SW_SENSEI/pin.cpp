#include "pin.h"

PIN::PIN()
{
	_type = ePinType::PIN_DISABLE;
	_value = 0;
	_precValue = 0;
	_sendingMode = eSendingMode::SENDING_MODE_ON_REQUEST;
    _deltaTicksContinuousMode = 0;
	_pinValueChanged=false;
}

PIN::PIN(SetupPin* setupPin)
{
	PIN();
	_sendingMode = setupPin->sendingMode;
    _deltaTicksContinuousMode = setupPin->deltaTicksContinuousMode;
}

PIN::~PIN()
{

}

bool PIN::isPinValueChanged()
{
	return _pinValueChanged;
}

uint8_t PIN::getSendingMode()
{
	return _sendingMode;
}

uint16_t PIN::getDeltaTicksContinuousMode()
{
	return _deltaTicksContinuousMode;
}

uint8_t PIN::getPinType()
{
	return _type;
}

void PIN::setPinValue(uint16_t value)
{
    _value = value;
	_checkPinChange();
}

void PIN::_checkPinChange()
{
	_pinValueChanged = !(_value == _precValue);
	_precValue = _value;
}

uint16_t PIN::getPinValue()
{
    return _value;
}
