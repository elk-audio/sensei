#include "pin.h"

using namespace sensei;

PIN::PIN()
{
	_type = ePinType::PIN_DISABLE;
	_value = 0;
	_precValue = 0;
	_sendingMode = eSendingMode::SENDING_MODE_ON_REQUEST;
  _deltaTicksContinuousMode = 0;
	_pinValueChanged=false;
	_pinValueChangedInsideInterval=false;
}

PIN::PIN(SetupPin* setupPin)
{
	PIN();
	_sendingMode = setupPin->sendingMode;
  _deltaTicksContinuousMode = setupPin->deltaTicksContinuousMode;
	_ticksForSending = _deltaTicksContinuousMode;
}

PIN::~PIN()
{

}

bool PIN::isMomentToSendValue()
{
	if (_type == ePinType::PIN_DIGITAL_INPUT)
	{
		return isPinValueChanged();
	}
	else
	{
		if (_pinValueChanged)
		{
			_pinValueChangedInsideInterval=true;
		}
		_ticksForSending++;
		if (_ticksForSending >= _deltaTicksContinuousMode)
		{
			_ticksForSending = 0;

			if (_pinValueChangedInsideInterval)
			{
				_pinValueChangedInsideInterval = false;
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}
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

void PIN::_setPinValue(uint16_t value)
{
  _value = value;
	_checkPinChange();
	_precValue = _value;
}

void PIN::setPinValue(uint16_t value)
{
    _setPinValue(value);
}

void PIN::_checkPinChange()
{
	_pinValueChanged = !(_value == _precValue);
}

uint16_t PIN::getPinValue()
{
    return _value;
}
