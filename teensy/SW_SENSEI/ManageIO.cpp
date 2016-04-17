#include "ManageIO.h"

ManageIO::ManageIO()
{
	nPin = 0;
	nMul=0;
	nDigitalPin = 0;
	nDigitalBank=0;
	systemInitialized=false;
}

int32_t ManageIO::setSystem(uint16_t _nPin, uint16_t _nDigitalPin)
{

	nPin = _nPin;
	nDigitalPin = _nDigitalPin;

	if (nPin%CHANNELS_PER_MULTIPLEXER!=0)
	{
		return SENSEI_ERROR_CODE::INCORRECT_NUMBER_OF_PINS;
	}

	if (_nDigitalPin%CHANNELS_PER_SHIFT_REGISTER!=0)
	{
		return SENSEI_ERROR_CODE::INCORRECT_NUMBER_OF_DIGITAL_PINS;
	}

	nMul=nPin/CHANNELS_PER_MULTIPLEXER;

	nDigitalBank = static_cast<uint8_t>(ceil(static_cast<float>(nDigitalPin) / CHANNELS_PER_SHIFT_REGISTER));
	vBankDigitalPin = new uint8_t[nDigitalBank];


	vPin.clear();
	vPin.reserve(nPin);
	for (uint16_t idxPin = 0; idxPin < nPin; idxPin++)
	{
		vPin.push_back(new PIN);
	}

	for (uint16_t idxBank = 0; idxBank < nDigitalBank; idxBank++)
	{
		vBankDigitalPin[idxBank] = 0x00;
	}

	setDigitalPin();

	systemInitialized=true;

	return SENSEI_ERROR_CODE::OK;
}

uint8_t ManageIO::getNmultiplexer()
{
	return nMul;
}

bool ManageIO::isPinValueChanged(uint16_t _idxPin)
{
	return vPin[_idxPin]->isPinValueChanged();
}

int32_t ManageIO::setPinValue(uint16_t _idxPin, uint16_t _value)
{
	if (!systemInitialized)
	return SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;

	if (_idxPin >= nPin)
	return SENSEI_ERROR_CODE::IDX_PIN_NOT_VALID;

	if (getPinType(_idxPin)!=ePinType::PIN_DISABLE)
	{
		vPin[_idxPin]->setPinValue(_value);
		return SENSEI_ERROR_CODE::OK;
	}
	else
	{
		return SENSEI_ERROR_CODE::CMD_NOT_ALLOWED;
	}
}

uint16_t ManageIO::getPinValue(uint16_t _idxPin)
{
	if ((!systemInitialized) || (_idxPin >= nPin))
	return 0;
	return vPin[_idxPin]->getPinValue();
}

int32_t ManageIO::getPinValue(uint16_t _idxPin,uint16_t& _value)
{
	if (!systemInitialized)
	return SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;

	if (_idxPin >= nPin)
	return SENSEI_ERROR_CODE::IDX_PIN_NOT_VALID;

	_value=vPin[_idxPin]->getPinValue();

	return SENSEI_ERROR_CODE::OK;

}

int32_t ManageIO::configurePin(ePinType type, SetupPin* setupPin)
{
	if (setupPin->idxPin >= nPin)
	return SENSEI_ERROR_CODE::IDX_PIN_NOT_VALID;
	//SerialDebug.println("PRE setPin");
	delete vPin[setupPin->idxPin];

	switch (type)
	{
		case PIN_ANALOG_INPUT:
		//SerialDebug.println("PIN_ANALOG_INPUT");

		vPin[setupPin->idxPin] = new AnalogInput(setupPin);
		break;

		case PIN_DIGITAL_INPUT:
		//	SerialDebug.println("PIN_DIGITAL_INPUT");
		vPin[setupPin->idxPin] = new DigitalInput(setupPin);

		break;

		case PIN_DIGITAL_OUTPUT:
		//SerialDebug.println("PIN_DIGITAL_OUTPUT");
		vPin[setupPin->idxPin] = new DigitalOutput();
		break;

		case PIN_DISABLE:
		vPin[setupPin->idxPin] = new PIN();
		break;
	};

	//SerialDebug.println("POST setPin");
	return SENSEI_ERROR_CODE::OK;

}

uint16_t ManageIO::getNumberOfPins()
{
	return nPin;
}

uint16_t ManageIO::getNumberOfDigitalPins()
{
	return nDigitalPin;
}

bool ManageIO::isSystemInitialized()
{
	return systemInitialized;

}

bool ManageIO::isPinInitialized(uint16_t _idxPin)
{
	if (vPin[_idxPin]->type == ePinType::PIN_DISABLE)
	return false;
	else
	return true;
}

uint8_t ManageIO::getPinType(uint16_t _idxPin)
{
	return vPin[_idxPin]->type;
}

int32_t ManageIO::setDigitalBank(uint16_t _idxBank, uint8_t _value)
{
	if (!systemInitialized)
	return SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;

	if (_idxBank >= nDigitalBank)
	return SENSEI_ERROR_CODE::DIGITAL_OUTPUT_IDX_BANK_NOT_VALID;

	vBankDigitalPin[_idxBank] = _value;

	setDigitalPin();
	return SENSEI_ERROR_CODE::OK;

}

int32_t ManageIO::setDigitalPin(uint16_t _idxPin, bool _value)
{

	if (!systemInitialized)
	return SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;

	if (_idxPin >= nDigitalPin)
	return SENSEI_ERROR_CODE::DIGITAL_OUTPUT_IDX_PIN_NOT_VALID;

	uint16_t idxBank = floor(static_cast<float>(_idxPin) / 8);
	uint8_t idxBit = _idxPin - 8 * idxBank;

	if (_value)
	{
		//vBankDigitalPin[idxBank] = vBankDigitalPin[idxBank] | (static_cast<uint8_t>(0x01) << idxBit);
		vBankDigitalPin[idxBank] = arm_bit_set(vBankDigitalPin[idxBank], idxBit);

	}
	else
	{
		//vBankDigitalPin[idxBank] = vBankDigitalPin[idxBank] & ~(static_cast<uint8_t>(0x01) << idxBit);
		vBankDigitalPin[idxBank] = arm_bit_clear(vBankDigitalPin[idxBank], idxBit);
	}

	setDigitalPin();

	return SENSEI_ERROR_CODE::OK;

}

void ManageIO::setDigitalPin()
{
	//noInterrupts();

	digitalWrite(ST, 0);
	//delayMicroseconds(1);

	for (uint16_t idxBank = 0; idxBank < nDigitalBank; idxBank++)
	{
		shiftOut(DS, SH, LSBFIRST, vBankDigitalPin[nDigitalBank-idxBank-1]);
	}

	digitalWrite(ST, 1);
	//delayMicroseconds(1);

	//interrupts();
}


ManageIO::~ManageIO()
{

}
