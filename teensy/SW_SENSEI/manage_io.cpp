#include "manage_io.h"

using namespace sensei;

ManageIO::ManageIO()
{
	_nPin = 0;
	_nMul = 0;
	_nDigitalPin = 0;
	_nDigitalBank = 0;
	_systemInitialized=false;
}

void ManageIO::hardwareAcquisition()
{
	// Hardware Polling
	if (isSystemInitialized())
	{
		uint16_t idxPin;
		for (uint8_t ch = 0; ch < CHANNELS_PER_MULTIPLEXER; ch++)
		{
			Byte S_Address;
			S_Address.value = ch;

			digitalWrite(S0, S_Address.bit0);
			digitalWrite(S1, S_Address.bit1);
			digitalWrite(S2, S_Address.bit2);
			digitalWrite(S3, S_Address.bit3);

			for (uint8_t idxMul = 0; idxMul < getNmultiplexer(); idxMul++)
			{
				idxPin = ch + idxMul * CHANNELS_PER_MULTIPLEXER;

				switch(getPinType(idxPin))
				{
				   case ePinType::PIN_DISABLE:
					   pinMode(Z1 + VERSOR_Z_PINS * idxMul, INPUT_PULLUP);
				   break;

					case ePinType::PIN_DIGITAL_INPUT:
						pinMode(Z1 + VERSOR_Z_PINS * idxMul, INPUT_PULLUP);
						delayMicroseconds(5); //to define
						setPinValue(idxPin, digitalRead(Z1 + VERSOR_Z_PINS * idxMul));
					break;

				   case ePinType::PIN_DIGITAL_OUTPUT:
					   pinMode(Z1 + VERSOR_Z_PINS * idxMul, OUTPUT);
					   delayMicroseconds(1);
					   digitalWrite(Z1 + VERSOR_Z_PINS * idxMul,static_cast<bool>(getPinValue(idxPin)));
					   delayMicroseconds(1);
					   digitalWrite(Z1 + VERSOR_Z_PINS * idxMul,LOW);
					   pinMode(Z1 + VERSOR_Z_PINS * idxMul, INPUT_PULLDOWN);
				   break;

					case ePinType::PIN_ANALOG_INPUT:
						pinMode(Z1 + VERSOR_Z_PINS * idxMul, INPUT);
						delayMicroseconds(3);
						setPinValue(idxPin,analogRead(Z1 + VERSOR_Z_PINS * idxMul));
					break;
				}
			} //Mul
		} //Ch
	} //manageIO.isSystemInitialized()
}

int32_t ManageIO::setSystem(uint16_t nPin, uint16_t nDigitalPin)
{
	if (nPin % CHANNELS_PER_MULTIPLEXER != 0)
		return SENSEI_ERROR_CODE::INCORRECT_NUMBER_OF_PINS;

	if (nDigitalPin % CHANNELS_PER_SHIFT_REGISTER != 0)
		return SENSEI_ERROR_CODE::INCORRECT_NUMBER_OF_DIGITAL_PINS;

    _nPin = nPin;
	_nDigitalPin = nDigitalPin;
	_nMul=_nPin/CHANNELS_PER_MULTIPLEXER;
	_nDigitalBank = static_cast<uint8_t>(ceil(static_cast<float>(_nDigitalPin) / CHANNELS_PER_SHIFT_REGISTER));

    //Set vector PIN
	if (_systemInitialized)
	{
		for (uint16_t idxPin = 0; idxPin < _nPin; idxPin++)
		{
			delete(_vPin[idxPin]);
		}
	}

	_vPin.clear();
	_vPin.reserve(_nPin);
	for (uint16_t idxPin = 0; idxPin < _nPin; idxPin++)
	{
		_vPin.push_back(new PIN);
	}

    //Set vector DigitalBank
    _vBankDigitalPin.clear();
    _vBankDigitalPin.reserve(_nDigitalBank);
	for (uint16_t idxBank = 0; idxBank < _nDigitalBank; idxBank++)
	{
		_vBankDigitalPin.push_back(0);
	}

	setDigitalPin();

	_systemInitialized=true;

	#ifdef POWER_ON_IMU_WITH_DIGITAL_PIN
	  setDigitalPin(PIN_POWER_IMU,true);
	#endif
	
	return SENSEI_ERROR_CODE::OK;
}

uint8_t ManageIO::getNmultiplexer()
{
	return _nMul;
}

uint8_t ManageIO::getSendingMode(uint16_t idxPin)
{
	if ((_systemInitialized) && (idxPin<_nPin))
		return _vPin[idxPin]->getSendingMode();
	else
		return 0;
}

uint16_t ManageIO::getDeltaTicksContinuousMode(uint16_t idxPin)
{
	if ((_systemInitialized) && (idxPin<_nPin))
		return _vPin[idxPin]->getDeltaTicksContinuousMode();
	else
		return 0;
}

bool ManageIO::isMomentToSendValue(uint16_t idxPin)
{
	return _vPin[idxPin]->isMomentToSendValue();
}

bool ManageIO::isPinValueChanged(uint16_t idxPin)
{
	if ((_systemInitialized) && (idxPin<_nPin))
		return _vPin[idxPin]->isPinValueChanged();
	else
		return 0;
}

int32_t ManageIO::setPinValue(uint16_t idxPin, uint16_t value)
{
	if (!_systemInitialized)
		return SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;

	if (idxPin >= _nPin)
		return SENSEI_ERROR_CODE::IDX_PIN_NOT_VALID;

	if (getPinType(idxPin)!=ePinType::PIN_DISABLE)
	{
		_vPin[idxPin]->setPinValue(value);
		return SENSEI_ERROR_CODE::OK;
	}
	else
	{
		return SENSEI_ERROR_CODE::CMD_NOT_ALLOWED;
	}
}

uint16_t ManageIO::getPinValue(uint16_t idxPin)
{
	if ((_systemInitialized) && (idxPin<_nPin))
		return _vPin[idxPin]->getPinValue();
	else
		return 0;
}

int32_t ManageIO::getPinValue(uint16_t idxPin,uint16_t& value)
{
	if (!_systemInitialized)
		return SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;

	if (idxPin >= _nPin)
		return SENSEI_ERROR_CODE::IDX_PIN_NOT_VALID;

	value=_vPin[idxPin]->getPinValue();

	return SENSEI_ERROR_CODE::OK;
}

int32_t ManageIO::configurePin(ePinType type, SetupPin* setupPin)
{
	if (setupPin->idxPin >= _nPin)
		return SENSEI_ERROR_CODE::IDX_PIN_NOT_VALID;

	delete _vPin[setupPin->idxPin];

	switch (type)
	{
		case PIN_ANALOG_INPUT:
			_vPin[setupPin->idxPin] = new AnalogInput(setupPin);
		break;

		case PIN_DIGITAL_INPUT:
			_vPin[setupPin->idxPin] = new DigitalInput(setupPin);
		break;

		case PIN_DIGITAL_OUTPUT:
			_vPin[setupPin->idxPin] = new DigitalOutput();
		break;

		case PIN_DISABLE:
			_vPin[setupPin->idxPin] = new PIN();
		break;

		default:
		return SENSEI_ERROR_CODE::INCORRECT_PARAMETER_TYPE;
	};

	return SENSEI_ERROR_CODE::OK;
}

uint16_t ManageIO::getNumberOfPins()
{
	return _nPin;
}

uint16_t ManageIO::getNumberOfDigitalPins()
{
	return _nDigitalPin;
}

bool ManageIO::isSystemInitialized()
{
	return _systemInitialized;
}

bool ManageIO::isPinInitialized(uint16_t idxPin)
{
	if (_vPin[idxPin]->getPinType() == ePinType::PIN_DISABLE)
		return false;
	else
		return true;
}

uint8_t ManageIO::getPinType(uint16_t idxPin)
{
	if ((_systemInitialized) && (idxPin<_nPin))
		return _vPin[idxPin]->getPinType();
	else
		return ePinType::PIN_DISABLE;
}

int32_t ManageIO::setDigitalBank(uint16_t idxBank, uint8_t value)
{
	if (!_systemInitialized)
		return SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;

	if (idxBank >= _nDigitalBank)
		return SENSEI_ERROR_CODE::DIGITAL_OUTPUT_IDX_BANK_NOT_VALID;

	_vBankDigitalPin[idxBank] = value;

	setDigitalPin();
	return SENSEI_ERROR_CODE::OK;

}

int32_t ManageIO::setDigitalPin(uint16_t idxPin, bool value)
{

	if (!_systemInitialized)
		return SENSEI_ERROR_CODE::SYSTEM_NOT_INITIALIZED;

	if (idxPin >= _nDigitalPin)
		return SENSEI_ERROR_CODE::DIGITAL_OUTPUT_IDX_PIN_NOT_VALID;

	uint16_t idxBank = floor(static_cast<float>(idxPin) / 8);
	uint8_t idxBit = idxPin - 8 * idxBank;

	if (value)
	{
		//_vBankDigitalPin[idxBank] = _vBankDigitalPin[idxBank] | (static_cast<uint8_t>(0x01) << idxBit);
		_vBankDigitalPin[idxBank] = arm_bit_set(_vBankDigitalPin[idxBank], idxBit);
	}
	else
	{
		//_vBankDigitalPin[idxBank] = _vBankDigitalPin[idxBank] & ~(static_cast<uint8_t>(0x01) << idxBit);
		_vBankDigitalPin[idxBank] = arm_bit_clear(_vBankDigitalPin[idxBank], idxBit);
	}

	setDigitalPin();

	return SENSEI_ERROR_CODE::OK;
}

void ManageIO::setDigitalPin()
{
	//noInterrupts();

	digitalWrite(ST, 0);
	//delayMicroseconds(1);

	for (uint16_t idxBank = 0; idxBank < _nDigitalBank; idxBank++)
	{
		shiftOut(DS, SH, LSBFIRST, _vBankDigitalPin[_nDigitalBank-idxBank-1]);
	}

	digitalWrite(ST, 1);
	//delayMicroseconds(1);

	//interrupts();
}

ManageIO::~ManageIO()
{

}
