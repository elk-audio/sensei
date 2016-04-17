#ifndef MANAGEIO_H
#define MANAGEIO_H

#include "Arduino.h"
#include "common.h"

#include "PIN.h"
#include "AnalogInput.h"
#include "DigitalInput.h"
#include "DigitalOutput.h"

#include "ManageIMU.h"

class ManageIO{
  public:

    ManageIO();
	~ManageIO();

    int32_t setSystem(uint16_t _nPin, uint16_t _nDigitalPin);

    ManageIMU imu;

    int32_t setPinValue(uint16_t _idxPin, uint16_t _value);
    int32_t getPinValue(uint16_t _idxPin, uint16_t &_value);
    uint16_t getPinValue(uint16_t _idxPin);

	int32_t configurePin(ePinType type, SetupPin* setupPin);
	uint8_t getPinType(uint16_t _idxPin);
    bool isPinValueChanged(uint16_t _idxPin);
	bool isPinInitialized(uint16_t _idxPin);
	int32_t setDigitalBank(uint16_t _idxBank, uint8_t value);
	int32_t setDigitalPin(uint16_t _idxPin, bool _value);
    uint8_t getNmultiplexer();

	uint16_t getNumberOfPins();
	uint16_t getNumberOfDigitalPins();
    bool isSystemInitialized();


  private:

     std::vector<PIN*> vPin;

     void setDigitalPin();

     uint8_t nMul;
	 uint16_t nPin;
	 uint16_t nDigitalPin;
	 uint16_t nDigitalBank;

	 uint8_t* vBankDigitalPin;
     bool systemInitialized;
};

#endif // MANAGEIO_H
