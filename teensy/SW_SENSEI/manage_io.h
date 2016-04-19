#ifndef MANAGEIO_H
#define MANAGEIO_H

#include "common.h"
#include "pin.h"
#include "analog_input.h"
#include "digital_input.h"
#include "digital_output.h"

#include "manage_imu.h"

class ManageIO{
  public:

    ManageIO();
	~ManageIO();

    ManageIMU imu;

    int32_t setSystem(uint16_t nPin, uint16_t _nDigitalPin);

    int32_t setPinValue(uint16_t idxPin, uint16_t value);
    int32_t getPinValue(uint16_t idxPin, uint16_t& value);

    uint16_t getPinValue(uint16_t idxPin);
	int32_t configurePin(ePinType type, SetupPin* setupPin);
	uint8_t getPinType(uint16_t idxPin);
    bool isPinValueChanged(uint16_t idxPin);
	bool isPinInitialized(uint16_t idxPin);
	int32_t setDigitalBank(uint16_t idxBank, uint8_t value);
	int32_t setDigitalPin(uint16_t idxPin, bool value);
    uint8_t getNmultiplexer();
	uint16_t getNumberOfPins();
	uint16_t getNumberOfDigitalPins();
    void hardwareAcquisition();
    bool isSystemInitialized();
    uint8_t getSendingMode(uint16_t idxPin);
    uint16_t getDeltaTicksContinuousMode(uint16_t idxPin);

  private:
     std::vector<PIN*> _vPin;
     uint8_t _nMul;
	 uint16_t _nPin;
	 uint16_t _nDigitalPin;
	 uint16_t _nDigitalBank;

     std::vector<uint8_t> _vBankDigitalPin;
     bool _systemInitialized;

     void setDigitalPin();

};

#endif // MANAGEIO_H
