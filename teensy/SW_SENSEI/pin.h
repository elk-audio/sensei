#ifndef PIN_H
#define PIN_H

#include "common.h"

class PIN{
  public:
	PIN();
    PIN(SetupPin* setupPin);
	virtual ~PIN();

    virtual void setPinValue(uint16_t value);
    uint16_t getPinValue();
    uint8_t getPinType();
    bool isPinValueChanged();
    uint8_t getSendingMode();
    uint16_t getDeltaTicksContinuousMode();
    bool isMomentToSendValue();

protected:
    uint8_t _sendingMode;
    uint16_t _deltaTicksContinuousMode;
    bool _pinValueChanged;
    ePinType _type;
    uint16_t _value;
    uint16_t _precValue;
    bool _isInitialized;
    void _checkPinChange();
    uint16_t _ticksRemainingForSending;
    bool _pinValueChangedInsideInterval;
};

#endif // PIN_H
