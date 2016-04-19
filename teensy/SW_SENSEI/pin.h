#ifndef PIN_H
#define PIN_H

#include "common.h"

class PIN{
  public:
	PIN();
    PIN(SetupPin* setupPin);
	virtual ~PIN();
	bool isInitialized;

    virtual void setPinValue(uint16_t _value);
    uint16_t getPinValue();
    bool isPinValueChanged();

    uint8_t sendingMode;
    uint16_t deltaTicksContinuousMode;
    bool pinValueChanged;
    ePinType type;
    uint16_t value;
    uint16_t precValue;
    void checkPinChange();

  private:

};

#endif // PIN_H
