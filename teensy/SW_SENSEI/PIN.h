#ifndef PIN_H
#define PIN_H

#include "common.h"

class PIN{
  public:
	 PIN();
	virtual ~PIN();
	bool isInitialized;

    virtual void setPinValue(uint16_t _value);
    uint16_t getPinValue();
    bool isPinValueChanged();

    //TODO Mettere privati
    bool pinValueChanged;
    ePinType type;
    uint16_t value;
    uint16_t precValue;

  private:

};


#endif
