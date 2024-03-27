#pragma once

#include "Arduino.h"
#include "Updatable.h"

class Kicker : public Updatable
{
public:
    Kicker(int pin, uint32_t kick_delay, uint32_t kick_timeout)
    {
        this->pin = pin;
        this->kick_delay = kick_delay;
        this->kick_timeout = kick_timeout;
        this->last_kick = 0;
    }
    ERROR_TYPE kick()
    {
        if(millis() < last_kick + kick_timeout)
            return KICKER_WAIT_TIMEOUT;
        digitalWrite(pin, HIGH);
        last_kick = millis();
        return NO_ERRORS;
    }
    ERROR_TYPE update() override
    {
        if(millis() > last_kick + kick_delay)
        {
            digitalWrite(pin, LOW);
        }
        return NO_ERRORS;
    }

protected:
    int pin;
    uint32_t kick_delay;
    uint32_t kick_timeout;
    uint32_t last_kick;
};
