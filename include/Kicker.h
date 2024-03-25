#pragma once

#include "Arduino.h"
#include "Updatable.h"

class Kicker : public Updatable
{
public:
    Kicker(int pin, uint32_t kick_time, uint32_t kick_timeout)
    {
        this->pin = pin;
        this->kick_time = kick_time;
        this->kick_timeout = kick_timeout;
        this->last_kick = 0;
    }
    uint16_t kick()
    {
        digitalWrite(pin, HIGH);
        last_kick = millis();
    }
    ERROR_TYPE update() override
    {
    }

protected:
    int pin;
    uint32_t kick_time;
    uint32_t kick_timeout;
    uint32_t last_kick;
};