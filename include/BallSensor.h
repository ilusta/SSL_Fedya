#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"


class BallSensor : public Updatable{
    private:
    int pin;
    bool value;
    uint16_t analogValue, threshold; 
    
    public:
    BallSensor(int pin, uint16_t threshold){
        this->pin = pin;
        this->threshold = threshold;

        pinMode(pin, INPUT);
    }

    void setThreshold(uint16_t threshold){
        this->threshold = threshold;
    }

    bool getValue(){
        return value;
    }

    uint16_t getAnalogValue(){
        return analogValue;
    }

    uint16_t update() override{
        analogValue = analogRead(pin);//*0.2 + analogValue*0.8;
        value = analogValue < threshold;

        return NO_ERRORS;
    }
};
