#pragma once

#include <Arduino.h>

#include "Updatable.h"
#include "Errors.h"

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

    bool getAnalogValue(){
        return analogValue;
    }

    uint16_t update() override{
        analogValue = analogRead(pin);
        value = analogValue > threshold;
    }
};