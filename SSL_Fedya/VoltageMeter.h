#ifndef VOLTAGE_METER_H
#define VOLTAGE_METER_H

#include <Arduino.h>

#include "Updatable.h"
#include "Errors.h"

uint8_t calcLinearPercentage(float v, float min, float max){
        return constrain((v - min) * 100 / (max - min), 0, 100);
}

class VoltageMeter : public Updatable{
    private:
    uint8_t pin;
    float coef, min, max;
    float voltage;
    uint8_t percentage;
    uint8_t (*calcPercentage)(float v, float min, float max);
    
    
    public:
    VoltageMeter(uint8_t pin, float coef, float min, float max, uint8_t (*calcPercentage)(float v, float min, float max) = calcLinearPercentage){
        this->pin = pin;
        this->coef = coef;
        this->min = min;
        this->max = max;
        this->calcPercentage = calcPercentage;

        pinMode(pin, INPUT);
        update();
    }

    uint16_t update() override{
        voltage = analogRead(pin) * coef;
        percentage = (*calcPercentage)(voltage, min, max);

        return NO_ERRORS;
    }

    float getVoltage(){
        return voltage;
    }

    uint8_t getPercentage(){
        return percentage;
    }
};

#endif
