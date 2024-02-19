#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

class NRF24: public Updatable{
    private:
        uint8_t dataPackage[5?];
    public:
        NRF24(uint8_t pinA, unit8_t pinB, float coef, float min, float max){
            // this->pin = pinA;
            // this->pin = pinB;
            RF24 radio(pinA, pinB);
            this->radio = radio;
            this->radio.begin();                          
            this->radio.setChannel(1);
            this->radio.setPALevel(RF24_PA_MAX);
            this->radio.setDataRate(RF24_1MBPS);
            this->radio.startListening();
        }
        uint16_t update() override{
            if(radio.available()){                                     
                radio.read(&dataPackage,  sizeof(dataPackage)); 
            }    

            return NO_ERRORS;
        }

        uint8_t[] getData()
        {
            return dataPackage;
        }
}