#pragma once

#include <SPI.h>
#include "RF24.h"


#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

struct data {
  uint8_t op_addr;
  uint8_t speed_x;
  uint8_t speed_y;
  uint8_t speed_w;
  uint8_t voltage;
  uint8_t flags;
};

class NRF24: public Updatable{
    private:
        RF24 rad;
        data dataPackage;
    public:
        
        NRF24(uint8_t ce, uint8_t csn):rad(ce, csn){}

        void init(){
            rad.begin();                          
            rad.setChannel      (27);                                // Указываем канал передачи данных (от 0 до 125), 27 - значит приём данных осуществляется на частоте 2,427 ГГц.
            rad.setDataRate     (RF24_1MBPS);                        // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек.
            rad.setPALevel      (RF24_PA_MAX);
            rad.openReadingPipe (0, 0xAABBCCDD11LL);  
            rad.startListening();
        }
        uint16_t update() override{
            Serial.println("data");
            if(this->rad.available()){                                     
                rad.read(&dataPackage,  sizeof(dataPackage)); 
            }    

            return NO_ERRORS;
        }

        data getData()
        {
            return dataPackage;
        }
};
