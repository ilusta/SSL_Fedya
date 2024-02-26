#pragma once

//#include <nRF24L01.h>
//#include <printf.h>
#include <SPI.h>
#include "RF24.h"
//#include <RF24_config.h>


#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"
RF24 radio(10,11);
int      myData[5];
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
      
        data dataPackage;
    public:
        NRF24(){
//            radio(pinA, pinB);
//            this->radio = radio;
        }

        void init(){
            radio.begin();                          
//            radio.setChannel(1);
//            radio.setPALevel(RF24_PA_MAX);
//            radio.setDataRate(RF24_1MBPS);
              radio.setChannel      (27);                                // Указываем канал передачи данных (от 0 до 125), 27 - значит приём данных осуществляется на частоте 2,427 ГГц.
              radio.setDataRate     (RF24_1MBPS);                        // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек.
              radio.setPALevel      (RF24_PA_MAX);
            radio.openReadingPipe (0, 0xAABBCCDD11LL);  
            radio.startListening();
        }
        uint16_t update() override{
            Serial.println("data");
            if(radio.available()){                                     
                radio.read(&dataPackage,  sizeof(dataPackage)); 
            }    

            return NO_ERRORS;
        }

        data getData()
        {
            return dataPackage;
        }
};
