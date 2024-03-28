#pragma once

#include <SPI.h>
#include "RF24.h"


#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

struct data {
  double op_addr;
  double speed_x;
  double speed_y;
  double speed_w;
  double voltage;
  uint8_t flags;
};

class NRF24: public Updatable{
    private:
        RF24 rad;
        data dataPackage;
        bool availableFlag;

    public:
        
        NRF24(uint8_t ce, uint8_t csn):rad(ce, csn){}

        uint16_t init(){
            if(!rad.begin()) return NRF_CONNECTION_ERROR;

			uint16_t error = NO_ERRORS;

            rad.setChannel(0x4c);                               				// Указываем канал передачи данных (от 0 до 125), 27 - значит приём данных осуществляется на частоте 2,427 ГГц.
            if(!rad.setDataRate(RF24_2MBPS)) error |= NRF_DATA_RATE_ERROR;		// Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек.
            rad.setPALevel(RF24_PA_MAX);
        
            rad.enableDynamicPayloads();
            rad.setCRCLength(RF24_CRC_16);
            rad.setAutoAck(1, false);
            
            rad.setAddressWidth(3);

            byte self_addr[]{0xAB, 0xAD, 0xAF};
            rad.openReadingPipe(1, self_addr);
            
            rad.startListening();
            rad.powerUp();

            availableFlag = true;

            return error;
        }

        uint16_t update() override{
            if(this->rad.available()){                                     
                
                byte recv[6];
                rad.read(&recv,  sizeof(recv)); 
                dataPackage.op_addr = recv[0];
                if (recv[1] <= 127)
                {
                  dataPackage.speed_x = recv[1];  
                }
                else
                {
                  dataPackage.speed_x = -(255 - recv[1]); 
                }
                if (recv[2] <= 127)
                {
                  dataPackage.speed_y = recv[2];  
                }
                else
                {
                  dataPackage.speed_y = -(255 - recv[2]);
                }

                if (recv[3] <= 127)
                {
                  dataPackage.speed_w = recv[3];  
                }
                else
                {
                  dataPackage.speed_w = -(255 - recv[3]);
                }
                dataPackage.voltage = recv[4];
                dataPackage.flags = recv[5];
                //Serial.println(recv[5]);

                availableFlag = true;
            }    

            return NO_ERRORS;
        }

        data getData()
        {   
            availableFlag = false;
            return dataPackage;
        }

        bool available(){
            return availableFlag;
        }
};
