#pragma once

#include <SPI.h>
#include "RF24.h"

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

struct data
{
  uint8_t op_addr;
  int8_t speed_x;
  int8_t speed_y;
  int8_t speed_w;
  uint8_t voltage;
  uint8_t flags;
};

class NRF24 : public Updatable
{
private:
  RF24 rad;
  data dataPackage;
  bool availableFlag;

  CHANNEL channel;

  bool checkChannel(uint8_t op_addr)
  {
    return channel == op_addr - 16;
  }

public:
  NRF24(uint8_t ce, uint8_t csn) : rad(ce, csn) {}

  ERROR_TYPE init()
  {
    if (!rad.begin())
      return NRF_CONNECTION_ERROR;

    ERROR_TYPE error = NO_ERRORS;

    rad.setChannel(0x4c); // Указываем канал передачи данных (от 0 до 125), 27 - значит приём данных осуществляется на частоте 2,427 ГГц.
    if (!rad.setDataRate(RF24_2MBPS))
      error |= NRF_DATA_RATE_ERROR; // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек.
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

  ERROR_TYPE update() override
  {
    while (this->rad.available())
    {
      byte recv[6];
      rad.read(&recv, sizeof(recv));

      if(checkChannel(recv[0]))
      {
        dataPackage.op_addr = recv[0];
        dataPackage.speed_x = recv[1];
        dataPackage.speed_y = recv[2];
        dataPackage.speed_w = recv[3];
        dataPackage.voltage = recv[4];
        dataPackage.flags = recv[5];
        availableFlag = true;
      }
    }

    return NO_ERRORS;
  }

  void setChannel(CHANNEL channel)
  {
    this->channel = channel;
  }

  data getData()
  {
    availableFlag = false;
    return dataPackage;
  }

  bool available()
  {
    return availableFlag;
  }
};
