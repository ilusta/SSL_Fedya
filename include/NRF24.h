#pragma once

#include <SPI.h>
#include "RF24.h"

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

// struct data
// {
//   union
//   {
//     uint8_t raw[6];
//     struct
//     {
//       uint8_t packet_id : 3;
//       uint8_t op_addr : 5;
//     };
//     struct
//     {
//       uint8_t op_addr_byte;
//       int8_t target_xi;
//       int8_t target_yi;
//       int8_t target_w;
//       uint8_t kicker_voltage;
//       uint8_t flags;
//     } p0;
//     struct
//     {
//       uint8_t op_addr_byte;
//       int8_t real_x;
//       int8_t real_y;
//       int8_t real_theta;
//       uint8_t kicker_voltage;
//       uint8_t flags;
//     } p1;
//     struct
//     {
//       uint8_t op_addr_byte;
//       int8_t target_x;
//       int8_t target_y;
//       int8_t target_theta;
//       int8_t target_xi;
//       int8_t target_yi;
//     } p2;
//   };
// };
struct data{
      uint8_t op_addr;
      int8_t target_x;
      int8_t target_y;
      int8_t real_x;
      int8_t real_y;
      int8_t real_theta;      
      int8_t target_theta;
      int8_t target_xi;
      int8_t target_yi;
      uint8_t kicker_voltage;
      uint8_t flags;
};

class NRF24 : public Updatable
{
private:
  RF24 rad;
  data dataPackage;
  bool availableFlag;

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
    if (this->rad.available())
    {

      byte recv[8];
      rad.read(&recv, sizeof(recv));
      // for(size_t i = 0; i < 8; i++)
      // {
      //   dataPackage.raw[i] = recv[i];
      // }
      dataPackage.op_addr = recv[0];
      dataPackage.target_x = recv[1];
      dataPackage.target_y = recv[2];
      dataPackage.real_x = recv[3];
      dataPackage.real_y = recv[4];
      dataPackage.real_theta = recv[5];      
      dataPackage.target_theta = recv[6];
      dataPackage.target_xi = recv[7];
      dataPackage.target_yi = recv[8];
      dataPackage.kicker_voltage = recv[9];
      dataPackage.flags = recv[10];
      availableFlag = true;
    }

    return NO_ERRORS;
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
