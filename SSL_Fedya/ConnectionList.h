#pragma once

#include "Arduino.h"


//-------------------------LEDS----------------------------
#define LED_GREEN           23
#define LED_BLUE            22


//----------------------INDICATOR--------------------------
#define INDICATOR_A         24
#define INDICATOR_B         25
#define INDICATOR_C         26
#define INDICATOR_D         27
#define INDICATOR_E         28
#define INDICATOR_F         29
#define INDICATOR_G         30
#define INDICATOR_DOT       31


//-----------------------BUTTONS---------------------------
#define BUTTON_CHANEL_PLUS  15
#define BUTTON_CHANEL_MINUS 16
#define BUTTON_ENTER        17


//------------------------MOTORS---------------------------
#define DRIBBLER            13

#define MOTOR1_IN1          5
#define MOTOR1_IN2          4
#define MOTOR1_ENCA_PIN     2
#define MOTOR1_ENCA_CH      0
#define MOTOR1_ENCB_PIN     3
#define MOTOR1_ENCB_CH      1

#define MOTOR2_IN1          7
#define MOTOR2_IN2          6
#define MOTOR2_ENCA_PIN     18
#define MOTOR2_ENCA_CH      5
#define MOTOR2_ENCB_PIN     19
#define MOTOR2_ENCB_CH      4

#define MOTOR3_IN1          8
#define MOTOR3_IN2          9
#define MOTOR3_ENCA_PIN     20
#define MOTOR3_ENCA_CH      3
#define MOTOR3_ENCB_PIN     21
#define MOTOR3_ENCB_CH      2


//------------------------KICKER---------------------------
#define KICKER              14      //HIGH for a kick


//-----------------------SENSORS---------------------------
#define BATTERY_VOLTAGE     A0
#define BALL_SENSOR         A1


//-------------------------SPI-----------------------------
#define SPI_MOSI            51
#define SPI_MISO            50
#define SPI_SCK             52


//-------------------------NRF-----------------------------
#define NRF_CHIP_ENABLE     10
#define NRF_CHIP_SELECT     11


//-----------------------MPU-9250--------------------------
#define NRF_CHIP_SELECT     12
