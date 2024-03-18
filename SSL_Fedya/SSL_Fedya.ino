#include "Arduino.h"
#include "BallSensor.h"
#include "Button.h"
#include "ConnectionList.h"
#include "EEPROM.h"
#include "Indicator.h"
#include "Motor.h"
#include "VoltageMeter.h"
#include <SPI.h>
#include "RF24.h"
#include "NRF24.h"
//#include <math.h>



#define BATTERY_WARNING_VOLTAGE                 11.1    //Volts
#define BATTERY_CRITICAL_VOLTAGE                9.6     //Volts
#define BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME   5000    //Miliseconds

#define MOTORS_MAX_SPEED                        6.0     //Rotations/second
#define MOTORS_PPR                              495     //Encoder pulses per rotation
#define MOTORS_PID_KP                           4.0
#define MOTORS_PID_KD                           0.03
#define MOTORS_PID_KI                           100.0
#define MOTORS_PID_MAX_INTEGRATED_ERROR         10.0

#define BALL_SENSOR_THRESHOLD                   500     //from 0 to 1024

#define KICK_TIME                               20      //Miliseconds
#define KICK_TIMEOUT                            1000    //Miliseconds


//RF24 rad(10, 11);

//Peripheral
#define perihN 10
Button buttonChannelPlus = Button(BUTTON_CHANEL_PLUS);
Button buttonChannelMinus = Button(BUTTON_CHANEL_MINUS);
Button buttonEnter = Button(BUTTON_ENTER);
VoltageMeter batteryVoltage = VoltageMeter(BATTERY_VOLTAGE, 5*2.5/1024.0, BATTERY_CRITICAL_VOLTAGE, 12.4);
BallSensor ballSensor = BallSensor(BALL_SENSOR, BALL_SENSOR_THRESHOLD);
Indicator indicator = Indicator(INDICATOR_A, INDICATOR_B, INDICATOR_C, INDICATOR_D, INDICATOR_E, INDICATOR_F, INDICATOR_G, INDICATOR_DOT);
Motor motor1 = Motor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENCB_PIN, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
Motor motor2 = Motor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_ENCB_PIN, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
Motor motor3 = Motor(MOTOR3_IN1, MOTOR3_IN2, MOTOR3_ENCB_PIN, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
NRF24 nrf = NRF24(10, 11);//NRF
//IMU

Updatable* peripheral[perihN] ={
    &buttonChannelPlus,
    &buttonChannelMinus,
    &buttonEnter,
    &batteryVoltage,
    &ballSensor,
    &motor1,
    &motor2,
    &motor3,
    &nrf,
    //&imu,
    &indicator
};

//struct data {
//  uint8_t op_addr;
//  uint8_t speed_x;
//  uint8_t speed_y;
//  uint8_t speed_w;
//  uint8_t voltage;
//  uint8_t flags;
//};

uint32_t error = NO_ERRORS;
bool initComplete = false;

uint8_t channel = 0;

uint32_t lowBatteryTimer = 0;
uint32_t kickTimer = 0;


void setup(){
    //Kicker pin initialization
    pinMode(KICKER, OUTPUT);
    digitalWrite(KICKER, LOW);

    //Show dash during initialization
    indicator.printDash();
    indicator.update();
    delay(100);

    nrf.init();
    //Serial for debug
    Serial.begin(115200);
    Serial.println("Initialization...");

    //Encoder interrupts handlers
    attachInterrupt(MOTOR1_ENCA_CH, [](){motor1.interruptHandler();}, RISING);
    attachInterrupt(MOTOR2_ENCA_CH, [](){motor2.interruptHandler();}, RISING);
    attachInterrupt(MOTOR3_ENCA_CH, [](){motor3.interruptHandler();}, RISING);
    //Enable PID speed control for all motors
    motor1.usePID(true);
    motor2.usePID(true);
    motor3.usePID(true);

    //Retrieve channel number from EEPROM
    channel = EEPROM.read(0);
    if(channel > 9) channel = 0;
    Serial.println("Channel: " + String(channel));

    //Print battery voltage
    batteryVoltage.update();
    Serial.println("Battery voltage: " + String(batteryVoltage.getVoltage()) + "V");

    //Check if initialization was complited successfully
    if(error == NO_ERRORS) {
        Serial.println("Initialization complete");
        initComplete = true;
    }
    else {
        Serial.println("Initialization error");
        initComplete = false;
    }
}

data control_data;

void loop(){
    //Update all perripheral
    update(1);
    control_data = nrf.getData();
//    Serial.print("op_addr: ");
//    Serial.print(nrf.getData().op_addr);
//    Serial.print(" speed_x: ");
//    Serial.print(nrf.getData().speed_x);
//    Serial.print(" speed_y: ");
//    Serial.print(nrf.getData().speed_y);
//    Serial.print(" speed_w: ");
//    Serial.print(nrf.getData().speed_w);
//    Serial.print(" voltage: ");
//    Serial.print(nrf.getData().voltage);
//    Serial.print(" flags: ");
//    Serial.println(nrf.getData().flags); 

    //Show ballSensor status on blue LED
    digitalWrite(LED_BLUE, ballSensor.getValue());

    //Update NRF channel number
    if(buttonChannelPlus.isReleased() && channel < 9){
        channel++;
        EEPROM.write(0, channel);
        Serial.println("Channel changed to: " + String(channel));
    }
    if(buttonChannelMinus.isReleased() && channel > 0){
        channel--;
        EEPROM.write(0, channel);
        Serial.println("Channel changed to: " + String(channel));
    }
    indicator.print(channel);

    //Check battery voltage
    if(batteryVoltage.getVoltage() < BATTERY_WARNING_VOLTAGE) indicator.printL();                   //Low battery warning
    if(batteryVoltage.getVoltage() > BATTERY_CRITICAL_VOLTAGE) lowBatteryTimer = millis();
    if(millis() - lowBatteryTimer < BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME){                         //Move disabled if battery was criticaly low for some time

        //Kick from enter button
//        Serial.println()
        if(buttonEnter.isReleased()) kick();
        
        if(channel == control_data.op_addr - 16)
        {
          if (control_data.flags & (0x01<<6)) kick();
          float speedy = control_data.speed_y * MOTORS_MAX_SPEED / 127.0;
          float speedx = control_data.speed_x * MOTORS_MAX_SPEED / 127.0;
          float speedw = control_data.speed_w * MOTORS_MAX_SPEED / 127.0;
          float podacha = min(MOTORS_MAX_SPEED, max(abs_(speedy), abs_(speedx)));
          Serial.print(speedx);
          Serial.print(" ");
          float alpha = atan2(speedx, -speedy);
          double speed1 = sin(float(alpha - 1.0/3.0*M_PI)) * podacha;
          double speed2 = sin(float(alpha - M_PI)) * podacha;
          double speed3 = sin(float(alpha + 1.0/3.0*M_PI)) * podacha;
          Serial.print(speedy);
          Serial.print(" ");
          Serial.print(control_data.speed_y);
          Serial.print(" ");
          Serial.println(podacha);
          motor1.setSpeed(speed1 + speedw);
          motor2.setSpeed(speed2 + speedw);
          motor3.setSpeed(speed3 + speedw);
        }
    }
    else{
        motor1.applySpeed(0);
        motor2.applySpeed(0);
        motor3.applySpeed(0);
    }

//    Serial.println("Voltage: " + String(batteryVoltage.getVoltage()) + "V; "
//     + "channel: " + String(channel) + "; ");// + String(motor3.getSpeed()) + " " + String(-5.0));
}


//Update all perripheral during given time
void update(uint32_t time){
    uint32_t timer = millis();
    error = NO_ERRORS;

    do{
        //Update everything except indicator 
        for(int i = 0; i < perihN-1; i++)
            error |= peripheral[i]->update();

        //Error handling
        if(initComplete && error == NO_ERRORS){
            //Show ok
            digitalWrite(LED_GREEN, HIGH);
        } else {
            //Show error
            digitalWrite(LED_GREEN, LOW);
            indicator.printError();
        }
        //Update indicator
        indicator.update();

    } while(millis() - timer < time);
}


//Kicker handling
void kick(){
    if(millis() - kickTimer > KICK_TIMEOUT){
        digitalWrite(KICKER, HIGH);
        delay(KICK_TIME);
        digitalWrite(KICKER, LOW);
        kickTimer = millis();
    }
}

float min_3(float x1, float x2, float x3)
{
  if (x1 > x2)
  {
    if (x1 > x3)
    {
      return x3;
    }
    else
    {
      return x1;
    }
  }
  else
  {
    if(x2 > x3)
    {
      return x3;
    }
    else
    {
      return x2;
    }
  }
}

float abs_(float x)
{
  if (x > 0)
  {
    return x;
  }
  else
  {
    return -x;
  }
}
