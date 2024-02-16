#include "Arduino.h"
#include "ConnectionList.h"
#include "Button.h"
#include "BallSensor.h"
#include "Indicator.h"
#include "VoltageMeter.h"
#include "Motor.h"
#include "EEPROM.h"


#define BATTERY_WARNING_VOLTAGE                 11.1    //Volts
#define BATTERY_CRITICAL_VOLTAGE                9.6     //Volts
#define BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME   10000   //Miliseconds

#define MOTORS_MAX_SPEED                        7.0     //Rotations/second
#define MOTORS_PPR                              495     //Encoder pulses per rotation
#define MOTORS_PID_KP                           1.0
#define MOTORS_PID_KD                           0.0
#define MOTORS_PID_KI                           0.0
#define MOTORS_PID_MAX_INTEGRATED_ERROR         0.0

#define BALL_SENSOR_THRESHOLD                   500     //from 0 to 1024


//Peripheral
#define perihN 9
Button buttonChannelPlus = Button(BUTTON_CHANEL_PLUS);
Button buttonChannelMinus = Button(BUTTON_CHANEL_MINUS);
Button buttonEnter = Button(BUTTON_ENTER);
VoltageMeter batteryVoltage = VoltageMeter(BATTERY_VOLTAGE, 5*2.5/1024.0, BATTERY_CRITICAL_VOLTAGE, 12.4);
BallSensor ballSensor = BallSensor(BALL_SENSOR, BALL_SENSOR_THRESHOLD);
Indicator indicator = Indicator(INDICATOR_A, INDICATOR_B, INDICATOR_C, INDICATOR_D, INDICATOR_E, INDICATOR_F, INDICATOR_G, INDICATOR_DOT);
Motor motor1 = Motor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENCA, MOTOR1_ENCB, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
Motor motor2 = Motor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_ENCA, MOTOR2_ENCB, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
Motor motor3 = Motor(MOTOR3_IN1, MOTOR3_IN2, MOTOR3_ENCA, MOTOR3_ENCB, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
//NRF
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
    //&nrf,
    //&imu,
    &indicator
};

uint32_t error = NO_ERRORS;
bool initComplete = false;

uint8_t channel = 0;

uint32_t lowBatteryTimer = 0;


void setup(){
    //Encoder interrupts handlers
    attachInterrupt(MOTOR1_ENCA, [](){motor1.interruptHandler();}, CHANGE);
    attachInterrupt(MOTOR2_ENCA, [](){motor2.interruptHandler();}, CHANGE);
    attachInterrupt(MOTOR3_ENCA, [](){motor3.interruptHandler();}, CHANGE);

    //Indicator test
    indicator.printL();
    indicator.update();
    delay(500);
    indicator.printError();
    indicator.update();
    delay(500);
    indicator.printDot();
    indicator.update();
    delay(500);
    indicator.clearDot();
    indicator.update();
    delay(500);
    indicator.printDot(true);
    indicator.update();
    delay(500);
    indicator.printDot(false);
    indicator.update();
    delay(500);
    for(int i = 0; i <= 9; i++){
        indicator.print(i);
        indicator.update();
        delay(500);
    }
    indicator.clear();
    indicator.update();
    delay(500);
    indicator.printDash();
    indicator.update();
    delay(500);

    //Retrieve channel number from EEPROM
    channel = EEPROM.read(0);
    if(channel > 9) channel = 0;

    //Check if initialization was complited successfully
    if(error = NO_ERRORS) initComplete = true;
    else initComplete = false;
}


void loop(){
    //Update all perripheral
    update(1);

    //Show ballSensor status on blue LED
    digitalWrite(LED_BLUE, ballSensor.getValue());

    //Update NRF channel number
    if(buttonChannelPlus.isReleased() && channel < 9){
        channel++;
        EEPROM.write(0, channel);
    }
    if(buttonChannelMinus.isReleased() && channel > 0){
        channel--;
        EEPROM.write(0, channel);
    }
    indicator.print(channel);

    //Check battery voltage
    if(batteryVoltage.getVoltage() < BATTERY_WARNING_VOLTAGE) indicator.printL();                   //Low battery warning
    if(batteryVoltage.getVoltage() > BATTERY_CRITICAL_VOLTAGE) lowBatteryTimer = millis();
    if(millis() - lowBatteryTimer < BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME){                         //Move disabled if battery was criticaly low for some time

        //motor1.setSpeed(0);
        //motor2.setSpeed(0);
        //motor3.setSpeed(0);
    }
    else{
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        motor3.setSpeed(0);
    }
}


void update(uint32_t time){
    uint32_t timer = millis();
    error = NO_ERRORS;

    do{
        for(int i = 0; i < perihN; i++)
            error |= peripheral[i]->update();

        //Error handling
        if(initComplete && error == NO_ERRORS){
            //Show ok
            digitalWrite(LED_GREEN, HIGH);
        } else {
            //Show error
            digitalWrite(LED_GREEN, LOW);
            indicator.printError();
            indicator.update();
        }

    } while(millis() - timer < time);

}