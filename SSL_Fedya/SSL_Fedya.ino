#include "Arduino.h"
#include "BallSensor.h"
#include "Button.h"
#include "ConnectionList.h"
#include "EEPROM.h"
#include "Indicator.h"
#include "Motor.h"
#include "VoltageMeter.h"


#define BATTERY_WARNING_VOLTAGE                 11.1    //Volts
#define BATTERY_CRITICAL_VOLTAGE                9.6     //Volts
#define BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME   5000    //Miliseconds

#define MOTORS_MAX_SPEED                        6.0     //Rotations/second
#define MOTORS_PPR                              495     //Encoder pulses per rotation
#define MOTORS_PID_KP                           4.0
#define MOTORS_PID_KD                           0.05
#define MOTORS_PID_KI                           100.0
#define MOTORS_PID_MAX_INTEGRATED_ERROR         10.0

#define BALL_SENSOR_THRESHOLD                   500     //from 0 to 1024

#define KICK_TIME                               10      //Miliseconds
#define KICK_TIMEOUT                            500    //Miliseconds


//Peripheral
#define perihN 9
Button buttonChannelPlus = Button(BUTTON_CHANEL_PLUS);
Button buttonChannelMinus = Button(BUTTON_CHANEL_MINUS);
Button buttonEnter = Button(BUTTON_ENTER);
VoltageMeter batteryVoltage = VoltageMeter(BATTERY_VOLTAGE, 5*2.5/1024.0, BATTERY_CRITICAL_VOLTAGE, 12.4);
BallSensor ballSensor = BallSensor(BALL_SENSOR, BALL_SENSOR_THRESHOLD);
Indicator indicator = Indicator(INDICATOR_A, INDICATOR_B, INDICATOR_C, INDICATOR_D, INDICATOR_E, INDICATOR_F, INDICATOR_G, INDICATOR_DOT);
Motor motor1 = Motor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENCB_PIN, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
Motor motor2 = Motor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_ENCB_PIN, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
Motor motor3 = Motor(MOTOR3_IN1, MOTOR3_IN2, MOTOR3_ENCB_PIN, MOTORS_MAX_SPEED, MOTORS_PPR, MOTORS_PID_KP, MOTORS_PID_KD, MOTORS_PID_KI, MOTORS_PID_MAX_INTEGRATED_ERROR);
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
uint32_t kickTimer = 0;


void setup(){
    //Kicker pin initialization
    pinMode(KICKER, OUTPUT);
    digitalWrite(KICKER, LOW);

    //Show dash during initialization
    indicator.printDash();
    indicator.update();
    delay(100);

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


void loop(){
    //Update all perripheral
    update(1);

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
        if(buttonEnter.isReleased()) kick();

        motor1.setSpeed(0.0);
        motor2.setSpeed(0.0);
        motor3.setSpeed(0.0);
    }
    else{
        motor1.applySpeed(0);
        motor2.applySpeed(0);
        motor3.applySpeed(0);
    }

    Serial.println("Voltage: " + String(batteryVoltage.getVoltage()) + "V; "
     + "channel: " + String(channel) + "; " + String(motor3.getSpeed()) + " " + String(-0.5));
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
