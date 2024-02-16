#pragma once

#include "Arduino.h"

#include "Updatable.h"
#include "Errors.h"

class Motor : public Updatable{
    public:
    Motor(int, int, int, int, double, int, double, double, double, double);
    void usePID(bool);
    void setSpeed(double speed);
    double getRotations();
    double getSpeed();
    uint16_t update() override;
    void interruptHandler();

    private:
    void applySpeed(double speed);

    int in1, in2;
    int encA, encB;

    bool usePIDFlag = true;
    double kP, kD, kI, maxIntegratedError;
    double oldError = 0;
    double integratedError = 0;

    int pulsesPerRotation; 
    volatile int32_t counter = 0;
    uint64_t timer = 0;
    double maxSpeed;
    double goalSpeed = 0, realSpeed = 0, rotations = 0;
};

//maxSpeed in rotations/second
Motor::Motor(int in1, int in2, int encA, int encB, double maxSpeed, int pulsesPerRotation, double kP, double kD, double kI, double maxIntegratedError){
    this->in1 = in1;
    this->in2 = in2;
    this->encA = encA;
    this->encB = encB;

    this->pulsesPerRotation = pulsesPerRotation;
    this->maxSpeed = maxSpeed;
    this->kP = kP;
    this->kD = kD;
    this->kI = kI;
    this->maxIntegratedError = maxIntegratedError;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(encA, INPUT);
    pinMode(encB, INPUT);

    applySpeed(0);
}

void Motor::usePID(bool enablePID){
    usePIDFlag = enablePID;
}

void Motor::setSpeed(double speed){
    goalSpeed = speed;
}

uint16_t Motor::update(){
    
    noInterrupts();
    int32_t c = counter;
    counter = 0;
    interrupts();

    double dTime = (millis()-timer)/1000.0;
    timer = millis();
    rotations += c/pulsesPerRotation;
    realSpeed = c/pulsesPerRotation/dTime;
    

    double u = 0;
    if(usePIDFlag){
        double error = realSpeed - goalSpeed;
        integratedError += error*dTime;
        if(integratedError > maxIntegratedError) integratedError = maxIntegratedError;
        if(integratedError < -maxIntegratedError) integratedError = -maxIntegratedError;
        u = error*kP + (error - oldError)*kD/dTime + integratedError*kI;
        oldError = error;
    }

    applySpeed(goalSpeed + u);

    return NO_ERRORS;
}

//speed in rotations/second
void Motor::applySpeed(double speed){
    int pwm = speed/maxSpeed*255;
    if(pwm > 255) pwm = 255;
    if(pwm < -255) pwm = -255;

    if(pwm >= 0){
        analogWrite(in1, 255);
        analogWrite(in2, 255 - pwm);
    }
    else{
        analogWrite(in1, 255 + pwm);
        analogWrite(in2, 255);
    }
}

double Motor::getRotations(){
    return rotations;
}

//Returns real speed measured with encoder in rotations/second
double Motor::getSpeed(){
    return realSpeed;
}

void Motor::interruptHandler(){
    bool a = digitalRead(encA);
    bool b = digitalRead(encB);

    if(a == 0){
        if(b == 1) counter++;
        else counter--;
    }
    else{
        if(b == 1) counter--;
        else counter++;
    }

}