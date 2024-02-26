#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"


class Motor : public Updatable{
    public:
    Motor(int, int, int, double, int, double, double, double, double);
    void usePID(bool);
    void setSpeed(double speed);
    void zeroRotations();
    double getRotations();
    double getSpeed();
    double getUnfilteredSpeed();
    uint16_t update() override;
    void interruptHandler();
    void applySpeed(double speed);

    private:
    int in1, in2;
    int encB;

    bool usePIDFlag = true;
    double kP, kD, kI, maxIntegratedError;
    double oldError = 0;
    double integratedError = 0;

    int pulsesPerRotation; 
    volatile int counter = 0;
    uint64_t timer = 0;
    double maxSpeed;
    double goalSpeed = 0, realSpeed = 0, rotations = 0;
    double realUnfilteredSpeed = 0, realUnfilteredSpeedOld = 0;
};

//maxSpeed in rotations/second
Motor::Motor(int in1, int in2, int encB, double maxSpeed, int pulsesPerRotation, double kP, double kD, double kI, double maxIntegratedError){
    this->in1 = in1;
    this->in2 = in2;
    this->encB = encB;

    this->pulsesPerRotation = pulsesPerRotation;
    this->maxSpeed = maxSpeed;
    this->kP = kP;
    this->kD = kD;
    this->kI = kI;
    this->maxIntegratedError = maxIntegratedError;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(encB, INPUT);

    applySpeed(0);
}

void Motor::usePID(bool enablePID){
    usePIDFlag = enablePID;
}

void Motor::setSpeed(double speed){
    goalSpeed = speed;
    if(goalSpeed > maxSpeed) goalSpeed = maxSpeed;
    if(goalSpeed < -maxSpeed) goalSpeed = -maxSpeed;
}

uint16_t Motor::update(){
    
    noInterrupts();
    float c = counter;
    counter = 0;
    interrupts();

    double dTime = (millis() - timer)/1000.0;
    timer = millis();
    rotations += double(c)/pulsesPerRotation;
    realUnfilteredSpeed = double(c)/pulsesPerRotation/dTime;
    realSpeed = 0.86*realSpeed + 0.07*(realUnfilteredSpeed + realUnfilteredSpeedOld);
    realUnfilteredSpeedOld = realUnfilteredSpeed;
    

    double u = 0;
    if(usePIDFlag){
        double error = goalSpeed - realSpeed;
        integratedError += error*dTime*kI;
        if(integratedError > maxIntegratedError) integratedError = maxIntegratedError;
        if(integratedError < -maxIntegratedError) integratedError = -maxIntegratedError;
        u = error*kP + (error - oldError)*kD/dTime + integratedError;
        oldError = error;

        if(u < 0.1 && u > -0.1) u = 0.0;
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
        analogWrite(in2, 255);
        analogWrite(in1, 255 - pwm);
    }
    else{
        analogWrite(in2, 255 + pwm);
        analogWrite(in1, 255);
    }
}

void Motor::zeroRotations(){
    rotations = 0.0;
}

double Motor::getRotations(){
    return rotations;
}

//Returns real speed measured with encoder in rotations/second
double Motor::getSpeed(){
    return realSpeed;
}

double Motor::getUnfilteredSpeed(){
    return realUnfilteredSpeed;
}

void Motor::interruptHandler(){
    if(digitalRead(encB) == 1){
        counter++;
    }
    else{
        counter--;
    }
}
