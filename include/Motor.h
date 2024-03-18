#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

#include "Tau.h"


class Motor : public Updatable{
public:
    Motor(int in1, int in2, int encB, float maxU, float maxSpeed, int pulsesPerRotation,
            float Ts, float gain, float T);
    void usePID(bool);
    void setSpeed(double speed);
    void zeroAngle();
    double getAngle();
    double getSpeed();
    uint16_t update() override; /*!< Обновить мотор. Вызывать раз в период квантования! */
    void interruptHandler();
    void applyU(float u);

private:
    int in1, in2;
    int encB;

    bool usePIDFlag = true;
    PIreg piReg;
    Saturation spdLimiter;
    FOD spdFilter;

    int pulsesPerRotation; /*!< мб не надо нам? */
    float pulses2rad; /*!< Коэффициент пересчета тиков в радианы */
    volatile int counter = 0;
    uint64_t timer = 0;
    float maxSpeed, maxU;
    float ke; /*!< Конструктивный коэффициент мотора [В/об] */

    float goalSpeed = 0, realSpeed = 0, angle = 0;
};

//maxSpeed in rad/second
Motor::Motor(int in1, int in2, int encB, float maxU, float maxSpeed, int pulsesPerRotation,
            float Ts, float gain, float T)
            : piReg(Ts, gain, T, maxU),
            spdLimiter(-maxSpeed, maxSpeed),
            spdFilter(Ts, Ts*2, true)
{
    this->in1 = in1;
    this->in2 = in2;
    this->encB = encB;

    this->pulsesPerRotation = pulsesPerRotation;
    this->pulses2rad = 2*M_PI/pulsesPerRotation;
    this->maxSpeed = maxSpeed;
    this->maxU = maxU;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(encB, INPUT);

    applyU(0);
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

    timer = millis();
    angle += c * pulses2rad;
    realSpeed = spdFilter.tick(angle);
    
    float feedforwardU = goalSpeed*ke;

    float feedbackU = piReg.tick(goalSpeed - realSpeed);

    applyU(feedforwardU + feedbackU);

    return NO_ERRORS;
}

//speed in rad/second
void Motor::applyU(float u){
    int pwm = u/maxU*255;
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

void Motor::zeroAngle(){
    angle = 0.0;
}

double Motor::getAngle(){
    return angle;
}

//Returns real speed measured with encoder in rad/second
double Motor::getSpeed(){
    return realSpeed;
}

void Motor::interruptHandler(){
    if(digitalRead(encB) == 1){
        counter++;
    }
    else{
        counter--;
    }
}
