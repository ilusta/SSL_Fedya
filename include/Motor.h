#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

#include "Tau.h"

class Motor : public Updatable
{
public:
    Motor(int in1, int in2, int encB, float maxU, float maxSpeed, float ke, int pulsesPerRotation,
          float Ts, float gain, float ki);
    void usePID(bool);
    /*!v Установить целевую скорость в [рад/с] */
    void setSpeed(double speed);
    void zeroAngle();
    double getAngle();
    double getSpeed();
    /*!v Обновить мотор. Вызывать раз в период квантования! */
    ERROR_TYPE update() override;
    void interruptHandler();
    void applyU(float u);

private:
    int in1, in2;
    int encB;

    bool usePIDFlag = true;
    PIreg piReg;
    Saturation spdLimiter;
    RateLimiter accLimiter;
    FOD spdFilter;
    RateLimiter UchangeLimiter;

    int pulsesPerRotation; /*!< мб не надо нам? */
    float pulses2rad;      /*!< Коэффициент пересчета тиков в радианы */
    volatile int counter = 0;
    uint64_t timer = 0;
    float maxSpeed, maxU;
    float ke; /*!< Конструктивный коэффициент мотора [В/об] */

    float goalSpeed = 0, realSpeed = 0, angle = 0;
};

// maxSpeed in rad/second
Motor::Motor(int in1, int in2, int encB, float maxU, float maxSpeed, float ke, int pulsesPerRotation,
             float Ts, float gain, float ki)
    : piReg(Ts, gain, ki, maxU),
      spdLimiter(-maxSpeed, maxSpeed),
      accLimiter(Ts, 400),
      spdFilter(Ts, Ts * 2, true),
      UchangeLimiter(Ts, 300 /* [V/s] */)
{
    this->in1 = in1;
    this->in2 = in2;
    this->encB = encB;

    this->pulsesPerRotation = pulsesPerRotation;
    this->pulses2rad = 2 * M_PI / pulsesPerRotation;
    this->maxSpeed = maxSpeed;
    this->maxU = maxU;
    this->ke = ke;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(encB, INPUT);

    applyU(0);
}

void Motor::usePID(bool enablePID)
{
    usePIDFlag = enablePID;
}

void Motor::setSpeed(double speed)
{
    goalSpeed = accLimiter.tick(spdLimiter.tick(speed));
}

ERROR_TYPE Motor::update()
{
    noInterrupts();
    float c = counter;
    counter = 0;
    interrupts();

    timer = millis();
    angle += c * pulses2rad;
    realSpeed = spdFilter.tick(angle);

    float feedforwardU = goalSpeed * ke;

    float feedbackU = usePIDFlag ? piReg.tick(goalSpeed - realSpeed) : 0;

    applyU(feedforwardU + feedbackU);

    return NO_ERRORS;
}

// speed in rad/second
void Motor::applyU(float u)
{
    float slowed_u = UchangeLimiter.tick(u);
    // float slowed_u = u;
    int pwm = slowed_u / maxU * 255;

    if (pwm > 255)
        pwm = 255;
    if (pwm < -255)
        pwm = -255;

    if (pwm >= 0)
    {
        analogWrite(in2, 255);
        analogWrite(in1, 255 - pwm);
    }
    else
    {
        analogWrite(in2, 255 + pwm);
        analogWrite(in1, 255);
    }
}

void Motor::zeroAngle()
{
    angle = 0.0;
}

double Motor::getAngle()
{
    return angle;
}

// Returns real speed measured with encoder in rad/second
double Motor::getSpeed()
{
    return realSpeed;
}

void Motor::interruptHandler()
{
    if (digitalRead(encB) == 1)
    {
        counter++;
    }
    else
    {
        counter--;
    }
}
