#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

#include "Tau.h"

struct MotorConnectionParams
{
    int IN1;
    int IN2;
    int ENCA_PIN;
    int ENCA_CH;
    //
    int ENCB_PIN;
    int ENCB_CH;

    int ENC_PPR; /*!< Pulses per rotation */
    float i; /*!< Gear ratio */
    //
    float ke;
    //
    volatile uint8_t *ENC_PORT;
    uint8_t ENC_MASK;
    uint8_t ENC_SHIFT;
};

struct MotorControllerParams
{
    float maxU;
    float maxSpeed;
    float maxAccel;
    float Ts;
    float kp;
    float ki;
    float speedFilterT;
    float maxUi; /*!< Max U change rate (derivative) */
};

class Motor : public Updatable, public MotorControllerParams, public MotorConnectionParams
{
public:
    Motor(MotorConnectionParams *mconnp, MotorControllerParams *mctrlp);
    void usePID(bool);
    /*!v Установить целевую скорость в [рад/с] */
    void setSpeed(double speed);
    void zeroAngle();
    double getAngle();
    int getTicks();
    int getPrevEnc() {return prevEnc;}
    double getSpeed();
    /*!v Обновить мотор. Вызывать раз в период квантования! */
    ERROR_TYPE update() override;
    void interruptHandler();
    void interruptHandlerB();
    void applyU(float u);

private:

    bool usePIDFlag = true;
    PIreg piReg;
    Saturation spdLimiter;
    RateLimiter accLimiter;
    FOD spdFilter;
    RateLimiter UchangeLimiter;

    float pulses2rad;      /*!< Коэффициент пересчета тиков в радианы */
    volatile int counter = 0;
    int encCounter = 0;
    uint64_t timer = 0;

    volatile uint8_t prevEnc;
    int8_t ett[4][4] = { 0 }; /*!< Encoder transition table */

    float goalSpeed = 0, realSpeed = 0, angle = 0;
};

// maxSpeed in rad/second
Motor::Motor(MotorConnectionParams *mconnp, MotorControllerParams *mctrlp)
    :   MotorConnectionParams(*mconnp),
        MotorControllerParams(*mctrlp),
        piReg(mctrlp->Ts, mctrlp->kp, mctrlp->ki, maxU),
        spdLimiter(-mctrlp->maxSpeed, mctrlp->maxSpeed),
        accLimiter(mctrlp->Ts, mctrlp->maxAccel),
        spdFilter(mctrlp->Ts, mctrlp->speedFilterT, true),
        UchangeLimiter(mctrlp->Ts, mctrlp->maxUi /* [V/s] */)
{
    this->pulses2rad = 2.0 * M_PI / (ENC_PPR * i);

    ett[0b00][0b10] = 1;
    ett[0b10][0b11] = 1;
    ett[0b11][0b01] = 1;
    ett[0b01][0b00] = 1;

    ett[0b00][0b01] = -1;
    ett[0b01][0b11] = -1;
    ett[0b11][0b10] = -1;
    ett[0b10][0b00] = -1;

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENCA_PIN, INPUT);
    pinMode(ENCB_PIN, INPUT);

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
    int c = counter;
    counter = 0;
    interrupts();

    timer = millis();
    encCounter += c;
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
        analogWrite(IN2, 255);
        analogWrite(IN1, 255 - pwm);
    }
    else
    {
        analogWrite(IN2, 255 + pwm);
        analogWrite(IN1, 255);
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

int Motor::getTicks()
{
    return encCounter;
}

// Returns real speed measured with encoder in rad/second
double Motor::getSpeed()
{
    return realSpeed;
}

void Motor::interruptHandler()
{
    uint8_t enc = ((*ENC_PORT) & ENC_MASK) >> ENC_SHIFT;
    counter += ett[prevEnc][enc];
    prevEnc = enc;
}
