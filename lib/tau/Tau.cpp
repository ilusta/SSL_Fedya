#include "Tau.h"

/*!v Tau Base class*/

TauBase::TauBase(float Ts)
{
    this->Ts = Ts;
    out = 0;
}

void TauBase::reset()
{

}

float TauBase::get_val()
{
    return out;
}

/*!v Integrator */

Integrator::Integrator(float Ts) : TauBase(Ts)
{
    reset();
}

void Integrator::set(float in)
{
    I = in;
}

void Integrator::reset()
{
    out = I = 0;
}

float Integrator::tick(float in)
{
    I += in * Ts;
    out = I;
}

/*!v First order high pass filter */

FOD::FOD(float Ts, float T, bool is_angle = false) : TauBase(Ts), I(Ts)
{
    this->T = T;
    this->is_angle = is_angle;
}

void FOD::reset()
{
    I.reset();
}

float FOD::tick(float in)
{
    float err = in - I.get_val();

    if(is_angle)
    {
        if(err > M_PI)
        {
            err -= M_2PI;
            I.set(I.get_val() + M_2PI);
        }
        else if(err < -M_2PI)
        {
            err += M_2PI;
            I.set(I.get_val() - M_2PI);
        }
    }

    out = err /  T;
    I.tick(out);
    return out;
}

/*!v First order low pass filter */

FOLP::FOLP(float Ts, float T) : TauBase(Ts), I(Ts)
{
    this->T = T;
}

void FOLP::reset()
{
    I.reset();
}

float FOLP::tick(float in)
{
    float err = in - out;
    out = err / T;
    I.tick(out);
    return out;
}

/*!v Rate limiter */

RateLimiter::RateLimiter(float Ts, float max_der) : 
    TauBase(Ts), I(Ts), sat(-max_der, max_der)
{
    this->max_der = max_der;
    this->gain = 1/Ts;
}

void RateLimiter::reset()
{
    I.reset();
}

float RateLimiter::tick(float in)
{
    float err = in - out;
    float satin = gain*err;
    return out = I.tick(sat.tick(satin));
}

PIreg::PIreg(float Ts, float gain, float T, float max_out) :
    TauBase(Ts), I(Ts), sat(-max_out, max_out)
{
    kp = gain;
    ki = ki/T;
}

float PIreg::tick(float in)
{
    out = kp * in + ki * I.tick(in);
    if(out != sat.tick(out))
    {
        I.tick(-in);
    }
    return out = sat.tick(out);
}
