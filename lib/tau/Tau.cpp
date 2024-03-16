#include "Tau.h"

TauBase::TauBase(float Ts)
{
    this->Ts = Ts;
}

void TauBase::reset()
{

}

float TauBase::get_val()
{
    return out;
}

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

float Integrator::process(float in)
{
    I += in * Ts;
    out = I;
}

FOD::FOD(float Ts, float T, bool is_angle = false) : TauBase(Ts), I(Ts)
{
    this->T = T;
    this->is_angle = is_angle;
    out = 0;
}

void FOD::reset()
{
    I.reset();
}

float FOD::process(float in)
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
    I.process(out);
    return out;
}

FOLP::FOLP(float Ts, float T) : TauBase(Ts), I(Ts)
{
    this->T = T;
    out = 0;
}

void FOLP::reset()
{
    I.reset();
}

float FOLP::process(float in)
{
    float err = in - out;
    out = I.process(err) / T;
    return out;
}
