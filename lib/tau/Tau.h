#pragma once

#define M_PI 3.1415926
#define M_2PI 6.2831853

/**
 * @brief Базовый класс для определения динамических звеньев
 * 
 */
class TauBase{
public:
    TauBase(float Ts);
    virtual void reset();
    virtual float process(float in) = 0;
    float get_val();

protected:
    float Ts;
    float out;
};

/**
 * @brief Интегратор
 * 
 */
class Integrator : public TauBase {
public:
    Integrator(float Ts);
    void set(float in);
    void reset() override;
    float process(float in) override;

protected:
    float I;
};

/**
 * @brief Реальное дифференцирующее звено первого порядка
 * 
 */
class FOD : public TauBase {
public:
    FOD(float Ts, float T, bool is_angle = false);
    void reset() override;
    float process(float x) override;

protected:
    Integrator I;
    float T;
    bool is_angle;
};

/**
 * @brief Фильтр низких частот первого порядка
 * 
 */
class FOLP : public TauBase {
public:
    FOLP(float Ts, float T);
    void reset() override;
    float process(float x) override;

private:
    Integrator I;
    float T;
};

// class PISD {
// public:
//     PISD(float dT, float gain, float kd, float ki, float max_out);
//     float process(float xerr, float x_i);
//     float get_val();

// private:
//     float _gain;
//     float _kd;
//     float _ki;
//     float _max_out;
//     Integrator _int;
//     float _out;
// };

// class RateLimiter {
// public:
//     RateLimiter(float Ts, float max_der);
//     float process(float x);
//     float get_val();

// private:
//     float _out;
//     Integrator _int;
//     float _k;
//     float _max_der;
// };
