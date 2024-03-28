#pragma once

#define TAU_PI 3.1415926
#define TAU_2PI 6.2831853

/**
 * @brief Базовый класс для определения динамических звеньев
 *
 */
class TauBase
{
public:
    TauBase(float Ts);
    virtual void reset();
    virtual float tick(float in) = 0;
    virtual float tick(float in, float in2) { return tick(in); }
    float get_val();

protected:
    float Ts;
    float out;
};

class Gain : public TauBase
{
public:
    Gain(float gain) : TauBase(0) { this->gain = gain; }
    float tick(float in) { return out = in * gain; }

protected:
    float gain;
};

/**
 * @brief Сумматор со сложением
 */
class Sum : public TauBase
{
public:
    Sum() : TauBase(0) {}
    float tick(float in) { return 0; }
    float tick(float in, float in2) { return out = in + in2; }
};

/**
 * @brief Сумматор с вычитанием
 */
class Sub : public TauBase
{
public:
    Sub() : TauBase(0) {}
    float tick(float in) { return 0; }
    float tick(float in, float in2) { return out = in - in2; }
};

/**
 * @brief Класс, позволяющий легко соединять цепочки из звеньев
 */
class Chain
{
public:
    Chain(float in) { val = in; }
    Chain &chain(TauBase *link)
    {
        val = link->tick(val);
        return *this;
    }
    Chain &chain2(TauBase *link, float in2)
    {
        val = link->tick(val, in2);
        return *this;
    }
    float get() { return val; }

protected:
    float val;
};

/**
 * @brief Интегратор
 *
 */
class Integrator : public TauBase
{
public:
    Integrator(float Ts);
    void set(float in);
    void reset() override;
    float tick(float in) override;

protected:
    float I;
};

/**
 * @brief Реальное дифференцирующее звено (фильтр высоких частот) первого порядка
 *
 */
class FOD : public TauBase
{
public:
    FOD(float Ts, float T, bool is_angle = false);
    void reset() override;
    float tick(float in) override;

protected:
    Integrator I;
    float T;
    bool is_angle;
};

/**
 * @brief Фильтр низких частот первого порядка
 *
 */
class FOLP : public TauBase
{
public:
    FOLP(float Ts, float T);
    void reset() override;
    float tick(float in) override;

protected:
    Integrator I;
    float T;
};

/**
 * @brief Звено насыщения
 *
 */
class Saturation : public TauBase
{
public:
    Saturation(float min_val, float max_val) : TauBase(0)
    {
        this->min_val = min_val;
        this->max_val = max_val;
    }
    void reset() override {}
    float tick(float in)
    {
        return out = (in < min_val) ? min_val : ((in > max_val) ? max_val : in);
    }

protected:
    float min_val, max_val;
};

/**
 * @brief Звено ограничения первой производной (скорости изменения)
 */
class RateLimiter : public TauBase
{
public:
    RateLimiter(float Ts, float max_der);
    void reset() override;
    float tick(float in) override;

protected:
    Integrator I;
    Saturation sat;
    float gain;
    float max_der;
};

class PIreg : public TauBase
{
public:
    PIreg(float Ts, float gain, float ki, float max_out);
    float tick(float in) override;

protected:
    float kp, ki;
    Integrator I;
    Saturation sat;
};

// class PISD : public TauBase {
// public:
//     PISD(float Ts, float gain, float kd, float ki, float max_out);
//     float tick(float in) override { return 0; }
//     float tick(float in, float in2) override;

// protected:
//     float gain;
//     float kd;
//     float ki;
//     float max_out;
//     Integrator I;
// };
