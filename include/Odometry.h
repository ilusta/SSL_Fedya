#pragma once

#include "Arduino.h"
#include "Updatable.h"
#include "Defines.h"
#include "Motor.h"
#include "Tau.h"

float constexpr alpha2 = 0.67 * M_PI;
float constexpr alpha3 = 1.33 * M_PI;
float constexpr r = MOTORS_WHEEL_RAD_MM / 1000.0;
float constexpr R = MOTORS_ROBOT_RAD_MM / 1000.0;

float constexpr sina2 = sin(alpha2);
float constexpr cosa2 = cos(alpha2);
float constexpr sina3 = sin(alpha3);
float constexpr cosa3 = cos(alpha3);
float constexpr sina2_a3 = sin(alpha2 - alpha3);
float constexpr den_1 = 1/(sina3 - sina2 + sina2_a3);

// r (w2 - w3 - w1 cos(alpha2) + w1 cos(alpha3) - w2 cos(alpha3) + w3 cos(alpha2))
// -------------------------------------------------------------------------------
//                 sin(alpha3) - sin(alpha2) + sin(alpha2 - alpha3)

//   r (w1 sin(alpha2) - w1 sin(alpha3) + w2 sin(alpha3) - w3 sin(alpha2))
// - ---------------------------------------------------------------------
//              sin(alpha3) - sin(alpha2) + sin(alpha2 - alpha3)

// r (w2 sin(alpha3) - w3 sin(alpha2) + w1 sin(alpha2 - alpha3))
// -------------------------------------------------------------
//      R (sin(alpha3) - sin(alpha2) + sin(alpha2 - alpha3))

class Odometry
{
protected:

    Integrator x_G_m, y_G_m, theta_G_rad;
    float x_aug_G_m, y_aug_G_m, theta_aug_G_rad;
    float x_i, y_i, theta_i;
    Motor *m1, *m2, *m3;
    float Ts;

    float comp_a = 0.1;

public:
    Odometry(float Ts, Motor *m1, Motor *m2, Motor *m3) : x_G_m(Ts), y_G_m(Ts), theta_G_rad(Ts)
    {
        this->m1 = m2;
        this->m2 = m3;
        this->m3 = m1;
    }

    float getX() { return x_G_m.get_val(); }
    float getY() { return y_G_m.get_val(); }
    float getTheta() { return theta_G_rad.get_val(); }

    void setAugment(float x_aug_G_mm, float y_aug_G_mm, float theta_aug_G_rad)
    {
        x_aug_G_m = x_aug_G_mm * 0.001;
        y_aug_G_m = y_aug_G_mm * 0.001;
        this->theta_aug_G_rad = theta_aug_G_rad;
    }

    void augmentX() { x_G_m.set(x_G_m.get_val() * (1-comp_a) + x_aug_G_m * 0.001 * comp_a); }
    void augmentY() { y_G_m.set(y_G_m.get_val() * (1-comp_a) + y_aug_G_m * 0.001 * comp_a); }
    void augmentTheta() { theta_G_rad.set(theta_G_rad.get_val() * (1-comp_a) + theta_aug_G_rad * comp_a); }

    void tick(float gyro_w)
    {
        float w1 = m1->getSpeed();
        float w2 = m2->getSpeed();
        float w3 = m3->getSpeed();

        get_derivatives(w1, w2, w3, x_i, y_i, theta_i);
        x_G_m.tick(x_i);
        y_G_m.tick(y_i);
        theta_G_rad.tick(theta_i);
        augmentX();
        augmentY();
        augmentTheta();
    }

    void get_derivatives(float w1, float w2, float w3, float &x_i, float &y_i, float &theta_i)
    {
        float sint = sin(theta_G_rad.get_val());
        float cost = cos(theta_G_rad.get_val());

        float xl_i = r * (w1*sina2 - w1*sina3 + w2*sina3 - w3*sina2) * den_1;
        float yl_i = r * (w2 - w3 - w1*cosa2 + w1*cosa3 - w2*cosa3 + w3*cosa2) * den_1;

        x_i = cost*xl_i - sint*yl_i;
        y_i = sint*xl_i + cost*yl_i;        
        theta_i = - r/R * (w2*sina3 - w3*sina2 + w1*sina2_a3) * den_1;
    }
};

