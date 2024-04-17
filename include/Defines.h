#pragma once

#define Ts_us                                   6000 /*!< sample time [microseconds]*/
#define Ts_s                                    ((float)Ts_us * 0.000001)

#define BATTERY_WARNING_VOLTAGE                 11.1    //Volts
#define BATTERY_CRITICAL_VOLTAGE                9.6     //Volts
#define BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME   5000    //Miliseconds

#define MOTORS_ROBOT_MAX_SPEED                  1500 /*!< [mm/s] */

#define MOTORS_MAX_U                            12.0 /*!< [V] */
#define MOTORS_MOVE_U                           2  /*!< [V] */
#define MOTORS_MAX_SPEED                        30.0     //rad/second
#define MOTORS_KE                               0.185   /*!< [V/rad/s] */

#define MOTORS_POPUGI_TO_XY_MM_S                10.0
#define MOTORS_POPUGI_TO_W_RAD_S                0.04
#define MOTORS_WHEEL_RAD_MM                     23.5
#define MOTORS_ROBOT_RAD_MM                     82.0

#define MOTORS_PPR                              48     //Encoder pulses per rotation
#define MOTORS_GEAR_RATIO                       20.4
#define MOTORS_TIME_CONSTANT                    0.030
#define MOTORS_Tu                               Ts_s
#define MOTORS_ABS_OPTIMUM_SETTING              2.0
#define MOTORS_PI_KI                            (MOTORS_KE/(MOTORS_ABS_OPTIMUM_SETTING*MOTORS_Tu))
#define MOTORS_PI_GAIN                          (MOTORS_PI_KI*MOTORS_TIME_CONSTANT)

#define BALL_SENSOR_THRESHOLD                   20     //from 0 to 1024

#define KICK_TIME                               50      //Miliseconds
#define KICK_TIMEOUT                            2000     //Miliseconds

#define CONNECTION_TIMEOUT                      100000    //Miliseconds

//////////////////////////
#define LOG(name, val)  + String("\t") + String(name) + String(": ") + String(val)