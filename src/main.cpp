#include "Arduino.h"
#include "BallSensor.h"
#include "Button.h"
#include "ConnectionList.h"
#include "EEPROM.h"
#include "Indicator.h"
#include "Motor.h"
#include "VoltageMeter.h"
#include <SPI.h>
#include "RF24.h"
#include "NRF24.h"
#include "Kicker.h"
#include "IMU.h"
#include "Odometry.h"
#include "Defines.h"
#include "BasicLinearAlgebra.h"

Button buttonChannelPlus(BUTTON_CHANEL_PLUS);
Button buttonChannelMinus(BUTTON_CHANEL_MINUS);
Button buttonEnter(BUTTON_ENTER);
VoltageMeter batteryVoltage(BATTERY_VOLTAGE, 5 * 2.5 / 1024.0, BATTERY_CRITICAL_VOLTAGE, 12.4);
BallSensor ballSensor(BALL_SENSOR, BALL_SENSOR_THRESHOLD);
Indicator indicator(INDICATOR_A, INDICATOR_B, INDICATOR_C, INDICATOR_D, INDICATOR_E, INDICATOR_F, INDICATOR_G, INDICATOR_DOT);

MotorConnectionParams mconnp1 =
{
    .IN1 =      MOTOR1_IN1,
    .IN2 =      MOTOR1_IN2,
    .ENCA_PIN = MOTOR1_ENCA_PIN,
    .ENCA_CH =  MOTOR1_ENCA_CH,
    .ENCB_PIN = MOTOR1_ENCB_PIN,
    .ENCB_CH =  MOTOR1_ENCB_CH,
    .ENC_PPR =  MOTORS_PPR,
    .i =        MOTORS_GEAR_RATIO,
    .ke =       MOTORS_KE,
    .ENC_PORT = MOTOR1_ENC_PORT,
    .ENC_MASK = MOTOR1_ENC_MASK,
    .ENC_SHIFT= MOTOR1_ENC_SHIFT,
    .ENC_DIR =  MOTOR1_ENC_DIR,
};
MotorConnectionParams mconnp2 =
{
    .IN1 =      MOTOR2_IN1,
    .IN2 =      MOTOR2_IN2,
    .ENCA_PIN = MOTOR2_ENCA_PIN,
    .ENCA_CH =  MOTOR2_ENCA_CH,
    .ENCB_PIN = MOTOR2_ENCB_PIN,
    .ENCB_CH =  MOTOR2_ENCB_CH,
    .ENC_PPR =  MOTORS_PPR,
    .i =        MOTORS_GEAR_RATIO,
    .ke =       MOTORS_KE,
    .ENC_PORT = MOTOR2_ENC_PORT,
    .ENC_MASK = MOTOR2_ENC_MASK,
    .ENC_SHIFT= MOTOR2_ENC_SHIFT,
    .ENC_DIR =  MOTOR2_ENC_DIR,
};
MotorConnectionParams mconnp3 =
{
    .IN1 =      MOTOR3_IN1,
    .IN2 =      MOTOR3_IN2,
    .ENCA_PIN = MOTOR3_ENCA_PIN,
    .ENCA_CH =  MOTOR3_ENCA_CH,
    .ENCB_PIN = MOTOR3_ENCB_PIN,
    .ENCB_CH =  MOTOR3_ENCB_CH,
    .ENC_PPR =  MOTORS_PPR,
    .i =        MOTORS_GEAR_RATIO,
    .ke =       MOTORS_KE,
    .ENC_PORT = MOTOR3_ENC_PORT,
    .ENC_MASK = MOTOR3_ENC_MASK,
    .ENC_SHIFT= MOTOR3_ENC_SHIFT,
    .ENC_DIR =  MOTOR3_ENC_DIR,
};

MotorControllerParams mctrlp = 
{
    .maxU = MOTORS_MAX_U,
    .moveU = MOTORS_MOVE_U,
    .maxSpeed = MOTORS_MAX_SPEED,
    .maxAccel = 9999,
    .Ts = Ts_s,
    .kp = MOTORS_PI_GAIN,
    .ki = MOTORS_PI_KI,
    .speedFilterT = 2*Ts_s,
    .maxUi = 9999
};

Motor motor1(&mconnp1, &mctrlp);
Motor motor2(&mconnp2, &mctrlp);
Motor motor3(&mconnp3, &mctrlp);

Kicker kicker(KICKER, KICK_TIME, KICK_TIMEOUT);

NRF24 nrf(NRF_CHIP_ENABLE, NRF_CHIP_SELECT);
IMU imu(IMU_CHIP_SELECT);

Odometry od(Ts_s, &motor1, &motor2, &motor3);

Updatable *input[] =
    {
        &buttonChannelPlus,
        &buttonChannelMinus,
        &buttonEnter,
        &batteryVoltage,
        &ballSensor,
        &nrf,
        &imu
};

Updatable *output[] =
    {
        &motor1,
        &motor2,
        &motor3,
        &kicker,
        &indicator
};

// Peripheral
#define INPUT_N (sizeof(input) / sizeof(input[0]))
#define OUTPUT_N (sizeof(output) / sizeof(output[0]))

data controlData;
bool autoKick = false;

bool initComplete = false;

uint8_t channel = 0;

uint32_t lowBatteryTimer = 0;
uint32_t kickTimer = 0;
uint32_t connectionTimer = 0;

ERROR_TYPE updateIN();
ERROR_TYPE updateOUT();
void kick();

enum ControlMode
{
    velocity,
    position
} controlMode;

struct Targets
{
    // Velocity control
    float xi_0_G_mm_s;
    float yi_0_G_mm_s;
    float w_0_G_rad_s;

    // Position control
    float x_0_G_mm;
    float y_0_G_mm;
    float theta_0_G_rad;
} targets;

void control_loop();

void setup()
{
    // Kicker pin initialization
    pinMode(KICKER, OUTPUT);
    digitalWrite(KICKER, LOW);

    // Show dash during initialization
    indicator.printDash();
    indicator.update();
    delay(100);

    // Serial for debug
    Serial.begin(115200);
    Serial.println("Initialization...");

    // Encoder interrupts handlers
    attachInterrupt(
        MOTOR1_ENCA_CH, []()
        { motor1.interruptHandler(); },
        CHANGE);
    attachInterrupt(
        MOTOR1_ENCB_CH, []()
        { motor1.interruptHandler(); },
        CHANGE);
    attachInterrupt(
        MOTOR2_ENCA_CH, []()
        { motor2.interruptHandler(); },
        CHANGE);
    attachInterrupt(
        MOTOR2_ENCB_CH, []()
        { motor2.interruptHandler(); },
        CHANGE);
    attachInterrupt(
        MOTOR3_ENCA_CH, []()
        { motor3.interruptHandler(); },
        CHANGE);
    attachInterrupt(
        MOTOR3_ENCB_CH, []()
        { motor3.interruptHandler(); },
        CHANGE);
    // Enable PID speed control for all motors
    motor1.usePID(true);
    motor2.usePID(true);
    motor3.usePID(true);

    ERROR_TYPE error = NO_ERRORS;
    // Initialize NRF
    error |= nrf.init();

    // Retrieve channel number from EEPROM
    channel = EEPROM.read(0);
    if (channel > 16)
        channel = 1;
    Serial.println("Channel: " + String(channel));

    // Print battery voltage
    batteryVoltage.update();
    Serial.println("Battery voltage: " + String(batteryVoltage.getVoltage()) + "V");

    imu.init();

    // Check if initialization was complited successfully
    if (error == NO_ERRORS)
    {
        Serial.println("Initialization complete");
        initComplete = true;
    }
    else
    {
        Serial.println("Initialization error: " + String(error));
        initComplete = false;
    }
}

uint32_t time_keeper = 0;
uint32_t time_delta;

void loop()
{
    while(micros() < time_keeper + Ts_us);
    time_delta = micros() - time_keeper;
    time_keeper = micros();

    // Update all perripheral
    uint32_t error = updateIN();

    // Read data from NRF
    if (nrf.available())
    {
        indicator.printDot();
        connectionTimer = millis();
        controlData = nrf.getData();
    }
    // Clear dot on indicator if no data was received in some time
    if (millis() - connectionTimer > 10)
    {
        indicator.clearDot();
    }

    // Show ballSensor status on blue LED
    digitalWrite(LED_BLUE, ballSensor.getValue());

    // Update NRF channel number
    if (buttonChannelPlus.isReleased() && channel < 16)
    {
        channel++;
        EEPROM.write(0, channel);
        Serial.println("Channel changed to: " + String(channel));
    }
    if (buttonChannelMinus.isReleased() && channel > 1)
    {
        channel--;
        EEPROM.write(0, channel);
        Serial.println("Channel changed to: " + String(channel));
    }
    indicator.print(channel);

    // Check battery voltage
    if (batteryVoltage.getVoltage() < BATTERY_WARNING_VOLTAGE)
        indicator.printL(); // Low battery warning
    if (batteryVoltage.getVoltage() > BATTERY_CRITICAL_VOLTAGE)
        lowBatteryTimer = millis();

    if (millis() - lowBatteryTimer < BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME)
    { // Move disabled if battery was criticaly low for some time

        // Kick from enter button
        if (buttonEnter.isReleased())
            kick();

        // Kick from enter button
        if (autoKick && ballSensor.getValue())
            kick();

        // Remote control
        if (millis() - connectionTimer < CONNECTION_TIMEOUT)
        {
            if (channel == controlData.op_addr - 16)
            {
                switch (controlData.packet_id)
                {
                case 0:
                    // Kick
                    if (controlData.p0.flags & (0x01 << 6))
                        kick();

                    // Auto kick
                    autoKick = controlData.p0.flags & (0x01<<4);

                    targets.yi_0_G_mm_s = controlData.p0.target_yi * MOTORS_POPUGI_TO_XY_MM_S;
                    targets.xi_0_G_mm_s = controlData.p0.target_xi * MOTORS_POPUGI_TO_XY_MM_S;
                    targets.w_0_G_rad_s = controlData.p0.target_w * MOTORS_POPUGI_TO_W_RAD_S;
                    
                    controlMode = velocity;
                    break;
                case 1:
                    // Kick
                    if (controlData.p0.flags & (0x01 << 6))
                        kick();

                    // Auto kick
                    autoKick = controlData.p0.flags & (0x01<<4);
                    
                    od.setAugment(
                        controlData.p1.real_x * MOTORS_POPUGI_TO_XY_MM,
                        controlData.p1.real_y * MOTORS_POPUGI_TO_XY_MM,
                        controlData.p1.real_theta * MOTORS_POPUGI_TO_W_RAD
                    );
                    break;
                
                case 2:
                    targets.x_0_G_mm = controlData.p2.target_x * MOTORS_POPUGI_TO_XY_MM;
                    targets.y_0_G_mm = controlData.p2.target_y * MOTORS_POPUGI_TO_XY_MM;
                    targets.theta_0_G_rad = controlData.p2.target_theta * MOTORS_POPUGI_TO_W_RAD;
                    
                    controlMode = position;
                    break;
                
                default:
                    break;
                }
            }

            control_loop();
        }
        else
        {
            motor1.setSpeed(0);
            motor2.setSpeed(0);
            motor3.setSpeed(0);

            autoKick = false;
        }
    }
    else
    {
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        motor3.setSpeed(0);
    }

    error |= updateOUT();

    // Error handling
    if (initComplete && error == NO_ERRORS)
    {
        // Show ok
        digitalWrite(LED_GREEN, HIGH);
    }
    else
    {
        // Show error
        digitalWrite(LED_GREEN, LOW);
        indicator.printError();
    }
    // Update indicator
    indicator.update();

    Serial.println("[" + String(time_delta) + "] " + "Voltage: " + String(batteryVoltage.getVoltage()) + "V; " 
            // // + "channel: " + String(channel)
            // // + "; ball sensor: " + String(ballSensor.getValue()) 
            // // + "/" + String(ballSensor.getAnalogValue())
            + " x = " + String(od.getX())
            + " y = " + String(od.getY())
            + " theta_G_rad = " + String(od.getTheta())
            // // LOG("m1 ticks", motor1.getTicks())
            // // LOG("m1 angle", motor1.getAngle())
            // LOG("m1 vel", motor1.getSpeed())
        );
}

// Update all input perripheral
ERROR_TYPE updateIN()
{
    ERROR_TYPE error = NO_ERRORS;

    // Update everything except indicator
    for (int i = 0; i < INPUT_N; i++)
    {
        error |= input[i]->update();
    }

    return error;
}

// Update all output perripheral
ERROR_TYPE updateOUT()
{
    ERROR_TYPE error = NO_ERRORS;

    // Update everything except indicator
    for (int i = 0; i < OUTPUT_N; i++)
    {
        error |= output[i]->update();
    }

    return error;
}

// Kicker handling
void kick()
{
    kicker.kick();
}

void control_loop()
{
    if(controlMode == position)
    {
        // Position control loop
        BLA::Matrix<2> pos_0_G_mm = {targets.x_0_G_mm, targets.y_0_G_mm};
        BLA::Matrix<2> pos_G_m = {od.getX(), od.getY()};
        BLA::Matrix<2> pos_G_mm = pos_G_m * BLA::Matrix<1>(1000);

        BLA::Matrix<2> pos_err_G_mm = pos_0_G_mm - pos_G_mm;

        static float constexpr pos_Tu = 0.15;
        static float constexpr pos_K = 1/(2*pos_Tu);

        BLA::Matrix<2> pos_u_G_mm_s = pos_err_G_mm * pos_K;

        float theta_G_rad = od.getTheta();
        BLA::Matrix<2,2> R = {cos(-theta_G_rad), -sin(-theta_G_rad), sin(-theta_G_rad), cos(-theta_G_rad)};

        BLA::Matrix<2> pos_u_L_mm_s = R*pos_u_G_mm_s;

        targets.xi_0_G_mm_s = pos_u_L_mm_s(0);
        targets.yi_0_G_mm_s = pos_u_L_mm_s(1);

        // Angle control loop
        targets.w_0_G_rad_s = (targets.theta_0_G_rad - theta_G_rad) * 8;
        targets.w_0_G_rad_s = constrain(targets.w_0_G_rad_s, -4, 4);
    }

    ////////////////////////////
    // Velocity control loop
    static RateLimiter spd_lim_x(Ts_s, 1000);
    static RateLimiter spd_lim_y(Ts_s, 1000);

    float limitedx_mms = spd_lim_x.tick(targets.xi_0_G_mm_s);
    float limitedy_mms = spd_lim_y.tick(targets.yi_0_G_mm_s);

    float speed_mms = constrain(
        sqrt(limitedx_mms*limitedx_mms + limitedy_mms*limitedy_mms),
        0,
        MOTORS_ROBOT_MAX_SPEED);
    float alpha = atan2(limitedx_mms, -limitedy_mms);

    static Integrator yawRate(Ts_s);

    // float w_feedback_rads = 6*yawRate.tick(speedw_rads - -imu.getYawRate());
    float w_feedback_rads = 0;

    float speedw_wheel_mms = targets.w_0_G_rad_s * MOTORS_ROBOT_RAD_MM + w_feedback_rads * MOTORS_ROBOT_RAD_MM;

    float speed1_mms = - speedw_wheel_mms + sin(alpha - 0.33 * M_PI) * speed_mms;
    float speed2_mms = - speedw_wheel_mms + sin(alpha - M_PI) * speed_mms;
    float speed3_mms = - speedw_wheel_mms + sin(alpha + 0.33 * M_PI) * speed_mms;

    float constexpr mms2rads = 1.0 / (MOTORS_WHEEL_RAD_MM);
    float speed1_rads = speed1_mms * mms2rads;
    float speed2_rads = speed2_mms * mms2rads;
    float speed3_rads = speed3_mms * mms2rads;


    float max_spd = max(max(fabs(speed1_rads), fabs(speed2_rads)), fabs(speed3_rads));
    float scaler = 1;
    if(max_spd > MOTORS_MAX_SPEED)
    {
        scaler = MOTORS_MAX_SPEED / fabs(max_spd);
    }

    // умножение быстрее деления
    motor1.setSpeed(speed1_rads * scaler);
    motor2.setSpeed(speed2_rads * scaler);
    motor3.setSpeed(speed3_rads * scaler);

    od.tick(imu.getYawRate());
}
