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
#include "Fast.h"
#include "Odometry.h"
#include "Defines.h"

Button buttonChannelPlus(BUTTON_CHANEL_PLUS);
Button buttonChannelMinus(BUTTON_CHANEL_MINUS);
Button buttonEnter(BUTTON_ENTER);
VoltageMeter batteryVoltage(BATTERY_VOLTAGE, 5 * 2.5 / 1024.0, BATTERY_CRITICAL_VOLTAGE, 12.4);
BallSensor ballSensor(BALL_SENSOR, BALL_SENSOR_THRESHOLD);
Indicator indicator(INDICATOR_A, INDICATOR_B, INDICATOR_C, INDICATOR_D, INDICATOR_E, INDICATOR_F, INDICATOR_G, INDICATOR_DOT);
Motor motor1(
    MOTOR1_IN1,
    MOTOR1_IN2,
    MOTOR1_ENCB_PIN,
    MOTORS_MAX_U,
    MOTORS_MAX_SPEED,
    MOTORS_KE,
    MOTORS_PPR,
    Ts_s,
    MOTORS_PI_GAIN,
    MOTORS_PI_KI);
Motor motor2(
    MOTOR2_IN1,
    MOTOR2_IN2,
    MOTOR2_ENCB_PIN,
    MOTORS_MAX_U,
    MOTORS_MAX_SPEED,
    MOTORS_KE,
    MOTORS_PPR,
    Ts_s,
    MOTORS_PI_GAIN,
    MOTORS_PI_KI);
Motor motor3(
    MOTOR3_IN1,
    MOTOR3_IN2,
    MOTOR3_ENCB_PIN,
    MOTORS_MAX_U,
    MOTORS_MAX_SPEED,
    MOTORS_KE,
    MOTORS_PPR,
    Ts_s,
    MOTORS_PI_GAIN,
    MOTORS_PI_KI);
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
        RISING);
    attachInterrupt(
        MOTOR2_ENCA_CH, []()
        { motor2.interruptHandler(); },
        RISING);
    attachInterrupt(
        MOTOR3_ENCA_CH, []()
        { motor3.interruptHandler(); },
        RISING);
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
            static float speedy_mms = 0, speedx_mms = 0, speedw_rads = 0;

            if (channel == controlData.op_addr - 16)
            {
                // Kick
                if (controlData.flags & (0x01 << 6))
                    kick();

                // Auto kick
                autoKick = controlData.flags & (0x01<<4);

                speedy_mms = controlData.speed_y * MOTORS_POPUGI_TO_XY_MM_S;
                speedx_mms = controlData.speed_x * MOTORS_POPUGI_TO_XY_MM_S;
                speedw_rads = controlData.speed_w * MOTORS_POPUGI_TO_W_RAD_S;
            }

            static RateLimiter spd_lim_x(Ts_s, 400000);
            static RateLimiter spd_lim_y(Ts_s, 400000);

            float limitedx_mms = spd_lim_x.tick(speedx_mms);
            float limitedy_mms = spd_lim_y.tick(speedy_mms);

            float speed_mms = constrain(
                sqrt(limitedx_mms*limitedx_mms + limitedy_mms*limitedy_mms),
                0,
                MOTORS_ROBOT_MAX_SPEED);
            // float alpha = ang_lim.tick(
            //     atan2(speedx_mms, -speedy_mms)
            // );
            float alpha = atan2(limitedx_mms, -limitedy_mms);

            // static PIreg yawRate(Ts_s, 0, 1, 4);s
            static Integrator yawRate(Ts_s);
            static FOD yawFod(Ts_s, 0.4, false);

            float w_feedback_rads = 6*yawRate.tick(speedw_rads - -imu.getYawRate());

            float speedw_wheel_mms = speedw_rads * MOTORS_ROBOT_RAD_MM + w_feedback_rads * MOTORS_ROBOT_RAD_MM; // * fabs(yawFod.tick(speed_mms));

            float speed1_mms = speedw_wheel_mms + sin(alpha - 0.33 * M_PI) * speed_mms;
            float speed2_mms = speedw_wheel_mms + sin(alpha - M_PI) * speed_mms;
            float speed3_mms = speedw_wheel_mms + sin(alpha + 0.33 * M_PI) * speed_mms;

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

            // Serial.println(
            //     + "\t" + String(speedw_rads)
            //     + "\t" + String(w_feedback_rads)
            //     + "\t" + String(imu.getYawRate())
            // //     // + " " + String(controlData.speed_x)
            // //     // + " " + String(controlData.speed_y)
            // //     // + " " + String(controlData.speed_w)
            // //     // + " " + String(speedx_mms)
            // //     // + " " + String(speedy_mms)
            // //     // + " " + String(speedw_rads)
            // //     + " " + String(speed_mms)
            // //     + " " + String(alpha)
            // //     + " " + String(speedw_wheel_mms)
            // //     + " " + String(speed1_mms)
            // //     + " " + String(speed2_mms)
            // //     + " " + String(speed3_mms)
            //     // + " " + String(time_keeper)
            //     // + " " + String(motor1.getSpeed())
            //     + " " + String(imu.getYawRate())
                // + " " + String(speed1_mms)
                // + " " + String(motor1.getSpeed())
            // );
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
    od.tick();

    Serial.println("[" + String(time_delta) + "] " + "Voltage: " + String(batteryVoltage.getVoltage()) + "V; " 
            // + "channel: " + String(channel)
            // + "; ball sensor: " + String(ballSensor.getValue()) 
            // + "/" + String(ballSensor.getAnalogValue())
            + " x = " + String(od.getX())
            + " y = " + String(od.getY())
            + " theta = " + String(od.getTheta()));
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
