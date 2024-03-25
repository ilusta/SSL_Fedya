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


#define BATTERY_WARNING_VOLTAGE                 11.1    //Volts
#define BATTERY_CRITICAL_VOLTAGE                9.6     //Volts
#define BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME   5000    //Miliseconds

#define MOTORS_MAX_SPEED                        6.0     //Rotations/second
#define MOTORS_PPR                              495     //Encoder pulses per rotation
#define MOTORS_PI_GAIN                          4.0
#define MOTORS_PI_TIME_CONSTANT                 0.05

#define BALL_SENSOR_THRESHOLD                   120     //from 0 to 1024

#define KICK_TIME                               10      //Miliseconds
#define KICK_TIMEOUT                            500     //Miliseconds

#define CONNECTION_TIMEOUT                      1000    //Miliseconds

// Peripheral
#define INPUT_N 6
#define OUTPUT_N 4

Button buttonChannelPlus = Button(BUTTON_CHANEL_PLUS);
Button buttonChannelMinus = Button(BUTTON_CHANEL_MINUS);
Button buttonEnter = Button(BUTTON_ENTER);
VoltageMeter batteryVoltage = VoltageMeter(BATTERY_VOLTAGE, 5 * 2.5 / 1024.0, BATTERY_CRITICAL_VOLTAGE, 12.4);
BallSensor ballSensor = BallSensor(BALL_SENSOR, BALL_SENSOR_THRESHOLD);
Indicator indicator = Indicator(INDICATOR_A, INDICATOR_B, INDICATOR_C, INDICATOR_D, INDICATOR_E, INDICATOR_F, INDICATOR_G, INDICATOR_DOT);
Motor motor1 = Motor(
    MOTOR1_IN1,
    MOTOR1_IN2,
    MOTOR1_ENCB_PIN,
    12,
    MOTORS_MAX_SPEED,
    MOTORS_PPR,
    0.005,
    MOTORS_PI_GAIN,
    MOTORS_PI_TIME_CONSTANT);
Motor motor2 = Motor(
    MOTOR2_IN1,
    MOTOR2_IN2,
    MOTOR2_ENCB_PIN,
    12,
    MOTORS_MAX_SPEED,
    MOTORS_PPR,
    0.005,
    MOTORS_PI_GAIN,
    MOTORS_PI_TIME_CONSTANT);
Motor motor3 = Motor(
    MOTOR3_IN1,
    MOTOR3_IN2,
    MOTOR3_ENCB_PIN,
    12,
    MOTORS_MAX_SPEED,
    MOTORS_PPR,
    0.005,
    MOTORS_PI_GAIN,
    MOTORS_PI_TIME_CONSTANT);
NRF24 nrf = NRF24(NRF_CHIP_ENABLE, NRF_CHIP_SELECT);
// TODO: imu

Updatable *input[INPUT_N] =
    {
        &buttonChannelPlus,
        &buttonChannelMinus,
        &buttonEnter,
        &batteryVoltage,
        &ballSensor,
        &nrf,
        //&imu,
};

Updatable *output[OUTPUT_N] =
    {
        &motor1,
        &motor2,
        &motor3,
        &indicator
};

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

    uint32_t error = NO_ERRORS;
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

void loop()
{
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

        // Remote control
        if (millis() - connectionTimer < CONNECTION_TIMEOUT)
        {
            if (channel == controlData.op_addr - 16)
            {

                // Kick
                if (controlData.flags & (0x01 << 6))
                    kick();

                //Auto kick
                autoKick = controlData.flags & (0x01<<4);

                float speedy = controlData.speed_y * MOTORS_MAX_SPEED / 127.0;
                float speedx = controlData.speed_x * MOTORS_MAX_SPEED / 127.0;
                float speedw = controlData.speed_w * MOTORS_MAX_SPEED / 127.0;
                float speed = min(MOTORS_MAX_SPEED, max(abs(speedy), abs(speedx)));
                float alpha = atan2(speedx, -speedy);

                float speed1 = speedw + sin(alpha - 0.33 * M_PI) * speed;
                float speed2 = speedw + sin(alpha - M_PI) * speed;
                float speed3 = speedw + sin(alpha + 0.33 * M_PI) * speed;

                motor1.setSpeed(speed1);
                motor2.setSpeed(speed2);
                motor3.setSpeed(speed3);
            }
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
        motor1.applyU(0);
        motor2.applyU(0);
        motor3.applyU(0);
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

    Serial.println("Voltage: " + String(batteryVoltage.getVoltage()) + "V; " + "channel: " + String(channel) + "; ball sensor: " + String(ballSensor.getValue()) + "/" + String(ballSensor.getAnalogValue()));
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
    if (millis() - kickTimer > KICK_TIMEOUT)
    {
        digitalWrite(KICKER, HIGH);
        delay(KICK_TIME);
        digitalWrite(KICKER, LOW);
        kickTimer = millis();
    }
}
