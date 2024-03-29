# SSL_Fedya
Low level software for SSL robots.

Designed for Arduino Mega 2560

## Description
After powering on initialization will began automaticly. During initialization dash will be shown on the indicator and "Initialization..." will be printed to the serial with 115200 baudrate. Channel number and battery voltage are printed to serial. If initialization is finished successfully, "Initialization complete" will be printed to serial. If any error occures during initialization, "Initialization error: " with error code will be printed and robot will be in error state permanently until power off. Error codes can be found in "Errors.h" file.

After initialization robot will be ready for work. If everything is OK radio channel number is shown on the indicator and green LED will be turned on. Some information like battery voltage, radio channel number and ball sensor status is constantly printed to serial.

Error state is signaled by "E" on the indicator and turned off green LED.

Robot constantly monitors battery voltage. Low battery state is shown by "L" on the indicator. If battery voltage is criticly low for some time motors and kicker will be disabled. Low battery voltage, critical voltage and maximum allowable time for critical low voltage are configured by defines "BATTERY_WARNING_VOLTAGE", "BATTERY_CRITICAL_VOLTAGE" and "BATTERY_CRITICAL_VOLTAGE_MAXIMUM_TIME" in "main.cpp" file.

During normal work radio channel number will be shown on the indicator. If selected channel is greater then 9, "H" will be show on the indicator. Channel number can be changed by clicking "CH+" or "CH-" buttons. Channel number changes immediately and it is stored in EEPROM. Channel number is automaticly restored from EEPROM after power cycle. Dot will be shown on the indicator for 10 milliseconds after each successfully received data packet. Robot automaticly stops if no data is received for time defined by "CONNECTION_TIMEOUT" in "main.cpp" file.

Kick can be performed by clicking "ENTER" button. Kick duration and timeout for recharging capacitors is configured by "KICK_TIME" and "KICK_TIMEOUT" defines in "main.cpp" file.

Ball sensor status is shown on blue LED. Ball sensor threshold is configured by "BALL_SENSOR_THRESHOLD" define in "main.cpp" file.

## Supported features:
- 3 DC motors with incremental encoders, LPF for encoder values, PID speed control for each motor
- Kicker
- Dribbler ball presence sensor
- Seven segment indicator for showing radio channel number and othe info
- Buttons for changing radio channel number and other purposes
- Battery voltmeter for undervoltage protection
- 2 LEDs
- NRF
- MPU-9250

## TODO:

