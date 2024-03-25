#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"

#define TIMEOUT_MS 10

class Button : public Updatable
{
public:
    Button(int);
    ERROR_TYPE update() override;
    bool isPressed();
    bool isClicked();
    bool isReleased();

private:
    int pin;
    long long timer;
    bool pressed = false;
    bool clicked = false;
    bool released = false;
    bool checked = false;
    bool prevState = false;
};

Button::Button(int pin)
{
    this->pin = pin;
    pinMode(pin, INPUT);
}

ERROR_TYPE Button::update()
{
    bool state = digitalRead(pin);

    if (checked)
    {
        clicked = false;
        released = false;
        checked = false;
    }

    if (state != pressed && millis() - timer > TIMEOUT_MS)
    {
        timer = millis();

        clicked |= !prevState && state;
        released |= prevState && !state;
        pressed = state;
    }

    prevState = state;

    return NO_ERRORS;
}

bool Button::isPressed()
{
    return pressed;
}

bool Button::isClicked()
{
    checked = true;
    return clicked;
}

bool Button::isReleased()
{
    checked = true;
    return released;
}
