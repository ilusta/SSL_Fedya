#pragma once

#include "Arduino.h"
#include "Errors.h"
#include "Updatable.h"


#define segmentN    8
//Segments location:
//      ----A----
//      |       |
//      F       B
//      |       |
//      ----G----
//      |       |
//      E       C
//      |       |
//      ----D----  dot


class Indicator : public Updatable{
    public:
    Indicator(int, int, int, int, int, int, int, int);
    uint16_t update() override;
    void clear();
    void printL();
    void printH();
    void printError();
    void printDash();
    void print(uint8_t);
    void printDot(bool);
    void inverseDot();
    void clearDot();

    private:
    int pins[8] = {0};
    bool state[8] = {0};
    uint8_t digits[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};     //Digits coded in hex for indicator. Bit 0 - segment A, bit 1 - segment B e.t.c
};


Indicator::Indicator(int a, int b, int c, int d, int e, int f, int g, int dot){
    pins[0] = a;
    pins[1] = b;
    pins[2] = c;
    pins[3] = d;
    pins[4] = e;
    pins[5] = f;
    pins[6] = g;
    pins[7] = dot;

    for(int i = 0; i < segmentN; i++){
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], 0);
    }
}

void Indicator::clear(){
    for(int i = 0; i < segmentN-1; i++)
        state[i] = 0;
}

void Indicator::print(uint8_t number){
    if(number > 9) printH();

    for(int i = 0; i < segmentN-1; i++)
        state[i] = digits[number] & (1 << i);
}

void Indicator::printDash(){
    for(int i = 0; i < segmentN-1; i++)
        state[i] = 0;

    state[6] = 1;
}

void Indicator::printH(){
    for(int i = 0; i < segmentN-1; i++)
        state[i] = 1;

    state[0] = 0;
    state[3] = 0;
}

void Indicator::printL(){
    for(int i = 0; i < segmentN-1; i++)
        state[i] = 0;

    state[3] = 1;
    state[4] = 1;
    state[5] = 1;
}

void Indicator::printError(){
    for(int i = 0; i < segmentN-1; i++)
        state[i] = 1;

    state[1] = 0;
    state[2] = 0;
}

void Indicator::inverseDot(){
    state[7] = !state[7];
}

void Indicator::printDot(bool print = 1){
    state[7] = print;
}

void Indicator::clearDot(){
    state[7] = 0;
}

uint16_t Indicator::update(){
    for(int i = 0; i < segmentN; i++)
        digitalWrite(pins[i], state[i]);

    return NO_ERRORS;
}
