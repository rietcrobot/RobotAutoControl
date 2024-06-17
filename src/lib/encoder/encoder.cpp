#include "encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, void (*isr)())
    : pinA(pinA), pinB(pinB), position(0) {
    attachInterrupt(digitalPinToInterrupt(pinA), isr, CHANGE);
}

void Encoder::begin() {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
}

int Encoder::read() {
    noInterrupts();
    int pos = position;
    interrupts();
    return pos;
}

void Encoder::update() {
    if (digitalRead(pinA) == digitalRead(pinB)) {
        position--;
    } else {
        position++;
    }
}
