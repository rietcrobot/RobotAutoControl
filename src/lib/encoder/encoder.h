#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB, void (*isr)());
    void begin();
    int read();
    void update();

private:
    uint8_t pinA;
    uint8_t pinB;
    volatile int position;
};

#endif
