#pragma once

#include <Arduino.h>

struct EncoderEvent {
    int8_t rotation;
    bool clicked;
};

class RotaryEncoder {
public:
    RotaryEncoder(uint8_t clkPin, uint8_t dtPin, uint8_t swPin);

    void begin();
    EncoderEvent read();

private:
    uint8_t clkPin;
    uint8_t dtPin;
    uint8_t swPin;
    uint8_t previousState;
    bool previousButtonState;
    unsigned long lastButtonChangeMs;
};
