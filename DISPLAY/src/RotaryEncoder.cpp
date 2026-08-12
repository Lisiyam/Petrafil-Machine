#include "RotaryEncoder.h"

namespace {
constexpr unsigned long ButtonDebounceMs = 40;

constexpr int8_t TransitionTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};
}

RotaryEncoder::RotaryEncoder(uint8_t clkPin, uint8_t dtPin, uint8_t swPin)
    : clkPin(clkPin),
      dtPin(dtPin),
      swPin(swPin),
      previousState(0),
            position(0),
      previousButtonState(HIGH),
      lastButtonChangeMs(0) {}

void RotaryEncoder::begin() {
    pinMode(clkPin, INPUT_PULLUP);
    pinMode(dtPin, INPUT_PULLUP);
    pinMode(swPin, INPUT_PULLUP);

    previousState = (digitalRead(clkPin) << 1) | digitalRead(dtPin);
    previousButtonState = digitalRead(swPin);
}

EncoderEvent RotaryEncoder::read() {
    EncoderEvent event{0, false};

    const uint8_t currentState = (digitalRead(clkPin) << 1) | digitalRead(dtPin);
    if (currentState != previousState) {
        const uint8_t transitionIndex = (previousState << 2) | currentState;
        const int8_t movement = TransitionTable[transitionIndex];

        position += movement;
        previousState = currentState;

        if (position >= 4) {
            event.rotation = 1;
            position = 0;
        } else if (position <= -4) {
            event.rotation = -1;
            position = 0;
        }
    }

    const bool buttonState = digitalRead(swPin);
    const unsigned long now = millis();
    if (buttonState != previousButtonState && (now - lastButtonChangeMs) > ButtonDebounceMs) {
        lastButtonChangeMs = now;
        previousButtonState = buttonState;

        if (buttonState == LOW) {
            event.clicked = true;
        }
    }

    return event;
}
