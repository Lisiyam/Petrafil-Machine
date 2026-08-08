#include "RotaryEncoder.h"

namespace {
constexpr unsigned long ButtonDebounceMs = 40;
}

RotaryEncoder::RotaryEncoder(uint8_t clkPin, uint8_t dtPin, uint8_t swPin)
    : clkPin(clkPin),
      dtPin(dtPin),
      swPin(swPin),
      previousState(0),
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
        const uint8_t transition = (previousState << 2) | currentState;

        if (transition == 0b0001 || transition == 0b0111 || transition == 0b1110 || transition == 0b1000) {
            event.rotation = 1;
        } else if (transition == 0b0010 || transition == 0b1011 || transition == 0b1101 || transition == 0b0100) {
            event.rotation = -1;
        }

        previousState = currentState;
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
