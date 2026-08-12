#include <Arduino.h>
#include "DisplayManager.h"
#include "RotaryEncoder.h"
#include "ScreenController.h"

constexpr uint8_t LcdAddress = 0x27;
constexpr uint8_t LcdColumns = 20;
constexpr uint8_t LcdRows = 4;

constexpr uint8_t EncoderClkPin = 19;
constexpr uint8_t EncoderDtPin = 18;
constexpr uint8_t EncoderSwPin = 23;

DisplayManager display(LcdAddress, LcdColumns, LcdRows);
RotaryEncoder encoder(EncoderClkPin, EncoderDtPin, EncoderSwPin);
ScreenController screenController(display);

void setup() {
    display.begin();
    encoder.begin();
    screenController.begin();
}

void loop() {
    const EncoderEvent input = encoder.read();
    screenController.update(input);
    delay(2);
}
