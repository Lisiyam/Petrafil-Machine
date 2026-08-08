#pragma once

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

class DisplayManager {
public:
    DisplayManager(uint8_t address, uint8_t columns, uint8_t rows);

    void begin();
    void clear();
    void printAt(uint8_t column, uint8_t row, const String& text);
    void printPadded(uint8_t column, uint8_t row, const String& text, uint8_t width);
    void setBacklight(bool enabled);

private:
    LiquidCrystal_I2C lcd;
    uint8_t columns;
    uint8_t rows;
};
