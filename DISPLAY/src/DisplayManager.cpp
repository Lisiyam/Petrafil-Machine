#include "DisplayManager.h"

DisplayManager::DisplayManager(uint8_t address, uint8_t columns, uint8_t rows)
    : lcd(address, columns, rows), columns(columns), rows(rows) {}

void DisplayManager::begin() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
}

void DisplayManager::clear() {
    lcd.clear();
}

void DisplayManager::printAt(uint8_t column, uint8_t row, const String& text) {
    if (column >= columns || row >= rows) {
        return;
    }

    lcd.setCursor(column, row);
    lcd.print(text);
}

void DisplayManager::printPadded(uint8_t column, uint8_t row, const String& text, uint8_t width) {
    String output = text;
    while (output.length() < width) {
        output += ' ';
    }

    if (output.length() > width) {
        output = output.substring(0, width);
    }

    printAt(column, row, output);
}

void DisplayManager::setBacklight(bool enabled) {
    if (enabled) {
        lcd.backlight();
    } else {
        lcd.noBacklight();
    }
}
