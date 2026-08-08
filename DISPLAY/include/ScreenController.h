#pragma once

#include <Arduino.h>
#include "DisplayManager.h"
#include "RotaryEncoder.h"

enum class ScreenState {
    Loading,
    Home,
    Menu
};

class ScreenController {
public:
    explicit ScreenController(DisplayManager& display);

    void begin();
    void update(const EncoderEvent& input);

private:
    enum class MenuItem {
        FilamentReset,
        Temperature,
        Stepper,
        Exit
    };

    DisplayManager& display;
    ScreenState screen;
    MenuItem cursor;
    bool editMode;
    bool loadingFinished;
    unsigned long loadingStartedMs;
    unsigned long lastRenderMs;

    int filamentCm;
    int temperatureC;
    int stepperValue;

    void updateLoading();
    void updateHome(const EncoderEvent& input);
    void updateMenu(const EncoderEvent& input);

    void renderLoading(uint8_t progress, const String& status);
    void renderHome();
    void renderMenu();

    void moveCursor(int8_t direction);
    void editSelectedValue(int8_t direction);
    void selectCurrentItem();
    uint8_t cursorIndex() const;
    void setCursorByIndex(uint8_t index);
};
