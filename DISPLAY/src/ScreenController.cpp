#include "ScreenController.h"

namespace {
constexpr unsigned long LoadingDurationMs = 3000;
constexpr unsigned long LoadingRenderMs = 150;
constexpr uint8_t LoadingBarWidth = 12;
constexpr int MinTemperature = 0;
constexpr int MaxTemperature = 300;
constexpr int MinStepper = 0;
constexpr int MaxStepper = 255;
}

ScreenController::ScreenController(DisplayManager& display)
    : display(display),
      screen(ScreenState::Loading),
      cursor(MenuItem::FilamentReset),
      editMode(false),
      loadingFinished(false),
      loadingStartedMs(0),
      lastRenderMs(0),
      filamentCm(0),
      temperatureC(0),
            stepperValue(0) {}

void ScreenController::begin() {
    loadingStartedMs = millis();
    renderLoading(0, "LOADING");
}

void ScreenController::update(const EncoderEvent& input) {
    switch (screen) {
        case ScreenState::Loading:
            updateLoading();
            break;
        case ScreenState::Home:
            updateHome(input);
            break;
        case ScreenState::Menu:
            updateMenu(input);
            break;
    }
}

void ScreenController::updateLoading() {
    const unsigned long now = millis();
    const unsigned long elapsed = now - loadingStartedMs;
    const uint8_t progress = min<uint8_t>(100, (elapsed * 100UL) / LoadingDurationMs);

    if (now - lastRenderMs >= LoadingRenderMs) {
        renderLoading(progress, progress >= 100 ? "READY" : "LOADING");
        lastRenderMs = now;
    }

    if (!loadingFinished && elapsed >= LoadingDurationMs + 700) {
        loadingFinished = true;
        screen = ScreenState::Home;
        renderHome();
    }
}

void ScreenController::updateHome(const EncoderEvent& input) {
    if (input.clicked) {
        screen = ScreenState::Menu;
        cursor = MenuItem::FilamentReset;
        editMode = false;
        renderMenu();
    }
}

void ScreenController::updateMenu(const EncoderEvent& input) {
    if (input.rotation != 0) {
        if (editMode) {
            editSelectedValue(input.rotation);
        } else {
            moveCursor(input.rotation);
        }
        renderMenu();
    }

    if (input.clicked) {
        selectCurrentItem();
    }
}

void ScreenController::renderLoading(uint8_t progress, const String& status) {
    const uint8_t filled = (progress * LoadingBarWidth) / 100;
    String bar = "[";
    for (uint8_t i = 0; i < LoadingBarWidth; i++) {
        bar += i < filled ? char(255) : ' ';
    }
    bar += "]";

    display.printPadded(0, 0, " PETRAFILE MACHINE ", 20);
    display.printPadded(0, 1, "                    ", 20);
    display.printPadded(0, 2, bar + " " + String(progress) + "%", 20);
    display.printPadded(0, 3, " STATUS: " + status, 20);
}

void ScreenController::renderHome() {
    display.printPadded(0, 0, " PETRAFILE MACHINE ", 20);
    display.printPadded(0, 1, "filamen : " + String(filamentCm) + " cm", 20);
    display.printPadded(0, 2, "suhu    : " + String(temperatureC) + " C  ", 20);
    display.printPadded(0, 3, "stepper : " + String(stepperValue), 20);
}

void ScreenController::renderMenu() {
    const bool onFilament = cursor == MenuItem::FilamentReset;
    const bool onTemperature = cursor == MenuItem::Temperature;
    const bool onStepper = cursor == MenuItem::Stepper;
    const bool onExit = cursor == MenuItem::Exit;

    display.printPadded(0, 0, String(onFilament ? ">" : " ") + "filamen: reset", 20);
    display.printPadded(0, 1, String(onTemperature ? ">" : " ") + "suhu   : " + String(temperatureC) + (editMode && onTemperature ? "*" : " ") + " 0-300", 20);
    display.printPadded(0, 2, String(onStepper ? ">" : " ") + "stepper: " + String(stepperValue) + (editMode && onStepper ? "*" : " ") + " 0-255", 20);
    display.printPadded(0, 3, String(onExit ? ">" : " ") + "      exit", 20);
}

void ScreenController::moveCursor(int8_t direction) {
    int nextIndex = cursorIndex() + (direction > 0 ? 1 : -1);

    if (nextIndex < 0) {
        nextIndex = 3;
    } else if (nextIndex > 3) {
        nextIndex = 0;
    }

    setCursorByIndex(nextIndex);
}

void ScreenController::editSelectedValue(int8_t direction) {
    const int step = direction > 0 ? 1 : -1;

    if (cursor == MenuItem::Temperature) {
        temperatureC = constrain(temperatureC + step, MinTemperature, MaxTemperature);
    } else if (cursor == MenuItem::Stepper) {
        stepperValue = constrain(stepperValue + step, MinStepper, MaxStepper);
    }
}

void ScreenController::selectCurrentItem() {
    if (cursor == MenuItem::FilamentReset) {
        filamentCm = 0;
        renderMenu();
        return;
    }

    if (cursor == MenuItem::Exit) {
        screen = ScreenState::Home;
        editMode = false;
        renderHome();
        return;
    }

    editMode = !editMode;
    renderMenu();
}

uint8_t ScreenController::cursorIndex() const {
    switch (cursor) {
        case MenuItem::FilamentReset:
            return 0;
        case MenuItem::Temperature:
            return 1;
        case MenuItem::Stepper:
            return 2;
        case MenuItem::Exit:
            return 3;
    }

    return 0;
}

void ScreenController::setCursorByIndex(uint8_t index) {
    switch (index) {
        case 0:
            cursor = MenuItem::FilamentReset;
            break;
        case 1:
            cursor = MenuItem::Temperature;
            break;
        case 2:
            cursor = MenuItem::Stepper;
            break;
        default:
            cursor = MenuItem::Exit;
            break;
    }
}
