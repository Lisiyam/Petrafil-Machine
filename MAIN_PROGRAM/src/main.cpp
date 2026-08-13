#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <Preferences.h>
#include <SPI.h>
#include <math.h>

// =====================================================
// LCD 20x4 I2C
// =====================================================

constexpr uint8_t LCD_ADDRESS = 0x27;
constexpr uint8_t LCD_COLUMNS = 20;
constexpr uint8_t LCD_ROWS = 4;

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
Preferences preferences;


// =====================================================
// ADS1118 + NTC
// =====================================================

constexpr uint8_t ADS_CS = 17;
constexpr uint8_t ADS_SCK = 5;
constexpr uint8_t ADS_MISO = 16;
constexpr uint8_t ADS_MOSI = 4;

SPISettings adsSPI(1000000, MSBFIRST, SPI_MODE1);
constexpr uint16_t ADS_CONFIG = 0xC383;

constexpr float VCC = 3.3;
constexpr float R_BOTTOM = 10000.0;
constexpr float NTC_R25 = 100000.0;
constexpr float NTC_T25 = 298.15;
constexpr float NTC_BETA = 3950.0;

constexpr unsigned long TEMPERATURE_SAMPLE_MS = 500;
constexpr unsigned long ADS_CONVERSION_MS = 10;

double temperatureC = 0.0;
bool temperatureValid = false;
bool adsConversionPending = false;
unsigned long lastTemperatureSampleMs = 0;
unsigned long adsConversionStartedMs = 0;


// =====================================================
// HEATER + PURE PID
// =====================================================

constexpr uint8_t HEATER_PIN = 25;
constexpr uint8_t HEATER_PWM_CHANNEL = 0;
constexpr uint32_t PWM_FREQ = 5000;
constexpr uint8_t PWM_RES = 8;
constexpr double MAX_TEMP = 280.0;

double setpointC = 260.0;
double pidOutput = 0.0;
uint8_t heaterPWM = 0;
bool heaterFault = false;

double Kp = 19.192109;
double Ki = 0.366318;
double Kd = 251.378249;

PID hotendPID(&temperatureC, &pidOutput, &setpointC, Kp, Ki, Kd, DIRECT);


// =====================================================
// STEPPER
// =====================================================

constexpr uint8_t STEP_PIN = 27;
constexpr uint8_t DIR_PIN = 26;
constexpr uint8_t EN_PIN = 14;

constexpr unsigned int MIN_STEP_INTERVAL_US = 500;
constexpr unsigned int MAX_STEP_INTERVAL_US = 5000;
constexpr unsigned int STEP_PULSE_WIDTH_US = 4;

uint8_t stepperSpeed = 0;
bool stepPinHigh = false;
unsigned long lastStepUs = 0;
unsigned long stepPulseStartedUs = 0;


// =====================================================
// ROTARY ENCODER
// =====================================================

constexpr uint8_t LENGTH_ENC_CLK = 32;
constexpr uint8_t LENGTH_ENC_DT = 33;

constexpr uint8_t UI_ENC_CLK = 19;
constexpr uint8_t UI_ENC_DT = 18;
constexpr uint8_t UI_ENC_SW = 23;

constexpr unsigned long BUTTON_DEBOUNCE_MS = 40;
constexpr int FILAMENT_CM_PER_DETENT = 1;

struct EncoderEvent {
  int8_t rotation;
  bool clicked;
};

const int8_t TRANSITION_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

class RotaryEncoder {
public:
  RotaryEncoder(uint8_t clkPin, uint8_t dtPin, uint8_t swPin = 255)
      : clkPin(clkPin),
        dtPin(dtPin),
        swPin(swPin),
        previousState(0),
        position(0),
        previousButtonState(HIGH),
        lastButtonChangeMs(0) {}

  void begin() {
    pinMode(clkPin, INPUT_PULLUP);
    pinMode(dtPin, INPUT_PULLUP);

    if (hasButton()) {
      pinMode(swPin, INPUT_PULLUP);
      previousButtonState = digitalRead(swPin);
    }

    previousState = (digitalRead(clkPin) << 1) | digitalRead(dtPin);
  }

  EncoderEvent read() {
    EncoderEvent event{0, false};

    const uint8_t currentState = (digitalRead(clkPin) << 1) | digitalRead(dtPin);
    if (currentState != previousState) {
      const uint8_t transitionIndex = (previousState << 2) | currentState;
      const int8_t movement = TRANSITION_TABLE[transitionIndex];

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

    if (hasButton()) {
      const bool buttonState = digitalRead(swPin);
      const unsigned long now = millis();

      if (buttonState != previousButtonState && (now - lastButtonChangeMs) > BUTTON_DEBOUNCE_MS) {
        lastButtonChangeMs = now;
        previousButtonState = buttonState;

        if (buttonState == LOW) {
          event.clicked = true;
        }
      }
    }

    return event;
  }

private:
  uint8_t clkPin;
  uint8_t dtPin;
  uint8_t swPin;
  uint8_t previousState;
  int8_t position;
  bool previousButtonState;
  unsigned long lastButtonChangeMs;

  bool hasButton() const {
    return swPin != 255;
  }
};

RotaryEncoder lengthEncoder(LENGTH_ENC_CLK, LENGTH_ENC_DT);
RotaryEncoder uiEncoder(UI_ENC_CLK, UI_ENC_DT, UI_ENC_SW);

long filamentLengthCm = 0;


// =====================================================
// UI STATE
// =====================================================

enum class ScreenState {
  Loading,
  Home,
  Menu
};

enum class MenuItem {
  FilamentReset,
  Temperature,
  Stepper,
  Exit
};

ScreenState screen = ScreenState::Loading;
MenuItem cursor = MenuItem::FilamentReset;
bool editMode = false;
bool loadingFinished = false;
unsigned long loadingStartedMs = 0;
unsigned long lastRenderMs = 0;

constexpr unsigned long LOADING_DURATION_MS = 3000;
constexpr unsigned long LOADING_RENDER_MS = 150;
constexpr unsigned long HOME_RENDER_MS = 500;
constexpr uint8_t LOADING_BAR_WIDTH = 12;

constexpr int MIN_SETPOINT_C = 0;
constexpr int MAX_SETPOINT_C = 270;
constexpr int MIN_STEPPER_SPEED = 0;
constexpr int MAX_STEPPER_SPEED = 255;
constexpr int DEFAULT_SETPOINT_C = 260;
constexpr int DEFAULT_STEPPER_SPEED = 0;


// =====================================================
// LCD HELPERS
// =====================================================

void printPadded(uint8_t column, uint8_t row, String text, uint8_t width) {
  while (text.length() < width) {
    text += ' ';
  }

  if (text.length() > width) {
    text = text.substring(0, width);
  }

  lcd.setCursor(column, row);
  lcd.print(text);
}

String temperatureText() {
  if (!temperatureValid) {
    return "--.-";
  }

  return String(temperatureC, 1);
}


// =====================================================
// ADS1118 + TEMPERATURE
// =====================================================

uint16_t readADS1118() {
  SPI.beginTransaction(adsSPI);
  digitalWrite(ADS_CS, LOW);
  const uint16_t raw = SPI.transfer16(ADS_CONFIG);
  digitalWrite(ADS_CS, HIGH);
  SPI.endTransaction();

  return raw;
}

float calculateTemperature(float resistance) {
  if (resistance <= 0) {
    return NAN;
  }

  const float temperatureK =
      1.0 /
      ((1.0 / NTC_T25) + (1.0 / NTC_BETA) * log(resistance / NTC_R25));

  return temperatureK - 273.15;
}

float rawToTemperature(uint16_t raw) {
  const int16_t signedRaw = static_cast<int16_t>(raw);
  const float voltage = signedRaw * 4.096 / 32768.0;

  if (voltage <= 0.001 || voltage >= VCC) {
    return NAN;
  }

  const float ntcResistance = R_BOTTOM * ((VCC / voltage) - 1.0);
  return calculateTemperature(ntcResistance);
}

void serviceTemperature() {
  const unsigned long now = millis();

  if (!adsConversionPending && (now - lastTemperatureSampleMs >= TEMPERATURE_SAMPLE_MS)) {
    readADS1118();
    adsConversionStartedMs = now;
    adsConversionPending = true;
    return;
  }

  if (adsConversionPending && (now - adsConversionStartedMs >= ADS_CONVERSION_MS)) {
    const float newTemperature = rawToTemperature(readADS1118());
    temperatureValid = !isnan(newTemperature);

    if (temperatureValid) {
      temperatureC = newTemperature;
    }

    adsConversionPending = false;
    lastTemperatureSampleMs = now;
  }
}


// =====================================================
// HEATER CONTROL
// =====================================================

void setHeaterPWM(uint8_t pwm) {
  heaterPWM = pwm;
  ledcWrite(HEATER_PWM_CHANNEL, pwm);
}

void serviceHeater() {
  if (!temperatureValid) {
    setHeaterPWM(0);
    return;
  }

  if (temperatureC >= MAX_TEMP) {
    heaterFault = true;
    setHeaterPWM(0);
    hotendPID.SetMode(MANUAL);
    return;
  }

  if (heaterFault) {
    setHeaterPWM(0);
    return;
  }

  if (hotendPID.Compute()) {
    pidOutput = constrain(pidOutput, 0, 255);
    setHeaterPWM(static_cast<uint8_t>(pidOutput));
  }
}


// =====================================================
// STEPPER CONTROL
// =====================================================

unsigned int stepIntervalUs() {
  return map(stepperSpeed, 1, 255, MAX_STEP_INTERVAL_US, MIN_STEP_INTERVAL_US);
}

void serviceStepper() {
  if (stepperSpeed == 0 || heaterFault) {
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(EN_PIN, HIGH);
    stepPinHigh = false;
    return;
  }

  digitalWrite(EN_PIN, LOW);

  const unsigned long now = micros();

  if (stepPinHigh && (now - stepPulseStartedUs >= STEP_PULSE_WIDTH_US)) {
    digitalWrite(STEP_PIN, LOW);
    stepPinHigh = false;
    return;
  }

  if (!stepPinHigh && (now - lastStepUs >= stepIntervalUs())) {
    digitalWrite(STEP_PIN, HIGH);
    stepPinHigh = true;
    stepPulseStartedUs = now;
    lastStepUs = now;
  }
}


// =====================================================
// FILAMENT LENGTH
// =====================================================

void serviceLengthEncoder() {
  const EncoderEvent event = lengthEncoder.read();

  if (event.rotation != 0) {
    filamentLengthCm += event.rotation * FILAMENT_CM_PER_DETENT;

    if (filamentLengthCm < 0) {
      filamentLengthCm = 0;
    }
  }
}


// =====================================================
// SCREEN RENDER
// =====================================================

void renderLoading(uint8_t progress, const String& status) {
  const uint8_t filled = (progress * LOADING_BAR_WIDTH) / 100;
  String bar = "[";

  for (uint8_t i = 0; i < LOADING_BAR_WIDTH; i++) {
    bar += i < filled ? char(255) : ' ';
  }

  bar += "]";

  printPadded(0, 0, " PETRAFIL MACHINE ", 20);
  printPadded(0, 1, " STRIP PET SYSTEM ", 20);
  printPadded(0, 2, bar + " " + String(progress) + "%", 20);
  printPadded(0, 3, " STATUS: " + status, 20);
}

void renderHome() {
  printPadded(0, 0, " PETRAFIL MACHINE ", 20);
  printPadded(0, 1, "suhu    : " + temperatureText() + " C", 20);
  printPadded(0, 2, "filamen : " + String(filamentLengthCm) + " cm", 20);
  printPadded(0, 3, "stepper : " + String(stepperSpeed) + (heaterFault ? " FAULT" : ""), 20);
}

void renderMenu() {
  const bool onFilament = cursor == MenuItem::FilamentReset;
  const bool onTemperature = cursor == MenuItem::Temperature;
  const bool onStepper = cursor == MenuItem::Stepper;
  const bool onExit = cursor == MenuItem::Exit;

  printPadded(0, 0, String(onFilament ? ">" : " ") + "filamen: reset", 20);
  printPadded(0, 1, String(onTemperature ? ">" : " ") + "suhu   : " + String(static_cast<int>(setpointC)) + (editMode && onTemperature ? "*" : " ") + " C", 20);
  printPadded(0, 2, String(onStepper ? ">" : " ") + "stepper: " + String(stepperSpeed) + (editMode && onStepper ? "*" : " "), 20);
  printPadded(0, 3, String(onExit ? ">" : " ") + "      exit", 20);
}


// =====================================================
// SCREEN CONTROL
// =====================================================

uint8_t cursorIndex() {
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

void setCursorByIndex(uint8_t index) {
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

void moveCursor(int8_t direction) {
  int nextIndex = cursorIndex() + (direction > 0 ? 1 : -1);

  if (nextIndex < 0) {
    nextIndex = 3;
  } else if (nextIndex > 3) {
    nextIndex = 0;
  }

  setCursorByIndex(nextIndex);
}

void editSelectedValue(int8_t direction) {
  const int step = direction > 0 ? 1 : -1;

  if (cursor == MenuItem::Temperature) {
    setpointC = constrain(static_cast<int>(setpointC) + step, MIN_SETPOINT_C, MAX_SETPOINT_C);
  } else if (cursor == MenuItem::Stepper) {
    stepperSpeed = constrain(static_cast<int>(stepperSpeed) + step, MIN_STEPPER_SPEED, MAX_STEPPER_SPEED);
  }
}

void saveCurrentParameter() {
  if (cursor == MenuItem::Temperature) {
    preferences.putInt("setpoint", static_cast<int>(setpointC));
  } else if (cursor == MenuItem::Stepper) {
    preferences.putUChar("stepper", stepperSpeed);
  }
}

void selectCurrentItem() {
  if (cursor == MenuItem::FilamentReset) {
    filamentLengthCm = 0;
    renderMenu();
    return;
  }

  if (cursor == MenuItem::Exit) {
    screen = ScreenState::Home;
    editMode = false;
    renderHome();
    return;
  }

  if (editMode) {
    saveCurrentParameter();
  }

  editMode = !editMode;
  renderMenu();
}

void updateLoading() {
  const unsigned long now = millis();
  const unsigned long elapsed = now - loadingStartedMs;
  const uint8_t progress = min<uint8_t>(100, (elapsed * 100UL) / LOADING_DURATION_MS);

  if (now - lastRenderMs >= LOADING_RENDER_MS) {
    renderLoading(progress, progress >= 100 ? "READY" : "LOADING");
    lastRenderMs = now;
  }

  if (!loadingFinished && elapsed >= LOADING_DURATION_MS + 700) {
    loadingFinished = true;
    screen = ScreenState::Home;
    renderHome();
  }
}

void updateHome(const EncoderEvent& input) {
  const unsigned long now = millis();

  if (input.clicked) {
    screen = ScreenState::Menu;
    cursor = MenuItem::FilamentReset;
    editMode = false;
    renderMenu();
    return;
  }

  if (now - lastRenderMs >= HOME_RENDER_MS) {
    renderHome();
    lastRenderMs = now;
  }
}

void updateMenu(const EncoderEvent& input) {
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

void serviceDisplay() {
  const EncoderEvent input = uiEncoder.read();

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


// =====================================================
// SERIAL DEBUG
// =====================================================

void printStatus() {
  static unsigned long lastPrintMs = 0;
  const unsigned long now = millis();

  if (now - lastPrintMs < 1000) {
    return;
  }

  lastPrintMs = now;

  Serial.print("TEMP=");
  Serial.print(temperatureText());
  Serial.print(" C | SET=");
  Serial.print(setpointC, 1);
  Serial.print(" C | PWM=");
  Serial.print(heaterPWM);
  Serial.print(" | STEP=");
  Serial.print(stepperSpeed);
  Serial.print(" | FILAMENT=");
  Serial.print(filamentLengthCm);
  Serial.print(" cm | ");
  Serial.println(heaterFault ? "FAULT" : "RUN");
}


// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  preferences.begin("petrafil", false);
  setpointC = preferences.getInt("setpoint", DEFAULT_SETPOINT_C);
  stepperSpeed = preferences.getUChar("stepper", DEFAULT_STEPPER_SPEED);

  pinMode(ADS_CS, OUTPUT);
  digitalWrite(ADS_CS, HIGH);
  SPI.begin(ADS_SCK, ADS_MISO, ADS_MOSI, ADS_CS);

  ledcSetup(HEATER_PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(HEATER_PIN, HEATER_PWM_CHANNEL);

  setHeaterPWM(0);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(EN_PIN, HIGH);

  lengthEncoder.begin();
  uiEncoder.begin();

  hotendPID.SetOutputLimits(0, 255);
  hotendPID.SetSampleTime(500);
  hotendPID.SetMode(AUTOMATIC);

  loadingStartedMs = millis();
  lastTemperatureSampleMs = millis() - TEMPERATURE_SAMPLE_MS;
  renderLoading(0, "LOADING");

  Serial.println();
  Serial.println("========================================");
  Serial.println("          PETRAFIL MACHINE");
  Serial.println("========================================");
  Serial.println("Pure PID hotend + strip PET filament system");
  Serial.println();
}


// =====================================================
// LOOP
// =====================================================

void loop() {
  serviceTemperature();
  serviceHeater();
  serviceStepper();
  serviceLengthEncoder();
  serviceDisplay();
  printStatus();
}
